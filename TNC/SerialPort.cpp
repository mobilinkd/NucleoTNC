// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef NUCLEOTNC
#include "Log.h"
#include "bm78.h"
#endif
#include "SerialPort.hpp"
#include "PortInterface.h"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "main.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <atomic>

#ifdef NUCLEOTNC
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef& huart_serial = huart2;
#else
extern UART_HandleTypeDef huart3;
UART_HandleTypeDef& huart_serial = huart3;
#endif

extern osMessageQId ioEventQueueHandle;

std::atomic<uint32_t> uart_error{HAL_UART_ERROR_NONE};

std::atomic<bool> txDoneFlag{true};

uint8_t tmpBuffer[mobilinkd::tnc::TX_BUFFER_SIZE];
uint8_t tmpBuffer2[mobilinkd::tnc::TX_BUFFER_SIZE];

constexpr const int RX_BUFFER_SIZE = 127;
unsigned char rxBuffer[RX_BUFFER_SIZE * 2];

// 3 chunks of 128 bytes.  The first byte in each chunk is the length.
typedef mobilinkd::tnc::memory::Pool<
    3, RX_BUFFER_SIZE + 1> serial_pool_type;
serial_pool_type serialPool;

#ifndef NUCLEOTNC
void log_frame(mobilinkd::tnc::hdlc::IoFrame* frame)
{
    int pos = 0;
    for (auto c: *frame) {
        if (isprint(int(c))) pos += sprintf((char*)tmpBuffer2 + pos, " %c ", c);
        else pos += sprintf((char*)tmpBuffer2 + pos, "/%02x", c);
        if (pos > 80) {
          DEBUG((char*)tmpBuffer2);
          pos = 0;
        }
    }
    DEBUG((char*)tmpBuffer2);
}
#endif

// HAL does not have
HAL_StatusTypeDef UART_DMAPauseReceive(UART_HandleTypeDef *huart)
{
  /* Process Locked */
  __HAL_LOCK(huart);

  if ((huart->RxState == HAL_UART_STATE_BUSY_RX) &&
      (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)))
  {
    /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Disable the UART DMA Rx request */
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

HAL_StatusTypeDef UART_DMAResumeReceive(UART_HandleTypeDef *huart)
{
  /* Process Locked */
  __HAL_LOCK(huart);

  if (huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    /* Clear the Overrun flag before resuming the Rx transfer */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

    /* Reenable PE and ERR (Frame error, noise error, overrun error) interrupts */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the UART DMA Rx request */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

extern "C" void startSerialTask(void const* arg) __attribute__((optimize("-O1")));

void startSerialTask(void const* arg)
{
    using namespace mobilinkd::tnc;

    auto serialPort = static_cast<const SerialPort*>(arg);

    const uint8_t FEND = 0xC0;
    const uint8_t FESC = 0xDB;
    const uint8_t TFEND = 0xDC;
    const uint8_t TFESC = 0xDD;

    enum State {WAIT_FBEGIN, WAIT_FRAME_TYPE, WAIT_FEND, WAIT_ESCAPED};

    State state = WAIT_FBEGIN;

    hdlc::IoFrame* frame = hdlc::acquire_wait();

    HAL_UART_Receive_DMA(&huart_serial, rxBuffer, RX_BUFFER_SIZE * 2);
    __HAL_UART_ENABLE_IT(&huart_serial, UART_IT_IDLE);

    while (true) {
        osEvent evt = osMessageGet(serialPort->queue(), osWaitForever);

        if (evt.status != osEventMessage) {
            continue;
        }

        if (evt.value.v < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
        {
            // Error received.
            hdlc::release(frame);
#ifndef NUCLEOTNC
            ERROR("UART Error: %08lx", uart_error.load());
#endif
            uart_error.store(HAL_UART_ERROR_NONE);
            frame = hdlc::acquire_wait();
            HAL_UART_Receive_DMA(&huart_serial, rxBuffer, RX_BUFFER_SIZE * 2);
            __HAL_UART_ENABLE_IT(&huart_serial, UART_IT_IDLE);
            continue;
        }

        auto block = (serial_pool_type::chunk_type*) evt.value.p;
        auto data = static_cast<unsigned char*>(block->buffer);

        uint8_t end = data[0] + 1;
        for (uint8_t i = 1; i != end; ++i) {
            uint8_t c = data[i];
            switch (state) {
            case WAIT_FBEGIN:
                if (c == FEND) state = WAIT_FRAME_TYPE;
                break;
            case WAIT_FRAME_TYPE:
                if (c == FEND) break;   // Still waiting for FRAME_TYPE.
                frame->type(c);
                state = WAIT_FEND;
                break;
            case WAIT_FEND:
                switch (c) {
                case FESC:
                    state = WAIT_ESCAPED;
                    break;
                case FEND:
                    frame->source(frame->source() & 7);
                    if (osMessagePut(
                        ioEventQueueHandle,
                        reinterpret_cast<uint32_t>(frame),
                        osWaitForever) != osOK)
                    {
                        hdlc::release(frame);
                    }

                    if (hdlc::ioFramePool().size() < (hdlc::ioFramePool().capacity() / 4))
                    {
                        UART_DMAPauseReceive(&huart_serial);
                        while (hdlc::ioFramePool().size() < (hdlc::ioFramePool().capacity() / 2))
                        {
                            osThreadYield();
                        }
                        UART_DMAResumeReceive(&huart_serial);
                    }

                    frame = hdlc::acquire_wait();
                    state = WAIT_FBEGIN;
                    break;
                default:
                    if (not frame->push_back(c)) {
                        hdlc::release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::acquire_wait();
                    }
                }
                break;
            case WAIT_ESCAPED:
                state = WAIT_FEND;
                switch (c) {
                case TFESC:
                    if (not frame->push_back(FESC)) {
                        hdlc::release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::acquire_wait();
                    }
                    break;
                case TFEND:
                    if (not frame->push_back(FEND)) {
                        hdlc::release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::acquire_wait();
                    }
                    break;
                default:
                    hdlc::release(frame);
                    state = WAIT_FBEGIN;  // Drop frame;
                    frame = hdlc::acquire_wait();
                }
                break;
            }
        }
        serialPool.deallocate(block);
    }
}


extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*)
{
    txDoneFlag = true;
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t len = (RX_BUFFER_SIZE * 2) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (len == 0) return;
    if (len > RX_BUFFER_SIZE) {
        len = RX_BUFFER_SIZE; // wrapped.
    }

    auto block = serialPool.allocate();
    if (!block) return;
    memmove(block->buffer + 1, rxBuffer, len);
    block->buffer[0] = len;
    auto status = osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), (uint32_t) block, 0);
    if (status != osOK) serialPool.deallocate(block);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t hdmarx = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint32_t len = 0;
    if (hdmarx > RX_BUFFER_SIZE) {
        len = RX_BUFFER_SIZE;   // wrapped.
    } else {
        len = RX_BUFFER_SIZE - hdmarx;
    }
    if (len == 0) return;

    auto block = serialPool.allocate();
    if (!block) return;
    memmove(block->buffer + 1, rxBuffer + RX_BUFFER_SIZE, len);
    block->buffer[0] = len;
    auto status = osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), (uint32_t) block, 0);
    if (status != osOK) serialPool.deallocate(block);
}

extern "C" void idleInterruptCallback(UART_HandleTypeDef* huart)
{
    uint32_t len = (RX_BUFFER_SIZE * 2) - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (len == 0) return;

    auto block = serialPool.allocate();
    if (!block) return;

    HAL_UART_AbortReceive(huart);

    if (len > RX_BUFFER_SIZE) {
        // Second half
        len = len - RX_BUFFER_SIZE;
        memmove(block->buffer + 1, rxBuffer + RX_BUFFER_SIZE, len);
    } else {
        // First half
        memmove(block->buffer + 1, rxBuffer, len);
    }

    block->buffer[0] = len;

    HAL_UART_Receive_DMA(huart, rxBuffer, RX_BUFFER_SIZE * 2);

    auto status = osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), (uint32_t) block, 0);
    if (status != osOK) serialPool.deallocate(block);

}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), huart->ErrorCode, 0);
    uart_error.store((huart->gState<<16) | huart->ErrorCode);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_READY;
}

namespace mobilinkd { namespace tnc {

void SerialPort::init()
{
    if (serialTaskHandle_) return;

    osMessageQDef(uartQueue, 32, void*);
    queue_ = osMessageCreate(osMessageQ(uartQueue), 0);

    osMutexDef(uartMutex);
    mutex_ = osMutexCreate(osMutex(uartMutex));

    osThreadDef(serialTask, startSerialTask, osPriorityAboveNormal, 0, 128);
    serialTaskHandle_ = osThreadCreate(osThread(serialTask), this);
#ifndef NUCLEOTNC
    DEBUG("serialTaskHandle_ = %p", serialTaskHandle_);
#endif
}

bool SerialPort::open()
{
    if (open_ or !serialTaskHandle_) return open_;

    open_ = true;
    return open_;
}

void SerialPort::close()
{
    open_ = false;
}

bool SerialPort::write(const uint8_t* data, uint32_t size, uint8_t type, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    using ::mobilinkd::tnc::kiss::slip_encoder;

    auto slip_iter = slip_encoder((const char*)data, size);
    auto slip_end = slip_encoder();

    size_t pos = 0;
    memset(tmpBuffer, 0, TX_BUFFER_SIZE);

    tmpBuffer[pos++] = 0xC0;   // FEND
    tmpBuffer[pos++] = type;   // KISS Data Frame

    while (slip_iter != slip_end) {
        tmpBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {

            while (!txDoneFlag) osThreadYield();
            memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
            txDoneFlag = false;

            while (open_ and HAL_UART_Transmit_DMA(&huart_serial, TxBuffer, TX_BUFFER_SIZE) == HAL_BUSY)
            {
                if (osKernelSysTick() - start > timeout) {
                    osMutexRelease(mutex_);
                    txDoneFlag = true;
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
            memset(TxBuffer, 0, TX_BUFFER_SIZE);
        }
    }

    // Buffer has room for at least one more byte.
    tmpBuffer[pos++] = 0xC0;
    while (!txDoneFlag) osThreadYield();
    memcpy(TxBuffer, tmpBuffer, pos);
    txDoneFlag = false;
    while (open_ and HAL_UART_Transmit_DMA(&huart_serial, TxBuffer, pos) == HAL_BUSY)
    {
        if (osKernelSysTick() - start > timeout) {
            osMutexRelease(mutex_);
            txDoneFlag = true;
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
}

bool SerialPort::write(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    size_t pos = 0;
    memset(TxBuffer, 0, TX_BUFFER_SIZE);

    auto first = data;
    auto last = data + size;

    while (first != last) {
        TxBuffer[pos++] = *first++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and HAL_UART_Transmit(&huart_serial, TxBuffer, TX_BUFFER_SIZE, timeout) == HAL_BUSY)
            {
                if (osKernelSysTick() - start > timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
            memset(TxBuffer, 0, TX_BUFFER_SIZE);
        }
    }

    while (open_ and HAL_UART_Transmit(&huart_serial, TxBuffer, TX_BUFFER_SIZE, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() - start > timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    while (open_ and HAL_UART_Transmit(&huart_serial, (uint8_t*)"\r\n", 2, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() - start > timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
}

/*
 * Abort the DMA transmission. Release the mutex and the frame.  Set the
 * txDoneFlag so other writes may be attempted.
 *
 * This really sucks. The BM78 seems to just give up the ghost in BLE mode
 * when connected for long periods of time (and long is relative, but
 * typically more than an hour).  To deal with this, we reset the device
 * and the client needs to attempt to reconnect when disconnection is
 * detected.
 */
bool SerialPort::abort_tx(hdlc::IoFrame* frame)
{
    HAL_UART_AbortTransmit(&huart_serial);
    hdlc::release(frame);
#ifndef NUCLEOTNC
    WARN("SerialPort::write timed out -- DMA aborted.");
    HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
    bm78_wait_until_ready();
#endif
    txDoneFlag = true;
    osMutexRelease(mutex_);
    return false;
}

bool SerialPort::write(hdlc::IoFrame* frame, uint32_t timeout)
{
    if (!open_) {
        hdlc::release(frame);
        return false;
    }

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK) {
        hdlc::release(frame);
        return false;
    }

    using ::mobilinkd::tnc::kiss::slip_encoder2;

    hdlc::IoFrame::iterator begin = frame->begin();
    hdlc::IoFrame::iterator end = frame->begin();
    std::advance(end, frame->size() - 2);           // Drop FCS

    slip_encoder2 slip_iter{begin};
    slip_encoder2 slip_end{end};

    size_t pos = 0;

    tmpBuffer[pos++] = 0xC0;   // FEND
    tmpBuffer[pos++] = static_cast<int>((frame->source() | frame->type()) & 0x7F);   // KISS Data Frame

    while (slip_iter != slip_end)
    {
        tmpBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {
            while (!txDoneFlag) {
                // txDoneFlag set in HAL_UART_TxCpltCallback() above when DMA completes.
                if (osKernelSysTick() - start > timeout) {
                    return abort_tx(frame); // Abort DMA xfer on timeout.
                } else {
                    osThreadYield();
                }
            }
            memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
            txDoneFlag = false;
            while (open_ and HAL_UART_Transmit_DMA(&huart_serial, TxBuffer, TX_BUFFER_SIZE) == HAL_BUSY)
            {
                // This should not happen.  HAL_BUSY should not occur when txDoneFlag set.
                if (osKernelSysTick() - start > timeout) {
                    return abort_tx(frame); // Abort DMA xfer on timeout.
                } else {
                    osThreadYield();
                }
            }
            pos = 0;
        }
    }

    // Buffer has room for at least one more byte.
    tmpBuffer[pos++] = 0xC0;

    while (!txDoneFlag) {
        // txDoneFlag set in HAL_UART_TxCpltCallback() above when DMA completes.
        if (osKernelSysTick() - start > timeout) {
            return abort_tx(frame); // Abort DMA xfer on timeout.
        } else {
            osThreadYield();
        }
    }

    memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
    txDoneFlag = false;
    while (open_ and HAL_UART_Transmit_DMA(&huart_serial, TxBuffer, pos) == HAL_BUSY) {
        // This should not happen.  HAL_BUSY should not occur when txDoneFlag set.
        if (osKernelSysTick() - start > timeout) {
            return abort_tx(frame); // Abort DMA xfer on timeout.
        } else {
            osThreadYield();
        }
    }

    osMutexRelease(mutex_);
    hdlc::release(frame);

    DEBUG("SerialPort::write COMPLETE");

    return true;
}


SerialPort* getSerialPort()
{
    static SerialPort instance;
    return &instance;
}

}} // mobilinkd::tnc
