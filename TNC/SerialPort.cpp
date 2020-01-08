// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "SerialPort.hpp"
#include "SerialPort.h"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "main.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <atomic>

extern UART_HandleTypeDef huart2;
extern osMessageQId ioEventQueueHandle;

extern "C" void startSerialTask(void const* arg);

osThreadId serialTaskHandle;
uint32_t serialTaskBuffer[ 256 ];
osStaticThreadDef_t serialTaskControlBlock;
osThreadStaticDef(serialTask, startSerialTask, osPriorityNormal, 0, 128, serialTaskBuffer, &serialTaskControlBlock);

constexpr const int RX_BUFFER_SIZE = 127;
unsigned char rxBuffer[RX_BUFFER_SIZE * 2];

// 3 chunks of 128 bytes.  The first byte in each chunk is the length.
typedef mobilinkd::tnc::memory::Pool<
    3, RX_BUFFER_SIZE + 1> serial_pool_type;
serial_pool_type serialPool;

extern "C" void startSerialTask(void const* arg)
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

    HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE * 2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    while (true) {
        osEvent evt = osMessageGet(serialPort->queue(), osWaitForever);

        if (evt.status != osEventMessage) {
            continue;
        }

        auto block = (serial_pool_type::chunk_type*) evt.value.p;
        auto data = static_cast<unsigned char*>(block->buffer);

        uint8_t len = data[0];
        for (uint8_t i = 0; i != len; ++i) {
            uint8_t c = data[i+1];
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
                    frame->source(hdlc::IoFrame::SERIAL_DATA);
                    osMessagePut(ioEventQueueHandle, reinterpret_cast<uint32_t>(frame), osWaitForever);
                    frame = hdlc::acquire_wait();
                    state = WAIT_FBEGIN;
                    break;
                default:
                    if (not frame->push_back(c)) {
                        hdlc::ioFramePool().release(frame);
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
                        hdlc::ioFramePool().release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::acquire_wait();
                    }
                    break;
                case TFEND:
                    if (not frame->push_back(FEND)) {
                        hdlc::ioFramePool().release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::acquire_wait();
                    }
                    break;
                default:
                    hdlc::ioFramePool().release(frame);
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
    osMutexRelease(mobilinkd::tnc::getSerialPort()->mutex_);
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t len = (RX_BUFFER_SIZE * 2) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (len == 0) return;

    auto block = serialPool.allocate();
    if (!block) return;
    memmove(block->buffer + 1, rxBuffer, len);
    auto status = osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), (uint32_t) block, 0);
    if (status != osOK) serialPool.deallocate(block);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (len == 0) return;

    auto block = serialPool.allocate();
    if (!block) return;
    memmove(block->buffer + 1, rxBuffer + RX_BUFFER_SIZE, len);
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

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*)
{
    Error_Handler();
}

namespace mobilinkd { namespace tnc {

void SerialPort::init()
{
    if (serialTaskHandle_) return;

    osMessageQDef(uartQueue, 32, void*);
    queue_ = osMessageCreate(osMessageQ(uartQueue), 0);

    osMutexDef(uartMutex);
    mutex_ = osMutexCreate(osMutex(uartMutex));

    serialTaskHandle = osThreadCreate(osThread(serialTask), this);

    serialTaskHandle_ = serialTaskHandle;
    // DEBUG("serialTaskHandle_ = %p", serialTaskHandle_);
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

    TxBuffer[pos++] = 0xC0;   // FEND
    TxBuffer[pos++] = type;   // KISS Data Frame

    while (slip_iter != slip_end) {
        TxBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY)
            {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
        }
    }

    // Buffer has room for at least one more byte.
    TxBuffer[pos++] = 0xC0;
    while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
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

    auto first = data;
    auto last = data + size;

    while (first != last) {
        TxBuffer[pos++] = *first++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY)
            {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
        }
    }

    while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    while (open_ and HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
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

    auto slip_iter = slip_encoder2(begin);
    auto slip_end = slip_encoder2(end);

    size_t pos = 0;

    TxBuffer[pos++] = 0xC0;   // FEND
    TxBuffer[pos++] = static_cast<int>(frame->type());   // KISS Data Frame

    while (slip_iter != slip_end) {
        TxBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY)
            {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    hdlc::release(frame);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
        }
    }

    // Buffer has room for at least one more byte.
    TxBuffer[pos++] = 0xC0;
    while (open_ and HAL_UART_Transmit(&huart2, TxBuffer, pos, timeout) == HAL_BUSY) {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            hdlc::release(frame);
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);
    hdlc::release(frame);

    return true;
}


SerialPort* getSerialPort()
{
    static SerialPort instance;
    return &instance;
}


}} // mobilinkd::tnc

void initSerial()
{
    mobilinkd::tnc::getSerialPort()->init();
}

int openSerial()
{
    mobilinkd::tnc::PortInterface* tmp = mobilinkd::tnc::getSerialPort();
    tmp->open();
    if (mobilinkd::tnc::ioport != tmp and tmp->isOpen())
    {
        std::swap(tmp, mobilinkd::tnc::ioport);
        if (tmp) tmp->close();
        return true;
    }
    return mobilinkd::tnc::ioport == tmp;
}

int writeSerial(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    return mobilinkd::tnc::getSerialPort()->write(data, size, timeout);
}

