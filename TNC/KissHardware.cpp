// Copyright 2018-2022 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "KissHardware.hpp"
#include "PortInterface.hpp"
#include "AudioInput.hpp"
#include "AudioLevel.hpp"
#include "IOEventTask.h"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "HDLCEncoder.hpp"
#ifndef NUCLEOTNC
#include "KissHardware.h"
#endif

#include <array>
#include <cstdio>
#include <cstring>
#include <alloca.h>

#ifdef NUCLEOTNC
extern I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef& eeprom_i2c = hi2c3;
#else
extern I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef& eeprom_i2c = hi2c1;
#endif

extern RTC_HandleTypeDef hrtc;

#ifndef NUCLEOTNC
int powerOnViaUSB(void)
{
    return mobilinkd::tnc::kiss::settings().options & KISS_OPTION_VIN_POWER_ON;
}

int powerOffViaUSB(void)
{
    return mobilinkd::tnc::kiss::settings().options & KISS_OPTION_VIN_POWER_OFF;
}
#endif

namespace mobilinkd { namespace tnc { namespace kiss {

#if defined(NUCLEOTNC)
const char FIRMWARE_VERSION[] = "2.4.4";
const char HARDWARE_VERSION[] = "Mobilinkd NucleoTNC";
#elif defined(STM32L433xx)
const char FIRMWARE_VERSION[] = "2.4.4";
const char HARDWARE_VERSION[] = "Mobilinkd TNC3 2.1.1";
#elif defined(STM32L4P5xx)
const char FIRMWARE_VERSION[] = "2.4.4";
const char HARDWARE_VERSION[] = "Mobilinkd TNC3+ Rev A";
#endif
Hardware& settings()
{
    static Hardware instance __attribute__((section(".bss3")));
    return instance;
}

const uint8_t* get_rtc_datetime()
{
    static uint8_t buffer[8]; // YYMMDDWWHHMMSS

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    ::memset(&sTime, 0, sizeof(sTime));
    ::memset(&sDate, 0, sizeof(sDate));

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

    buffer[0] = sDate.Year;
    buffer[1] = sDate.Month;
    buffer[2] = sDate.Date;
    buffer[3] = sDate.WeekDay;
    buffer[4] = sTime.Hours;
    buffer[5] = sTime.Minutes;
    buffer[6] = sTime.Seconds;
    buffer[7] = 0;

    return buffer;
}

// TODO: determine why this is now necessary.
void set_rtc_datetime(const uint8_t* buffer) __attribute__((optimize("-O0")));

void set_rtc_datetime(const uint8_t* buffer)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    ::memset(&sTime, 0, sizeof(sTime));
    ::memset(&sDate, 0, sizeof(sDate));

    sDate.Year = buffer[0];
    sDate.Month = buffer[1];
    sDate.Date = buffer[2];
    sDate.WeekDay = buffer[3];
    sTime.Hours = buffer[4];
    sTime.Minutes = buffer[5];
    sTime.Seconds = buffer[6];

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
}

void Hardware::set_txdelay(uint8_t value) {
    txdelay = value;
    update_crc();
}
void Hardware::set_ppersist(uint8_t value) {
    ppersist = value;
    update_crc();
}
void Hardware::set_slottime(uint8_t value) {
    slot = value;
    update_crc();
}
void Hardware::set_txtail(uint8_t value) {
    txtail = value;
    update_crc();
}
void Hardware::set_duplex(uint8_t value) {
    duplex = value;
    update_crc();
}

void reply8(uint8_t cmd, uint8_t result) {
    uint8_t data[2] { cmd, result };
    ioport->write(data, 2, 6, osWaitForever);
}

void reply16(uint8_t cmd, uint16_t result) {
    uint8_t data[3] { cmd, uint8_t((result >> 8) & 0xFF), uint8_t(result & 0xFF) };
    ioport->write(data, 3, 6, osWaitForever);
}

void reply(uint8_t cmd, const uint8_t* data, uint16_t len) {
    uint8_t* buffer = static_cast<uint8_t*>(alloca(len + 1));
    buffer[0] = cmd;
    for (uint16_t i = 0; i != len; i++)
        buffer[i + 1] = data[i];
    ioport->write(buffer, len + 1, 6, osWaitForever);
}

template <size_t N>
void reply_ext(const std::array<uint8_t, N>& cmd, const uint8_t* data, uint16_t len) {
    auto buffer = static_cast<uint8_t*>(alloca(len + N));
    if (buffer == nullptr) return;
    std::copy(std::begin(cmd), std::end(cmd), buffer);
    for (uint16_t i = 0; i != len and data[i] != 0; i++)
        buffer[i + N] = data[i];
    ioport->write(buffer, len + 2, 6, osWaitForever);
}

template <size_t N>
inline void ext_reply(const std::array<uint8_t, N>& cmd, uint8_t result) {
    std::array<uint8_t, 1 + N> buffer;
    std::copy(std::begin(cmd), std::end(cmd), std::begin(buffer));
    buffer[N] = result;
    ioport->write(buffer.data(), N + 1, 6, osWaitForever);
}

template <size_t M, size_t N>
void ext_reply(const std::array<uint8_t, M>& cmd, const std::array<uint8_t, N>& result) {
    std::array<uint8_t, M + N> data;
    std::copy(std::begin(cmd), std::end(cmd), std::begin(data));
    std::copy(std::begin(result), std::end(result), std::begin(data) + M);
    ioport->write(data.data(), M + N, 6, osWaitForever);
}

void Hardware::get_alias(uint8_t alias) {
    uint8_t result[14];
    if (alias >= NUMBER_OF_ALIASES or not aliases[alias].set) return;
    result[0] = alias;
    memcpy(result + 1, aliases[alias].call.data(), aliases[alias].call.size());
    result[9] = aliases[alias].set;
    result[10] = aliases[alias].use;
    result[11] = aliases[alias].insert_id;
    result[12] = aliases[alias].preempt;
    result[13] = aliases[alias].hops;
    reply_ext(hardware::EXT_GET_ALIASES, result, 14);
}

void Hardware::set_alias(const hdlc::IoFrame* frame) {
  UNUSED(frame);
}

void Hardware::announce_input_settings()
{
    reply16(hardware::GET_INPUT_GAIN, input_gain);
    reply8(hardware::GET_INPUT_TWIST, rx_twist);
}

AFSKTestTone& getAFSKTestTone() {
     static AFSKTestTone testTone;
     return testTone;
}

void Hardware::handle_request(hdlc::IoFrame* frame)
{
    auto it = frame->begin();
    uint8_t command = *it++;

    switch (command) {
    case hardware::SEND_MARK:
    case hardware::SEND_SPACE:
    case hardware::SEND_BOTH:
    case hardware::SET_OUTPUT_GAIN:
    case hardware::SET_OUTPUT_OFFSET:
    case hardware::SET_OUTPUT_TWIST:
        break;
    default:
        getAFSKTestTone().stop();
    }

    switch (command) {

    case hardware::SAVE:
    case hardware::SAVE_EEPROM_SETTINGS:
        update_crc();
        store();
        reply8(hardware::SAVE_EEPROM_SETTINGS, hardware::OK);
        break;
    case hardware::POLL_INPUT_LEVEL:
        TNC_DEBUG("POLL_INPUT_VOLUME");
        reply8(hardware::POLL_INPUT_LEVEL, 0);
        osMessagePut(audioInputQueueHandle, audio::POLL_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
            osWaitForever);
        break;
    case hardware::STREAM_INPUT_LEVEL:
        TNC_DEBUG("STREAM_INPUT_VOLUME");
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;
    case hardware::GET_BATTERY_LEVEL:
      TNC_DEBUG("GET_BATTERY_LEVEL");
      osMessagePut(audioInputQueueHandle, audio::POLL_BATTERY_LEVEL,
          osWaitForever);
      osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
          osWaitForever);
        break;
    case hardware::SEND_MARK:
        TNC_DEBUG("SEND_MARK");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().mark();
        break;
    case hardware::SEND_SPACE:
        TNC_DEBUG("SEND_SPACE");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().space();
        break;
    case hardware::SEND_BOTH:
        TNC_DEBUG("SEND_BOTH");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().both();
        break;
    case hardware::STOP_TX:
        TNC_DEBUG("STOP_TX");
        getAFSKTestTone().stop();
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        break;
    case hardware::RESET:
        TNC_DEBUG("RESET");
        osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
            osWaitForever);
        break;
    case hardware::SET_OUTPUT_GAIN:
        output_gain = *it << 8;
        ++it;
        output_gain += *it;
        TNC_DEBUG("SET_OUTPUT_GAIN = %hd", output_gain);
        audio::setAudioOutputLevel();
        update_crc();
        [[fallthrough]];
    case hardware::GET_OUTPUT_GAIN:
        TNC_DEBUG("GET_OUTPUT_GAIN");
        reply16(hardware::GET_OUTPUT_GAIN, output_gain);
        break;

    case hardware::STREAM_DCD_VALUE:
        TNC_DEBUG("STREAM_DCD_VALUE");
        break;

    case hardware::POLL_INPUT_TWIST:
      TNC_DEBUG("POLL_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::POLL_TWIST_LEVEL,
          osWaitForever);
      osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
          osWaitForever);
        break;

    case hardware::STREAM_AVG_INPUT_TWIST:
      TNC_DEBUG("STREAM_AVG_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::STREAM_AVERAGE_TWIST_LEVEL,
          osWaitForever);
        break;

    case hardware::STREAM_INPUT_TWIST:
      TNC_DEBUG("STREAM_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::STREAM_INSTANT_TWIST_LEVEL,
          osWaitForever);
        break;

    case hardware::ADJUST_INPUT_LEVELS:
        TNC_DEBUG("ADJUST_INPUT_LEVELS");
        osMessagePut(audioInputQueueHandle, audio::AUTO_ADJUST_INPUT_LEVEL,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;

    case hardware::SET_INPUT_GAIN:
        input_gain = *it << 8;
        ++it;
        input_gain += *it;
        TNC_DEBUG("SET_INPUT_GAIN = %d", input_gain);
        update_crc();
        osMessagePut(audioInputQueueHandle, audio::UPDATE_SETTINGS,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        [[fallthrough]];
    case hardware::GET_INPUT_GAIN:
        TNC_DEBUG("GET_INPUT_GAIN");
        reply16(hardware::GET_INPUT_GAIN, input_gain);
        break;

    case hardware::SET_INPUT_TWIST:
        TNC_DEBUG("SET_INPUT_TWIST");
        rx_twist = *it;
        update_crc();
        osMessagePut(audioInputQueueHandle, audio::UPDATE_SETTINGS,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        [[fallthrough]];
    case hardware::GET_INPUT_TWIST:
        TNC_DEBUG("GET_INPUT_TWIST");
        reply8(hardware::GET_INPUT_TWIST, rx_twist);
        break;

    case hardware::SET_OUTPUT_TWIST:
        tx_twist = *it;
        if (tx_twist < 0) tx_twist = 0;
        if (tx_twist > 100) tx_twist = 100;
        TNC_DEBUG("SET_OUTPUT_TWIST: %d", int(tx_twist));
        getModulator().init(*this);
        update_crc();
        [[fallthrough]];
    case hardware::GET_OUTPUT_TWIST:
        TNC_DEBUG("GET_OUTPUT_TWIST");
        reply8(hardware::GET_OUTPUT_TWIST, tx_twist);
        break;

    case hardware::STREAM_AMPLIFIED_INPUT:
        TNC_DEBUG("STREAM_AMPLIFIED_INPUT");
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;

    case hardware::GET_TXDELAY:
        TNC_DEBUG("GET_TXDELAY");
        reply8(hardware::GET_TXDELAY, txdelay);
        break;

    case hardware::GET_PERSIST:
        TNC_DEBUG("GET_PERSIST");
        reply8(hardware::GET_PERSIST, ppersist);
        break;

    case hardware::GET_TIMESLOT:
        TNC_DEBUG("GET_TIMESLOT");
        reply8(hardware::GET_TIMESLOT, slot);
        break;

    case hardware::GET_TXTAIL:
        TNC_DEBUG("GET_TXTAIL");
        reply8(hardware::GET_TXTAIL, txtail);
        break;

    case hardware::GET_DUPLEX:
        TNC_DEBUG("GET_DUPLEX");
        reply8(hardware::GET_DUPLEX, duplex);
        break;

    case hardware::GET_FIRMWARE_VERSION:
        TNC_DEBUG("GET_FIRMWARE_VERSION");
        reply(hardware::GET_FIRMWARE_VERSION, (uint8_t*) FIRMWARE_VERSION,
          sizeof(FIRMWARE_VERSION) - 1);
        break;
    case hardware::GET_HARDWARE_VERSION:
        TNC_DEBUG("GET_HARDWARE_VERSION");
        reply(hardware::GET_HARDWARE_VERSION, (uint8_t*) HARDWARE_VERSION,
          sizeof(HARDWARE_VERSION) - 1);
        break;

    case hardware::GET_SERIAL_NUMBER:
        TNC_DEBUG("GET_SERIAL_NUMBER");
        reply(hardware::GET_SERIAL_NUMBER, (uint8_t*) serial_number_64,
            sizeof(serial_number_64) - 1);
        break;

    case hardware::SET_PTT_CHANNEL:
        TNC_DEBUG("SET_PTT_CHANNEL");
        if (*it) {
            options &= ~KISS_OPTION_PTT_SIMPLEX;
            osMessagePut(ioEventQueueHandle, CMD_SET_PTT_MULTIPLEX, osWaitForever);
        } else {
            options |= KISS_OPTION_PTT_SIMPLEX;
            osMessagePut(ioEventQueueHandle, CMD_SET_PTT_SIMPLEX, osWaitForever);
        }
        update_crc();
        break;
    case hardware::GET_PTT_CHANNEL:
        TNC_DEBUG("GET_PTT_CHANNEL");
        reply8(hardware::GET_PTT_CHANNEL,
            options & KISS_OPTION_PTT_SIMPLEX ? 0 : 1);
        break;

    case hardware::SET_PASSALL:
        TNC_DEBUG("SET_PASSALL");
        if (*it) {
          options |= KISS_OPTION_PASSALL;
        } else {
          options &= ~KISS_OPTION_PASSALL;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_PASSALL:
        TNC_DEBUG("GET_PASSALL");
        reply8(hardware::GET_PASSALL, options & KISS_OPTION_PASSALL ? 1 : 0);
        break;

    case hardware::SET_RX_REV_POLARITY:
        TNC_DEBUG("SET_RX_REV_POLARITY");
        if (*it) {
          options |= KISS_OPTION_RX_REV_POLARITY;
        } else {
          options &= ~KISS_OPTION_RX_REV_POLARITY;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_RX_REV_POLARITY:
        TNC_DEBUG("GET_RX_REV_POLARITY");
        reply8(hardware::GET_RX_REV_POLARITY, options & KISS_OPTION_RX_REV_POLARITY ? 1 : 0);
        break;

    case hardware::SET_TX_REV_POLARITY:
        TNC_DEBUG("SET_TX_REV_POLARITY");
        if (*it) {
          options |= KISS_OPTION_TX_REV_POLARITY;
        } else {
          options &= ~KISS_OPTION_TX_REV_POLARITY;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_TX_REV_POLARITY:
        TNC_DEBUG("GET_TX_REV_POLARITY");
        reply8(hardware::GET_TX_REV_POLARITY, options & KISS_OPTION_TX_REV_POLARITY ? 1 : 0);
        break;

#ifndef NUCLEOTNC
    case hardware::SET_USB_POWER_OFF:
        TNC_DEBUG("SET_USB_POWER_OFF");
        if (*it) {
          options |= KISS_OPTION_VIN_POWER_OFF;
        } else {
          options &= ~KISS_OPTION_VIN_POWER_OFF;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_USB_POWER_OFF:
        TNC_DEBUG("GET_USB_POWER_OFF");
        reply8(hardware::GET_USB_POWER_OFF,
            options & KISS_OPTION_VIN_POWER_OFF ? 1 : 0);
        break;

    case hardware::SET_USB_POWER_ON:
        TNC_DEBUG("SET_USB_POWER_ON");
        if (*it) {
          options |= KISS_OPTION_VIN_POWER_ON;
        } else {
          options &= ~KISS_OPTION_VIN_POWER_ON;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_USB_POWER_ON:
        TNC_DEBUG("GET_USB_POWER_ON");
        reply8(hardware::GET_USB_POWER_ON,
            options & KISS_OPTION_VIN_POWER_ON ? 1 : 0);
        break;
#endif

    case hardware::SET_DATETIME:
        TNC_DEBUG("SET_DATETIME");
        set_rtc_datetime(static_cast<const uint8_t*>(&*it));
        [[fallthrough]];
    case hardware::GET_DATETIME:
        TNC_DEBUG("GET_DATETIME");
        reply(hardware::GET_DATETIME, get_rtc_datetime(), 7);
        break;

    case hardware::GET_CAPABILITIES:
        TNC_DEBUG("GET_CAPABILITIES");
#ifndef NUCLEOTNC
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|hardware::CAP_BATTERY_LEVEL|
            hardware::CAP_ADJUST_INPUT|hardware::CAP_DFU_FIRMWARE);
#else
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|
            hardware::CAP_ADJUST_INPUT|
            hardware::CAP_DFU_FIRMWARE);
#endif
        break;

    case hardware::GET_ALL_VALUES:
        TNC_DEBUG("GET_ALL_VALUES");
        // GET_API_VERSION must always come first.
        reply16(hardware::GET_API_VERSION, hardware::KISS_API_VERSION);
#ifndef NUCLEOTNC
        osMessagePut(audioInputQueueHandle, audio::POLL_BATTERY_LEVEL,
            osWaitForever);
        reply8(hardware::GET_USB_POWER_OFF, options & KISS_OPTION_VIN_POWER_OFF ? 1 : 0);
        reply8(hardware::GET_USB_POWER_ON, options & KISS_OPTION_VIN_POWER_ON ? 1 : 0);
        reply(hardware::GET_MAC_ADDRESS, mac_address, sizeof(mac_address));
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|hardware::CAP_BATTERY_LEVEL|
            hardware::CAP_ADJUST_INPUT|hardware::CAP_DFU_FIRMWARE);
#else
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|
            hardware::CAP_ADJUST_INPUT|
            hardware::CAP_DFU_FIRMWARE);
#endif
        osMessagePut(audioInputQueueHandle, audio::POLL_TWIST_LEVEL,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        reply(hardware::GET_FIRMWARE_VERSION, (uint8_t*) FIRMWARE_VERSION,
          sizeof(FIRMWARE_VERSION) - 1);
        reply(hardware::GET_HARDWARE_VERSION, (uint8_t*) HARDWARE_VERSION,
          sizeof(HARDWARE_VERSION) - 1);
        reply(hardware::GET_SERIAL_NUMBER, (uint8_t*) serial_number_64,
            sizeof(serial_number_64) - 1);
        reply16(hardware::GET_OUTPUT_GAIN, output_gain);
        reply8(hardware::GET_OUTPUT_TWIST, tx_twist);
        reply16(hardware::GET_INPUT_GAIN, input_gain);
        reply8(hardware::GET_INPUT_TWIST, rx_twist);
        reply8(hardware::GET_TXDELAY, txdelay);
        reply8(hardware::GET_PERSIST, ppersist);
        reply8(hardware::GET_TIMESLOT, slot);
        reply8(hardware::GET_TXTAIL, txtail);
        reply8(hardware::GET_DUPLEX, duplex);
        reply8(hardware::GET_PTT_CHANNEL,
            options & KISS_OPTION_PTT_SIMPLEX ? 0 : 1);
        reply8(hardware::GET_PASSALL, options & KISS_OPTION_PASSALL ? 1 : 0);
        reply16(hardware::GET_MIN_INPUT_GAIN, 0);   // Constants for this FW
        reply16(hardware::GET_MAX_INPUT_GAIN, 4);   // Constants for this FW
        reply8(hardware::GET_MIN_INPUT_TWIST, -3);  // Constants for this FW
        reply8(hardware::GET_MAX_INPUT_TWIST, 9);   // Constants for this FW
        ext_reply(hardware::EXT_GET_MODEM_TYPE, modem_type);
        ext_reply(hardware::EXT_GET_MODEM_TYPES, supported_modem_types);
        if (*error_message) {
            reply(hardware::GET_ERROR_MSG, (uint8_t*) error_message, sizeof(error_message));
        }
        // GET_DATETIME must always be last.  iOS config app depends on it.
        reply(hardware::GET_DATETIME, get_rtc_datetime(), 7);
        reply8(hardware::GET_RX_REV_POLARITY, options & KISS_OPTION_RX_REV_POLARITY ? 1 : 0);
        reply8(hardware::GET_TX_REV_POLARITY, options & KISS_OPTION_TX_REV_POLARITY ? 1 : 0);
        break;
    default:
        if (command > 0xC0)
        {
            handle_ext_request(frame);
        }
        else
        {
            ERROR("Unknown hardware request");
        }
    }
}

void Hardware::handle_ext_request(hdlc::IoFrame* frame) {
    auto it = frame->begin();
    ++it;
    // Currently only supports 2-byte extended commands.
    uint8_t ext_command = *it++;

    switch (ext_command) {

    case hardware::EXT_SET_MODEM_TYPE[1]:
        TNC_DEBUG("SET_MODEM_TYPE");
        if ((*it == hardware::MODEM_TYPE_1200)
            or (*it == hardware::MODEM_TYPE_9600)
            or (*it == hardware::MODEM_TYPE_M17))
        {
            modem_type = *it;
            TNC_DEBUG(modem_type_lookup[modem_type]);
            update_crc();
        }
        else
        {
            ERROR("Unsupported modem type");
        }
        osMessagePut(hdlcOutputQueueHandle, 0, osWaitForever);      // Reset encoder/modulator.
        osMessagePut(audioInputQueueHandle, audio::UPDATE_SETTINGS, // Reset decoder/demodulator.
            osWaitForever);
        [[fallthrough]];
    case hardware::EXT_GET_MODEM_TYPE[1]:
        TNC_DEBUG("EXT_GET_MODEM_TYPE");
        ext_reply(hardware::EXT_GET_MODEM_TYPE, modem_type);
        break;
    case hardware::EXT_GET_MODEM_TYPES[1]:
        TNC_DEBUG("EXT_GET_MODEM_TYPES");
        ext_reply(hardware::EXT_GET_MODEM_TYPES, supported_modem_types);
        break;
    default:
        ERROR("Unknown extended hardware request");
    }
}

Hardware tmpHardware;

bool Hardware::load()
{
    INFO("Loading settings from EEPROM");

    memset(&tmpHardware, 0, sizeof(Hardware));

    if (!I2C_Storage::load(tmpHardware)) {
        ERROR("EEPROM read failed");
        return false;
    }

    if (tmpHardware.crc_ok())
    {
        memcpy(this, &tmpHardware, sizeof(Hardware));
        return true;
    }
    ERROR("EEPROM CRC error");
    return false;
}

bool Hardware::store() const
{
    INFO("Saving settings to EEPROM");

    if (!I2C_Storage::store(*this)) {
        ERROR("EEPROM write failed");
        return false;
    }

    INFO("EEPROM saved checksum is: %04hx (crc = %04hx)", checksum, crc());

    return crc_ok();
}

bool I2C_Storage::load(void* ptr, size_t len)
{
    if (HAL_I2C_Init(&eeprom_i2c) != HAL_OK) CxxErrorHandler();

    TNC_DEBUG("Attempting to read %d bytes from EEPROM...", len);

    uint32_t timeout = 1000;    // systicks... milliseconds

    auto tmp = static_cast<uint8_t*>(ptr);
    auto result = HAL_I2C_Mem_Read(&eeprom_i2c, i2c_address, 0,
        I2C_MEMADD_SIZE_16BIT, tmp, len, timeout);
    if (result != HAL_OK) CxxErrorHandler();

    if (HAL_I2C_DeInit(&eeprom_i2c) != HAL_OK) CxxErrorHandler();

    return true;
}

bool I2C_Storage::store(const void* ptr, size_t len)
{
    if (HAL_I2C_Init(&eeprom_i2c) != HAL_OK) CxxErrorHandler();

    auto tmp = const_cast<uint8_t*>(static_cast<const uint8_t*>(ptr));

    uint32_t index = 0;
    size_t remaining = len;
    while (remaining > page_size)
    {
        auto result = HAL_I2C_Mem_Write(&eeprom_i2c, i2c_address, index, I2C_MEMADD_SIZE_16BIT, tmp + index, page_size, 20);
        if (result != HAL_OK) {
            ERROR("EEPROM write block error = %lu.", eeprom_i2c.ErrorCode);
            if (HAL_I2C_DeInit(&eeprom_i2c) != HAL_OK) CxxErrorHandler();
            return false;
        }
        osDelay(write_time);
        index += page_size;
        remaining -= page_size;
    }

    while (remaining) {
        auto result = HAL_I2C_Mem_Write(&eeprom_i2c, i2c_address, index, I2C_MEMADD_SIZE_16BIT, tmp + index, remaining, 20);
        if (result != HAL_OK) {
            ERROR("EEPROM write remainder error = %lu.", eeprom_i2c.ErrorCode);
            if (HAL_I2C_DeInit(&eeprom_i2c) != HAL_OK) CxxErrorHandler();
            return false;
        }
        osDelay(write_time);
        index += remaining;
        remaining = 0;
    }

    if (HAL_I2C_DeInit(&eeprom_i2c) != HAL_OK) CxxErrorHandler();

    return true;
}

}}} // mobilinkd::tnc::kiss

