// Copyright 2018-2019 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "KissHardware.hpp"
#include "PortInterface.hpp"
#include "AudioInput.hpp"
#include "AudioLevel.hpp"
#include "IOEventTask.h"
#include <ModulatorTask.hpp>

#include <memory>
#include <cstdio>

extern I2C_HandleTypeDef hi2c3;

namespace mobilinkd { namespace tnc { namespace kiss {

const char FIRMWARE_VERSION[] = "1.0.1";
const char HARDWARE_VERSION[] = "Mobilinkd Nucleo32 Breadboard TNC";

Hardware& settings()
{
    static Hardware instance __attribute__((section(".bss3")));
    return instance;
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

inline void reply(uint8_t cmd, const uint8_t* data, uint16_t len) {
    uint8_t* buffer = static_cast<uint8_t*>(alloca(len + 1));
    buffer[0] = cmd;
    for (uint16_t i = 0; i != len; i++)
        buffer[i + 1] = data[i];
    ioport->write(buffer, len + 1, 6, osWaitForever);
}

inline void reply_ext(uint8_t ext, uint8_t cmd, const uint8_t* data, uint16_t len) {
    uint8_t* buffer = static_cast<uint8_t*>(alloca(len + 2));
    buffer[0] = ext;
    buffer[1] = cmd;
    for (uint16_t i = 0; i != len and data[i] != 0; i++)
        buffer[i + 2] = data[i];
    ioport->write(buffer, len + 2, 6, osWaitForever);
}

void Hardware::get_alias(uint8_t alias) {
    uint8_t result[14];
    if (alias >= NUMBER_OF_ALIASES or not aliases[alias].set) return;
    result[0] = alias;
    memcpy(result + 1, aliases[alias].call, CALLSIGN_LEN);
    result[9] = aliases[alias].set;
    result[10] = aliases[alias].use;
    result[11] = aliases[alias].insert_id;
    result[12] = aliases[alias].preempt;
    result[13] = aliases[alias].hops;
    reply_ext(hardware::EXTENDED_CMD, hardware::EXT_GET_ALIASES, result, 14);
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
        reply8(hardware::OK, hardware::SAVE_EEPROM_SETTINGS);
        break;
    case hardware::POLL_INPUT_LEVEL:
        DEBUG("POLL_INPUT_VOLUME");
        reply8(hardware::POLL_INPUT_LEVEL, 0);
        osMessagePut(audioInputQueueHandle, audio::POLL_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
            osWaitForever);
        break;
    case hardware::STREAM_INPUT_LEVEL:
        DEBUG("STREAM_INPUT_VOLUME");
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;
    case hardware::GET_BATTERY_LEVEL:
      DEBUG("GET_BATTERY_LEVEL");
      osMessagePut(audioInputQueueHandle, audio::POLL_BATTERY_LEVEL,
          osWaitForever);
      osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
          osWaitForever);
        break;
    case hardware::SEND_MARK:
        DEBUG("SEND_MARK");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().mark();
        break;
    case hardware::SEND_SPACE:
        DEBUG("SEND_SPACE");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().space();
        break;
    case hardware::SEND_BOTH:
        DEBUG("SEND_BOTH");
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        getAFSKTestTone().both();
        break;
    case hardware::STOP_TX:
        DEBUG("STOP_TX");
        getAFSKTestTone().stop();
        osMessagePut(audioInputQueueHandle, audio::IDLE,
            osWaitForever);
        break;
    case hardware::RESET:
        DEBUG("RESET");
        osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
            osWaitForever);
        break;
    case hardware::SET_OUTPUT_GAIN:
        DEBUG("SET_OUTPUT_VOLUME");
        output_gain = *it << 8;
        ++it;
        output_gain += *it;
        DEBUG("SET_OUTPUT_GAIN = %d", int(output_gain));
        audio::setAudioOutputLevel();
        update_crc();
        [[fallthrough]];
    case hardware::GET_OUTPUT_GAIN:
        DEBUG("GET_OUTPUT_VOLUME");
        reply16(hardware::GET_OUTPUT_GAIN, output_gain);
        break;

    case hardware::STREAM_DCD_VALUE:
        DEBUG("STREAM_DCD_VALUE");
        break;

    case hardware::POLL_INPUT_TWIST:
      DEBUG("POLL_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::POLL_TWIST_LEVEL,
          osWaitForever);
      osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
          osWaitForever);
        break;

    case hardware::STREAM_AVG_INPUT_TWIST:
      DEBUG("STREAM_AVG_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::STREAM_AVERAGE_TWIST_LEVEL,
          osWaitForever);
        break;

    case hardware::STREAM_INPUT_TWIST:
      DEBUG("STREAM_INPUT_TWIST");
      osMessagePut(audioInputQueueHandle, audio::STREAM_INSTANT_TWIST_LEVEL,
          osWaitForever);
        break;

    case hardware::ADJUST_INPUT_LEVELS:
        DEBUG("ADJUST_INPUT_LEVELS");
        osMessagePut(audioInputQueueHandle, audio::AUTO_ADJUST_INPUT_LEVEL,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;

    case hardware::SET_VERBOSITY:
        DEBUG("SET_VERBOSITY");
        log_level = *it ? Log::Level::debug : Log::Level::warn;
        Log().setLevel(*it ? Log::Level::debug : Log::Level::warn);
        update_crc();
        [[fallthrough]];
    case hardware::GET_VERBOSITY:
        DEBUG("GET_VERBOSITY");
        reply8(hardware::GET_VERBOSITY, log_level == Log::Level::debug);
        break;
    case hardware::SET_INPUT_GAIN:
        input_gain = *it << 8;
        ++it;
        input_gain += *it;
        DEBUG("SET_INPUT_GAIN = %d", input_gain);
        update_crc();
        osMessagePut(audioInputQueueHandle, audio::UPDATE_SETTINGS,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        [[fallthrough]];
    case hardware::GET_INPUT_GAIN:
        DEBUG("GET_INPUT_GAIN");
        reply16(hardware::GET_INPUT_GAIN, input_gain);
        break;
    case hardware::SET_INPUT_TWIST:
        DEBUG("SET_INPUT_TWIST");
        rx_twist = *it;
        update_crc();
        osMessagePut(audioInputQueueHandle, audio::UPDATE_SETTINGS,
            osWaitForever);
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        [[fallthrough]];
    case hardware::GET_INPUT_TWIST:
        DEBUG("GET_INPUT_TWIST");
        reply8(hardware::GET_INPUT_TWIST, rx_twist);
        break;

    case hardware::SET_OUTPUT_TWIST:
        tx_twist = *it;
        if (tx_twist < 0) tx_twist = 0;
        if (tx_twist > 100) tx_twist = 100;
        DEBUG("SET_OUTPUT_TWIST: %d", int(tx_twist));
        getModulator().init(*this);
        update_crc();
        [[fallthrough]];
    case hardware::GET_OUTPUT_TWIST:
        DEBUG("GET_OUTPUT_TWIST");
        reply8(hardware::GET_OUTPUT_TWIST, tx_twist);
        break;

    case hardware::STREAM_AMPLIFIED_INPUT:
        DEBUG("STREAM_AMPLIFIED_INPUT");
        osMessagePut(audioInputQueueHandle, audio::STREAM_AMPLIFIED_INPUT_LEVEL,
            osWaitForever);
        break;

    case hardware::GET_TXDELAY:
        DEBUG("GET_TXDELAY");
        reply8(hardware::GET_TXDELAY, txdelay);
        break;
    case hardware::GET_PERSIST:
        DEBUG("GET_PERSIST");
        reply8(hardware::GET_PERSIST, ppersist);
        break;
    case hardware::GET_TIMESLOT:
        DEBUG("GET_TIMESLOT");
        reply8(hardware::GET_TIMESLOT, slot);
        break;
    case hardware::GET_TXTAIL:
        DEBUG("GET_TXTAIL");
        reply8(hardware::GET_TXTAIL, txtail);
        break;
    case hardware::GET_DUPLEX:
        DEBUG("GET_DUPLEX");
        reply8(hardware::GET_DUPLEX, duplex);
        break;

    case hardware::GET_FIRMWARE_VERSION:
        DEBUG("GET_FIRMWARE_VERSION");
        reply(hardware::GET_FIRMWARE_VERSION, (uint8_t*) FIRMWARE_VERSION,
          sizeof(FIRMWARE_VERSION) - 1);
        break;
    case hardware::GET_HARDWARE_VERSION:
        DEBUG("GET_HARDWARE_VERSION");
        reply(hardware::GET_HARDWARE_VERSION, (uint8_t*) HARDWARE_VERSION,
          sizeof(HARDWARE_VERSION) - 1);
        break;

    case hardware::GET_SERIAL_NUMBER:
        DEBUG("GET_SERIAL_NUMBER");
        reply(hardware::GET_SERIAL_NUMBER, (uint8_t*) serial_number_64,
            sizeof(serial_number_64) - 1);
        break;

    case hardware::SET_PTT_CHANNEL:
        DEBUG("SET_PTT_CHANNEL");
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
        DEBUG("GET_PTT_CHANNEL");
        reply8(hardware::GET_PTT_CHANNEL,
            options & KISS_OPTION_PTT_SIMPLEX ? 0 : 1);
        break;

    case hardware::SET_USB_POWER_OFF:
        DEBUG("SET_USB_POWER_OFF");
        if (*it) {
          options |= KISS_OPTION_VIN_POWER_OFF;
        } else {
          options &= ~KISS_OPTION_VIN_POWER_OFF;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_USB_POWER_OFF:
        DEBUG("GET_USB_POWER_OFF");
        reply8(hardware::GET_USB_POWER_OFF,
            options & KISS_OPTION_VIN_POWER_OFF ? 1 : 0);
        break;

    case hardware::SET_USB_POWER_ON:
        DEBUG("SET_USB_POWER_ON");
        if (*it) {
          options |= KISS_OPTION_VIN_POWER_ON;
        } else {
          options &= ~KISS_OPTION_VIN_POWER_ON;
        }
        update_crc();
        [[fallthrough]];
    case hardware::GET_USB_POWER_ON:
        DEBUG("GET_USB_POWER_ON");
        reply8(hardware::GET_USB_POWER_ON,
            options & KISS_OPTION_VIN_POWER_ON ? 1 : 0);
        break;

    case hardware::GET_CAPABILITIES:
        DEBUG("GET_CAPABILITIES");
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|
            hardware::CAP_ADJUST_INPUT|
            hardware::CAP_DFU_FIRMWARE);
        break;

    case hardware::GET_ALL_VALUES:
        DEBUG("GET_ALL_VALUES");
        reply16(hardware::GET_API_VERSION, hardware::KISS_API_VERSION);
        osMessagePut(audioInputQueueHandle, audio::POLL_BATTERY_LEVEL,
            osWaitForever);
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
        reply16(hardware::GET_CAPABILITIES,
            hardware::CAP_EEPROM_SAVE|
            hardware::CAP_ADJUST_INPUT|
            hardware::CAP_DFU_FIRMWARE);
        reply16(hardware::GET_MIN_INPUT_GAIN, 0);   // Constants for this FW
        reply16(hardware::GET_MAX_INPUT_GAIN, 4);   // Constants for this FW
        reply8(hardware::GET_MIN_INPUT_TWIST, -3);  // Constants for this FW
        reply8(hardware::GET_MAX_INPUT_TWIST, 9);   // Constants for this FW

        break;
    case hardware::EXTENDED_CMD:
        handle_ext_request(frame);
        break;
    default:
        ERROR("Unknown hardware request");
    }
}

inline void ext_reply(uint8_t cmd, uint8_t result) {
    uint8_t data[3] { hardware::EXTENDED_CMD, cmd, result };
    ioport->write(data, 3, 6);
}

void Hardware::handle_ext_request(hdlc::IoFrame* frame) {
    auto it = frame->begin();
    ++it;
    uint8_t ext_command = *it++;

    switch (ext_command) {
    case hardware::EXT_GET_MODEM_TYPE:
        DEBUG("EXT_GET_MODEM_TYPE");
        ext_reply(hardware::EXT_GET_MODEM_TYPE, 1);
        break;
    case hardware::EXT_SET_MODEM_TYPE:
        DEBUG("EXT_SET_MODEM_TYPE");
        ext_reply(hardware::EXT_OK, hardware::EXT_SET_MODEM_TYPE);
        break;
    case hardware::EXT_GET_MODEM_TYPES:
        DEBUG("EXT_GET_MODEM_TYPES");
        ext_reply(hardware::EXT_GET_MODEM_TYPES, 1);
        break;
    }
}

bool Hardware::load()
{
    INFO("Loading settings from EEPROM");

    Hardware tmp;

    memset(&tmp, 0, sizeof(Hardware));

    if (!I2C_Storage::load(tmp)) {
        ERROR("EEPROM read failed");
        return false;
    }

    if (tmp.crc_ok())
    {
        memcpy(this, &tmp, sizeof(Hardware));
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

    INFO("EEPROM saved checksum is: %04x (crc = %04x)", checksum, crc());

    return crc_ok();
}

bool I2C_Storage::load(void* ptr, size_t len)
{
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) CxxErrorHandler();

    DEBUG("Attempting to read %d bytes from EEPROM...", len);

    uint32_t timeout = 1000;    // systicks... milliseconds

    auto tmp = static_cast<uint8_t*>(ptr);
    auto result = HAL_I2C_Mem_Read(&hi2c3, i2c_address, 0,
        I2C_MEMADD_SIZE_16BIT, tmp, len, timeout);
    if (result != HAL_OK) CxxErrorHandler();

    if (HAL_I2C_DeInit(&hi2c3) != HAL_OK) CxxErrorHandler();

    return true;
}

bool I2C_Storage::store(const void* ptr, size_t len)
{
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) CxxErrorHandler();

    auto tmp = const_cast<uint8_t*>(static_cast<const uint8_t*>(ptr));

    uint32_t index = 0;
    size_t remaining = len;
    while (remaining > page_size)
    {
        auto result = HAL_I2C_Mem_Write(&hi2c3, i2c_address, index, I2C_MEMADD_SIZE_16BIT, tmp + index, page_size, 20);
        if (result != HAL_OK) {
            ERROR("EEPROM write block error = %lu.", hi2c3.ErrorCode);
            if (HAL_I2C_DeInit(&hi2c3) != HAL_OK) CxxErrorHandler();
            return false;
        }
        osDelay(write_time);
        index += page_size;
        remaining -= page_size;
    }

    while (remaining) {
        auto result = HAL_I2C_Mem_Write(&hi2c3, i2c_address, index, I2C_MEMADD_SIZE_16BIT, tmp + index, remaining, 20);
        if (result != HAL_OK) {
            ERROR("EEPROM write remainder error = %lu.", hi2c3.ErrorCode);
            if (HAL_I2C_DeInit(&hi2c3) != HAL_OK) CxxErrorHandler();
            return false;
        }
        osDelay(write_time);
        index += remaining;
        remaining = 0;
    }

    if (HAL_I2C_DeInit(&hi2c3) != HAL_OK) CxxErrorHandler();

    return true;
}

}}} // mobilinkd::tnc::kiss

