// Copyright 2015-2021 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Log.h"
#include "HdlcFrame.hpp"
#include "AFSKTestTone.hpp"

#include <cstdint>
#include <cstring>
#include <memory>
#include <array>

extern "C" void updatePtt(void);

namespace mobilinkd { namespace tnc { namespace kiss {

extern const char FIRMWARE_VERSION[];
extern const char HARDWARE_VERSION[];

AFSKTestTone& getAFSKTestTone();

namespace hardware {

/**
 * This indicates the API version of the device.  API versions are used to
 * indicate to the user that the config app may need to be upgraded because
 * the device is using a newer configuration API.
 *
 * The minor version should be updated whenever the API is extended (new
 * GET/SET or CAP types added.
 *
 * The major version should be updated whenever non-backwards compatible
 * changes to the API are made.
 */
constexpr const uint16_t KISS_API_VERSION = 0x0202;

constexpr const uint16_t CAP_DCD = 0x0100;
constexpr const uint16_t CAP_SQUELCH = 0x0200;
constexpr const uint16_t CAP_INPUT_ATTEN = 0x0400;
constexpr const uint16_t CAP_FIRMWARE_VERSION = 0x0800;
constexpr const uint16_t CAP_BATTERY_LEVEL = 0x1000;
constexpr const uint16_t CAP_BT_CONN_TRACK = 0x2000;
constexpr const uint16_t CAP_BT_NAME_CHANGE = 0x4000;
constexpr const uint16_t CAP_BT_PIN_CHANGE = 0x8000;
constexpr const uint16_t CAP_VERBOSE_ERROR = 0x0001;
constexpr const uint16_t CAP_EEPROM_SAVE = 0x0002;
constexpr const uint16_t CAP_ADJUST_INPUT = 0x0004; // Auto-adjust input levels.
constexpr const uint16_t CAP_DFU_FIRMWARE = 0x0008; // DFU firmware style.

constexpr const uint8_t SAVE = 0; // Save settings to EEPROM.
constexpr const uint8_t SET_OUTPUT_GAIN = 1;
constexpr const uint8_t SET_INPUT_GAIN = 2;
constexpr const uint8_t SET_SQUELCH_LEVEL = 3;      // deprecated.
constexpr const uint8_t POLL_INPUT_LEVEL = 4;
constexpr const uint8_t STREAM_INPUT_LEVEL = 5;
constexpr const uint8_t GET_BATTERY_LEVEL = 6;
constexpr const uint8_t SEND_MARK = 7;
constexpr const uint8_t SEND_SPACE = 8;
constexpr const uint8_t SEND_BOTH = 9;
constexpr const uint8_t STOP_TX = 10;
constexpr const uint8_t RESET = 11;
constexpr const uint8_t GET_OUTPUT_GAIN = 12;
constexpr const uint8_t GET_INPUT_GAIN = 13;
constexpr const uint8_t GET_SQUELCH_LEVEL = 14;
constexpr const uint8_t STREAM_DCD_VALUE = 15;

constexpr const uint8_t SET_VERBOSITY = 16;
constexpr const uint8_t GET_VERBOSITY = 17;

constexpr const uint8_t SET_INPUT_OFFSET = 18;
constexpr const uint8_t GET_INPUT_OFFSET = 19;
constexpr const uint8_t SET_OUTPUT_OFFSET = 20;
constexpr const uint8_t GET_OUTPUT_OFFSET = 21;
constexpr const uint8_t SET_LOWPASS_FREQ = 22;
constexpr const uint8_t GET_LOWPASS_FREQ = 23;
constexpr const uint8_t SET_INPUT_TWIST = 24;
constexpr const uint8_t GET_INPUT_TWIST = 25;
constexpr const uint8_t SET_OUTPUT_TWIST = 26;
constexpr const uint8_t GET_OUTPUT_TWIST = 27;

constexpr const uint8_t STREAM_RAW_INPUT = 28;
constexpr const uint8_t STREAM_AMPLIFIED_INPUT = 29;
constexpr const uint8_t STREAM_FILTERED_INPUT = 30;
constexpr const uint8_t STREAM_OUTPUT = 31;

constexpr const uint8_t OK = 32;                  // Acknowledge SET commands.

constexpr const uint8_t GET_TXDELAY = 33;
constexpr const uint8_t GET_PERSIST = 34;
constexpr const uint8_t GET_TIMESLOT = 35;
constexpr const uint8_t GET_TXTAIL = 36;
constexpr const uint8_t GET_DUPLEX = 37;

constexpr const uint8_t GET_FIRMWARE_VERSION = 40;
constexpr const uint8_t GET_HARDWARE_VERSION = 41;
constexpr const uint8_t SAVE_EEPROM_SETTINGS = 42;
constexpr const uint8_t ADJUST_INPUT_LEVELS = 43;       // Auto-adjust levels.
constexpr const uint8_t POLL_INPUT_TWIST = 44;
constexpr const uint8_t STREAM_AVG_INPUT_TWIST = 45;
constexpr const uint8_t STREAM_INPUT_TWIST = 46;
constexpr const uint8_t GET_SERIAL_NUMBER = 47;
constexpr const uint8_t GET_MAC_ADDRESS = 48;
constexpr const uint8_t GET_DATETIME = 49;
constexpr const uint8_t SET_DATETIME = 50;
constexpr const uint8_t GET_ERROR_MSG = 51;
constexpr const uint8_t GET_SNR = 52;
constexpr const uint8_t GET_BER = 53;

constexpr const uint8_t SET_BLUETOOTH_NAME = 65;
constexpr const uint8_t GET_BLUETOOTH_NAME = 66;
constexpr const uint8_t SET_BLUETOOTH_PIN = 67; // Danger Will Robinson.
constexpr const uint8_t GET_BLUETOOTH_PIN = 68;
constexpr const uint8_t SET_BT_CONN_TRACK = 69; // Bluetooth connection tracking
constexpr const uint8_t GET_BT_CONN_TRACK = 70; // Bluetooth connection tracking
constexpr const uint8_t SET_BT_MAJOR_CLASS = 71; // Bluetooth Major Class
constexpr const uint8_t GET_BT_MAJOR_CLASS = 72; // Bluetooth Major Class

constexpr const uint8_t SET_USB_POWER_ON = 73; // Power on when USB power available
constexpr const uint8_t GET_USB_POWER_ON = 74;
constexpr const uint8_t SET_USB_POWER_OFF = 75; // Power off when USB power unavailable
constexpr const uint8_t GET_USB_POWER_OFF = 76;
constexpr const uint8_t SET_BT_POWER_OFF = 77; // Power off after n seconds w/o BT conn
constexpr const uint8_t GET_BT_POWER_OFF = 78;

constexpr const uint8_t SET_PTT_CHANNEL = 79; // Which PTT line to use (currently 0 or 1,
constexpr const uint8_t GET_PTT_CHANNEL = 80; // multiplex or simplex)

constexpr const uint8_t SET_PASSALL = 81;   // Allow invalid CRC through when
constexpr const uint8_t GET_PASSALL = 82;   // true (1).
constexpr const uint8_t SET_RX_REV_POLARITY = 83;   // Reverse RX polarity for
constexpr const uint8_t GET_RX_REV_POLARITY = 84;   // 4-FSK modes when true (1).
constexpr const uint8_t SET_TX_REV_POLARITY = 85;   // Reverse TX polarity for
constexpr const uint8_t GET_TX_REV_POLARITY = 86;   // 4-FSK modes when true (1).

constexpr const uint8_t GET_MIN_OUTPUT_TWIST = 119;  ///< int8_t (may be negative).
constexpr const uint8_t GET_MAX_OUTPUT_TWIST = 120;  ///< int8_t (may be negative).
constexpr const uint8_t GET_MIN_INPUT_TWIST = 121;  ///< int8_t (may be negative).
constexpr const uint8_t GET_MAX_INPUT_TWIST = 122;  ///< int8_t (may be negative).
constexpr const uint8_t GET_API_VERSION = 123;      ///< uint16_t (major/minor)
constexpr const uint8_t GET_MIN_INPUT_GAIN = 124;   ///< int8_t (may be negative/attenuated).
constexpr const uint8_t GET_MAX_INPUT_GAIN = 125;   ///< int8_t (may be negative/attenuated).
constexpr const uint8_t GET_CAPABILITIES = 126;   ///< Send all capabilities.
constexpr const uint8_t GET_ALL_VALUES = 127;     ///< Send all settings & versions.

/**
 * Extended commands are two+ bytes in length.  They follow the model used
 * by UTF-8 to extend the command set.  Extended commands start at C1:80
 * and go through DF:BF (11 significant bits), then proceed to E0:80:80
 * through EF:BF:BF (16 significant bits).
 *
 * To avoid special KISS characters, we skip the following byte values in
 * the first byte:
 *
 * - 0xC0 / FEND
 * - 0xDB / FESC
 * - 0xDC / TFEND
 * - 0xDD / TFESC
 */
constexpr const uint8_t EXTENDED_CMD = 0xC1;

constexpr const uint8_t EXT_OK = 0x80;

constexpr std::array<uint8_t, 2> EXT_GET_MODEM_TYPE = {0xC1, 0x81};
constexpr std::array<uint8_t, 2> EXT_SET_MODEM_TYPE = {0xC1, 0x82};
constexpr std::array<uint8_t, 2> EXT_GET_MODEM_TYPES = {0xC1, 0x83};    ///< Return a list of supported modem types

constexpr std::array<uint8_t, 2> EXT_GET_ALIASES = {0xC1, 0x88};        ///< Number of aliases supported
constexpr std::array<uint8_t, 2> EXT_GET_ALIAS = {0xC1, 0x89};          ///< Alias number (uint8_t), 8 characters, 5 bytes (set, use, insert_id, preempt, hops)
constexpr std::array<uint8_t, 2> EXT_SET_ALIAS = {0xC1, 0x8A};          ///< Alias number (uint8_t), 8 characters, 5 bytes (set, use, insert_id, preempt, hops)

constexpr std::array<uint8_t, 2> EXT_GET_BEACON_SLOTS = {0xC1, 0x8C};   ///< Number of beacons supported
constexpr std::array<uint8_t, 2> EXT_GET_BEACON = {0xC1, 0x8D};         ///< Beacon number (uint8_t), uint16_t interval in seconds, 3 NUL terminated strings (callsign, path, text)
constexpr std::array<uint8_t, 2> EXT_SET_BEACON = {0xC1, 0x8E};         ///< Beacon number (uint8_t), uint16_t interval in seconds, 3 NUL terminated strings (callsign, path, text)

/*
 * Modem type values 0x00 - 0x7F are single-byte types.  Modem type values
 * starting at 0xC0 are multi-byte values, and should follow the model used
 * by UTF-8 to extend the modem types, should that ever be necessary.
 */
constexpr uint8_t MODEM_TYPE_1200 = 1;
constexpr uint8_t MODEM_TYPE_300 = 2;
constexpr uint8_t MODEM_TYPE_9600 = 3;
constexpr uint8_t MODEM_TYPE_PSK31 = 4;
constexpr uint8_t MODEM_TYPE_M17 = 5;

// Boolean options.
#define KISS_OPTION_CONN_TRACK      0x01
#define KISS_OPTION_VERBOSE         0x02
#define KISS_OPTION_VIN_POWER_ON    0x04  // Power on when plugged in to USB
#define KISS_OPTION_VIN_POWER_OFF   0x08  // Power off when unplugged from USB
#define KISS_OPTION_PTT_SIMPLEX     0x10  // Simplex PTT (the default)
#define KISS_OPTION_PASSALL         0x20  // Ignore invalid CRC.
#define KISS_OPTION_RX_REV_POLARITY 0x40  // Reverse Polarity on RX when set.
#define KISS_OPTION_TX_REV_POLARITY 0x80  // Reverse Polarity on TX when set.

#ifndef NUCLEOTNC
const char TOCALL[] = "APML30"; // Update for every feature change.
#else
const char TOCALL[] = "APML00"; // Update for every feature change.
#endif

} // hardware

const size_t CALLSIGN_LEN = 8;
using call_t = std::array<char, CALLSIGN_LEN>;

struct Alias {
    call_t call;                ///< Callsign.  Pad unused with NUL.
    bool set;                   ///< Alias is configured.
    bool use;                   ///< Use this alias.
    bool insert_id;             ///< Tracing.
    bool preempt;               ///< Allow out of order pathing.
    uint8_t hops;
}; // size = 10

const size_t BEACON_PATH_LEN = 30;
const size_t BEACON_TEXT_LEN = 128;

struct Beacon {
    call_t dest;                        ///< callsign.  Pad unused with NUL.
    uint8_t path[BEACON_PATH_LEN + 1];  ///< NUL terminated string.
    uint8_t text[BEACON_TEXT_LEN + 1];  ///< NUL terminated string.
    uint16_t seconds;                   ///< Number of seconds between beacons.
}; // size = 170

const size_t NUMBER_OF_ALIASES = 8;     // 80 bytes
const size_t NUMBER_OF_BEACONS = 4;     // 680 bytes

/**
 * Values from the KISS settings (including hardware settings) which are
 * stored in EEPROM.
 */
struct Hardware
{
    static constexpr std::array<const char*, 6> modem_type_lookup = {
        "NOT SET",
        "AFSK1200",
        "AFSK300",
        "FSK9600",
        "PSK31",
        "M17"
    };

    // This must match the constants defined above.
    enum ModemType {
        AFSK1200 = hardware::MODEM_TYPE_1200,
        AFSK300 = hardware::MODEM_TYPE_300,
        FSK9600 = hardware::MODEM_TYPE_9600,
        PSK31 = hardware::MODEM_TYPE_PSK31,
        M17 = hardware::MODEM_TYPE_M17
    };

    static constexpr std::array<uint8_t, 3> supported_modem_types = {
        hardware::MODEM_TYPE_1200,
        hardware::MODEM_TYPE_9600,
        hardware::MODEM_TYPE_M17
    };

    uint8_t txdelay;        ///< How long in 10mS units to wait for TX to settle before starting data
    uint8_t ppersist;       ///< Likelihood of taking the channel when its not busy
    uint8_t slot;           ///< How long in 10mS units to wait between sampling the channel to see if free
    uint8_t txtail;         ///< How long in 10mS units to wait after the data before keying off the transmitter
    uint8_t duplex;         ///< Ignore current channel activity - just key up
    uint8_t modem_type;     ///< Modem type.
    uint16_t output_gain;   ///< output volume (0-256).
    uint16_t input_gain;    ///< input volume (0-256).
    int8_t tx_twist;        ///< 0 to 100 (50 = even).
    int8_t rx_twist;        ///< 0, 3, 6 dB
    uint8_t log_level;      ///< Log level (0 - 4 : debug - severe).

    uint16_t options;       ///< boolean options

    /// Callsign.   Pad unused with NUL.
    call_t mycall;

    uint8_t dedupe_seconds;          ///< number of seconds to dedupe packets.
    Alias aliases[NUMBER_OF_ALIASES];   ///< Digipeater aliases
    Beacon beacons[NUMBER_OF_BEACONS];  ///< Beacons
    uint16_t checksum;      ///< Validity check of param data (CRC16)

    uint16_t crc() const {

        uint32_t crc = HAL_CRC_Calculate(
            &hcrc, (uint32_t*) this, sizeof(Hardware) - 2);

        return crc & 0xFFFF;
    }

    void update_crc() {
        checksum = crc();
        INFO("EEPROM checksum = %04hx", checksum);
    }

    bool crc_ok() const {
        auto result = (crc() == checksum);
        if (!result) {
            WARN("CRC mismatch %04hx != %04hx", checksum, crc());
        }
        return result;
    }

    /**
     * Configure hardware settings.  Load up the defaults.  Call load() to
     * load values from EEPROM and save() to store the settings in EEPROM.
     *
     */
    void init()
    {
      txdelay = 30;
      ppersist = 64;
      slot = 10;
      txtail = 1;
      duplex = 0;
      modem_type = ModemType::AFSK1200;
      output_gain = 63;
      input_gain = 0;   // 0-4 on TNC3
      tx_twist = 50;
      rx_twist = 0;
      log_level = Log::Level::debug;

      options = KISS_OPTION_PTT_SIMPLEX;

      /// Callsign.   Pad unused with NUL.
      strcpy(mycall.data(), "NOCALL");

      dedupe_seconds = 30;
      memset(aliases, 0, sizeof(aliases));
      memset(beacons, 0, sizeof(beacons));
      update_crc();

      DEBUG("Settings initialized");
    }

    void debug() {
#ifdef KISS_LOGGING
        DEBUG("Hardware Settings (size=%d):", sizeof(Hardware));
        DEBUG("TX Delay: %d", (int)txdelay);
        DEBUG("P* Persistence: %d", (int)ppersist);
        DEBUG("Slot Time: %d", (int)slot);
        DEBUG("TX Tail: %d", (int)txtail);
        DEBUG("Duplex: %d", (int)duplex);
        DEBUG("Modem Type: %s", modem_type_lookup[modem_type]);
        DEBUG("TX Gain: %d", (int)output_gain);
        DEBUG("RX Gain: %d", (int)input_gain);
        DEBUG("TX Twist: %d", (int)tx_twist);
        DEBUG("RX Twist: %d", (int)rx_twist);
        DEBUG("Log Level: %d", (int)log_level);
        DEBUG("Options:  %04hx", options);
        DEBUG("MYCALL: %s", mycall.data());
        DEBUG("Dedupe time (secs): %d", (int)dedupe_seconds);
        DEBUG("Aliases:");
        for (auto& a : aliases) {
            if (!a.set) continue;
            DEBUG(" call: %s", a.call.data());
            DEBUG(" use: %d", (int)a.use);
            DEBUG(" insert: %d", (int)a.insert_id);
            DEBUG(" preempt: %d", (int)a.preempt);
            DEBUG(" hops: %d", (int)a.hops);
        }
        DEBUG("Beacons:");
        for (auto& b : this->beacons) {
            if (b.seconds == 0) continue;
            DEBUG(" dest: %s", b.dest.data());
            DEBUG(" path: %s", (char*)b.path);
            DEBUG(" text: %s", (char*)b.text);
            DEBUG(" frequency (secs): %d", (int)b.seconds);
        }
        DEBUG("Checksum: %04hx", checksum);
#endif
    }

    bool load();
    bool store() const;

    void set_txdelay(uint8_t value);
    void set_ppersist(uint8_t value);
    void set_slottime(uint8_t value);
    void set_txtail(uint8_t value);
    void set_duplex(uint8_t value);

    void handle_request(hdlc::IoFrame* frame) __attribute__((optimize("-O2")));
    void handle_ext_request(hdlc::IoFrame* frame);

    void get_aliases();
    void get_alias(uint8_t alias);
    void set_alias(const hdlc::IoFrame* frame);

    bool rx_rev_polarity() const
    {
        return (options & KISS_OPTION_RX_REV_POLARITY) != 0;
    }

    bool tx_rev_polarity() const
    {
        return (options & KISS_OPTION_TX_REV_POLARITY) != 0;
    }

    void announce_input_settings();

}; // 812 bytes

extern Hardware& settings();

struct I2C_Storage
{
    constexpr static const uint16_t i2c_address{EEPROM_ADDRESS};
    constexpr static const uint16_t capacity{EEPROM_CAPACITY};
    constexpr static const uint16_t page_size{EEPROM_PAGE_SIZE};
    constexpr static const uint32_t write_time{EEPROM_WRITE_TIME};

    static bool load(void* ptr, size_t len);

    template <typename T>
    static bool load(T& t) {
        return load(&t, sizeof(T));
    }

    static bool store(const void* ptr, size_t len);

    template <typename T>
    static bool store(const T& t) {
        return store(&t, sizeof(T));
    }
};

void reply8(uint8_t cmd, uint8_t result) __attribute__((noinline));

void reply16(uint8_t cmd, uint16_t result) __attribute__((noinline));

void reply(uint8_t cmd, const uint8_t* data, uint16_t len) __attribute__((noinline));

}}} // mobilinkd::tnc::kiss
