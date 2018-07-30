// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__KISS_HARDWARE_HPP_
#define MOBILINKD__TNC__KISS_HARDWARE_HPP_

#include <Log.h>
#include "HdlcFrame.hpp"

#include <cstdint>
#include <cstring>
#include <memory>

extern "C" void updatePtt(void);

namespace mobilinkd { namespace tnc { namespace kiss {

extern const char FIRMWARE_VERSION[];
extern const char HARDWARE_VERSION[];

namespace hardware {

const uint16_t CAP_DCD = 0x0001;
const uint16_t CAP_SQUELCH = 0x0002;
const uint16_t CAP_INPUT_ATTEN = 0x0004;
const uint16_t CAP_FIRMWARE_VERSION = 0x0008;
const uint16_t CAP_BATTERY_LEVEL = 0x0010;
const uint16_t CAP_BT_CONN_TRACK = 0x0020;
const uint16_t CAP_BT_NAME_CHANGE = 0x0040;
const uint16_t CAP_BT_PIN_CHANGE = 0x0080;
const uint16_t CAP_VERBOSE_ERROR = 0x0100;
const uint16_t CAP_EEPROM_SAVE = 0x0200;
const uint16_t CAP_ADJUST_INPUT = 0x0400;

const uint8_t SAVE = 0; // Save settings to EEPROM.
const uint8_t SET_OUTPUT_GAIN = 1;
const uint8_t SET_INPUT_GAIN = 2;
const uint8_t SET_SQUELCH_LEVEL = 3;
const uint8_t POLL_INPUT_LEVEL = 4;
const uint8_t STREAM_INPUT_LEVEL = 5;
const uint8_t GET_BATTERY_LEVEL = 6;
const uint8_t SEND_MARK = 7;
const uint8_t SEND_SPACE = 8;
const uint8_t SEND_BOTH = 9;
const uint8_t STOP_TX = 10;
const uint8_t RESET = 11;
const uint8_t GET_OUTPUT_GAIN = 12;
const uint8_t GET_INPUT_ATTEN = 13;
const uint8_t GET_SQUELCH_LEVEL = 14;
const uint8_t STREAM_DCD_VALUE = 15;

const uint8_t SET_VERBOSITY = 16;
const uint8_t GET_VERBOSITY = 17;

const uint8_t SET_INPUT_OFFSET = 18;
const uint8_t GET_INPUT_OFFSET = 19;
const uint8_t SET_OUTPUT_OFFSET = 20;
const uint8_t GET_OUTPUT_OFFSET = 21;
const uint8_t SET_LOWPASS_FREQ = 22;
const uint8_t GET_LOWPASS_FREQ = 23;
const uint8_t SET_INPUT_TWIST = 24;
const uint8_t GET_INPUT_TWIST = 25;
const uint8_t SET_OUTPUT_TWIST = 26;
const uint8_t GET_OUTPUT_TWIST = 27;

const uint8_t STREAM_RAW_INPUT = 28;
const uint8_t STREAM_AMPLIFIED_INPUT = 29;
const uint8_t STREAM_FILTERED_INPUT = 30;
const uint8_t STREAM_OUTPUT = 31;

const uint8_t OK = 32;                  // Acknowledge SET commands.

const uint8_t GET_TXDELAY = 33;
const uint8_t GET_PERSIST = 34;
const uint8_t GET_TIMESLOT = 35;
const uint8_t GET_TXTAIL = 36;
const uint8_t GET_DUPLEX = 37;

const uint8_t GET_FIRMWARE_VERSION = 40;
const uint8_t GET_HARDWARE_VERSION = 41;
const uint8_t SAVE_EEPROM_SETTINGS = 42;
const uint8_t ADJUST_INPUT_LEVELS = 43;
const uint8_t POLL_INPUT_TWIST = 44;
const uint8_t STREAM_AVG_INPUT_TWIST = 45;
const uint8_t STREAM_INPUT_TWIST = 46;

const uint8_t SET_BLUETOOTH_NAME = 65;
const uint8_t GET_BLUETOOTH_NAME = 66;
const uint8_t SET_BLUETOOTH_PIN = 67; // Danger Will Robinson.
const uint8_t GET_BLUETOOTH_PIN = 68;
const uint8_t SET_BT_CONN_TRACK = 69; // Bluetooth connection tracking
const uint8_t GET_BT_CONN_TRACK = 70; // Bluetooth connection tracking
const uint8_t SET_BT_MAJOR_CLASS = 71; // Bluetooth Major Class
const uint8_t GET_BT_MAJOR_CLASS = 72; // Bluetooth Major Class

const uint8_t SET_USB_POWER_ON = 73; // Power on when USB power available
const uint8_t GET_USB_POWER_ON = 74;
const uint8_t SET_USB_POWER_OFF = 75; // Power off when USB power unavailable
const uint8_t GET_USB_POWER_OFF = 76;
const uint8_t SET_BT_POWER_OFF = 77; // Power off after n seconds w/o BT conn
const uint8_t GET_BT_POWER_OFF = 78;

const uint8_t SET_PTT_CHANNEL = 79; // Which PTT line to use (currently 0 or 1,
const uint8_t GET_PTT_CHANNEL = 80; // multiplex or simplex)

const uint8_t GET_CAPABILITIES = 126;   ///< Send all capabilities.
const uint8_t GET_ALL_VALUES = 127;     ///< Send all settings & versions.

/**
 * Extended commands are two+ bytes in length.  They start at 80:00
 * and go through BF:FF (14 significant bits), then proceed to C0:00:00
 * through CF:FF:FF (20 more significant bits).
 *
 * If needed, the commands can be extended to 9 nibbles (D0 - DF),
 * 13 nibbles (E0-EF) and 17 nibbles (F0-FF).
 */
const uint8_t EXTENDED_CMD = 128;

const uint8_t EXT_OK = 0;
const uint8_t EXT_GET_MODEM_TYPE = 1;
const uint8_t EXT_SET_MODEM_TYPE = 2;
const uint8_t EXT_GET_MODEM_TYPES = 3;  ///< Return a list of supported modem types

const uint8_t EXT_GET_ALIASES = 8;    ///< Number of aliases supported
const uint8_t EXT_GET_ALIAS = 9;      ///< Alias number (uint8_t), 8 characters, 5 bytes (set, use, insert_id, preempt, hops)
const uint8_t EXT_SET_ALIAS = 10;     ///< Alias number (uint8_t), 8 characters, 5 bytes (set, use, insert_id, preempt, hops)

const uint8_t EXT_GET_BEACONS = 12;   ///< Number of beacons supported
const uint8_t EXT_GET_BEACON = 13;    ///< Beacon number (uint8_t), uint16_t interval in seconds, 3 NUL terminated strings (callsign, path, text)
const uint8_t EXT_SET_BEACON = 14;    ///< Beacon number (uint8_t), uint16_t interval in seconds, 3 NUL terminated strings (callsign, path, text)

const uint8_t MODEM_TYPE_1200 = 1;
const uint8_t MODEM_TYPE_300 = 2;
const uint8_t MODEM_TYPE_9600 = 3;
const uint8_t MODEM_TYPE_PSK31 = 4;

// Boolean options.
#define KISS_OPTION_CONN_TRACK      0x01
#define KISS_OPTION_VERBOSE         0x02
#define KISS_OPTION_VIN_POWER_ON    0x04  // Power on when plugged in to USB
#define KISS_OPTION_VIN_POWER_OFF   0x08  // Power off when plugged in to USB
#define KISS_OPTION_PTT_SIMPLEX     0x10  // Simplex PTT (the default)

const char TOCALL[] = "APML50"; // Update for every feature change.

} // hardware

const size_t CALLSIGN_LEN = 8;

struct Alias {
    uint8_t call[CALLSIGN_LEN];   ///< Callsign.  Pad unused with NUL.
    bool set;                     ///< Alias is configured.
    bool use;                     ///< Use this alias.
    bool insert_id;               ///< Tracing.
    bool preempt;                 ///< Allow out of order pathing.
    uint8_t hops;
}; // size = 10

const size_t BEACON_PATH_LEN = 30;
const size_t BEACON_TEXT_LEN = 128;

struct Beacon {
    uint8_t dest[CALLSIGN_LEN];         ///< callsign.  Pad unused with NUL.
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
    enum ModemType {
        AFSK1200 = 1,
        AFSK300,
        FSK9600,
        PSK31
    };

    uint8_t txdelay;       ///< How long in 10mS units to wait for TX to settle before starting data
    uint8_t ppersist;      ///< Likelihood of taking the channel when its not busy
    uint8_t slot;          ///< How long in 10mS units to wait between sampling the channel to see if free
    uint8_t txtail;         ///< How long in 10mS units to wait after the data before keying off the transmitter
    uint8_t duplex;         ///< Ignore current channel activity - just key up
    uint8_t modem_type;         ///< Modem type.
    uint16_t output_gain; ///< output volume (0-256).
    uint16_t input_gain;  ///< input volume (0-256).
    int8_t tx_twist;           ///< 0 to 100 (50 = even).
    int8_t rx_twist;            ///< 0, 3, 6 dB
    uint8_t log_level;          ///< Log level (0 - 4 : debug - severe).

    uint16_t options;           ///< boolean options

    /// Callsign.   Pad unused with NUL.
    uint8_t mycall[CALLSIGN_LEN];

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
        INFO("EEPROM checksum = %04xs", checksum);
    }

    bool crc_ok() const {
        return crc() == checksum;
    }

    /**
     * Configure hardware settings.  Load up the defaults.  Call load() to
     * load values from EEPROM and save() to store the settings in EEPROM.
     *
     */
    void init()
    {
      if (crc_ok()) {
        DEBUG("CRC OK");
        return;
      }

      DEBUG("CRC FAILED");
      DEBUG("checksum 0x%04x != CRC 0x%04x", checksum, crc());

      txdelay = 30;
      ppersist = 64;
      slot = 10;
      txtail = 1;
      duplex = 0;
      modem_type = ModemType::AFSK1200;
      output_gain = 63;
      input_gain = 0;
      tx_twist = 50;
      rx_twist = 0;
      log_level = Log::Level::debug;

      options = KISS_OPTION_PTT_SIMPLEX;

      /// Callsign.   Pad unused with NUL.
      strcpy((char*)mycall, "MYCALL");

      dedupe_seconds = 30;
      memset(aliases, 0, sizeof(aliases));
      memset(beacons, 0, sizeof(beacons));
      update_crc();

      updatePtt();

      debug();

      DEBUG("Settings initialized");
    }

    void debug() {
        DEBUG("Hardware Settings (size=%d):", sizeof(Hardware));
        DEBUG("TX Delay: %d", (int)txdelay);
        DEBUG("P* Persistence: %d", (int)ppersist);
        DEBUG("Slot Time: %d", (int)slot);
        DEBUG("TX Tail: %d", (int)txtail);
        DEBUG("Duplex: %d", (int)duplex);
        DEBUG("Modem Type: %d", (int)modem_type);
        DEBUG("TX Gain: %d", (int)output_gain);
        DEBUG("RX Gain: %d", (int)input_gain);
        DEBUG("TX Twist: %d", (int)tx_twist);
        DEBUG("RX Twist: %d", (int)rx_twist);
        DEBUG("Log Level: %d", (int)log_level);
        DEBUG("Options: %d", (int)options);
        DEBUG("MYCALL: %s", (char*) mycall);
        DEBUG("Dedupe time (secs): %d", (int)dedupe_seconds);
        DEBUG("Aliases:");
        for (auto& a : aliases) {
            if (!a.set) continue;
            DEBUG(" call: %s", (char*)a.call);
            DEBUG(" use: %d", (int)a.use);
            DEBUG(" insert: %d", (int)a.insert_id);
            DEBUG(" preempt: %d", (int)a.preempt);
            DEBUG(" hops: %d", (int)a.hops);
        }
        DEBUG("Beacons:");
        for (auto& b : this->beacons) {
            if (b.seconds == 0) continue;
            DEBUG(" dest: %s", (char*)b.dest);
            DEBUG(" path: %s", (char*)b.path);
            DEBUG(" text: %s", (char*)b.text);
            DEBUG(" frequency (secs): %d", (int)b.seconds);
        }
        DEBUG("Checksum: %04xs", checksum);
    }

#if 1
    bool load();

    bool store() const;
#endif

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

}; // 812 bytes

extern Hardware& settings();

struct I2C_Storage
{
    constexpr static const uint16_t i2c_address{0xA0};
    constexpr static const uint16_t capacity{4096};
    constexpr static const uint16_t page_size{32};
    constexpr static const uint32_t write_time{5};

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

}}} // mobilinkd::tnc::kiss

#endif // INMOBILINKD__TNC__KISS_HARDWARE_HPP_C_KISSHARDWARE_HPP_
