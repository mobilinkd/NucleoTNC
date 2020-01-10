// Copyright 2017-2019 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioLevel.hpp"
#include "Log.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "PortInterface.hpp"
#include "main.h"
#include "AudioInput.hpp"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "KissHardware.hpp"
#include "ModulatorTask.hpp"
#include "SerialPort.h"
#include "Led.h"
#include "main.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern osMessageQId hdlcOutputQueueHandle;

static PTT getPttStyle(const mobilinkd::tnc::kiss::Hardware& hardware)
{
    return hardware.options & KISS_OPTION_PTT_SIMPLEX ? PTT::SIMPLEX : PTT::MULTIPLEX;
}

void startIOEventTask(void const*)
{
    using namespace mobilinkd::tnc;

    initSerial();
    openSerial();

    print_startup_banner();

    auto& hardware = kiss::settings();
    if (!hardware.load() or !hardware.crc_ok())
    {
        hardware.init();
        hardware.store();
    }
    hardware.init();

    osMutexRelease(hardwareInitMutexHandle);

    hardware.debug();

    audio::init_log_volume();
    audio::setAudioOutputLevel();
    audio::setAudioInputLevels();
    setPtt(getPttStyle(hardware));

    osMessagePut(audioInputQueueHandle, mobilinkd::tnc::audio::DEMODULATOR,
        osWaitForever);

    /* Infinite loop */
    for (;;)
    {
        osEvent evt = osMessageGet(ioEventQueueHandle, osWaitForever);
        if (evt.status != osEventMessage)
            continue;

        uint32_t cmd = evt.value.v;
        if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
        {
            switch (cmd) {
            case CMD_USER_BUTTON_DOWN:
                INFO("Button Down");
                osMessagePut(audioInputQueueHandle,
                    mobilinkd::tnc::audio::AUTO_ADJUST_INPUT_LEVEL,
                    osWaitForever);
                osMessagePut(audioInputQueueHandle,
                    mobilinkd::tnc::audio::DEMODULATOR, osWaitForever);
                break;
            case CMD_USER_BUTTON_UP:
                DEBUG("Button Up");
                break;
            case CMD_SET_PTT_SIMPLEX:
                getModulator().set_ptt(&simplexPtt);
                break;
            case CMD_SET_PTT_MULTIPLEX:
                getModulator().set_ptt(&multiplexPtt);
                break;
            default:
                WARN("unknown command = %04x", static_cast<unsigned int>(cmd));
                break;
            }
            continue;
        }

        using hdlc::IoFrame;

        auto frame = static_cast<IoFrame*>(evt.value.p);

        switch (frame->source()) {
        case IoFrame::RF_DATA:
//      DEBUG("RF frame");
            if (!ioport->write(frame, 100))
            {
                ERROR("Timed out sending frame");
            }
            break;
        case IoFrame::SERIAL_DATA:
//      DEBUG("Serial frame");
            if ((frame->type() & 0x0F) == IoFrame::DATA)
            {
                if (osMessagePut(hdlcOutputQueueHandle,
                    reinterpret_cast<uint32_t>(frame),
                    osWaitForever) != osOK)
                {
                    ERROR("Failed to write frame to TX queue");
                    hdlc::release(frame);
                }
            }
            else
            {
                kiss::handle_frame(frame->type(), frame);
            }
            break;
        case IoFrame::DIGI_DATA:
//      DEBUG("Digi frame");
            if (osMessagePut(hdlcOutputQueueHandle,
                reinterpret_cast<uint32_t>(frame),
                osWaitForever) != osOK)
            {
                hdlc::release(frame);
            }
            break;
        case IoFrame::FRAME_RETURN:
            hdlc::release(frame);
            break;
        }
    }
}

namespace mobilinkd {
namespace tnc {

void print_startup_banner()
{
#ifdef KISS_LOGGING
    uint32_t* uid = (uint32_t*) UID_BASE;  // STM32L4xx (same for 476 and 432)

    INFO("%s version %s", mobilinkd::tnc::kiss::HARDWARE_VERSION,
        mobilinkd::tnc::kiss::FIRMWARE_VERSION);
    INFO("CPU core clock: %luHz", SystemCoreClock);
    INFO("    Device UID: %08lX %08lX %08lX", uid[0], uid[1], uid[2]);

    uint8_t* version_ptr = (uint8_t*) 0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
#endif
}

}
} // mobilinkd::tnc
