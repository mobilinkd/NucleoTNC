// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include <AudioLevel.hpp>
#include <Log.h>
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

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern osMessageQId hdlcOutputQueueHandle;

void startIOEventTask(void const*)
{
  initSerial();
  openSerial();

  mobilinkd::tnc::audio::init_log_volume();
  mobilinkd::tnc::print_startup_banner();


  auto& hardware = mobilinkd::tnc::kiss::settings();
  if (!hardware.crc_ok()) {
      hardware.init();
      if (!hardware.load()) hardware.store();
  }
  hardware.debug();
  strcpy((char*)hardware.mycall, "WX9O");
  hardware.update_crc();

  mobilinkd::tnc::audio::setAudioOutputLevel();
  mobilinkd::tnc::audio::setAudioInputLevels();

  osMessagePut(audioInputQueueHandle,
    mobilinkd::tnc::audio::DEMODULATOR, osWaitForever);

  /* Infinite loop */
  for(;;)
  {
    osEvent evt = osMessageGet(ioEventQueueHandle, osWaitForever);
    if (evt.status != osEventMessage) continue;

    uint32_t cmd = evt.value.v;
    if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
    {
      switch (cmd) {
      case CMD_USER_BUTTON_DOWN:
        INFO("Button Down");
        osMessagePut(audioInputQueueHandle,
          mobilinkd::tnc::audio::AUTO_ADJUST_INPUT_LEVEL, osWaitForever);
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

    using mobilinkd::tnc::hdlc::IoFrame;

    auto frame = static_cast<IoFrame*>(evt.value.p);

    switch (frame->source()) {
    case IoFrame::RF_DATA:
//      DEBUG("RF frame");
      if (!mobilinkd::tnc::ioport->write(frame, 100)) {
        ERROR("Timed out sending frame");
        mobilinkd::tnc::hdlc::release(frame);
      }
      break;
    case IoFrame::SERIAL_DATA:
//      DEBUG("Serial frame");
      if ((frame->type() & 0x0F) == IoFrame::DATA) {
        if (osMessagePut(hdlcOutputQueueHandle, reinterpret_cast<uint32_t>(frame),
          osWaitForever) != osOK) {
          ERROR("Failed to write frame to TX queue");
          mobilinkd::tnc::hdlc::release(frame);
        }
      } else {
        mobilinkd::tnc::kiss::handle_frame(frame->type(), frame);
      }
      break;
    case IoFrame::DIGI_DATA:
//      DEBUG("Digi frame");
      if (osMessagePut(hdlcOutputQueueHandle, reinterpret_cast<uint32_t>(frame),
        osWaitForever) != osOK) {
        mobilinkd::tnc::hdlc::release(frame);
      }
      break;
    case IoFrame::FRAME_RETURN:
      mobilinkd::tnc::hdlc::release(frame);
      break;
    }
  }
}

namespace mobilinkd { namespace tnc {

void print_startup_banner()
{
    uint32_t* uid = (uint32_t*)0x1FFF7590;  // STM32L4xx (same for 476 and 432)

    INFO("%s version %s", mobilinkd::tnc::kiss::HARDWARE_VERSION, mobilinkd::tnc::kiss::FIRMWARE_VERSION);
    INFO("CPU core clock: %luHz", SystemCoreClock);
    INFO(" Serial number: %08lX %08lX %08lX", uid[0], uid[1], uid[2]);

    uint8_t* version_ptr = (uint8_t*)0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
}

}} // mobilinkd::tnc
