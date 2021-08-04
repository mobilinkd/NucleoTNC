// Copyright 2015-2021 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "ModulatorTask.hpp"
#include "HDLCEncoder.hpp"
#include "Modulator.hpp"
#include "M17Modulator.h"
#include "M17Encoder.h"
#include "Fsk9600Modulator.hpp"
#include "AFSKModulator.hpp"
#include "KissHardware.hpp"
#include "main.h"
#include "Log.h"

mobilinkd::tnc::SimplexPTT simplexPtt;
mobilinkd::tnc::MultiplexPTT multiplexPtt;

mobilinkd::tnc::Modulator* modulator;
mobilinkd::Encoder* encoder;

// DMA Conversion half complete.
extern "C" void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef*) {
    osEvent evt = osMessageGet(dacOutputQueueHandle, 0);
    if (evt.status == osEventMessage) {
        modulator->fill_first(evt.value.v);
    } else {
        modulator->empty_first();
    }
}

extern "C" void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef*) {
    osEvent evt = osMessageGet(dacOutputQueueHandle, 0);
    if (evt.status == osEventMessage) {
        modulator->fill_last(evt.value.v);
    } else {
        modulator->empty_last();
    }
}

extern "C" void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef*) {
    modulator->abort();
}

mobilinkd::tnc::Modulator& getModulator()
{
    using namespace mobilinkd::tnc;

    static AFSKModulator afsk1200modulator(dacOutputQueueHandle, &simplexPtt);
    static Fsk9600Modulator fsk9600modulator(dacOutputQueueHandle, &simplexPtt);
    static M17Modulator m17modulator(dacOutputQueueHandle, &simplexPtt);

    switch (kiss::settings().modem_type)
    {
    case kiss::Hardware::ModemType::FSK9600:
        return fsk9600modulator;
    case kiss::Hardware::ModemType::AFSK1200:
        return afsk1200modulator;
    case kiss::Hardware::ModemType::M17:
        return m17modulator;
    default:
        CxxErrorHandler();
    }
}

mobilinkd::Encoder& getEncoder()
{
    using namespace mobilinkd::tnc;

    static hdlc::Encoder hdlcEncoder(hdlcOutputQueueHandle);
    static mobilinkd::M17Encoder m17Encoder(hdlcOutputQueueHandle);

    switch (kiss::settings().modem_type)
    {
    case kiss::Hardware::ModemType::FSK9600:
        return hdlcEncoder;
    case kiss::Hardware::ModemType::AFSK1200:
        return hdlcEncoder;
    case kiss::Hardware::ModemType::M17:
        return m17Encoder;
    default:
        ERROR("Invalid modem type %d", int(kiss::settings().modem_type));
        CxxErrorHandler();
    }
}

void setPtt(PTT ptt)
{
    switch (ptt) {
    case PTT::SIMPLEX:
        getModulator().set_ptt(&simplexPtt);
        break;
    case PTT::MULTIPLEX:
        getModulator().set_ptt(&multiplexPtt);
        break;
    }
}

void updatePtt()
{
    using namespace mobilinkd::tnc::kiss;

    if (settings().options & KISS_OPTION_PTT_SIMPLEX)
        getModulator().set_ptt(&simplexPtt);
    else
        getModulator().set_ptt(&multiplexPtt);
}

void updateModulator()
{
    using namespace mobilinkd::tnc::kiss;

    modulator = &getModulator();
    encoder = &getEncoder();
    modulator->init(settings());
    updatePtt();
    encoder->updateModulator();
    encoder->update_settings();
}

void startModulatorTask(void const*)
{
    using namespace mobilinkd::tnc::kiss;

    // Wait until hardware is initialized before creating modulator.
    osMutexWait(hardwareInitMutexHandle, osWaitForever);

    while (true)
    {
        modulator = &(getModulator());
        encoder = &(getEncoder());

        updatePtt();

        getModulator().init(settings());

        encoder->update_settings();
        encoder->run();
    }
}
