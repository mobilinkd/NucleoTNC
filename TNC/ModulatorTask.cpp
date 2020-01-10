// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "ModulatorTask.hpp"
#include "Fsk9600Modulator.hpp"
#include "AFSKModulator.hpp"
#include "KissHardware.hpp"
#include "main.h"

mobilinkd::tnc::SimplexPTT simplexPtt;
mobilinkd::tnc::MultiplexPTT multiplexPtt;

mobilinkd::tnc::Modulator* modulator;
mobilinkd::tnc::hdlc::Encoder* encoder;

// DMA Conversion half complete.
extern "C" void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef*) {
    osEvent evt = osMessageGet(dacOutputQueueHandle, 0);
    if (evt.status == osEventMessage) {
        modulator->fill_first(evt.value.v);
    } else {
        modulator->empty();
    }
}

extern "C" void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef*) {
    osEvent evt = osMessageGet(dacOutputQueueHandle, 0);
    if (evt.status == osEventMessage) {
        modulator->fill_last(evt.value.v);
    } else {
        modulator->empty();
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

    switch (kiss::settings().modem_type)
    {
    case kiss::Hardware::ModemType::FSK9600:
        return fsk9600modulator;
    case kiss::Hardware::ModemType::AFSK1200:
        return afsk1200modulator;
    default:
        _Error_Handler(__FILE__, __LINE__);
    }
}

mobilinkd::tnc::hdlc::Encoder& getEncoder() {
    static mobilinkd::tnc::hdlc::Encoder instance(hdlcOutputQueueHandle, &getModulator());
    return instance;
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

void startModulatorTask(void const*) {

    using namespace mobilinkd::tnc::kiss;

    // Wait until hardware is initialized before creating modulator.
    osMutexWait(hardwareInitMutexHandle, osWaitForever);

    modulator = &(getModulator());
    encoder = &(getEncoder());

    updatePtt();

    getModulator().init(settings());

    encoder->tx_delay(settings().txdelay);
    encoder->p_persist(settings().ppersist);
    encoder->slot_time(settings().slot);
    encoder->tx_tail(settings().txtail);

    encoder->run();
}
