// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "ModulatorTask.hpp"
#include "KissHardware.hpp"

mobilinkd::tnc::SimplexPTT simplexPtt;
mobilinkd::tnc::MultiplexPTT multiplexPtt;

mobilinkd::tnc::AFSKModulator* modulator;
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

mobilinkd::tnc::AFSKModulator& getModulator() {
    static mobilinkd::tnc::AFSKModulator instance(dacOutputQueueHandle, &simplexPtt);
    return instance;
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
        modulator->set_ptt(&simplexPtt);
    else
        modulator->set_ptt(&multiplexPtt);
}

void startModulatorTask(void const*) {

    using namespace mobilinkd::tnc::kiss;

    modulator = &(getModulator());
    encoder = &(getEncoder());

    updatePtt();

    modulator->set_twist(settings().tx_twist);

    encoder->tx_delay(settings().txdelay);
    encoder->p_persist(settings().ppersist);
    encoder->slot_time(settings().slot);
    encoder->tx_tail(settings().txtail);

    encoder->run();
}
