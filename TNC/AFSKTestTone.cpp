// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AFSKTestTone.hpp"
#include "ModulatorTask.hpp"

void startAfskToneTask(void const* arg)
{
    using mobilinkd::tnc::AFSKTestTone;

    auto test = static_cast<const AFSKTestTone*>(arg);

    while (true) {
        switch (test->state()) {
        case AFSKTestTone::State::NONE:
            osThreadYield();
            break;
        case AFSKTestTone::State::MARK:
        case AFSKTestTone::State::SPACE:
        case AFSKTestTone::State::BOTH:
            test->fill();
            break;
        default:
            break;
        }
    }
}

uint32_t testToneTaskBuffer[ 256 ];
osStaticThreadDef_t testToneTaskControlBlock;
static osThreadStaticDef(testToneTask, startAfskToneTask, osPriorityIdle, 0,
    256, testToneTaskBuffer, &testToneTaskControlBlock);

namespace mobilinkd { namespace tnc {

AFSKTestTone::AFSKTestTone()
{
  testToneTask_ = osThreadCreate(osThread(testToneTask), this);
  osThreadSuspend(testToneTask_);
}

void AFSKTestTone::transmit(State prev)
{
    if (prev == State::NONE) {
      osThreadResume(testToneTask_);
    }
}

void AFSKTestTone::mark()
{
    auto prev = state_;
    state_ = State::MARK;
    transmit(prev);
}

void AFSKTestTone::space()
{
    auto prev = state_;
    state_ = State::SPACE;
    transmit(prev);
}

void AFSKTestTone::both()
{
    auto prev = state_;
    state_ = State::BOTH;
    transmit(prev);
}

void AFSKTestTone::stop()
{
    if (state_ == State::NONE) return;

    state_ = State::NONE;
    getModulator().abort();
    osThreadSuspend(testToneTask_);
}

void AFSKTestTone::fill() const
{
    static State current = State::SPACE;

    switch (state_) {
    case AFSKTestTone::State::NONE:
        return;
    case AFSKTestTone::State::MARK:
        getModulator().send(true);
        break;
    case AFSKTestTone::State::SPACE:
        getModulator().send(false);
        break;
    case AFSKTestTone::State::BOTH:
        getModulator().send(current == State::SPACE);
        current = (current == State::MARK ? State::SPACE : State::MARK);
        break;
    default:
        break;
    }
}

}} // mobilinkd::tnc
