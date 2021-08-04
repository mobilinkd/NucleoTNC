// Copyright 2017-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "LEDIndicator.h"
#include "main.h"

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_tim.h>
#include <stm32l4xx_hal_tim_ex.h>
#include <cmsis_os.h>

#include <functional>
#include <atomic>

#include <stdint.h>

extern TIM_HandleTypeDef LED_PWM_TIMER_HANDLE;

namespace mobilinkd {
namespace tnc {


/**
 * No connection shows a low, slow breathing. Each breath inhale takes
 * for 500ms, is held for 500ms, and exhaled in 500ms. This is repeated
 * every 10 seconds.  Maximum brightness is 20%.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 300ms(30)
 *  - hold 400ms (40)
 *  - ramp down 300ms (30)
 *  - wait 9000ms (900)
 *
 *
 */
struct NoConnection
{
    enum STATE
    {
        RAMP_UP_1, WAIT_1, RAMP_DN_1, WAIT_2
    };

    int count { 0 };
    int state { RAMP_UP_1 };

    int operator()()
    {
        int result;
        switch (state) {
        case RAMP_UP_1:
            result = count * 40;
            if (count == 49)
            {
                count = 0;
                state = WAIT_1;
            }
            else
            {
                ++count;
            }
            break;
        case WAIT_1:
            result = 2000;
            if (count == 49)
            {
                state = RAMP_DN_1;
                count = 49;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_1:
            result = count * 40;
            if (count == 0)
            {
                count = 0;
                state = WAIT_2;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_2:
            result = 0;
            if (count == 849)
            {
                state = RAMP_UP_1;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        }
        return result;
    }
};

/**
 * Bluetooth connection shows a double blip. Each blip lasts for 200ms
 * and is separated by 200ms, and is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 4400ms (440)
 *
 *
 */
struct BluetoothConnection
{
    enum STATE
    {
        RAMP_UP_1, RAMP_DN_1, WAIT_1, RAMP_UP_2, RAMP_DN_2, WAIT_2
    };

    int count { 0 };
    int pulse { 0 };
    int state { RAMP_UP_1 };
    int ramp[10] =
        { 1564, 3090, 4540, 5878, 7071, 8090, 8910, 9510, 9877, 9999 };

    int operator()()
    {
        int result;
        switch (state) {
        case RAMP_UP_1:
            result = ramp[count] / 2;
            if (count == 9)
            {
                state = RAMP_DN_1;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_1:
            result = ramp[count] / 2;
            if (count == 0)
            {
                state = WAIT_1;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_1:
            result = 0;
            if (count == 19)
            {
                state = RAMP_UP_2;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_UP_2:
            result = ramp[count] / 2;
            if (count == 9)
            {
                state = RAMP_DN_2;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_2:
            result = ramp[count] / 2;
            if (count == 0)
            {
                state = WAIT_2;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_2:
            result = 0;
            if (count == 439)
            {
                state = RAMP_UP_1;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        }
        return result;
    }
};

/**
 * USB connection shows a triple blip. Each blip lasts for 200ms. The
 * first two are separated by 400ms.  The third comes 200ms later.  This
 * is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 400ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 3800ms (440)
 *
 *
 */
struct USBConnection
{
    enum STATE
    {
        RAMP_UP_1,
        RAMP_DN_1,
        WAIT_1,
        RAMP_UP_2,
        RAMP_DN_2,
        WAIT_2,
        RAMP_UP_3,
        RAMP_DN_3,
        WAIT_3
    };

    int count { 0 };
    int pulse { 0 };
    int state { RAMP_UP_1 };
    int ramp[10] =
        { 1564, 3090, 4540, 5878, 7071, 8090, 8910, 9510, 9877, 9999 };

    int operator()()
    {
        int result;
        switch (state) {
        case RAMP_UP_1:
            result = ramp[count];
            if (count == 9)
            {
                state = RAMP_DN_1;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_1:
            result = ramp[count];
            if (count == 0)
            {
                state = WAIT_1;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_1:
            result = 0;
            if (count == 39)
            {
                state = RAMP_UP_2;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_UP_2:
            result = ramp[count];
            if (count == 9)
            {
                state = RAMP_DN_2;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_2:
            result = ramp[count];
            if (count == 0)
            {
                state = WAIT_2;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_2:
            result = 0;
            if (count == 19)
            {
                state = RAMP_UP_3;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_UP_3:
            result = ramp[count];
            if (count == 9)
            {
                state = RAMP_DN_3;
            }
            else
            {
                ++count;
            }
            break;
        case RAMP_DN_3:
            result = ramp[count];
            if (count == 0)
            {
                state = WAIT_3;
            }
            else
            {
                --count;
            }
            break;
        case WAIT_3:
            result = 0;
            if (count == 379)
            {
                state = RAMP_UP_1;
                count = 0;
            }
            else
            {
                ++count;
            }
            break;
        }
        return result;
    }
};

struct Flash
{
    enum class STATE
    {
        RAMP_UP, ON, RAMP_DN, OFF
    };

    typedef std::atomic<STATE> state_type;
    typedef std::function<int(void)> function_type;

    constexpr static const int ramp[10] =
        { 1564, 3090, 4540, 5878, 7071, 8090, 8910, 9510, 9877, 9999 };
#ifndef NUCLEOTNC
    constexpr static const uint32_t BLUE_CHANNEL = TIM_CHANNEL_1;
    constexpr static const uint32_t GREEN_CHANNEL = TIM_CHANNEL_2;
    constexpr static const uint32_t RED_CHANNEL = TIM_CHANNEL_3;
#else
    constexpr static const uint32_t BLUE_CHANNEL = TIM_CHANNEL_3;   // YELLOW...
    constexpr static const uint32_t GREEN_CHANNEL = TIM_CHANNEL_2;
    constexpr static const uint32_t RED_CHANNEL = TIM_CHANNEL_1;
#endif

    int gr_count { 9 };
    state_type gr_state { STATE::OFF };
    int rd_count { 9 };
    state_type rd_state { STATE::OFF };

    NoConnection noConnection;
    BluetoothConnection btConnection;
    USBConnection usbConnection;

    function_type blue_func { noConnection };

    int blue()
    {
        return blue_func();
    }

    int plain(state_type& state, int& counter, uint32_t channel)
    {
        int result = 0;
        switch (state) {
        case STATE::RAMP_UP:
            result = ramp[rd_count] / 3;
            if (counter == 9)
            {
                state = STATE::ON;
            }
            else
            {
                ++counter;
            }
            break;
        case STATE::ON:
            result = ramp[counter] / 3;
            break;
        case STATE::RAMP_DN:
            result = ramp[counter] / 3;
            if (counter == 0)
            {
                state = STATE::OFF;
                HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, channel);
            }
            else
            {
                --counter;
            }
            break;
        case STATE::OFF:
            result = 0;
            break;
        }
        return result;
    }


    int green()
    {
        return plain(gr_state, gr_count, GREEN_CHANNEL);
    }

    int red()
    {
        return plain(rd_state, rd_count, RED_CHANNEL);
    }

    void dcd_on()
    {
        auto expected = STATE::OFF;
        if (gr_state.compare_exchange_strong(expected, STATE::RAMP_UP))
        {
            HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, GREEN_CHANNEL);
        }
        else
        {
            gr_state = STATE::RAMP_UP;
        }

    }
    void dcd_off()
    {

        if (gr_state != STATE::OFF)
            gr_state = STATE::RAMP_DN;

    }

    void tx_on()
    {
        auto expected = STATE::OFF;
        if (rd_state.compare_exchange_strong(expected, STATE::RAMP_UP))
        {
            // PWM Channel must match
            HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, RED_CHANNEL);
        }
        else
        {
            rd_state = STATE::RAMP_UP;
        }
    }

    void tx_off()
    {

        if (rd_state != STATE::OFF)
            rd_state = STATE::RAMP_DN;

    }

    void disconnect()
    {
        blue_func = noConnection;
        HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
    }

    void usb()
    {
        blue_func = usbConnection;
        HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
    }

    void bt()
    {
        blue_func = btConnection;
        HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
    }
};

Flash& flash()
{
    static Flash blinker;
    return blinker;
}


}
} // mobilinkd::tnc

void LED_TIMER_PeriodElapsedCallback()
{
    using mobilinkd::tnc::flash;

    // CCR registers must match the TIM_CHANNEL used for each LED in Flash.
#ifndef NUCLEOTNC
    LED_PWM_TIMER_HANDLE.Instance->CCR1 = flash().blue();
    LED_PWM_TIMER_HANDLE.Instance->CCR2 = flash().green();
    LED_PWM_TIMER_HANDLE.Instance->CCR3 = flash().red();
#else
    LED_PWM_TIMER_HANDLE.Instance->CCR1 = flash().red();
    LED_PWM_TIMER_HANDLE.Instance->CCR2 = flash().green();
    LED_PWM_TIMER_HANDLE.Instance->CCR3 = flash().blue(); // YELLOW
#endif
}

void indicate_turning_on(void)
{
    HAL_TIM_Base_Start_IT(&LED_PWM_TIMER_HANDLE);
    tx_on();
    rx_on();
}

void indicate_initializing_ble(void)
{
    tx_off();
}

void indicate_on()
{
    tx_off();
    rx_off();
}

void indicate_waiting_to_connect(void)
{
    mobilinkd::tnc::flash().disconnect();
}

void indicate_connected_via_usb(void)
{
    mobilinkd::tnc::flash().usb();
}

void indicate_connected_via_ble(void)
{
    mobilinkd::tnc::flash().bt();
}

void tx_on(void)
{
    mobilinkd::tnc::flash().tx_on();
}

void tx_off(void)
{
    mobilinkd::tnc::flash().tx_off();
}

// DCD is active.
void rx_on()
{
    mobilinkd::tnc::flash().dcd_on();
}

// DCD is active.
void rx_off()
{
    mobilinkd::tnc::flash().dcd_off();
}
