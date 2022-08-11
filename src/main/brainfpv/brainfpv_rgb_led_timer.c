/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_BRAINFPV_RGB_LED_TIMER)

#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "common/color.h"
#include "common/colorconversion.h"

#include "brainfpv_rgb_led_timer.h"

#define NUM_LEDS 2
#define RGB_LED_PWM_FREQ_KHZ 45
#define RGB_LED_PWM_PERIOD 256

extern const timerHardware_t timerHardwareRgbLed[3];
extern const hsvColor_t hsv[];

static pwmOutputPort_t rgbLedChannels[3];

static rgbColor24bpp_t ledColors[NUM_LEDS] = {{.rgb.r = 0x9B, .rgb.g = 0x5E, .rgb.b = 0},
                                              {.rgb.r = 255, .rgb.g = 0, .rgb.b = 0}};

static bool ledOn[NUM_LEDS] = { 0 };

void brainFpvRgbLedTimerInit(void)
{

// XXX fix this!
#if BRAINFPV_RGB_LED_TIMER_NO == 5
    __HAL_RCC_TIM5_CLK_ENABLE();
#else
#error not supported
#endif

    for (int i=0; i < 3; i++) {
        const timerHardware_t * timerHardware = &timerHardwareRgbLed[i];

        rgbLedChannels[i].io = IOGetByTag(timerHardware->tag);
        IOInit(rgbLedChannels[i].io, OWNER_SYSTEM, i);

        IOConfigGPIOAF(rgbLedChannels[i].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        pwmOutConfig(&rgbLedChannels[i].channel, timerHardware, (RGB_LED_PWM_PERIOD * RGB_LED_PWM_FREQ_KHZ * 1000), RGB_LED_PWM_PERIOD, 0, 0);
    }

    // Start timer
    TIM_HandleTypeDef htim;
    htim.Instance = timerHardwareRgbLed[0].tim;
    HAL_TIM_Base_Start(&htim);
}

void brainFPVRgbLedSetLedColor(int led, colorId_e color, uint8_t brightness)
{
    if (led >= NUM_LEDS) {
        return;
    }

    hsvColor_t hsv_color;
    rgbColor24bpp_t * rgb_color;

    hsv_color = hsv[color];
    hsv_color.v = brightness;

    rgb_color = hsvToRgb24(&hsv_color);
    memcpy(&ledColors[led], rgb_color, sizeof(rgbColor24bpp_t));
}

static void rgbSet(uint8_t r, uint8_t g, uint8_t b)
{
//    if (rgbLedChannels[0].channel.ccr == NULL) {
//        return;
//    }

    *rgbLedChannels[0].channel.ccr = r;
    *rgbLedChannels[1].channel.ccr = g;
    *rgbLedChannels[2].channel.ccr = b;
}

void brainFPVRgbLedSet(int led, bool on)
{
    if (led >= NUM_LEDS) {
        return;
    }

    if (on) {
        rgbSet(ledColors[led].rgb.r, ledColors[led].rgb.g, ledColors[led].rgb.b);
        ledOn[led] = true;
    }
    else {
        // check if another LED is still on
        ledOn[led] = false;
        bool allLedsOff = true;
        for (int i = 0; i < NUM_LEDS; i++) {
            if (ledOn[i]) {
                rgbSet(ledColors[i].rgb.r, ledColors[i].rgb.g, ledColors[i].rgb.b);
                allLedsOff = false;
                break;
            }
        }
        if (allLedsOff) {
            rgbSet(0, 0, 0);
        }
    }
}

void brainFPVRgbLedToggle(int led)
{
    if (led >= NUM_LEDS) {
        return;
    }

    bool on = !ledOn[led];
    brainFPVRgbLedSet(led, on);
}

#endif /* defined(USE_BRAINFPV_RGB_LED_TIMER) */
