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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/stm32/timer_def.h"

#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
#include "brainfpv_rgb_led_timer.h"
#endif

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
const timerHardware_t timerHardwareRgbLed[3] = {
    DEF_TIM(TIM15, CH1, PE5, TIMER_OUTPUT_INVERTED,  0,  0 ), // R
    DEF_TIM(TIM15, CH2, PE6, TIMER_OUTPUT_INVERTED,  0,  0 ), // G
    DEF_TIM(TIM14, CH1, PA7, TIMER_OUTPUT_INVERTED,  0,  0 ), // B
};
#endif

bool brainfpv_settings_updated_from_cms = false;

extern bfOsdConfig_t bfOsdConfigCms;
extern brainFpvSystemConfig_t brainFpvSystemConfigCms;

void brainFPVUpdateSettings(void)
{
    const brainFpvSystemConfig_t * brainFpvSystemConfigUse;

    if (brainfpv_settings_updated_from_cms) {
        brainFpvSystemConfigUse = &brainFpvSystemConfigCms;
    }
    else {
        brainFpvSystemConfigUse = brainFpvSystemConfig();
    }

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
    brainFPVRgbLedSetLedColor(0, brainFpvSystemConfigUse->status_led_color, brainFpvSystemConfigUse->status_led_brightness);
#endif
}
