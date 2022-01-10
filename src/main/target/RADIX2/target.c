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
#include "drivers/timer_def.h"

#include "fpga_drv.h"
#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"

#include "brainfpv/auto_sync_threshold.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM12, CH1, PB14,  TIM_USE_PPM,                 0,  0,  0 ), // PPM input
    DEF_TIM(TIM14, CH1, PA7,   TIM_USE_CAMERA_CONTROL,      0,  0,  0 ),

    DEF_TIM(TIM2,  CH1, PA0,   TIM_USE_MOTOR,               0,  0,  0 ), // M1
	DEF_TIM(TIM3,  CH2, PB5,   TIM_USE_MOTOR,               0,  1,  0 ), // M2
	DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_MOTOR,               0,  2,  0 ), // M3
    DEF_TIM(TIM4,  CH2, PD13,  TIM_USE_MOTOR,               0,  3,  0 ), // M4

    DEF_TIM(TIM8,  CH4, PC9,   TIM_USE_MOTOR,               0,  7,  2 ), // M5
    DEF_TIM(TIM8,  CH3, PC8,   TIM_USE_MOTOR,               0,  6,  2 ), // M6
    DEF_TIM(TIM1,  CH3, PE13,  TIM_USE_MOTOR,               0,  5,  1 ), // M7
    DEF_TIM(TIM1,  CH2, PE11,  TIM_USE_MOTOR,               0,  4,  1 ), // M8

    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_NONE,                 0,  0,  0 ), // Also TX2. Can be used for servo
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_NONE,                 0,  0,  0 ), // Also RX2. Can be used for servo
};

bool brainfpv_settings_updated_from_cms = false;

extern bfOsdConfig_t bfOsdConfigCms;
extern brainFpvSystemConfig_t brainFpvSystemConfigCms;

void brainFPVUpdateSettings(void) {
    const bfOsdConfig_t * bfOsdConfigUse;
    const brainFpvSystemConfig_t * brainFpvSystemConfigUse;

    if (brainfpv_settings_updated_from_cms) {
        bfOsdConfigUse = &bfOsdConfigCms;
        brainFpvSystemConfigUse = &brainFpvSystemConfigCms;
    }
    else {
        bfOsdConfigUse = bfOsdConfig();
        brainFpvSystemConfigUse = brainFpvSystemConfig();
    }

    if (bfOsdConfigUse->sync_threshold_mode == SYNC_THRESHOLD_MANUAL) {
        brainFpvOsdSetSyncThresholdMv(4 * bfOsdConfigUse->sync_threshold);
    }

    BRAINFPVFPGA_SetXOffset(bfOsdConfigUse->x_offset);
    BRAINFPVFPGA_SetXScale(brainFpvOsdGetXScale());
    BRAINFPVFPGA_SetStatusLEDColor(brainFpvSystemConfigUse->status_led_color, brainFpvSystemConfigUse->status_led_brightness);

    brainfpv_settings_updated_from_cms = false;
}
