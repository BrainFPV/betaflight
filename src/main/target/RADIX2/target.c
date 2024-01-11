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

#include "fpga_drv.h"
#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"
#include "brainfpv/auto_sync_threshold.h"

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
