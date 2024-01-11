/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "fpga_drv.h"
#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"
#include "brainfpv/ir_transponder.h"

bool brainfpv_settings_updated_from_cms = false;

extern bfOsdConfig_t bfOsdConfigCms;

void brainFPVUpdateSettings(void) {
    const bfOsdConfig_t * bfOsdConfigUse;

    if (brainfpv_settings_updated_from_cms)
        bfOsdConfigUse = &bfOsdConfigCms;
    else
        bfOsdConfigUse = bfOsdConfig();

    if (!bfOsdConfigUse->invert){
        BRAINFPVFPGA_SetBwLevels(bfOsdConfigUse->black_level, bfOsdConfigUse->white_level);
    }
    else {
        BRAINFPVFPGA_SetBwLevels(bfOsdConfigUse->white_level, bfOsdConfigUse->black_level);
    }

    BRAINFPVFPGA_SetSyncThreshold(bfOsdConfigUse->sync_threshold);
    BRAINFPVFPGA_SetXOffset(bfOsdConfigUse->x_offset);
    BRAINFPVFPGA_SetXScale(brainFpvOsdGetXScale());
    BRAINFPVFPGA_Set3DConfig(bfOsdConfigUse->sbs_3d_enabled, bfOsdConfigUse->sbs_3d_right_eye_offset);

    if (bfOsdConfigUse->ir_system == 1) {
        uint8_t ir_data[6];
        ir_generate_ilap_packet(bfOsdConfigUse->ir_ilap_id, ir_data, 6);
        BRAINFPVFPGA_SetIRData(ir_data, 6);
        BRAINFPVFPGA_SetIRProtocol(BRAINFPVFPGA_IR_PROTOCOL_ILAP);
    }

    if (bfOsdConfigUse->ir_system == 2) {
        uint8_t ir_data[4];
        ir_generate_trackmate_packet(bfOsdConfigUse->ir_trackmate_id, ir_data, 6);
        BRAINFPVFPGA_SetIRData(ir_data, 4);
        BRAINFPVFPGA_SetIRProtocol(BRAINFPVFPGA_IR_PROTOCOL_TRACKMATE);
    }
    brainfpv_settings_updated_from_cms = false;
}
