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

#pragma once

#include <stdint.h>
#include "pg/pg.h"
#include "io/ledstrip.h"


typedef struct brainFpvSystemConfig_s {
    colorId_e status_led_color;
    uint8_t status_led_brightness;
    uint8_t dji_osd_warnings_info;
    uint8_t dji_osd_rssi_use_lq;
} brainFpvSystemConfig_t;

PG_DECLARE(brainFpvSystemConfig_t, brainFpvSystemConfig);

typedef enum {
    BRAINFPV_REQ_NONE,
    BRAINFPV_REQ_UPDATE_HW_SETTINGS,
    BRAINFPV_REQ_SAVE_SETTINGS,
    BRAINFPV_REQ_SAVE_SETTINGS_REBOOT
} BrainFPVSystemReq_t;

void brainFPVSystemInit(void);
void brainFPVSystemSetReq(BrainFPVSystemReq_t req);
void brainFPVSystemCheck(void);
uint16_t brainFPVSystemGetCPULoad(void);

// implemented in osd_elements.c
#define BRAINFPV_DJI_OSD_ELEM_LENGTH 12
void brainFPVRenderCraftNameWarningsDji(char * buffer, int bufferLength);
