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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "drivers/system.h"

//#include "common/typeconversion.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "config/feature.h"

#include "brainfpv/video.h"
#include "brainfpv/osd_utils.h"
#include "brainfpv/ir_transponder.h"
#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"

#include "cli/settings.h"

#if defined(USE_BRAINFPV_OSD)
bfOsdConfig_t bfOsdConfigCms;
uint8_t logo_on_arming;
#endif

brainFpvSystemConfig_t brainFpvSystemConfigCms;

static const void *menuBrainFPVOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    memcpy(&brainFpvSystemConfigCms, brainFpvSystemConfig(), sizeof(brainFpvSystemConfig_t));

#if defined(USE_BRAINFPV_OSD)
    memcpy(&bfOsdConfigCms, bfOsdConfig(), sizeof(bfOsdConfig_t));
    logo_on_arming = osdConfig()->logo_on_arming;
#endif

    return NULL;
}

static const void *menuBrainFPVOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(brainFpvSystemConfigMutable(), &brainFpvSystemConfigCms, sizeof(brainFpvSystemConfig_t));

#if defined(USE_BRAINFPV_OSD)
    memcpy(bfOsdConfigMutable(), &bfOsdConfigCms, sizeof(bfOsdConfig_t));
    osdConfigMutable()->logo_on_arming = logo_on_arming;
#endif

    return NULL;
}

#if defined(USE_BRAINFPV_OSD)
OSD_UINT8_t entryAhiSteps =  {&bfOsdConfigCms.ahi_steps, 0, 9, 1};
const char *STICKS_DISPLAY_NAMES[] = {"OFF", "MODE2", "MODE1"};
OSD_TAB_t entrySticksDisplay = {&bfOsdConfigCms.sticks_display, 2, &STICKS_DISPLAY_NAMES[0]};

#if (NUM_USER_FONTS > 1)
const char *FONT_NAMES[] = {"DEFAULT", "LARGE", "BOLD"};
OSD_TAB_t entryOSDFont = {&bfOsdConfigCms.font, 2, &FONT_NAMES[0]};
#endif

#if defined(BRAINFPV_OSD_WHITE_LEVEL_MIN)
OSD_UINT8_t entryWhiteLevel =  {&bfOsdConfigCms.white_level, BRAINFPV_OSD_WHITE_LEVEL_MIN, BRAINFPV_OSD_WHITE_LEVEL_MAX, 1};
OSD_UINT8_t entryBlackLevel =  {&bfOsdConfigCms.black_level, BRAINFPV_OSD_BLACK_LEVEL_MIN, BRAINFPV_OSD_BLACK_LEVEL_MAX, 1};
#endif

#if defined(USE_BRAINFPV_AUTO_SYNC_THRESHOLD)
const char *SYNC_TH_MODE_NAMES[] = {"MANUAL", "AUTO"};
OSD_TAB_t entrySyncThMode = {&bfOsdConfigCms.sync_threshold_mode, 2, &SYNC_TH_MODE_NAMES[0]};
#endif

OSD_UINT8_t entrySyncTh =  {&bfOsdConfigCms.sync_threshold, BRAINFPV_OSD_SYNC_TH_MIN, BRAINFPV_OSD_SYNC_TH_MAX, 1};
OSD_INT8_t entryXoff =  {&bfOsdConfigCms.x_offset, -8, 7, 1};
OSD_INT8_t entryXScale =  {&bfOsdConfigCms.x_scale_diff, -3, 3, 1};
OSD_UINT8_t entry3DShift =  {&bfOsdConfigCms.sbs_3d_right_eye_offset, 10, 40, 1};
OSD_UINT16_t entryMapMaxDist =  {&bfOsdConfigCms.map_max_dist_m, 10, 32767, 10};

OSD_Entry cmsx_menuBrainFPVOsdEntries[] =
{
    {"--- BRAIN OSD ---", OME_Label, NULL, NULL},
    {"AHI STEPS", OME_UINT8, NULL, &entryAhiSteps},
    {"ALTITUDE SCALE", OME_Bool, NULL, &bfOsdConfigCms.altitude_scale},
    {"SPEED SCALE", OME_Bool, NULL, &bfOsdConfigCms.speed_scale},
    {"MAP", OME_Bool, NULL, &bfOsdConfigCms.map},
    {"MAP MAX DIST M", OME_UINT16, NULL, &entryMapMaxDist},
    {"SHOW STICKS", OME_TAB, NULL, &entrySticksDisplay},
#if (NUM_USER_FONTS > 1)
    {"FONT", OME_TAB, NULL, &entryOSDFont},
#endif
#if defined(BRAINFPV_OSD_WHITE_LEVEL_MIN)
    {"OSD WHITE", OME_UINT8, NULL, &entryWhiteLevel},
    {"OSD BLACK", OME_UINT8, NULL, &entryBlackLevel},
#endif
    {"INVERT", OME_Bool, NULL, &bfOsdConfigCms.invert},
#if defined(USE_BRAINFPV_AUTO_SYNC_THRESHOLD)
    {"OSD SYNC TH MODE", OME_TAB, NULL, &entrySyncThMode},
#endif
    {"OSD SYNC TH", OME_UINT8, NULL, &entrySyncTh},
    {"OSD X OFF", OME_INT8, NULL, &entryXoff},
    {"OSD X SC", OME_INT8, NULL, &entryXScale},
    {"3D MODE", OME_Bool, NULL, &bfOsdConfigCms.sbs_3d_enabled},
    {"3D R SHIFT", OME_UINT8, NULL, &entry3DShift},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainFPVOsd = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVOsdEntries,
};

const char * HD_FRAME_NAMES[] = {"OFF", "FULL", "CORNER"};
OSD_TAB_t entryHdFrameMode = {&bfOsdConfigCms.hd_frame, 2, &HD_FRAME_NAMES[0]};
OSD_UINT8_t entryHdFrameWidth = {&bfOsdConfigCms.hd_frame_width, 20, 255, 1};
OSD_UINT8_t entryHdFrameHeight = {&bfOsdConfigCms.hd_frame_height, 20, 255, 1};
OSD_INT8_t entryHdFrameHOffset = {&bfOsdConfigCms.hd_frame_h_offset, -100, 100, 1};
OSD_INT8_t entryHdFrameVOffset = {&bfOsdConfigCms.hd_frame_v_offset, -100, 100, 1};

OSD_Entry cmsx_menuBrainFPVHdFrameEntries[] =
{
    {"-- HD CAM FRAME --", OME_Label, NULL, NULL},
    {"MODE", OME_TAB, NULL, &entryHdFrameMode},
    {"WIDTH", OME_UINT8, NULL, &entryHdFrameWidth},
    {"HEIGHT", OME_UINT8, NULL, &entryHdFrameHeight},
    {"H OFFSET", OME_INT8, NULL, &entryHdFrameHOffset},
    {"V OFFSET", OME_INT8, NULL, &entryHdFrameVOffset},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainFPVHdFrame = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVHdFrameEntries,
};

#if defined(USE_BRAINFPV_IR_TRANSPONDER)

const char * IR_NAMES[] = {"OFF", "I-LAP", "TRACKMATE"};
OSD_TAB_t entryIRSys = {&bfOsdConfigCms.ir_system, 2, &IR_NAMES[0]};
OSD_UINT32_t entryIRIlap =  {&bfOsdConfigCms.ir_ilap_id, 0, 9999999, 1};
OSD_UINT16_t entryIRTrackmate =  {&bfOsdConfigCms.ir_trackmate_id, 0, 4095, 1};

OSD_Entry cmsx_menuBrainFPVIrEntries[] =
{
    {"-- IR TRANSPONDER --", OME_Label, NULL, NULL},
    {"IR SYS", OME_TAB, NULL, &entryIRSys},
    {"I LAP ID", OME_UINT32, NULL, &entryIRIlap},
    {"TRACKMATE ID", OME_UINT16, NULL, &entryIRTrackmate},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainFPVIr = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVIrEntries,
};

#endif /* defined(USE_BRAINFPV_IR_TRANSPONDER) */

const char * LOGO_ON_ARM_OPT_NAMES[] = {"OFF", "ON", "FIRST"};
OSD_TAB_t entryLogoOnArmingMode = {&logo_on_arming, 3, &LOGO_ON_ARM_OPT_NAMES[0]};

#endif /* defined(USE_BRAINFPV_OSD) */

OSD_UINT8_t entryLEDBrightness =  {&brainFpvSystemConfigCms.status_led_brightness, 0, 255, 1};

OSD_Entry cmsx_menuBrainFPVEntires[] =
{
    {"--- BRAINFPV ---", OME_Label, NULL, NULL},
#if defined(USE_BRAINFPV_OSD)
    {"BRAIN OSD", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVOsd},
    {"HD FRAME", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVHdFrame},
#endif

#if defined(USE_BRAINFPV_RGB_STATUS_LED)
    {"LED COLOR",  OME_TAB,   NULL, &(OSD_TAB_t){&brainFpvSystemConfigCms.status_led_color, COLOR_COUNT - 1, lookupTableLedstripColors }},
    {"LED BRIGHTNESS ",  OME_UINT8, NULL, &entryLEDBrightness},
#endif

#if defined(USE_BRAINFPV_IR_TRANSPONDER)
    {"IR TRANSPONDER", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVIr},
#endif /* defined(USE_BRAINFPV_IR_TRANSPONDER) */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
    {"SPECTROGRAPH", OME_Bool, NULL, &bfOsdConfigCms.spec_enabled},
#endif /* defined(USE_BRAINFPV_SPECTROGRAPH) */
#if defined(USE_BRAINFPV_OSD)
    {"SHOW LOGO ON ARM", OME_TAB, NULL, &entryLogoOnArmingMode},
    {"SHOW PILOT LOGO", OME_Bool, NULL, &bfOsdConfigCms.show_pilot_logo},
#endif
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuBrainFPV = {
    .onEnter = menuBrainFPVOnEnter,
    .onExit = menuBrainFPVOnExit,
    .entries = cmsx_menuBrainFPVEntires,
};
#endif // CMS
