
#ifndef BRAINFPV_OSD_H
#define BRAINFPV_OSD_H

#include <stdint.h>
#include "brainfpv/video.h"
#include "pg/pg.h"

#include "osd/osd_elements.h"

typedef struct bfOsdConfig_s {
    uint8_t sync_threshold;
    uint8_t white_level;
    uint8_t black_level;
    int8_t x_offset;
    uint8_t x_scale;
    uint8_t sbs_3d_enabled;
    uint8_t sbs_3d_right_eye_offset;
    uint8_t font;
    uint8_t ir_system;
    uint16_t ir_trackmate_id;
    uint32_t ir_ilap_id;
    uint8_t ahi_steps;
    uint8_t bmi160foc;
    uint16_t bmi160foc_ret;
    uint8_t altitude_scale;
    uint8_t speed_scale;
    uint8_t map;
    uint16_t map_max_dist_m;
    uint8_t sticks_display;
    uint8_t show_pilot_logo;
    uint8_t invert;
    uint8_t hd_frame;
    uint8_t hd_frame_width;
    uint8_t hd_frame_height;
    int8_t hd_frame_h_offset;
    int8_t hd_frame_v_offset;
    uint8_t crsf_link_stats;
    uint8_t crsf_link_stats_power;
    uint8_t crsf_link_stats_rssi;
    uint8_t crsf_link_stats_snr;
    int8_t crsf_link_stats_snr_threshold;
    uint8_t spec_enabled;
    int8_t x_scale_diff;
    uint8_t sync_threshold_mode;
} bfOsdConfig_t;

typedef enum {
    CRSF_OFF = 0,
    CRSF_LQ_LOW = 1,
    CRSF_SNR_LOW = 2,
    CRSF_ON = 3,
} CrsfMode_t;

PG_DECLARE(bfOsdConfig_t, bfOsdConfig);

void brainFpvOsdInit(void);
void osdMain(void);
void resetBfOsdConfig(bfOsdConfig_t *bfOsdConfig);

void brainFpvOsdSetTempFont(uint8_t font);
void brainFpvOsdResetTempFont(void);

//void brainFpvOsdArtificialHorizon(void);
void brainFpvOsdCenterMark(void);
void brainFpvOsdUserLogo(uint16_t x, uint16_t y);
void brainFpvOsdMainLogo(uint16_t x, uint16_t y);

void osdElementDummy_BrainFPV(osdElementParms_t *element);
void osdElementArtificialHorizon_BrainFPV(osdElementParms_t *element);
void osdElementGpsHomeDirection_BrainFPV(osdElementParms_t *element);
void osdElementCraftName_BrainFPV(osdElementParms_t *element);
void osdBackgroundCrosshairs_BrainFPV(osdElementParms_t *element);
void osdElementRssi_BrainFPV(osdElementParms_t *element);
void osdElementLinkQuality_BrainFPV(osdElementParms_t *element);

#if defined(BRAINFPV_OSD_USE_STM32CMP)
void brainFpvOsdSetSyncThresholdMv(uint16_t threshold_mv);
#endif

uint8_t brainFpvOsdGetXScale(void);
#endif /* BRAINFPV_OSD */
