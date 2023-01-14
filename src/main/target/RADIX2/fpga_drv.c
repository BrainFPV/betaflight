/**
 ******************************************************************************
 * @addtogroup dRonin Targets
 * @{
 * @addtogroup BrainRE1 support files
 * @{
 *
 * @file       fpga_drv.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      Driver for the RE1 custom FPGA
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "platform.h"

#if defined(USE_BRAINFPV_FPGA)

#include "fpga_drv.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/bus_spi.h"
#include "drivers/time.h"
#include "common/colorconversion.h"

/**
* RE1 Register Specification
* ##########################
*
* Address  Name   Type  Length  Default  Description
* 0x00     HWREV  R     1       NA       Hardware/FPGA revision
* 0x01     CFG    R/W   1       0x00     Configuration
*                                        CFG[0] : 0: 2BIT_PIXEL 1: 4BIT_PIXEL
* 0x06     XCFG   R/W   1       0x00     OSD X axis configuration
*                                        XCFG[7:4] x-axis stretch
*                                        XCFG[3:0] x-axis offset
* 0x07     XCFG2  R/W   1       0x00     XCFG2[6]  : 0: normal 1: SBS3D mode
*                                        XCFG2[5:0]: right-eye x offset
* 0x0F     LED    W     3072    0x00     WS2812B LED data
*/

enum re1fpga_register {
    BRAINFPVFPGA_REG_HWREV   = 0x00,
    BRAINFPVFPGA_REG_CFG     = 0x01,
    BRAINFPVFPGA_REG_XCFG    = 0x06,
    BRAINFPVFPGA_REG_XCFG2   = 0x07,
    BRAINFPVFPGA_REG_LED_R   = 0x09,
    BRAINFPVFPGA_REG_LED_G   = 0x0A,
    BRAINFPVFPGA_REG_LED_B   = 0x0B,
    BRAINFPVFPGA_REG_OSDC_0  = 0x0C,
    BRAINFPVFPGA_REG_OSDC_1  = 0x0D,
    BRAINFPVFPGA_REG_LED     = 0x0F,
};

struct re1_shadow_reg {
    uint8_t reg_hwrev;
    uint8_t reg_cfg;
    uint8_t reg_xcfg;
    uint8_t reg_xcfg2;
    uint8_t reg_led_r;
    uint8_t reg_led_g;
    uint8_t reg_led_b;
    uint8_t reg_osdc_0;
    uint8_t reg_osdc_1;
};

extDevice_t fpga_spi_dev;

static bool fpga_initialized = false;

static volatile struct re1_shadow_reg shadow_reg;
static IO_t re1FPGACsPin = IO_NONE;
static IO_t re1FPGAResetPin = IO_NONE;

static int32_t BRAINFPVFPGA_WriteReg(uint8_t reg, uint8_t data, uint8_t mask);
static int32_t BRAINFPVFPGA_WriteRegDirect(enum re1fpga_register reg, uint8_t data);

#if defined(BRAINFPV_FPGA_INCLUDE_BITSTREAM)
static int32_t BRAINFPVFPGA_LoadBitstream(void);
#endif

/**
 * @brief Initialize the driver
 * @return 0 for success
 */
int32_t BRAINFPVFPGA_Init(bool load_config)
{
	UNUSED(load_config);

    if (!spiSetBusInstance(&fpga_spi_dev, SPI_DEV_TO_CFG(spiDeviceByInstance(BRAINFPVFPGA_SPI_INSTANCE)))) {
        return -1;
    }

    re1FPGACsPin = IOGetByTag(IO_TAG(BRAINFPVFPGA_CS_PIN));
    IOInit(re1FPGACsPin, OWNER_OSD, 0);
    IOConfigGPIO(re1FPGACsPin, SPI_IO_CS_CFG);
    IOHi(re1FPGACsPin);
    fpga_spi_dev.busType_u.spi.csnPin = re1FPGACsPin;

    spiSetClkDivisor(&fpga_spi_dev, BRAINFPVFPGA_SPI_DIVISOR);

    /* Configure 16MHz clock output to FPGA */
    IOInit(IOGetByTag(IO_TAG(BRAINFPVFPGA_CLOCK_PIN)), OWNER_OSD, 0);
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

    /* Configure reset pin */
    re1FPGAResetPin = IOGetByTag(IO_TAG(BRAINFPVFPGA_RESET_PIN));
    IOInit(re1FPGAResetPin, OWNER_OSD, 0);
    IOConfigGPIO(re1FPGAResetPin, IOCFG_OUT_PP);

    // Give the PLL some time to stabilize
    delay(1);

    // Reset the FPGA
    IOHi(re1FPGAResetPin);
    delay(1);
    IOLo(re1FPGAResetPin);
    delay(1);

    // Initialize registers
#if (VIDEO_BITS_PER_PIXEL == 4)
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_CFG, 0x01);
#else
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_CFG, 0x00);
    // OSD colors for 2 bit mode
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_OSDC_0, 0x1C);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_OSDC_1, 0x02);
#endif
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_XCFG, 0x08);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_XCFG2, 0x10);

    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_LED_R, 0x00);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_LED_G, 0x00);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_LED_B, 0xFF);

    fpga_initialized = true;

    return 0;
}

/**
 * @brief Writes one byte to the BRAINFPVFPGA register
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * @returns 0 when success
 */
static int32_t BRAINFPVFPGA_WriteReg(enum re1fpga_register reg, uint8_t data, uint8_t mask)
{
    volatile uint8_t* cur_reg;

    switch (reg) {
    case BRAINFPVFPGA_REG_HWREV:
        return -2;
    case BRAINFPVFPGA_REG_CFG:
        cur_reg = &shadow_reg.reg_cfg;
        break;
    case BRAINFPVFPGA_REG_XCFG:
        cur_reg = &shadow_reg.reg_xcfg;
        break;
    case BRAINFPVFPGA_REG_XCFG2:
        cur_reg = &shadow_reg.reg_xcfg2;
        break;
    case BRAINFPVFPGA_REG_LED_R:
        cur_reg = &shadow_reg.reg_led_r;
        break;
    case BRAINFPVFPGA_REG_LED_G:
        cur_reg = &shadow_reg.reg_led_g;
        break;
    case BRAINFPVFPGA_REG_LED_B:
        cur_reg = &shadow_reg.reg_led_b;
        break;
    default:
    case BRAINFPVFPGA_REG_LED:
        return -2;
    }

    uint8_t new_data = (*cur_reg & ~mask) | (data & mask);

    if (new_data == *cur_reg) {
        return 0;
    }

    *cur_reg = new_data;

    return BRAINFPVFPGA_WriteRegDirect(reg, new_data);
}

static int32_t BRAINFPVFPGA_WriteRegDirect(enum re1fpga_register reg, uint8_t data)
{
    spiWriteReg(&fpga_spi_dev, reg, data);

    switch (reg) {
        case BRAINFPVFPGA_REG_LED:
        case BRAINFPVFPGA_REG_HWREV:
            break;
        case BRAINFPVFPGA_REG_CFG:
            shadow_reg.reg_cfg = data;
            break;
        case BRAINFPVFPGA_REG_XCFG:
            shadow_reg.reg_xcfg = data;
            break;
        case BRAINFPVFPGA_REG_XCFG2:
            shadow_reg.reg_xcfg2 = data;
            break;
        case BRAINFPVFPGA_REG_LED_R:
            shadow_reg.reg_led_r = data;
            break;
        case BRAINFPVFPGA_REG_LED_G:
            shadow_reg.reg_led_g = data;
            break;
        case BRAINFPVFPGA_REG_LED_B:
            shadow_reg.reg_led_b = data;
            break;
        case BRAINFPVFPGA_REG_OSDC_0:
            shadow_reg.reg_osdc_0 = data;
            break;
        case BRAINFPVFPGA_REG_OSDC_1:
            shadow_reg.reg_osdc_1 = data;
            break;
    }

    return 0;
}

/**
 * @brief Get the Hardware
 */
uint8_t BRAINFPVFPGA_GetHWRevision(void)
{
    return shadow_reg.reg_hwrev;
}


/**
 * @brief Set programmable LED (WS2812B) colors
 */
int32_t BRAINFPVFPGA_SetLEDs(uint8_t * led_data, uint16_t n_leds)
{
    if (!fpga_initialized) {
        return 0;
    }

    n_leds = MIN(n_leds, 1024);

    spiWriteRegBuf(&fpga_spi_dev, BRAINFPVFPGA_REG_LED, led_data, 3 * n_leds);

    return 0;
}


/**
 * @brief Set OSD x offset
 */
void BRAINFPVFPGA_SetXOffset(int8_t x_offset)
{
    //x_offset += 5;
    if (x_offset >= 7)
        x_offset = 7;
    if (x_offset < -8)
        x_offset = -8;

    uint8_t value = 8 + x_offset;
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_XCFG, value, 0x0F);
}

/**
 * @brief Set OSD x scale
 */
void BRAINFPVFPGA_SetXScale(uint8_t x_scale)
{
    x_scale = (x_scale & 0x0F) << 4;
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_XCFG, x_scale, 0xF0);
}

/**
 * @brief Set 3D mode configuration
 */
void BRAINFPVFPGA_Set3DConfig(bool enabled, uint8_t x_shift_right)
{
    uint8_t cfg;
    uint8_t enabled_data = enabled ? 1 : 0;

    cfg = (enabled_data << 6) | (x_shift_right & 0x3F);
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_XCFG2, cfg, 0xFF);
}

extern const hsvColor_t hsv[];

void BRAINFPVFPGA_SetStatusLEDColor(colorId_e color, uint8_t brightness)
{
    hsvColor_t hsv_color;
    rgbColor24bpp_t * rgb_color;

    hsv_color = hsv[color];
    hsv_color.v = brightness;

    rgb_color = hsvToRgb24(&hsv_color);

    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_LED_R, rgb_color->rgb.r, 0xFF);
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_LED_G, rgb_color->rgb.g, 0xFF);
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_LED_B, rgb_color->rgb.b, 0xFF);
}

#endif /* defined(USE_BRAINFPV_FPGA) */




