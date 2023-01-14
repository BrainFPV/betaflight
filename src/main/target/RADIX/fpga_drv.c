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

#include "stm32f4xx_rcc.h"

#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/bus_spi.h"
#include "drivers/time.h"


/**
* RE1 Register Specification
* ##########################
*
* Address  Name   Type  Length  Default  Description
* 0x00     HWREV  R     1       NA       Hardware/FPGA revision
* 0x01     CFG    R/W   1       0x00     CFG[0]: Serial RX inverter
*                                                0: normal 1: invert
*                                        CFG[1]: MultiPort TX mode
*                                                0: output 1: bidirectional
*                                        CFG[2]: MultiPort TX inverter
*                                                0: normal 1: inverted
*                                        CFG[3]: MultiPort TX pullup/down
*                                                0: pullup / down disabled
*                                                1: pullup / down enabled
*                                        CFG[4]: MultiPort TX pullup/down
*                                                0: pull-down
*                                                1: pull-up
*                                        CFG[5]: LEDCFG 0: status: blue custom: green
*                                                       1: status: green custom: blue
*                                        CFG[6]: 0: BUZZER DC drive
*                                                1: BUZZER 4kHz drive
* 0x02     CTL    R/W   1       0x00     CTL[0]: BUZZER 0: off, 1: on
*                                        CTL[1]: Custom LED: 0: off, 1: on
*                                        CTL[2]: Alarm LED: 0: off, 1: on
* 0x03     BLACK  R/W   1       XXX      OSD black level
* 0x04     WHITE  R/W   1       XXX      OSD white level
* 0x05     THR    R/W   1       XXX      OSD sync detect threshold
* 0x06     XCFG   R/W   1       0x00     OSD X axis configuration
*                                        XCFG[7:4] x-axis stretch
*                                        XCFG[3:0] x-axis offset
* 0x07     XCFG2  R/W   1       0x00     XCFG2[6]  : 0: normal 1: SBS3D mode
*                                        XCFG2[5:0]: right-eye x offset
* 0x08     IRCFG  R/W   1       0x00     IRCFG[0:3] IR Protocol
*                                        0x0: OFF / STM32 controlled
*                                        0x1: I-Lap / Trackmate
*                                        0x2: XX
*                                        0x3: XX
*                                        IRCFG[7:4] IR Power
* 0x09     IRDATA W     16      0x00     IR tranponder data
* 0x0F     LED    W     3072    0x00     WS2812B LED data
*/

enum re1fpga_register {
    BRAINFPVFPGA_REG_HWREV   = 0x00,
    BRAINFPVFPGA_REG_CFG     = 0x01,
    BRAINFPVFPGA_REG_CTL     = 0x02,
    BRAINFPVFPGA_REG_BLACK   = 0x03,
    BRAINFPVFPGA_REG_WHITE   = 0x04,
    BRAINFPVFPGA_REG_THR     = 0x05,
    BRAINFPVFPGA_REG_XCFG    = 0x06,
    BRAINFPVFPGA_REG_XCFG2   = 0x07,
    BRAINFPVFPGA_REG_IRCFG   = 0x08,
    BRAINFPVFPGA_REG_IRDATA  = 0x09,
    BRAINFPVFPGA_REG_LED     = 0x0F,
};

struct re1_shadow_reg {
    uint8_t reg_hwrev;
    uint8_t reg_cfg;
    uint8_t reg_ctl;
    uint8_t reg_black;
    uint8_t reg_white;
    uint8_t reg_thr;
    uint8_t reg_xcfg;
    uint8_t reg_xcfg2;
    uint8_t reg_ircfg;
};

extDevice_t fpga_spi_dev;

static bool fpga_initialized = false;

static volatile struct re1_shadow_reg shadow_reg;
static IO_t re1FPGACsPin = IO_NONE;
static IO_t re1FPGACdonePin = IO_NONE;
static IO_t re1FPGACresetPin = IO_NONE;
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

    if (!spiSetBusInstance(&fpga_spi_dev, SPI_DEV_TO_CFG(spiDeviceByInstance(BRAINFPVFPGA_SPI_INSTANCE)))) {
        return -1;
    }

    re1FPGACsPin = IOGetByTag(IO_TAG(BRAINFPVFPGA_CS_PIN));
    IOInit(re1FPGACsPin, OWNER_OSD, 0);
    IOConfigGPIO(re1FPGACsPin, SPI_IO_CS_CFG);
    IOHi(re1FPGACsPin);

    fpga_spi_dev.busType_u.spi.csnPin = re1FPGACsPin;

    spiSetClkDivisor(&fpga_spi_dev, BRAINFPVFPGA_SPI_DIVISOR);

    if (load_config) {
        /* Configure the CDONE and CRESETB pins */
        re1FPGACdonePin = IOGetByTag(IO_TAG(BRAINFPVFPGA_CDONE_PIN));
        IOInit(re1FPGACdonePin, OWNER_OSD, 0);
        IOConfigGPIO(re1FPGACdonePin, IOCFG_IN_FLOATING);

        re1FPGACresetPin = IOGetByTag(IO_TAG(BRAINFPVFPGA_CRESET_PIN));
        IOInit(re1FPGACresetPin, OWNER_OSD, 0);
        IOConfigGPIO(re1FPGACresetPin, GPIO_OType_OD);
#if defined(BRAINFPV_FPGA_INCLUDE_BITSTREAM)
        if (BRAINFPVFPGA_LoadBitstream() < 0) {
            return -2;
        }
#else
        // CRESETB low for 1ms
        IOLo(re1FPGACresetPin);
        delay(1);
        IOHi(re1FPGACresetPin);

        for (int i=0; i<100; i++) {
            if (IORead(re1FPGACdonePin))
                break;
            delay(1);
        }

        if (!IORead(re1FPGACdonePin))
            return -2;
#endif
    }

    /* Configure 16MHz clock output to FPGA */
    IOInit(IOGetByTag(IO_TAG(BRAINFPVFPGA_CLOCK_PIN)), OWNER_OSD, 0);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(BRAINFPVFPGA_CLOCK_PIN)), IOCFG_AF_PP, GPIO_AF_MCO);
    RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
    RCC_MCO1Cmd(ENABLE);

    /* Configure reset pin */
    IOInit(IOGetByTag(IO_TAG(BRAINFPVFPGA_RESET_PIN)), OWNER_OSD, 0);
    re1FPGAResetPin = IOGetByTag(IO_TAG(BRAINFPVFPGA_RESET_PIN));
    IOInit(re1FPGAResetPin, OWNER_OSD, 0);
    IOConfigGPIO(re1FPGAResetPin, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN));

    // Give the PLL some time to stabilize
    delay(1);

    // Reset the FPGA
    IOHi(re1FPGAResetPin);
    delay(1);
    IOLo(re1FPGAResetPin);
    delay(1);

    // Initialize registers
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_CFG, 0x00);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_CTL, 0x00);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_BLACK, 35);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_WHITE, 110);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_THR, 30);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_XCFG, 0x08);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_XCFG2, 0x10);
    BRAINFPVFPGA_WriteRegDirect(BRAINFPVFPGA_REG_IRCFG, 0x00);

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
    case BRAINFPVFPGA_REG_CTL:
        cur_reg = &shadow_reg.reg_ctl;
        break;
    case BRAINFPVFPGA_REG_BLACK:
        cur_reg = &shadow_reg.reg_black;
        break;
    case BRAINFPVFPGA_REG_WHITE:
        cur_reg = &shadow_reg.reg_white;
        break;
    case BRAINFPVFPGA_REG_THR:
        cur_reg = &shadow_reg.reg_thr;
        break;
    case BRAINFPVFPGA_REG_XCFG:
        cur_reg = &shadow_reg.reg_xcfg;
        break;
    case BRAINFPVFPGA_REG_XCFG2:
        cur_reg = &shadow_reg.reg_xcfg2;
        break;
    case BRAINFPVFPGA_REG_IRCFG:
        cur_reg = &shadow_reg.reg_ircfg;
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
        case BRAINFPVFPGA_REG_CTL:
            shadow_reg.reg_ctl = data;
            break;
        case BRAINFPVFPGA_REG_BLACK:
            shadow_reg.reg_black = data;
            break;
        case BRAINFPVFPGA_REG_WHITE:
            shadow_reg.reg_white = data;
            break;
        case BRAINFPVFPGA_REG_THR:
            shadow_reg.reg_thr = data;
            break;
        case BRAINFPVFPGA_REG_XCFG:
            shadow_reg.reg_xcfg = data;
            break;
        case BRAINFPVFPGA_REG_XCFG2:
            shadow_reg.reg_xcfg2 = data;
            break;
        case BRAINFPVFPGA_REG_IRCFG:
            shadow_reg.reg_ircfg = data;
            break;
        case BRAINFPVFPGA_REG_IRDATA:
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
 * @brief Set the protocol for the IR transmitter
 */
int32_t BRAINFPVFPGA_SetIRProtocol(enum re1fpga_ir_protocols ir_protocol)
{
    uint8_t value = 0;

    switch (ir_protocol) {
    case BRAINFPVFPGA_IR_PROTOCOL_OFF:
        value = 0;
        break;
    case BRAINFPVFPGA_IR_PROTOCOL_ILAP:
    case BRAINFPVFPGA_IR_PROTOCOL_TRACKMATE:
        /* They use the same basic protocol */
        value = 0x01;
        break;
    }

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_IRCFG, value, 0x0F);
}

/**
 * @brief Set IR emitter data
 */
int32_t BRAINFPVFPGA_SetIRData(uint8_t * ir_data, uint8_t n_bytes)
{
    if (n_bytes > 16)
        return - 1;

    spiWriteRegBuf(&fpga_spi_dev, BRAINFPVFPGA_REG_IRDATA, ir_data, n_bytes);

    return 0;
}

/**
 * @brief Enable / disable the serial RX inverter
 */
int32_t BRAINFPVFPGA_SerialRxInvert(bool invert)
{
    uint8_t data;
    if (invert) {
        data = 0x01;
    }
    else {
        data = 0x00;
    }
    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CFG, data, 0x01);
}

/**
 * @brief Set MultiPort TX pin mode
 */
int32_t BRAINFPVFPGA_MPTxPinMode(bool bidrectional, bool invert)
{
    uint8_t data = 0;

    data |= bidrectional << 1;
    data |= invert << 2;

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CFG, data, 0x06);
}

/**
 * @brief Set MultiPort TX pull-up / pull-down resistor
 */
int32_t BRAINFPVFPGA_MPTxPinPullUpDown(bool enable, bool pullup)
{
    uint8_t data = 0;

    data |= enable << 3;
    data |= pullup << 4;

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CFG, data, 0x18);
}

/**
 * @brief Set buzzer type
 */
int32_t BRAINFPVFPGA_SetBuzzerType(enum re1fpga_buzzer_types type)
{
    uint8_t data;
    if (type == BRAINFPVFPGA_BUZZER_DC) {
        data = 0;
    }
    else {
        data = 255;
    }
    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CFG, data, 0x40);
}


/**
 * @brief Enable / disable buzzer
 */
int32_t BRAINFPVFPGA_Buzzer(bool enable)
{
    if (!fpga_initialized) {
        return 0;
    }

    uint8_t data = enable;

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CTL, data, 0x01);
}

/**
 * @brief Toggle buzzer
 */
int32_t BRAINFPVFPGA_BuzzerToggle(void)
{
    if (!fpga_initialized) {
        return 0;
    }

    uint8_t data = shadow_reg.reg_ctl ^ 0x01;
    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CTL, data, 0x01);
}

/**
 * @brief Enable / disable alarm LED
 */
int32_t BRAINFPVFPGA_AlarmLED(bool enable)
{
    if (!fpga_initialized) {
        return 0;
    }

    uint8_t data = 0;

    if (enable)
        data = 0x04;

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CTL, data, 0x04);
}

/**
 * @brief Toggle Alarm LED
 */
int32_t BRAINFPVFPGA_AlarmLEDToggle(void)
{
    if (!fpga_initialized) {
        return 0;
    }

    uint8_t data = shadow_reg.reg_ctl ^ 0x04;
    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CTL, data, 0x04);
}

/**
 * @brief Set the notification LED colors
 */
int32_t BRAINFPVFPGA_SetNotificationLedColor(enum re1fpga_led_colors led_colors)
{
    if (!fpga_initialized) {
        return 0;
    }

    uint8_t value;

    if (led_colors == BRAINFPVFPGA_STATUS_BLUE_CUSTOM_GREEN)
        value = 0;
    else
        value = 255;

    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_CFG, value, 0x20);
}

/**
 * @brief Set OSD black and white levels
 */
void BRAINFPVFPGA_SetBwLevels(uint8_t black, uint8_t white)
{
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_BLACK, black, 0xFF);
    BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_WHITE, white, 0xFF);
}

/**
 * @brief Set the threshold for the video sync pulse detector
 */
int32_t BRAINFPVFPGA_SetSyncThreshold(uint8_t threshold)
{
    return BRAINFPVFPGA_WriteReg(BRAINFPVFPGA_REG_THR, threshold, 0xFF);
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

#if defined(BRAINFPV_FPGA_INCLUDE_BITSTREAM)

#define MAX_BITSTREAM_SIZE 71256

#define DUMMY_BUFFER_SIZE (200 / 8 + 1)


//#include <pios_stringify.h>	/* __stringify */

#define BS_PAYLOAD_FILE /home/martin/data/brain_fpv/product_dev/radix_fc/fpga/code/re1_fpga/re1_fpga_Implmnt/sbt/outputs/bitmap/re1fpga_bitmap.bin

/* Indirect stringification.  Doing two levels allows the parameter to be a
 * macro itself.  For example, compile with -DFOO=bar, __stringify(FOO)
 * converts to "bar".
 *
 * Copied from linux kernel:
 *    http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/include/linux/stringify.h
 */
#define __stringify_1(x...)     #x
#define __stringify(x...)       __stringify_1(x)


asm(
	".section .rodata\n"

	".global _bs_payload_start\n"
	"_bs_payload_start:\n"
	".incbin \"" __stringify(BS_PAYLOAD_FILE) "\"\n"
	".global _bs_payload_end\n"
	"_bs_payload_end:\n"

	".global _bs_payload_size\n"
	"_bs_payload_size:\n"
	".word _bs_payload_end - _bs_payload_start\n"
	".previous\n"
);

/* The ADDRESSES of the _bu_payload_* symbols are the important data */
extern const uint32_t _bs_payload_start;
extern const uint32_t _bs_payload_end;
extern const uint32_t _bs_payload_size;


static int32_t BRAINFPVFPGA_LoadBitstream(void)
{
    int32_t retval = 0;

    // Drive CSN line low
    BRAINFPVFPGA_ClaimBus();

    // CRESETB low for 1ms
    IOLo(re1FPGACresetPin);
    delay(1);
    IOHi(re1FPGACresetPin);

    // Wait for 1ms
    delay(1);

    // Send bitstream file
    uint8_t b;
    uint8_t *send_buffer = (uint8_t *)&_bs_payload_start;

    /* Make sure the RXNE flag is cleared by reading the DR register */
    b = BRAINFPVFPGA_SPI_INSTANCE->DR;

    for (int i=0; i < MAX_BITSTREAM_SIZE + 20; i++) {
        /* get the byte to send */
        b = (i < MAX_BITSTREAM_SIZE) ? *(send_buffer++) : 0x00;

        /* Start the transfer */
        BRAINFPVFPGA_SPI_INSTANCE->DR = b;

        /* Wait until the TXE goes high */
        while (!(BRAINFPVFPGA_SPI_INSTANCE->SR & SPI_I2S_FLAG_TXE));
    }

    /* Wait for SPI transfer to have fully completed */
    while (BRAINFPVFPGA_SPI_INSTANCE->SR & SPI_I2S_FLAG_BSY);

    // Sample CDONE pin
    for (int i=0; i<100; i++) {
        if (IORead(re1FPGACdonePin))
            break;
        delay(1);
    }

    if (!IORead(re1FPGACdonePin))
        retval = -2;

    BRAINFPVFPGA_ReleaseBus();

    return retval;
}

#endif /* defined(BRAINFPV_FPGA_INCLUDE_BITSTREAM) */

#endif /* defined(USE_BRAINFPV_FPGA) */




