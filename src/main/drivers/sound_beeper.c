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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/pwm_output.h"

#include "pg/beeper_dev.h"

#if defined(USE_BRAINFPV_FPGA_BUZZER)
#include "fpga_drv.h"
#endif

#include "sound_beeper.h"


#if !defined(USE_BRAINFPV_FPGA_BUZZER)
static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
static uint16_t beeperFrequency = 0;

#if defined(USE_PWM_OUTPUT) && !defined(USE_BRAINFPV_FPGA_BUZZER)
static pwmOutputPort_t beeperPwm;
static uint16_t freqBeep = 0;

static void pwmWriteBeeper(bool on)
{
    if (!beeperPwm.io) {
        return;
    }

    if (on) {
        *beeperPwm.channel.ccr = (PWM_TIMER_1MHZ / freqBeep) / 2;
        beeperPwm.enabled = true;
    } else {
        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
    }
}

static void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperPwm.enabled);
}

static void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    const timerHardware_t *timer = timerAllocate(tag, OWNER_BEEPER, 0);
    IO_t beeperIO = IOGetByTag(tag);

    if (beeperIO && timer) {
        beeperPwm.io = beeperIO;
        IOInit(beeperPwm.io, OWNER_BEEPER, 0);
        IOConfigGPIOAF(beeperPwm.io, IOCFG_AF_PP, timer->alternateFunction);
        freqBeep = frequency;
        pwmOutConfig(&beeperPwm.channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / freqBeep, (PWM_TIMER_1MHZ / freqBeep) / 2, 0);

        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
    }
}
#endif
#endif

void systemBeep(bool onoff)
{
#if !defined(USE_BRAINFPV_FPGA_BUZZER)
    if (beeperFrequency == 0) {
        IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    }
#ifdef USE_PWM_OUTPUT
    else {
        pwmWriteBeeper(onoff);
    }
#endif
#else
    BRAINFPVFPGA_Buzzer(onoff);
#endif
}

void systemBeepToggle(void)
{
#if !defined(USE_BRAINFPV_FPGA_BUZZER)
    if (beeperFrequency == 0) {
        IOToggle(beeperIO);
    }
#ifdef USE_PWM_OUTPUT
    else {
        pwmToggleBeeper();
    }
#endif
#else
     BRAINFPVFPGA_BuzzerToggle();
#endif
}

void beeperInit(const beeperDevConfig_t *config)
{
#if defined(USE_BRAINFPV_FPGA_BUZZER)
    UNUSED(config);
#else
#ifdef USE_BEEPER
    beeperFrequency = config->frequency;
    if (beeperFrequency == 0) {
        beeperIO = IOGetByTag(config->ioTag);
        beeperInverted = config->isInverted;
        if (beeperIO) {
            IOInit(beeperIO, OWNER_BEEPER, 0);
            IOConfigGPIO(beeperIO, config->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
        }
        systemBeep(false);
    }
#ifdef USE_PWM_OUTPUT
    else {
        const ioTag_t beeperTag = config->ioTag;
        beeperPwmInit(beeperTag, beeperFrequency);
    }
#endif
#else
    UNUSED(config);
#endif
#endif
}
