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
#include "brainfpv/brainfpv_system.h"

#include "fc/init.h"

#include "scheduler/scheduler.h"

#ifndef NOMAIN
#if !defined(USE_CHIBIOS)
void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        scheduler();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
#endif

#if defined(USE_CHIBIOS)
#include <string.h>
#include "ch.h"
#include "hal_st.h"
#include "nvic.h"

extern uint32_t __process_stack_end__;
extern uint32_t __process_stack_base__;

binary_semaphore_t gyroSem;

void appIdleHook(void)
{
    // Called when the scheduler has no tasks to run
}


#if defined(USE_BRAINFPV_OSD)
#include "brainfpv/brainfpv_osd.h"
#include "drivers/osd.h"
#include "drivers/display.h"
#include "io/displayport_max7456.h"
#include "pg/vcd.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "cms/cms.h"


displayPort_t *osdDisplayPort;

static THD_WORKING_AREA(waOSDThread, 1024);
static THD_FUNCTION(OSDThread, arg)
{
    (void)arg;
    chRegSetThreadName("OSD");

    brainFpvOsdInit();
    osdMain();
}
#endif /* defined(USE_BRAINFPV_OSD) */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
#include "brainfpv/spectrograph.h"
extern binary_semaphore_t spectrographDataReadySemaphore;

static THD_WORKING_AREA(waSpecThread, 512);
static THD_FUNCTION(SpecThread, arg)
{
    (void)arg;
    chRegSetThreadName("Spectrograph");
    while (1) {
        // wait for data ready
        chBSemWait(&spectrographDataReadySemaphore);
        spectrographMain();
    }
}
#endif /* defined(USE_BRAINFPV_SPECTROGRAPH) */

//#define USE_DUMMY_TASK
#if defined(USE_DUMMY_TASK)
static THD_WORKING_AREA(waDummyThread, 512);
static THD_FUNCTION(DummyThread, arg)
{
    (void)arg;
    chRegSetThreadName("Dummy");
    while (1) {
        chThdSleepMilliseconds(1);
    }
}
#endif

static THD_WORKING_AREA(waBetaFlightThread, 2 * 1024);
static THD_FUNCTION(BetaFlightThread, arg)
{
    (void)arg;
    chRegSetThreadName("Betaflight");

    chBSemObjectInit(&gyroSem, FALSE);

#if defined(USE_BRAINFPV_SPECTROGRAPH)
    chBSemObjectInit(&spectrographDataReadySemaphore, FALSE);
#endif

    // Betaflight init
    init();

    // Start other threads
#if defined(USE_BRAINFPV_OSD)
    if (VideoIsInitialized()) {
        chThdCreateStatic(waOSDThread, sizeof(waOSDThread), NORMALPRIO, OSDThread, NULL);
    }
#endif /* USE_BRAINFPV_OSD */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
    if (bfOsdConfig()->spec_enabled) {
        spectrographInit();
        chThdCreateStatic(waSpecThread, sizeof(waSpecThread), LOWPRIO, SpecThread, NULL);
    }
#endif /* USE_BRAINFPV_SPECTROGRAPH */

#if defined(USE_DUMMY_TASK)
    chThdCreateStatic(waDummyThread, sizeof(waDummyThread), NORMALPRIO, DummyThread, NULL);
#endif

    // Run BF scheduler forever
    while(1) {
        scheduler();
    }
}

uint8_t safe_boot = 0;

#define CONTROL_MODE_PRIVILEGED             0
#define CONTROL_USE_PSP                     2
#define CONTROL_FPCA                        4
#define CRT0_CONTROL_INIT (CONTROL_USE_PSP | CONTROL_MODE_PRIVILEGED | CONTROL_FPCA)
int main(void)
{
    // Fill process stack
    memset((void*)&__process_stack_base__, CH_DBG_STACK_FILL_VALUE, &__process_stack_end__ - &__process_stack_base__ - 4);

    // init ChibiOS
    __set_PSP((uint32_t)&__process_stack_end__);
    asm("movs    r0, %[ctl]\n\t" // Switch to thread mode with PSP
        "msr     CONTROL, r0\n\t"
        "isb\n\t":: [ctl] "i" (CRT0_CONTROL_INIT) : "r0");

    stInit();
    chSysInit();

    // Start Betaflight thread
    chThdCreateStatic(waBetaFlightThread, sizeof(waBetaFlightThread), HIGHPRIO, BetaFlightThread, NULL);

    // sleep forever
    chThdSleep(TIME_INFINITE);
}

#endif /* defined(USE_CHIBIOS) */
#endif /* NOMAIN */
