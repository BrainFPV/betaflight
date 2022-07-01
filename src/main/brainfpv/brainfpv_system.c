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
#include <string.h>
#include <ctype.h>

#include "platform.h"
#include "brainfpv/brainfpv_system.h"

#include "config/config.h"
#include "common/printf_serial.h"
#include "flight/mixer.h"
#include "io/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/light_led.h"
#include "io/beeper.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "scheduler/scheduler.h"

#include "pg/serial_uart.h"

#include "ch.h"


#if defined(BRAINFPV)

static BrainFPVSystemReq_t brainfpv_req = BRAINFPV_REQ_NONE;

PG_RESET_TEMPLATE(brainFpvSystemConfig_t, brainFpvSystemConfig,
  .status_led_color = COLOR_BLUE,
  .status_led_brightness = 255,
  .bmi_bwp_norm = 1,
  .dji_osd_warnings_info = 1,
);

PG_REGISTER_WITH_RESET_TEMPLATE(brainFpvSystemConfig_t, brainFpvSystemConfig, PG_BRAINFPV_SYSTEM_CONFIG, 0);

#if defined(USE_VTXFAULT_PIN)
IO_t vtx_fault_pin;

static void vtxFaultInit(void)
{
    vtx_fault_pin = IOGetByTag(IO_TAG(VTXFAULT_PIN));

    IOInit(vtx_fault_pin,  OWNER_OSD, 0);
    IOConfigGPIO(vtx_fault_pin, IO_CONFIG(GPIO_MODE_INPUT, 0, GPIO_PULLUP));
}

static void vtxFaultCheck(void)
{
    static bool fault_detected = false;

    if (IORead(vtx_fault_pin) == false) {
        // over current condition detected
        LED1_ON;
        fault_detected = true;
    }
    else {
        if (fault_detected) {
            // over current condition has been cleared
            LED1_OFF;
            fault_detected = false;
        }
    }
}
#endif /* defined(USE_VTXFAULT_PIN) */

#if defined(BRAINFPV_DEBUG_PIN)
IO_t debugPin;
#endif


// CPU utilization measurement
uint16_t brainFPVSystemGetCPULoad(void)
{
    static uint32_t t_last_measurement = 0;
    static uint64_t idle_cycles_last = 0;
    uint16_t load = 0;

    uint32_t t_now = millis();
    uint32_t dt;
    uint64_t tmp;
    thread_t * idle_tp = chRegFindThreadByName("idle");

    if (idle_tp) {
        // idle cycles / s
        dt = (t_now - t_last_measurement);
        if (dt > 0) {
            tmp = 1000 * (idle_tp->stats.cumulative - idle_cycles_last) / dt;

            // load %
            load = 100 - (100 * tmp) / SystemCoreClock;
        }

        t_last_measurement = t_now;
        idle_cycles_last = idle_tp->stats.cumulative;
    }

    return load;
}

void brainFPVSystemInit(void)
{
#if defined(USE_VTXFAULT_PIN)
    vtxFaultInit();
#endif /* defined(USE_VTXFAULT_PIN) */

#if defined(USE_BRAINFPV_DEBUG_PRINTF)
    // UART pins are not yet configured when this is called
    serialPinConfig_t tmpSerialPinConfig;

    for (uint8_t i=0; i<UARTDEV_COUNT; i++) {
        tmpSerialPinConfig.ioTagRx[i] = uartHardware[i].rxPins[0].pin;
        tmpSerialPinConfig.ioTagTx[i] = uartHardware[i].txPins[0].pin;
    }

    tmpSerialPinConfig.ioTagRx[DEBUG_PRINTF_UARTDEV] = DEFIO_TAG_E(DEBUG_UART_RX_PIN);
    tmpSerialPinConfig.ioTagTx[DEBUG_PRINTF_UARTDEV] = DEFIO_TAG_E(DEBUG_UART_TX_PIN);

    uartPinConfigure(&tmpSerialPinConfig);

    serialUartConfigMutable(DEBUG_PRINTF_UARTDEV)->txDmaopt = DMA_OPT_UNUSED;
    serialUartConfigMutable(DEBUG_PRINTF_UARTDEV)->rxDmaopt = DMA_OPT_UNUSED;

    serialPort_t * port = uartOpen(DEBUG_PRINTF_UARTDEV, NULL, NULL, 1000000, MODE_RXTX, SERIAL_NOT_INVERTED);

    printfSerialInit();
    setPrintfSerialPort(port);
    tfp_printf("DEBUG Print Init\n\r");
#endif

#if defined(BRAINFPV_DEBUG_PIN)
   debugPin = IOGetByTag(IO_TAG(BRAINFPV_DEBUG_PIN));
   IOInit(debugPin,  OWNER_OSD, 0);
   IOConfigGPIO(debugPin, IO_CONFIG(GPIO_MODE_OUTPUT_PP, 0, GPIO_NOPULL));
   IOLo(debugPin);
#endif
}

#if defined(USE_BRAINFPV_BOOTLOADER)
typedef struct __attribute__((packed)) {
    uint32_t target_magic;
    uint32_t isr_vector_base;
} BrainFPVBlHeader_t;

const BrainFPVBlHeader_t __attribute__((section (".bl_header_section"))) __attribute__((used)) BRAINFPV_BL_HEADER = {
    .target_magic = BOOTLOADER_TARGET_MAGIC,
    .isr_vector_base = VECT_TAB_BASE,
};
#endif /* defined(USE_BRAINFPV_BOOTLOADER) */

#if defined(USE_CUSTOM_RESET)
void CustomSystemReset(void)
{
    IO_t reset_pin = IOGetByTag(IO_TAG(CUSTOM_RESET_PIN));
    IOInit(reset_pin, OWNER_PULLDOWN, 0);
    IOConfigGPIO(reset_pin, IOCFG_OUT_OD);

    __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                         buffered write are completed before reset */

    IOLo(reset_pin);
    for(;;)                                                           /* wait until reset */
    {
      __NOP();
    }
}
#endif /* defined(USE_CUSTOM_RESET) */


// Set the request
void brainFPVSystemSetReq(BrainFPVSystemReq_t req)
{
    brainfpv_req = req;
}


void saveConfigAndNotifyBrainFPV(void)
{
    schedulerIgnoreTaskExecTime();

    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

// Execute request (called from betaflight system task)
void brainFPVSystemCheck(void)
{
    switch(brainfpv_req) {
        case BRAINFPV_REQ_NONE:
            // Nothing to do
            break;
        case BRAINFPV_REQ_UPDATE_HW_SETTINGS:
            brainFPVUpdateSettings();
            break;
        case BRAINFPV_REQ_SAVE_SETTINGS:
            saveConfigAndNotifyBrainFPV();
            break;
        case BRAINFPV_REQ_SAVE_SETTINGS_REBOOT:
            saveConfigAndNotifyBrainFPV();

            stopMotors();
            motorShutdown();
            delay(200);

            systemReset();
            break;
    }
    brainfpv_req = BRAINFPV_REQ_NONE;

#if defined(USE_VTXFAULT_PIN)
        vtxFaultCheck();
#endif /* defined(USE_VTXFAULT_PIN) */
}





#endif
