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

#include <stdbool.h>

#define TARGET_BOARD_IDENTIFIER "RDX2HD"
#define USBD_PRODUCT_STRING "BrainFPV RADIX 2 HD"

// For ChibiOS
// Priority needs to be lower than any of the BF interrupts that don't use CH_IRQ_EPILOGUE
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  13

#define USE_BRAINFPV_BOOTLOADER
#define BOOTLOADER_TARGET_MAGIC 0x785E9A14

//#define USE_CUSTOM_RESET
#define CUSTOM_RESET_PIN PC13

#define VECT_TAB_BASE 0x24000000

#define USE_TARGET_CONFIG

#define BRAINFPV

#define USE_BRAINFPV_RGB_STATUS_LED
#define USE_BRAINFPV_RGB_LED_TIMER
#define BRAINFPV_RGB_LED_TIMER_NO 5
#define LED0_PIN                PA2
#define LED0_INVERTED
#define LED1_PIN                PA0
#define LED1_INVERTED

#define USE_BEEPER
#define BEEPER_PIN              PE5
#define BEEPER_INVERTED

#define USE_PINIO
#define PINIO1_PIN              PC14 // VREG HD
#define USE_PINIOBOX

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PB14

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PB8
#define UART4_TX_PIN            PB9

#define USE_UART5
#define UART5_RX_PIN            PB12
#define UART5_TX_PIN            PB13

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART7
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            NONE

#define USE_VCP
#define VBUS_SENSING_PIN        PA9
#define VBUS_SENSING_ENABLED
#define USE_USB48MHZ_PLL

#define SERIAL_PORT_COUNT       8

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PD7

#define USE_I2C
#define USE_I2C_DEVICE_1
#undef I2C1_OVERCLOCK
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define I2C_DEVICE              (I2CDEV_1)

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define MAG_I2C_INSTANCE      I2C_DEVICE

#undef USE_GYRO_REGISTER_DUMP

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define QUADSPI1_SCK_PIN PB2
#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10

#define QUADSPI1_BK2_IO0_PIN NONE
#define QUADSPI1_BK2_IO1_PIN NONE
#define QUADSPI1_BK2_IO2_PIN NONE
#define QUADSPI1_BK2_IO3_PIN NONE
#define QUADSPI1_BK2_CS_PIN NONE

#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define USE_FLASH_M25P16
#define FLASH_QUADSPI_INSTANCE QUADSPI
#define M25P16_FIRST_SECTOR     64
#define M25P16_SECTORS_SPARE_END 3
#define USE_FLASHFS
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_RAM

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_DETECT_PIN PA8
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11


#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_EXTI
#define USE_GYRO
#define USE_ACC
#undef USE_MULTI_GYRO

#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define USE_SPI_GYRO
#define USE_GYRO_SPI_BMI270
#define USE_ACCGYRO_BMI270
#undef USE_GYRO_DLPF_EXPERIMENTAL

//#define BMI270_SPI_DIVISOR   8

#define GYRO_1_EXTI_PIN           PB5
#define GYRO_1_CS_PIN             PD3
#define GYRO_1_SPI_INSTANCE       SPI1
#define GYRO_1_ALIGN              CW0_DEG
#define GYRO_1_ALIGN              CW0_DEG

#define USE_BARO
#define USE_BARO_DPS310

#define USE_ADC
#define USE_ADC_INTERNAL // ADC3

#define ADC1_INSTANCE ADC1
#define ADC2_INSTANCE ADC2 // not used
#define ADC3_INSTANCE ADC3 // ADC3 only for core temp and vrefint
#define RSSI_ADC_PIN            PC4
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PA6

#define BOARD_HAS_VOLTAGE_DIVIDER
#define ADC_VOLTAGE_REFERENCE_MV 3285
#define VBAT_SCALE_DEFAULT            176
#define CURRENT_METER_SCALE_DEFAULT   200
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#ifdef USE_DMA_SPEC
#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9
#else
#define ADC1_DMA_STREAM DMA2_Stream0
#define ADC3_DMA_STREAM DMA2_Stream1
#endif

#define LIGHT_WS2811_INVERTED

#define DEFAULT_FEATURES        (FEATURE_OSD)
#define SERIALRX_UART           SERIAL_PORT_USART3
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 9

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) )

#undef USE_DSHOT_BITBANG
#undef USE_BRUSHED_ESC_AUTODETECT

extern bool brainfpv_settings_updated_from_cms;

void CustomSystemReset(void);
void brainFPVUpdateSettings(void);

//#define USE_BRAINFPV_DEBUG_PRINTF
#if defined(USE_BRAINFPV_DEBUG_PRINTF)
#define DEBUG_PRINTF_UARTDEV UARTDEV_2
#define DEBUG_UART_RX_PIN    PD6
#define DEBUG_UART_TX_PIN    PD5
#define UART2_RX_DMA_OPT DMA_OPT_UNUSED
#define UART2_TX_DMA_OPT DMA_OPT_UNUSED
#endif

