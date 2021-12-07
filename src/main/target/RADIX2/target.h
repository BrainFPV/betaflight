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

//#define DEBUG_PRINTF

#define TARGET_BOARD_IDENTIFIER "RDX2"
#define USBD_PRODUCT_STRING "BrainFPV RADIX 2"

// For ChibiOS
// Priority needs to be lower than any of the BF interrupts that don't use CH_IRQ_EPILOGUE
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  13

#if !defined(USE_BRAINFPV_BOOTLOADER)
#define USE_BRAINFPV_BOOTLOADER
#endif
#define BOOTLOADER_TARGET_MAGIC 0x65DF92FE

//#define USE_CUSTOM_RESET
//#define CUSTOM_RESET_PIN PC13

#define VECT_TAB_BASE 0x24000000

#define USE_TARGET_CONFIG

#define USE_BRAINFPV_FPGA
#define BRAINFPVFPGA_SPI_INSTANCE SPI3
#define BRAINFPVFPGA_SPI_DIVISOR  8
#define BRAINFPVFPGA_CS_PIN       PE1
#define BRAINFPVFPGA_RESET_PIN    PC4
#define BRAINFPVFPGA_CLOCK_PIN    PA8


#define BRAINFPV
#define USE_MAX7456
#define USE_OSD
#define USE_CMS
#define OSD_CALLS_CMS
#define USE_BRAINFPV_OSD
#define BRAINFPV_OSD_CMS_BG_BOX
#define BRAINFPV_OSD_CMS_CURSOR_HIGHLIGHT
#define BRAINFPV_OSD_CMS_FANCY_TITLE_FONT 2
#define VIDEO_BITS_PER_PIXEL 4
#define INCLUDE_VIDEO_QUADSPI
#define VIDEO_QSPI_CLOCK_PIN PB2
#define VIDEO_QSPI_IO0_PIN   PD11
#define VIDEO_QSPI_IO1_PIN   PC10
#define VIDEO_QSPI_IO2_PIN   PE2
#define VIDEO_QSPI_IO3_PIN   PA1
#define VIDEO_VSYNC          PE3
#define VIDEO_HSYNC          PD5
//#define BRAINFPV_OSD_TEST
//#define BRAINFPV_OSD_SHOW_DRAW_TIME


#define BRAINFPV_OSD_USE_STM32CMP
#define BRAINFPV_OSD_STM32CMP_DAC_INSTANCE DAC1
#define BRAINFPV_OSD_STM32CMP_CMP_INSTANCE COMP2
#define BRAINFPV_OSD_STM32CMP_CMP_INPUT_PIN PE9
#define BRAINFPV_OSD_STM32CMP_CMP_OUTPUT_PIN PE8

#define BRAINFPV_OSD_SYNC_TH_DEFAULT 150
#define BRAINFPV_OSD_SYNC_TH_MIN 0
#define BRAINFPV_OSD_SYNC_TH_MAX 255

#define USE_BRAINFPV_SPECTROGRAPH

#define USE_BRAINFPV_RGB_STATUS_LED

#define LED0_PIN                PE6
#define LED0_INVERTED
#define LED1_PIN                PE7
#define LED1_INVERTED

#define USE_BEEPER
#define BEEPER_PIN              PD14
#define BEEPER_INVERTED

#define USE_PINIO
#define PINIO1_PIN              PD15 // VTX
#define PINIO2_PIN              PC15 // Video input
#define USE_PINIOBOX

#define USE_VTXFAULT_PIN
#define VTXFAULT_PIN            PD10

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PB15
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PD0
#define UART4_TX_PIN            PD1

#define USE_UART5
#define UART5_RX_PIN            PB12
#define UART5_TX_PIN            PB13

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_VCP
#define VBUS_SENSING_PIN        PA9
#define VBUS_SENSING_ENABLED
#define USE_USB48MHZ_PLL

#define SERIAL_PORT_COUNT       7

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PD7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC1

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN            PA15

#define USE_I2C
#define USE_I2C_DEVICE_1
#undef I2C1_OVERCLOCK
#define I2C1_SCL                PB8
#define I2C1_SDA                PB7
#define I2C_DEVICE              (I2CDEV_1)

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define MAG_I2C_INSTANCE      I2C_DEVICE

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_FIRST_SECTOR     32
#define M25P16_SECTORS_SPARE_END 3
#define FLASH_CS_PIN           PE14
#define FLASH_SPI_INSTANCE     SPI1
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_RAM

#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO
#define USE_ACC
#undef USE_MULTI_GYRO

#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define USE_SPI_GYRO
#define USE_GYRO_SPI_BMI270
#define USE_ACCGYRO_BMI270

// SPI2 is running at 80MHz
#define BMI270_SPI_DIVISOR   8

#define GYRO_1_EXTI_PIN           PE4
#define GYRO_1_CS_PIN             PE15
#define GYRO_1_SPI_INSTANCE       SPI2
#define GYRO_1_ALIGN              CW0_DEG
#define GYRO_1_ALIGN              CW0_DEG

#define USE_BARO
#define USE_BARO_BMP388
#define USE_BARO_2SMBP_02B

#define USE_ADC
#define USE_ADC_INTERNAL // ADC3

#define ADC1_INSTANCE ADC1
#define ADC2_INSTANCE ADC2 // not used
#define ADC3_INSTANCE ADC3 // ADC3 only for core temp and vrefint
#define RSSI_ADC_PIN            PC0
#define VBAT_ADC_PIN            PA6
#define CURRENT_METER_ADC_PIN   PB0

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

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(12) | TIM_N(14) | TIM_N(15))

#undef USE_DSHOT_BITBANG
#undef USE_BRUSHED_ESC_AUTODETECT

extern bool brainfpv_settings_updated_from_cms;

void CustomSystemReset(void);
void brainFPVUpdateSettings(void);
