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

#define USE_BRAINFPV_AUTO_SYNC_THRESHOLD
#define AUTO_SYNC_THRESHOLD_ADC_INSTANCE ADC2_INSTANCE
#define AUTO_SYNC_THRESHOLD_ADC_PIN PC3
#define AUTO_SYNC_THRESHOLD_ADC_CHANNEL ADC_CHANNEL_13

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
#define USE_SPI_DMA_ENABLE_LATE

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_SDI_PIN            PB4
#define SPI1_SDO_PIN            PD7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_SDI_PIN            PC2
#define SPI2_SDO_PIN            PC1

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_SDI_PIN            PC11
#define SPI3_SDO_PIN            PC12
#define SPI3_NSS_PIN            PA15
// Disable DMA for SPI3
#define SPI3_TX_DMA_OPT         -2
#define SPI3_RX_DMA_OPT         -2

#define USE_I2C
#define USE_I2C_DEVICE_1
#undef I2C1_OVERCLOCK
#define I2C1_SCL_PIN             PB8
#define I2C1_SDA_PIN             PB7
#define I2C_DEVICE               (I2CDEV_1)

#define USE_MAG
#define MAG_I2C_INSTANCE      I2C_DEVICE

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_SPI
#define M25P16_FIRST_SECTOR     32
#define M25P16_SECTORS_SPARE_END 3
#define FLASH_CS_PIN           PE14
#define FLASH_SPI_INSTANCE     SPI1
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define CONFIG_IN_EXTERNAL_FLASH
#define EEPROM_SIZE 8192
//#define CONFIG_IN_RAM

// Config uses one 64 kB flash sector
#define FLASH_PAGE_SIZE 0x10000

#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH

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
#undef USE_GYRO_REGISTER_DUMP

#define GYRO_1_EXTI_PIN           PE4
#define GYRO_1_CS_PIN             PE15
#define GYRO_1_SPI_INSTANCE       SPI2
#define GYRO_1_ALIGN              CW0_DEG
#define GYRO_1_ALIGN              CW0_DEG

#define USE_BARO
#define USE_BARO_BMP388
#define USE_BARO_2SMBP_02B
#define USE_BARO_DPS310

#define USE_ADC
#define USE_ADC_INTERNAL // ADC3

#define ADC1_INSTANCE ADC1
#define ADC2_INSTANCE ADC2 // not used
#define ADC3_INSTANCE ADC3 // ADC3 only for core temp and vrefint
#define ADC_RSSI_PIN  PC0
#define ADC_VBAT_PIN  PA6
#define ADC_CURR_PIN  PB0

#define BOARD_HAS_VOLTAGE_DIVIDER
#define ADC_VOLTAGE_REFERENCE_MV 3285
#define DEFAULT_VOLTAGE_METER_SCALE   176
#define DEFAULT_CURRENT_METER_SCALE   200
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

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

//#define USABLE_TIMER_CHANNEL_COUNT 12
//#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(12) | TIM_N(14) | TIM_N(15))

// Timers and outputs
#define USE_TIMER_UP_CONFIG

#define MOTOR1_PIN           PA0   // TIM2 CH1
#define MOTOR2_PIN           PB5   // TIM3 CH2
#define MOTOR3_PIN           PD12  // TIM4 CH1
#define MOTOR4_PIN           PD13  // TIM4 CH2
#define MOTOR5_PIN           PC9   // TIM8 CH4
#define MOTOR6_PIN           PC8   // TIM8 CH3
#define MOTOR7_PIN           PE13  // TIM1 CH3
#define MOTOR8_PIN           PE11  // TIM1 CH2
#define SERVO1_PIN           PA2   // TIM15 CH1, Also TX2
#define SERVO2_PIN           PA3   // TIM15 CH2, Also RX2
#define RX_PPM_PIN           PB14  // TIM12 CH1
#define CAMERA_CONTROL_PIN   PA7   // TIM14 CH1

// TIMER_PIN_MAPPING(index, pin, occurence in fullTimerHardware, dma opt)
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA0,  1,  0) \
    TIMER_PIN_MAP( 1, PB5,  1,  1) \
    TIMER_PIN_MAP( 2, PD12, 1,  2) \
    TIMER_PIN_MAP( 3, PD13, 1,  3) \
    TIMER_PIN_MAP( 4, PC9,  2,  4) \
    TIMER_PIN_MAP( 5, PC8,  2,  5) \
    TIMER_PIN_MAP( 6, PE13, 1,  6) \
    TIMER_PIN_MAP( 7, PE11, 1,  7) \
    TIMER_PIN_MAP( 8, PA2,  3, -1) \
    TIMER_PIN_MAP( 9, PA3,  3, -1) \
    TIMER_PIN_MAP(10, PB14, 2, -1) \
    TIMER_PIN_MAP(11, PA7,  4, -1)

//#define USE_BRAINFPV_DEBUG_PRINTF
#if defined(USE_BRAINFPV_DEBUG_PRINTF)
#define DEBUG_PRINTF_UARTDEV UARTDEV_2
#define DEBUG_UART_RX_PIN    PA3
#define DEBUG_UART_TX_PIN    PA2
#define UART2_RX_DMA_OPT DMA_OPT_UNUSED
#define UART2_TX_DMA_OPT DMA_OPT_UNUSED
#endif

#undef USE_DSHOT_BITBANG
#undef USE_BRUSHED_ESC_AUTODETECT

extern bool brainfpv_settings_updated_from_cms;

void CustomSystemReset(void);
void brainFPVUpdateSettings(void);
