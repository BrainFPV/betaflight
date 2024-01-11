TARGET_MCU        := STM32H750xx
TARGET_MCU_FAMILY := STM32H7

HSE_VALUE    = 16000000
START_ADDRESS = 0x000000

TARGET_FLASH_SIZE = 1024

FEATURES       += VCP ONBOARDFLASH SDCARD SDCARD_SDIO CHIBIOS BRAINFPV BRAINFPV_BL

TARGET_SRC += \
              drivers/accgyro/accgyro_mpu.c \
              drivers/accgyro/accgyro_spi_bmi270.c \
              $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
              drivers/barometer/barometer_dps310.c \
              $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c)))