BRAINFPV_H750_TARGETS += $(TARGET)
H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 16000000
START_ADDRESS = 0x000000

FEATURES       += VCP ONBOARDFLASH SDCARD SDCARD_SDIO CHIBIOS BRAINFPV BRAINFPV_BL

TARGET_SRC += \
              drivers/accgyro/accgyro_mpu.c \
              drivers/accgyro/accgyro_spi_bmi270.c \
              $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
              drivers/barometer/barometer_dps310.c \
              drivers/compass/compass_hmc5883l.c \
              drivers/compass/compass_qmc5883l.c \
              drivers/compass/compass_lis3mdl.c \
              drivers/compass/compass_ak8963.c \
              drivers/compass/compass_ak8975.c \
