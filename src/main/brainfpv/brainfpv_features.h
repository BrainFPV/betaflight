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

#define USE_PINIO

#define USE_SERIALRX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_GHST       // ImmersionRC Ghost Protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_FPORT      // FrSky FPort
#define USE_SERIALRX_XBUS       // JR
#define USE_SERIALRX_SRXL2      // Spektrum SRXL2 protocol

#define USE_TELEMETRY

#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_SMARTPORT
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_GHST
#define USE_TELEMETRY_SRXL
#define USE_CRSF_CMS_TELEMETRY
#define USE_CRSF_LINK_STATISTICS

#define USE_SERVOS

#define USE_VTX
#define USE_OSD
#define USE_OSD_SD
#define USE_OSD_HD
#define USE_VARIO

#define USE_CAMERA_CONTROL

#define USE_BLACKBOX
#define USE_GPS
#define USE_LED_STRIP

#define USE_ACRO_TRAINER

#if TARGET_FLASH_SIZE > 512
#define USE_ESCSERIAL

#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol

#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM

#define USE_BATTERY_CONTINUE
#define USE_DASHBOARD
#define USE_EMFAT_AUTORUN
#define USE_EMFAT_ICON
#define USE_ESCSERIAL_SIMONK
#define USE_GPS_PLUS_CODES
#define USE_SERIAL_4WAY_SK_BOOTLOADER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF
#endif

#if defined(STM32H7)
#define SLOW_CONST   __attribute__((section(".slow_const")))
#define SLOW_CODE    __attribute__((section(".slow_code")))
#else
#define SLOW_CONST
#define SLOW_CODE
#endif

