/**
 ******************************************************************************
 * @file       pios_ir_transponder.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      Generate packets for various infrared lap timin protocols
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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "common/maths.h"


#if defined(USE_BRAINFPV_IR_TRANSPONDER)

/* Based on code generated by pycrc v0.9, https://pycrc.org
 * Note: this is regular crc-16-ccitt CRC
 * using the configuration:
 *    Width         = 16
 *    Poly          = 0x1021
 *    Xor_In        = 0x0000
 *    ReflectIn     = False
 *    Xor_Out       = 0x0000
 *    ReflectOut    = False
 *    Algorithm     = bit-by-bit-fast
 * @brief Update a CRC with a data buffer
 * @param[in] crc Starting CRC value
 * @param[in] data Data buffer
 * @param[in] length Number of bytes to process
 * @returns Updated CRC
*****************************************************************************/
uint16_t CRC16_CCITT_updateCRC(uint16_t crc, const uint8_t *data, uint32_t data_len)
{
    uint32_t i;
    uint32_t bit;
    uint8_t c;

    while (data_len--) {
        c = *data++;
        for (i = 0x80; i > 0; i >>= 1) {
            bit = crc & 0x8000;
            if (c & i) {
                bit = !bit;
            }
            crc <<= 1;
            if (bit) {
                crc ^= 0x1021;
            }
        }
        crc &= 0xffff;
    }
    return crc & 0xffff;
}

/**
 * @brief Invert bits, needed for some protocols
 */
static void ir_invert_bits(uint8_t * data, uint8_t data_len)
{
    for (int i=0; i<data_len; i++) {
        data[i] = ~data[i];
    }
}

/**
 * @brief Generate an I-Lap packet
 * @note  Credit goes to Gareth Owenson for reverse engineering the protocol
 *        see http://blog.owenson.me/reversing-ilap-race-transponders
 */
void ir_generate_ilap_packet(uint32_t ilap_id, uint8_t * data, uint8_t data_len)
{
    if (data_len < 6)
        return;

    uint8_t crc_data[4] = {0, 0, 0, 0};

    uint8_t digit;

    // BCD encoded in reverse
    uint32_t decimal = 10000000;

    for (int pos=3; pos>=0; pos--) {
        digit = MIN(9U, (ilap_id / decimal));
        ilap_id -= digit * decimal;
        crc_data[pos] |= (digit << 4) & 0xF0;
        decimal /= 10;
        digit = MIN(9U, (ilap_id / decimal));
        ilap_id -= digit * decimal;
        crc_data[pos] |= digit & 0x0F;
        decimal /= 10;
    }
    crc_data[3] |= 0xf0;

    uint16_t crc = CRC16_CCITT_updateCRC(0x00, crc_data, 4);

    data[0] = crc_data[3];
    data[1] = (crc & 0xFF00) >> 8;
    data[2] = crc_data[2];
    data[3] = crc_data[1];
    data[4] = crc_data[0];
    data[5] = (crc & 0x00FF);

    ir_invert_bits(data, 6);
}

/**
 * @brief Generate a trackmate packet
 * @note Credit goes to Mitch Martin for reverse engineering the protocol
 *       see http://getglitched.com/?p=1288
 */
void ir_generate_trackmate_packet(uint16_t trackmate_id, uint8_t * data, uint8_t data_len)
{
    if (data_len < 4)
        return;

    uint8_t crc_data[2];
    crc_data[1] = trackmate_id & 0x00FF;
    crc_data[0] = (trackmate_id & 0xFF00) >> 8;

    uint16_t crc = CRC16_CCITT_updateCRC(0xFB1A, crc_data, 2);

    data[0] = (trackmate_id >> 8) & 0xFF;
    data[1] = trackmate_id & 0xFF;
    data[2] = (crc >> 8) & 0xFF;
    data[3] = crc & 0xFF;
}


#define MAX_ID_TRACKMATE 0xfff

// Get the next valid trackmate id
// It seems like the ID has some weird requirements:
// 1st nibble is 0, 2nd nibble is not 0, 1, 8, or F
// 3rd and 4th nibbles are not 0 or F
uint16_t ir_next_valid_trackmateid(uint16_t trackmate_id, int16_t step)
{
    uint8_t nibble;
    while (1){
        trackmate_id += step;
        if (trackmate_id > MAX_ID_TRACKMATE) {
            if (step > 0) {
                trackmate_id = 0;
            }
            else {
                trackmate_id = MAX_ID_TRACKMATE;
            }
        }
        // Test 2nd nibble
        nibble = (trackmate_id & 0x0F00) >> 8;
        if ((nibble == 0x00) || (nibble == 0x01) || (nibble == 0x08) || (nibble == 0x0F)) {
            // step through these quickly
            if (step > 0) {
                trackmate_id += 256;
            }
            else {
                trackmate_id -= 256;
            }
            continue;
        }
        // Test 3rd nibble
        nibble = (trackmate_id & 0x00F0) >> 4;
        if ((nibble == 0x00) || (nibble == 0x0F)) {
            continue;
        }
        // Test 4th nibble
        nibble = trackmate_id & 0x000F;
        if ((nibble == 0x00) || (nibble == 0x0F)) {
            continue;
        }
        // We have a valid ID
        break;
    }

    return trackmate_id;
}

#endif /* defined(USE_BRAINFPV_IR_TRANSPODER) */

