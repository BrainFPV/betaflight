
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



#ifndef FONTS_ARCH_H
#define FONTS_ARCH_H

#include <stdint.h>


struct FontEntry {
	uint8_t width;
	uint8_t height;
	const uint8_t* lookup;
	const uint16_t* data;
};

#define NUM_FONTS  5

#define BETAFLIGHT_DEFAULT 0
#define BETAFLIGHT_LARGE 1
#define BETAFLIGHT_BOLD 2
#define FONT8X10 3
#define FONT_OUTLINED8X8 4


#endif /* FONTS_ARCH_H */
