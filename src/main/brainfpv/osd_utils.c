/**
 ******************************************************************************
 * @addtogroup Tau Labs Modules
 * @{
 * @addtogroup OnScreenDisplay OSD Module
 * @brief OSD Utility Functions
 * @{
 *
 * @file       osd_utils.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2015
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2014.
 * @brief      OSD Utility Functions
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

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"
#if defined(USE_BRAINFPV_OSD)

#include "video.h"
#include "fonts.h"
#include "osd_utils.h"
#include "common/printf.h"

extern struct FontEntry* fonts[NUM_FONTS];

extern uint8_t *draw_buffer;
extern uint8_t *disp_buffer;

void clearGraphics(void)
{
	memset((uint8_t *)draw_buffer, 0, BUFFER_HEIGHT * BUFFER_WIDTH);
}


#if VIDEO_BITS_PER_PIXEL == 4


static FAST_DATA_ZERO_INIT uint16_t TWOBIT_TO_4BIT_VALUE[256];
static FAST_DATA_ZERO_INIT uint16_t TWOBIT_TO_4BIT_MASK[256];

static void fill_2bit_mask_table(void)
{
    uint8_t value;
    uint16_t mask_4bit;

    for (uint16_t value_2bit = 0; value_2bit < 256; value_2bit++) {
        mask_4bit = 0;
        for (uint8_t pxl_idx = 0; pxl_idx < 4; pxl_idx++) {
            value = (value_2bit >> (2 * (3 - pxl_idx)) & 0x03);
            if (value != 0) {
                mask_4bit |= 0xF << 4 * pxl_idx;
            }
        }
        TWOBIT_TO_4BIT_MASK[value_2bit] = mask_4bit;
    }
}

FAST_CODE_NOINLINE void set_text_color(OSDOSD_COLOR_t main_color, OSDOSD_COLOR_t outline_color)
{
    uint8_t value;
    uint16_t value_4bit;

    for (uint16_t value_2bit = 0; value_2bit < 256; value_2bit++) {
        value_4bit = 0;
        for (uint8_t pxl_idx = 0; pxl_idx < 4; pxl_idx++) {
            value = (value_2bit >> (2 * (3 - pxl_idx)) & 0x03);
            switch (value) {
                case 1:
                    // 2-bit black: outline
                    value_4bit |= (uint16_t)outline_color << 4 * pxl_idx;
                    break;
                case 3:
                    // 2-bit white: main color
                    value_4bit |= (uint16_t)main_color << 4 * pxl_idx;
                    break;
                default:
                    break;

            }
        }
        TWOBIT_TO_4BIT_VALUE[value_2bit] = value_4bit;
    }
    fill_2bit_mask_table();
}

FAST_CODE_NOINLINE static void draw_2bit_pixels_aligned(uint32_t addr, uint8_t value)
{
    uint16_t draw_value = TWOBIT_TO_4BIT_VALUE[value];
    uint16_t draw_mask = TWOBIT_TO_4BIT_MASK[value];

    uint16_t * p_draw = (uint16_t*)&draw_buffer[addr];

    *p_draw = (*p_draw & ~draw_mask) | draw_value;
}

FAST_CODE_NOINLINE static void draw_2bit_pixels_unaligned(uint32_t addr, uint8_t value)
{
    uint32_t draw_value = TWOBIT_TO_4BIT_VALUE[value];
    uint32_t draw_mask = TWOBIT_TO_4BIT_MASK[value];

    draw_value = draw_value << 4;
    draw_mask = draw_mask << 4;

    uint32_t * p_draw = (uint32_t*)&draw_buffer[addr];

    *p_draw = (*p_draw & ~draw_mask) | draw_value;
}
#endif



#if VIDEO_BITS_PER_PIXEL == 2

void draw_image(uint16_t x, uint16_t y, const struct Image * image)
{
    CHECK_COORDS_UNSIGNED(x + image->width, y);
	uint8_t byte_width = image->width / 4;

	uint8_t pixel_offset = 2 * (x % 4);
	uint8_t mask1 = 0xFF;
	uint8_t mask2 = 0x00;

	if (pixel_offset > 0) {
		for (uint8_t i = 0; i<pixel_offset; i++) {
			mask2 |= 0x01 << i;
		}
		mask1 = ~mask2;
	}

	for (uint16_t yp = 0; yp < image->height; yp++) {
		if ((y + yp) >  GRAPHICS_BOTTOM)
			break;
		for (uint16_t xp = 0; xp < byte_width; xp++) {
			draw_buffer[(y + yp) * BUFFER_WIDTH + xp + x / 4] |= (image->data[yp * byte_width + xp] & mask1) >> pixel_offset;
			if (pixel_offset > 0) {
				draw_buffer[(y + yp) * BUFFER_WIDTH + xp + x / 4 + 1] |= (image->data[yp * byte_width + xp] & mask2) <<
						(8 - pixel_offset);
			}
		}
	}
}

#elif VIDEO_BITS_PER_PIXEL == 4

void draw_image(uint16_t x, uint16_t y, const struct Image * image)
{
    CHECK_COORDS_UNSIGNED(x + image->width, y);
    int addr;
    uint8_t byte_width = image->width / 4;
    //uint8_t mask = CALC_BIT_MASK(x);
    uint8_t img_value;


    if ((x & 0x01) == 0x00) {
        // draw image aligned
        for (uint16_t yp = 0; yp < image->height; yp++) {
            if ((y + yp) >  GRAPHICS_BOTTOM)
                break;
            addr   = CALC_BUFF_ADDR(x, y + yp);
            for (uint16_t xp = 0; xp < byte_width; xp++) {
                img_value = image->data[yp * byte_width + xp];
                draw_2bit_pixels_aligned(addr, img_value);
                addr += 2;
            }
        }
    }
    else {
        // draw image mis-aligned
        for (uint16_t yp = 0; yp < image->height; yp++) {
            if ((y + yp) >  GRAPHICS_BOTTOM)
                break;
            addr   = CALC_BUFF_ADDR(x, y + yp);
            for (uint16_t xp = 0; xp < byte_width; xp++) {
                img_value = image->data[yp * byte_width + xp];
                draw_2bit_pixels_unaligned(addr, img_value);
                addr += 2;
            }
        }
    }
}
#endif


/**
 * draw_pixel draw a pixel
 *
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       color           color to use
 */
void draw_pixel(int x, int y, OSDOSD_COLOR_t color)
{
	CHECK_COORDS(x, y);
	// Determine the bit in the word to be set and the word
	// index to set it in.
	int addr   = CALC_BUFF_ADDR(x, y);
	uint8_t mask = CALC_BIT_MASK(x);
	uint8_t value = PACK_BITS(color);
	draw_WORD(draw_buffer, addr, mask, value);
}

/**
 * draw_hline: optimized horizontal line writing algorithm
 *
 * @param       x0      x0 coordinate
 * @param       x1      x1 coordinate
 * @param       y       y coordinate
 * @param       color
 */
void draw_hline(int x0, int x1, int y, OSDOSD_COLOR_t color)
{
	CHECK_COORD_Y(y);
	CLIP_COORD_X(x0);
	CLIP_COORD_X(x1);
	if (x0 > x1) {
		SWAP(x0, x1);
	}
	if (x0 == x1) {
		return;
	}
	/* This is an optimised algorithm for writing horizontal lines.
	 * We begin by finding the addresses of the x0 and x1 points. */
	int addr0     = CALC_BUFF_ADDR(x0, y);
	int addr1     = CALC_BUFF_ADDR(x1, y);
	int addr0_bit = CALC_BIT1_IN_WORD(x0);
	int addr1_bit = CALC_BIT0_IN_WORD(x1);
    uint8_t value = PACK_BITS(color);

	int mask, mask_l, mask_r, i;
	/* If the addresses are equal, we only need to write one word
	 * which is an island. */
	if (addr0 == addr1) {
		mask = COMPUTE_HLINE_ISLAND_MASK(addr0_bit, addr1_bit);
		draw_WORD(draw_buffer, addr0, mask, value);
	} else {
		/* Otherwise we need to write the edges and then the middle. */
		mask_l = COMPUTE_HLINE_EDGE_L_MASK(addr0_bit);
		mask_r = COMPUTE_HLINE_EDGE_R_MASK(addr1_bit);
		draw_WORD(draw_buffer, addr0, mask_l, value);
		draw_WORD(draw_buffer, addr1, mask_r, value);
		// Now write 0xffff words from start+1 to end-1.
		for (i = addr0 + 1; i <= addr1 - 1; i++) {
			uint8_t m = 0xff;
			draw_WORD(draw_buffer, i, m, value);
		}
	}
}

/**
 * draw_hline_outlined: outlined horizontal line with varying endcaps
 * Always uses draw buffer.
 *
 * @param       x0                      x0 coordinate
 * @param       x1                      x1 coordinate
 * @param       y                       y coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       line_color      color for line
 * @param       outline_color   color for outline
 */
void draw_hline_outlined(int x0, int x1, int y, int endcap0, int endcap1, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color)
{
	if (x0 > x1) {
		SWAP(x0, x1);
	}
	// Draw the main body of the line.
	draw_hline(x0 + 1, x1 - 1, y - 1, outline_color);
	draw_hline(x0 + 1, x1 - 1, y + 1, outline_color);
	draw_hline(x0 + 1, x1 - 1, y, line_color);

	// Draw the endcaps, if any.
	DRAW_ENDCAP_HLINE(endcap0, x0, y, outline_color, outline_color);
	DRAW_ENDCAP_HLINE(endcap1, x1, y, outline_color, outline_color);
}

/**
 * draw_vline: optimized vertical line writing algorithm
 *
 * @param       x       x coordinate
 * @param       y0      y0 coordinate
 * @param       y1      y1 coordinate
 * @param       color   color to use
 */
void draw_vline(int x, int y0, int y1, OSDOSD_COLOR_t color)
{
	CHECK_COORD_X(x);
	CLIP_COORD_Y(y0);
	CLIP_COORD_Y(y1);
	if (y0 > y1) {
		SWAP(y0, y1);
	}
	if (y0 == y1) {
		return;
	}
	/* This is an optimised algorithm for writing vertical lines.
	 * We begin by finding the addresses of the x,y0 and x,y1 points. */
	int addr0  = CALC_BUFF_ADDR(x, y0);
	int addr1  = CALC_BUFF_ADDR(x, y1);
	/* Then we calculate the pixel data to be written. */
	uint8_t mask = CALC_BIT_MASK(x);
	uint8_t value = PACK_BITS(color);
	/* Run from addr0 to addr1 placing pixels. Increment by the number
	 * of words n each graphics line. */
	for (int a = addr0; a <= addr1; a += BUFFER_WIDTH) {
		draw_WORD(draw_buffer, a, mask, value);
	}
}

/**
 * draw_vline_outlined: outlined vertical line with varying endcaps
 * Always uses draw buffer.
 *
 * @param       x                       x coordinate
 * @param       y0                      y0 coordinate
 * @param       y1                      y1 coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       line_color      color for line
 * @param       outline_color   color for outline
 */
void draw_vline_outlined(int x, int y0, int y1, int endcap0, int endcap1, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color)
{
	if (y0 > y1) {
		SWAP(y0, y1);
	}

	// Draw the main body of the line.
	draw_vline(x - 1, y0 + 1, y1 - 1, outline_color);
	draw_vline(x + 1, y0 + 1, y1 - 1, outline_color);
	draw_vline(x, y0 + 1, y1 - 1, line_color);
	// Draw the endcaps, if any.
	DRAW_ENDCAP_VLINE(endcap0, x, y0, outline_color, outline_color);
	DRAW_ENDCAP_VLINE(endcap1, x, y1, outline_color, outline_color);
}

/**
 * draw_filled_rectangle: draw a filled rectangle.
 *
 * Uses an optimized algorithm which is similar to the horizontal
 * line writing algorithm, but optimized for writing the lines
 * multiple times without recalculating lots of stuff.
 *
 * @param       buff    pointer to buffer to write in
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       width   rectangle width
 * @param       height  rectangle height
 * @param       color   color to use
 *
 */
void draw_filled_rectangle(int x, int y, int width, int height, OSDOSD_COLOR_t color)
{
	int yy, addr0_old, addr1_old;

	CHECK_COORDS(x, y);
	CHECK_COORDS(x + width, y + height);
	if (width <= 0 || height <= 0) {
		return;
	}
	// Calculate as if the rectangle was only a horizontal line. We then
	// step these addresses through each row until we iterate `height` times.
	int addr0     = CALC_BUFF_ADDR(x, y);
	int addr1     = CALC_BUFF_ADDR(x + width, y);
    int addr0_bit = CALC_BIT1_IN_WORD(x);
    int addr1_bit = CALC_BIT0_IN_WORD(x + width);
    uint8_t value = PACK_BITS(color);
	int mask, mask_l, mask_r, i;
	// If the addresses are equal, we need to write one word vertically.
	if (addr0 == addr1) {
		mask = COMPUTE_HLINE_ISLAND_MASK(addr0_bit, addr1_bit);
		while (height--) {
			draw_WORD(draw_buffer, addr0, mask, value);
			addr0 += BUFFER_WIDTH;
		}
	} else {
		// Otherwise we need to write the edges and then the middle repeatedly.
		mask_l    = COMPUTE_HLINE_EDGE_L_MASK(addr0_bit);
		mask_r    = COMPUTE_HLINE_EDGE_R_MASK(addr1_bit);
		// Write edges first.
		yy        = 0;
		addr0_old = addr0;
		addr1_old = addr1;
		while (yy < height) {
			draw_WORD(draw_buffer, addr0, mask_l, value);
			draw_WORD(draw_buffer, addr1, mask_r, value);
			addr0 += BUFFER_WIDTH;
			addr1 += BUFFER_WIDTH;
			yy++;
		}
		// Now write 0xffff words from start+1 to end-1 for each row.
		yy    = 0;
		addr0 = addr0_old;
		addr1 = addr1_old;
		while (yy < height) {
			for (i = addr0 + 1; i <= addr1 - 1; i++) {
				uint8_t m = 0xff;
				draw_WORD(draw_buffer, i, m, value);
			}
			addr0 += BUFFER_WIDTH;
			addr1 += BUFFER_WIDTH;
			yy++;
		}
	}
}


/**
 * draw_line: Draw a line of arbitrary angle.
 *
 * @param       buff    pointer to buffer to write in
 * @param       x0              first x coordinate
 * @param       y0              first y coordinate
 * @param       x1              second x coordinate
 * @param       y1              second y coordinate
 * @param       color           color
 */
void draw_line(int x0, int y0, int x1, int y1, OSDOSD_COLOR_t color)
{
	// Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	int steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep) {
		SWAP(x0, y0);
		SWAP(x1, y1);
	}
	if (x0 > x1) {
		SWAP(x0, x1);
		SWAP(y0, y1);
	}
    uint8_t value = PACK_BITS(color);

	int deltax     = x1 - x0;
	int deltay = abs(y1 - y0);
	int error      = deltax / 2;
	int ystep;
	int y = y0;
	int x; // , lasty = y, stox = 0;
	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}
	for (x = x0; x < x1; x++) {
		if (steep) {
			draw_pixel(y, x, value);
		} else {
			draw_pixel(x, y, value);
		}
		error -= deltay;
		if (error < 0) {
			y     += ystep;
			error += deltax;
		}
	}
}

/**
 * draw_line_outlined: Draw a line of arbitrary angle, with an outline.
 *
 * @param       x0                      first x coordinate
 * @param       y0                      first y coordinate
 * @param       x1                      second x coordinate
 * @param       y1                      second y coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       line_color      color for line
 * @param       outline_color   color for outline
 */
void draw_line_outlined(int x0, int y0, int x1, int y1,
						 __attribute__((unused)) int endcap0, __attribute__((unused)) int endcap1,
						 OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color)
{
	// Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	// This could be improved for speed.

	int steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		SWAP(x0, y0);
		SWAP(x1, y1);
	}
	if (x0 > x1) {
		SWAP(x0, x1);
		SWAP(y0, y1);
	}
	int deltax     = x1 - x0;
	int deltay = abs(y1 - y0);
	int error      = deltax / 2;
	int ystep;
	int y = y0;
	int x;
	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}
	// Draw the outline.
	for (x = x0; x < x1; x++) {
		if (steep) {
			draw_pixel(y - 1, x, outline_color);
			draw_pixel(y + 1, x, outline_color);
			draw_pixel(y, x - 1, outline_color);
			draw_pixel(y, x + 1, outline_color);
		} else {
			draw_pixel(x - 1, y, outline_color);
			draw_pixel(x + 1, y, outline_color);
			draw_pixel(x, y - 1, outline_color);
			draw_pixel(x, y + 1, outline_color);
		}
		error -= deltay;
		if (error < 0) {
			y     += ystep;
			error += deltax;
		}
	}
	// Now draw the innards.
	error = deltax / 2;
	y     = y0;
	for (x = x0; x < x1; x++) {
		if (steep) {
			draw_pixel(y, x, line_color);
		} else {
			draw_pixel(x, y, line_color);
		}
		error -= deltay;
		if (error < 0) {
			y     += ystep;
			error += deltax;
		}
	}
}

/**
 * draw_line_outlined_dashed: Draw a line of arbitrary angle, with an outline, potentially dashed.
 *
 * @param       x0              first x coordinate
 * @param       y0              first y coordinate
 * @param       x1              second x coordinate
 * @param       y1              second y coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       line_color      color for line
 * @param       outline_color   color for outline
 * @param       dots			0 = not dashed, > 0 = # of set/unset dots for the dashed innards
 */
void draw_line_outlined_dashed(int x0, int y0, int x1, int y1,
								__attribute__((unused)) int endcap0, __attribute__((unused)) int endcap1,
								OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color, int dots)
{
	// Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	// This could be improved for speed.

	int steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		SWAP(x0, y0);
		SWAP(x1, y1);
	}
	if (x0 > x1) {
		SWAP(x0, x1);
		SWAP(y0, y1);
	}
	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error  = deltax / 2;
	int ystep;
	int y = y0;
	int x;
	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}
	// Draw the outline.
	int dot_cnt = 0;
	int draw    = 1;
	for (x = x0; x < x1; x++) {
		if (dots && !(dot_cnt++ % dots)) {
			draw++;
		}
		if (draw % 2) {
			if (steep) {
				draw_pixel(y - 1, x, outline_color);
				draw_pixel(y + 1, x, outline_color);
				draw_pixel(y, x - 1, outline_color);
				draw_pixel(y, x + 1, outline_color);
			} else {
				draw_pixel(x - 1, y, outline_color);
				draw_pixel(x + 1, y, outline_color);
				draw_pixel(x, y - 1, outline_color);
				draw_pixel(x, y + 1, outline_color);
			}
		}
		error -= deltay;
		if (error < 0) {
			y     += ystep;
			error += deltax;
		}
	}
	// Now draw the innards.
	error = deltax / 2;
	y     = y0;
	dot_cnt = 0;
	draw    = 1;
	for (x = x0; x < x1; x++) {
		if (dots && !(dot_cnt++ % dots)) {
			draw++;
		}
		if (draw % 2) {
			if (steep) {
				draw_pixel(y, x, line_color);
			} else {
				draw_pixel(x, y, line_color);
			}
		}
		error -= deltay;
		if (error < 0) {
			y     += ystep;
			error += deltax;
		}
	}
}

/**
 * draw_word_misaligned_NAND: Write a misaligned word across two addresses
 * with an x offset, using a NAND mask.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 *
 * This is identical to calling draw_word_misaligned with a mode of 0 but
 * it doesn't go through a lot of switch logic which slows down text writing
 * a lot.
 */
void draw_word_misaligned_NAND(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff)
{
	uint16_t firstmask = word >> xoff;
	uint16_t lastmask  = word << (16 - xoff);

	draw_WORD_NAND(buff, addr + 1, firstmask & 0x00ff);
	draw_WORD_NAND(buff, addr, (firstmask & 0xff00) >> 8);
	if (xoff > 0) {
		draw_WORD_NAND(buff, addr + 2, (lastmask & 0xff00) >> 8);
	}
}

/**
 * draw_word_misaligned_OR: Write a misaligned word across two addresses
 * with an x offset, using an OR mask.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 *
 * This is identical to calling draw_word_misaligned with a mode of 1 but
 * it doesn't go through a lot of switch logic which slows down text writing
 * a lot.
 */
void draw_word_misaligned_OR(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff)
{
	uint16_t firstmask = word >> xoff;
	uint16_t lastmask  = word << (16 - xoff);

	draw_WORD_OR(buff, addr + 1, firstmask & 0x00ff);
	draw_WORD_OR(buff, addr, (firstmask & 0xff00) >> 8);
	if (xoff > 0) {
		draw_WORD_OR(buff, addr + 2, (lastmask & 0xff00) >> 8);
	}
}

/**
 * draw_word_misaligned_OR: Write a misaligned word across two addresses
 * with an x offset, using an OR mask.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 *
 * This is identical to calling draw_word_misaligned with a mode of 1 but
 * it doesn't go through a lot of switch logic which slows down text writing
 * a lot.
 */
void draw_word_misaligned_MASKED(uint8_t *buff, uint16_t word, uint16_t mask, unsigned int addr, unsigned int xoff)
{
	uint16_t firstword = (word >> xoff);
	uint16_t lastword  = word << (16 - xoff);
	uint16_t firstmask = (mask >> xoff);
	uint16_t lastmask  = mask << (16 - xoff);

	draw_WORD(buff, addr + 1, firstmask & 0x00ff, firstword & 0x00ff);
	draw_WORD(buff, addr, (firstmask & 0xff00) >> 8, (firstword & 0xff00) >> 8);
	if (xoff > 0) {
		draw_WORD(buff, addr + 2, (lastmask & 0xff00) >> 8, (lastword & 0xff00) >> 8);
	}
}

/**
 * draw_char: Draw a character on the current draw buffer.
 *
 * @param       ch           character to write
 * @param       x            x coordinate (left)
 * @param       y            y coordinate (top)
 * @param       font_info    font to use
 */
void draw_char(uint8_t ch, int x, int y, const struct FontEntry *font_info)
{
	int yy, row;
	uint16_t data16;

#if VIDEO_BITS_PER_PIXEL == 2
	uint16_t mask;
#endif

	ch = font_info->lookup[ch];
	if (ch == 255)
		return;

	// check if char is partly out of boundary
	uint8_t partly_out = (x < GRAPHICS_LEFT) || (x + font_info->width > GRAPHICS_RIGHT) || (y < GRAPHICS_TOP) || (y + font_info->height > GRAPHICS_BOTTOM);
	// check if char is totally out of boundary, if so return
	if (partly_out && ((x + font_info->width < GRAPHICS_LEFT) || (x > GRAPHICS_RIGHT) || (y + font_info->height < GRAPHICS_TOP) || (y > GRAPHICS_BOTTOM))) {
		return;
	}

	// Compute starting address of character
	int addr = CALC_BUFF_ADDR(x, y);
	int wbit = CALC_BIT_IN_WORD(x);
	row = ch * font_info->height;

	if (font_info->width > 8) {
		uint32_t data;
		for (yy = y; yy < y + font_info->height; yy++) {
			if (!partly_out || ((x >= GRAPHICS_LEFT) && (x + font_info->width <= GRAPHICS_RIGHT) && (yy >= GRAPHICS_TOP) && (yy <= GRAPHICS_BOTTOM))) {
				data = ((uint32_t*)font_info->data)[row];

#if VIDEO_BITS_PER_PIXEL == 2
				data16 = (data & 0xFFFF0000) >> 16;
				mask = data16 | (data16 << 1);
				draw_word_misaligned_MASKED(draw_buffer, data16, mask, addr, wbit);
				data16 = (data & 0x0000FFFF);
				mask = data16 | (data16 << 1);
				draw_word_misaligned_MASKED(draw_buffer, data16, mask, addr + 2, wbit);
#elif VIDEO_BITS_PER_PIXEL == 4
                data16 = (data & 0xFFFF0000) >> 16;

                if (wbit) {
                    draw_2bit_pixels_unaligned(addr, (data16 & 0xFF00) >> 8);
                    draw_2bit_pixels_unaligned(addr + 2, (data16 & 0xFF));
                    data16 = (data & 0x0000FFFF);
                    draw_2bit_pixels_unaligned(addr + 4, (data16 & 0xFF00) >> 8);
                    draw_2bit_pixels_unaligned(addr + 6, (data16 & 0xFF));
                }
                else {
                    draw_2bit_pixels_aligned(addr, (data16 & 0xFF00) >> 8);
                    draw_2bit_pixels_aligned(addr + 2, (data16 & 0xFF));
                    data16 = (data & 0x0000FFFF);
                    draw_2bit_pixels_aligned(addr + 4, (data16 & 0xFF00) >> 8);
                    draw_2bit_pixels_aligned(addr + 6, (data16 & 0xFF));
                }
#endif

			}
			addr += BUFFER_WIDTH;
			row++;
		}
	}
	else {
		uint16_t data;
		for (yy = y; yy < y + font_info->height; yy++) {
			if (!partly_out || ((x >= GRAPHICS_LEFT) && (x + font_info->width <= GRAPHICS_RIGHT) && (yy >= GRAPHICS_TOP) && (yy <= GRAPHICS_BOTTOM))) {
				data = font_info->data[row];
#if VIDEO_BITS_PER_PIXEL == 2
				mask = data | (data << 1);
				draw_word_misaligned_MASKED(draw_buffer, data, mask, addr, wbit);
#elif VIDEO_BITS_PER_PIXEL == 4
                if (wbit) {
                    draw_2bit_pixels_unaligned(addr, (data & 0xFF00) >> 8);
                    draw_2bit_pixels_unaligned(addr + 2, (data & 0xFF));
                }
                else {
                    draw_2bit_pixels_aligned(addr, (data & 0xFF00) >> 8);
                    draw_2bit_pixels_aligned(addr + 2, (data & 0xFF));
                }
#endif
			}
			addr += BUFFER_WIDTH;
			row++;
		}
	}
}


/**
 * fetch_font_info: Fetch font info structs.
 *
 * @param       font    font id
 */
const struct FontEntry * get_font_info(int font)
{
	if (font >= NUM_FONTS)
		return NULL;
	return fonts[font];
}

/**
 * calc_text_dimensions: Calculate the dimensions of a
 * string in a given font. Supports new lines and
 * carriage returns in text.
 *
 * @param       str                     string to calculate dimensions of
 * @param       font_info       font info structure
 * @param       xs                      horizontal spacing
 * @param       ys                      vertical spacing
 * @param       dim                     return result: struct FontDimensions
 */
void calc_text_dimensions(const char *str, const struct FontEntry *font, int xs, int ys, struct FontDimensions *dim)
{
	int max_length = 0, line_length = 0, lines = 1;

	while (*str != 0) {
		line_length++;
		if (*str == '\n' || *str == '\r') {
			if (line_length > max_length) {
				max_length = line_length;
			}
			line_length = 0;
			lines++;
		}
		str++;
	}
	if (line_length > max_length) {
		max_length = line_length;
	}
	dim->width  = max_length * (font->width + xs);
	dim->height = lines * (font->height + ys);
}

/**
 * draw_string: Draw a string on the screen with certain
 * alignment parameters.
 *
 * @param       str             string to write
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       xs              horizontal spacing
 * @param       ys              horizontal spacing
 * @param       va              vertical align
 * @param       ha              horizontal align
 * @param       font    font
 */
void draw_string(const char *str, int x, int y, int xs, int ys, int va, int ha, int font)
{
	int xx = 0, yy = 0, xx_original = 0;
	const struct FontEntry *font_info;
	struct FontDimensions dim;

	font_info = get_font_info(font);

	calc_text_dimensions(str, font_info, xs, ys, &dim);
	switch (va) {
	case TEXT_VA_TOP:
		yy = y;
		break;
	case TEXT_VA_MIDDLE:
		yy = y - (dim.height / 2) + 1;
		break;
	case TEXT_VA_BOTTOM:
		yy = y - dim.height;
		break;
	}

	switch (ha) {
	case TEXT_HA_LEFT:
		xx = x;
		break;
	case TEXT_HA_CENTER:
		xx = x - (dim.width / 2);
		break;
	case TEXT_HA_RIGHT:
		xx = x - dim.width;
		break;
	}
	// Then write each character.
	xx_original = xx;
	while (*str != 0) {
		if (*str == '\n' || *str == '\r') {
			yy += ys + font_info->height;
			xx  = xx_original;
		} else {
			if (xx >= 0 && xx < GRAPHICS_WIDTH_REAL) {
				draw_char(*str, xx, yy, font_info);
			}
			xx += font_info->width + xs;
		}
		str++;
	}
}

/**
 * Draw a polygon
 *
 */
void draw_polygon(int16_t x, int16_t y, float angle, const point_t * points, uint8_t n_points, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color)
{
	float sin_angle, cos_angle;
	int16_t x1, y1, x2, y2;

	sin_angle    = sinf(angle * (float)(M_PI / 180));
	cos_angle    = cosf(angle * (float)(M_PI / 180));

	x1 = roundf(cos_angle * points[0].x - sin_angle * points[0].y);
	y1 = roundf(sin_angle * points[0].x + cos_angle * points[0].y);
	x2 = 0; // so compiler doesn't give a warning
	y2 = 0;

	for (int i=0; i<n_points-1; i++)
	{
		x2 = roundf(cos_angle * points[i + 1].x - sin_angle * points[i + 1].y);
		y2 = roundf(sin_angle * points[i + 1].x + cos_angle * points[i + 1].y);

		draw_line_outlined(x + x1, y + y1, x + x2, y + y2, 2, 2, line_color, outline_color);
		x1 = x2;
		y1 = y2;
	}

	x1 = roundf(cos_angle * points[0].x - sin_angle * points[0].y);
	y1 = roundf(sin_angle * points[0].x + cos_angle * points[0].y);
	draw_line_outlined(x + x1, y + y1, x + x2, y + y2, 2, 2, line_color, outline_color);

	for (int i=0; i<n_points-1; i++)
	{
		x2 = roundf(cos_angle * points[i + 1].x - sin_angle * points[i + 1].y);
		y2 = roundf(sin_angle * points[i + 1].x + cos_angle * points[i + 1].y);

		draw_line(x + x1, y + y1, x + x2, y + y2, line_color);
		x1 = x2;
		y1 = y2;
	}

	x1 = roundf(cos_angle * points[0].x - sin_angle * points[0].y);
	y1 = roundf(sin_angle * points[0].x + cos_angle * points[0].y);

	draw_line( x + x1, y + y1, x + x2, y + y2, line_color);
}


/**
 * hud_draw_vertical_scale: Draw a vertical scale.
 *
 * @param       v                   value to display as an integer
 * @param       range               range about value to display (+/- range/2 each direction)
 * @param       halign              horizontal alignment: -1 = left, +1 = right.
 * @param       x                   x displacement
 * @param       y                   y displacement
 * @param       height              height of scale
 * @param       mintick_step        how often a minor tick is shown
 * @param       majtick_step        how often a major tick is shown
 * @param       mintick_len         minor tick length
 * @param       majtick_len         major tick length
 * @param       boundtick_len       boundary tick length
 * @param       max_val             maximum expected value (used to compute size of arrow ticker)
 * @param       flags               special flags (see hud.h.)
 */
#define VERTICAL_SCALE_FILLED_NUMBER
#define VSCALE_FONT FONT8X10
void osd_draw_vertical_scale(int v, int range, int halign, int x, int y, int height, int mintick_step, int majtick_step, int mintick_len,
                             int majtick_len, int boundtick_len, int flags)
{
    char temp[15];
    const struct FontEntry *font_info;
    struct FontDimensions dim;
    // Compute the position of the elements.
    int majtick_start = 0, majtick_end = 0, mintick_start = 0, mintick_end = 0, boundtick_start = 0, boundtick_end = 0;

    majtick_start   = x;
    mintick_start   = x;
    boundtick_start = x;
    if (halign == -1) {
        majtick_end     = x + majtick_len;
        mintick_end     = x + mintick_len;
        boundtick_end   = x + boundtick_len;
    } else if (halign == +1) {
        majtick_end     = x - majtick_len;
        mintick_end     = x - mintick_len;
        boundtick_end   = x - boundtick_len;
    }
    // Retrieve width of large font (font #0); from this calculate the x spacing.
    font_info = get_font_info(VSCALE_FONT);
    if (font_info == NULL)
        return;
    int arrow_len      = (font_info->height / 2) + 1;
    int text_x_spacing = (font_info->width / 2);
    int max_text_y     = 0, text_length = 0;
    int small_font_char_width = font_info->width + 1; // +1 for horizontal spacing = 1
    // For -(range / 2) to +(range / 2), draw the scale.
    int range_2 = range / 2; // , height_2 = height / 2;
    int r = 0, rr = 0, rv = 0, ys = 0, style = 0; // calc_ys = 0,
    // Iterate through each step.
    for (r = -range_2; r <= +range_2; r++) {
        style = 0;
        rr    = r + range_2 - v; // normalise range for modulo, subtract value to move ticker tape
        rv    = -rr + range_2; // for number display
        if (flags & HUD_VSCALE_FLAG_NO_NEGATIVE) {
            rr += majtick_step / 2;
        }
        if (rr % majtick_step == 0) {
            style = 1; // major tick
        } else if (rr % mintick_step == 0) {
            style = 2; // minor tick
        } else {
            style = 0;
        }
        if (flags & HUD_VSCALE_FLAG_NO_NEGATIVE && rv < 0) {
            continue;
        }
        if (style) {
            // Calculate y position.
            ys = ((long int)(r * height) / (long int)range) + y;
            // Depending on style, draw a minor or a major tick.
            if (style == 1) {
                draw_hline_outlined(majtick_start, majtick_end, ys, 2, 2, OSD_COLOR_WHITE, OSD_COLOR_BLACK);
                memset(temp, ' ', 10);
                tfp_sprintf(temp, "%d", rv);
                text_length = (strlen(temp) + 1) * small_font_char_width; // add 1 for margin
                if (text_length > max_text_y) {
                    max_text_y = text_length;
                }
                if (halign == -1) {
                    draw_string(temp, majtick_end + text_x_spacing + 1, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, FONT_OUTLINED8X8);
                } else {
                    draw_string(temp, majtick_end - text_x_spacing + 1, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_RIGHT, FONT_OUTLINED8X8);
                }
            } else if (style == 2) {
                draw_hline_outlined(mintick_start, mintick_end, ys, 2, 2, OSD_COLOR_WHITE, OSD_COLOR_BLACK);
            }
        }
    }
    // Generate the string for the value, as well as calculating its dimensions.
    memset(temp, ' ', 10);
    // my_itoa(v, temp);
    tfp_sprintf(temp, "%02d", v);
    // TODO: add auto-sizing.
    calc_text_dimensions(temp, font_info, 1, 0, &dim);
    int xx = 0, i = 0;
    if (halign == -1) {
        xx = majtick_end + text_x_spacing;
    } else {
        xx = majtick_end - text_x_spacing;
    }
    y++;
    uint8_t width =  dim.width + 4;
    // Draw an arrow from the number to the point.
    for (i = 0; i < arrow_len; i++) {
        if (halign == -1) {
            draw_pixel(xx - arrow_len + i, y - i - 1, OSD_COLOR_WHITE);
            draw_pixel(xx - arrow_len + i, y + i - 1, OSD_COLOR_WHITE);
#ifdef VERTICAL_SCALE_FILLED_NUMBER
            draw_hline(xx + width - 1, xx - arrow_len + i + 1, y - i - 1, OSD_COLOR_BLACK);
            draw_hline(xx + width - 1, xx - arrow_len + i + 1, y + i - 1, OSD_COLOR_BLACK);
#else
            draw_hline(xx + width - 1, xx - arrow_len + i + 1, y - i - 1, OSD_COLOR_TRANSP);
            draw_hline(xx + width - 1, xx - arrow_len + i + 1, y + i - 1, OSD_COLOR_TRANSP);
#endif
        } else {
            draw_pixel(xx + arrow_len - i, y - i - 1, OSD_COLOR_WHITE);
            draw_pixel(xx + arrow_len - i, y + i - 1, OSD_COLOR_WHITE);
#ifdef VERTICAL_SCALE_FILLED_NUMBER
            draw_hline(xx - width - 1, xx + arrow_len - i - 1, y - i - 1, OSD_COLOR_BLACK);
            draw_hline(xx - width - 1, xx + arrow_len - i - 1, y + i - 1, OSD_COLOR_BLACK);
#else
            draw_hline(xx - width - 1, xx + arrow_len - i - 1, y - i - 1, OSD_COLOR_TRANSP);
            draw_hline(xx - width - 1, xx + arrow_len - i - 1, y + i - 1, OSD_COLOR_TRANSP);
#endif
        }
    }
    if (halign == -1) {
        draw_hline(xx, xx + width -1, y - arrow_len, OSD_COLOR_WHITE);
        draw_hline(xx, xx + width - 1, y + arrow_len - 2, OSD_COLOR_WHITE);
        draw_vline(xx + width - 1, y - arrow_len, y + arrow_len - 2, OSD_COLOR_WHITE);
    } else {
        draw_hline(xx, xx - width - 1, y - arrow_len, OSD_COLOR_WHITE);
        draw_hline(xx, xx - width - 1, y + arrow_len - 2, OSD_COLOR_WHITE);
        draw_vline(xx - width - 1, y - arrow_len, y + arrow_len - 2, OSD_COLOR_WHITE);
    }
    // Draw the text.
    if (halign == -1) {
        draw_string(temp, xx + width / 2, y - 1, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, VSCALE_FONT);
    } else {
        draw_string(temp, xx - width / 2, y - 1, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, VSCALE_FONT);
    }

    y--;
    draw_hline_outlined(boundtick_start, boundtick_end, y + (height / 2), 2, 2, OSD_COLOR_WHITE, OSD_COLOR_BLACK);
    draw_hline_outlined(boundtick_start, boundtick_end, y - (height / 2), 2, 2, OSD_COLOR_WHITE, OSD_COLOR_BLACK);
}

#endif
