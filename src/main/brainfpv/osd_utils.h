/**
 ******************************************************************************
 * @addtogroup Tau Labs Modules
 * @{
 * @addtogroup OnScreenDisplay OSD Module
 * @brief OSD Utility Functions
 * @{
 *
 * @file       osd_utils.h
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
 */

#ifndef OSDUTILS_H
#define OSDUTILS_H

#include "fonts.h"
#include "images.h"
#include "video.h"

// Size of an array (num items.)
#define SIZEOF_ARRAY(x) (sizeof(x) / sizeof((x)[0]))

#define HUD_VSCALE_FLAG_CLEAR       1
#define HUD_VSCALE_FLAG_NO_NEGATIVE 2

#define PIXELS_PER_BYTE (8 / VIDEO_BITS_PER_PIXEL)

#if !defined(VIDEO_BITS_PER_PIXEL)
#define VIDEO_BITS_PER_PIXEL 2
#endif

#if VIDEO_BITS_PER_PIXEL == 2
// 2 bit mode: Leftmost pixel in byte is MSB
// MSB  P[P0.1, P0.0 | P1.1, P1.0 | P2.1, P2.0 | P3.1, P3.0] LSB


#define CALC_BIT_IN_WORD(x) (2 * ((x) & 3))
#define CALC_BITSHIFT_WORD(x) (2 * ((x) & 3))
#define CALC_BIT_MASK(x) (3 << (6 - CALC_BITSHIFT_WORD(x)))
//#define PACK_BITS(mask, level) (level << 7 | mask << 6 | level << 5 | mask << 4 | level << 3 | mask << 2 | level << 1 | mask)
#define PACK_BITS(color) ((color) << 6 | (color) << 4 | (color) << 2 | (color))
#define CALC_BIT0_IN_WORD(x)  (2 * ((x) & 3))
#define CALC_BIT1_IN_WORD(x)  (2 * ((x) & 3) + 1)
// Horizontal line calculations.
// Edge cases.
#define COMPUTE_HLINE_EDGE_L_MASK(b)      ((1 << (7 - (b))) - 1)
#define COMPUTE_HLINE_EDGE_R_MASK(b)      (~((1 << (6 - (b))) - 1))

#elif VIDEO_BITS_PER_PIXEL == 4
// 4 bit mode: Leftmost pixel in byte is LSB
// MSB  P[P1.3, P1.2  P1.1, P1.0 | P0.3, P0.2 P0.1, P0.0] LSB

#define CALC_BIT_IN_WORD(x) (4 * ((x) & 0x1))

#define CALC_BIT_MASK(x) ((x) & 0x1 ? 0xF0 : 0x0F)

#define PACK_BITS(color) ((color) << 4 | (color))

#define CALC_BIT0_IN_WORD(x)  ((x) & 0x1 ? 4 : 0)
#define CALC_BIT1_IN_WORD(x)  ((x) & 0x1 ? 7 : 3)

// Horizontal line calculations.
// Edge cases.
#define COMPUTE_HLINE_EDGE_L_MASK(b)    ((b) <= 3 ? 0xFF : 0xF0)
#define COMPUTE_HLINE_EDGE_R_MASK(b)    ((b) >= 3 ? 0xFF : 0x0F)
#else
#error "Only 2 or 4 bits per pixel are currently supported"
#endif

// Macros for computing addresses and bit positions.
#define CALC_BUFF_ADDR(x, y) (((x) / PIXELS_PER_BYTE) + ((y) * BUFFER_WIDTH))
#define DEBUG_DELAY
// Macro for writing a word with a mode (NAND = clear, OR = set, XOR = toggle)
// at a given position
#define draw_WORD_MODE(buff, addr, mask, mode) \
	switch (mode) { \
	case 0: buff[addr] &= ~mask; break; \
	case 1: buff[addr] |= mask; break; \
	case 2: buff[addr] ^= mask; break; }

#define draw_WORD_NAND(buff, addr, mask) { buff[addr] &= ~mask; DEBUG_DELAY; }
#define draw_WORD_OR(buff, addr, mask)   { buff[addr] |= mask; DEBUG_DELAY; }
#define draw_WORD_XOR(buff, addr, mask)  { buff[addr] ^= mask; DEBUG_DELAY; }
#define draw_WORD(buff, addr, mask, value)  { buff[addr] = (buff[addr] & ~mask) | (value & mask);}


// This computes an island mask.
#define COMPUTE_HLINE_ISLAND_MASK(b0, b1) (COMPUTE_HLINE_EDGE_L_MASK(b0) ^ COMPUTE_HLINE_EDGE_L_MASK(b1));

// Macro for initializing stroke/fill modes. Add new modes here
// if necessary.
#define SETUP_STROKE_FILL(stroke, fill, mode) \
	stroke = 0; fill = 0; \
	if (mode == 0) { stroke = 0; fill = 1; } \
	if (mode == 1) { stroke = 1; fill = 0; } \

// Line endcaps (for horizontal and vertical lines.)
#define ENDCAP_NONE  0
#define ENDCAP_ROUND 1
#define ENDCAP_FLAT  2

#define DRAW_ENDCAP_HLINE(e, x, y, lc, olc) \
	if ((e) == ENDCAP_ROUND) /* single pixel endcap */ \
{ draw_pixel(x, y, lc); } \
	else if ((e) == ENDCAP_FLAT) /* flat endcap: FIXME, quicker to draw a vertical line(?) */ \
{ draw_pixel(x, y - 1, lc); draw_pixel(x, y, lc); draw_pixel(x, y + 1, lc); }

#define DRAW_ENDCAP_VLINE(e, x, y, lc, olc) \
	if ((e) == ENDCAP_ROUND) /* single pixel endcap */ \
{ draw_pixel(x, y, lc); } \
	else if ((e) == ENDCAP_FLAT) /* flat endcap: FIXME, quicker to draw a horizontal line(?) */ \
{ draw_pixel(x - 1, y, lc); draw_pixel(x, y, lc); draw_pixel(x + 1, y, lc); }

// Macros for writing pixels in a midpoint circle algorithm.
#define CIRCLE_PLOT_8(buff, cx, cy, x, y, mode) \
	CIRCLE_PLOT_4(buff, cx, cy, x, y, mode); \
	if ((x) != (y)) { CIRCLE_PLOT_4(buff, cx, cy, y, x, mode); }

#define CIRCLE_PLOT_4(buff, cx, cy, x, y, mode) \
	draw_pixel(buff, (cx) + (x), (cy) + (y), mode); \
	draw_pixel(buff, (cx) - (x), (cy) + (y), mode); \
	draw_pixel(buff, (cx) + (x), (cy) - (y), mode); \
	draw_pixel(buff, (cx) - (x), (cy) - (y), mode);

// Font flags.
#define FONT_BOLD      1               // bold text (no outline)
#define FONT_INVERT    2               // invert: border white, inside black
// Text alignments.
#define TEXT_VA_TOP    0
#define TEXT_VA_MIDDLE 1
#define TEXT_VA_BOTTOM 2
#define TEXT_HA_LEFT   0
#define TEXT_HA_CENTER 1
#define TEXT_HA_RIGHT  2

// Max/Min macros
#define MAX3(a, b, c)                MAX(a, MAX(b, c))
#define MIN3(a, b, c)                MIN(a, MIN(b, c))
#define LIMIT(x, l, h)               MAX(l, MIN(x, h))

// Check if coordinates are valid. If not, return. Assumes signed coordinates for working correct also with values lesser than 0.
#define CHECK_COORDS(x, y)           if (x < GRAPHICS_LEFT || x > GRAPHICS_RIGHT || y < GRAPHICS_TOP || y > GRAPHICS_BOTTOM) { return; }
#define CHECK_COORDS_UNSIGNED(x, y)  if (x > GRAPHICS_RIGHT || y > GRAPHICS_BOTTOM) { return; }

#define CHECK_COORD_X(x)             if (x < GRAPHICS_LEFT || x > GRAPHICS_RIGHT) { return; }
#define CHECK_COORD_Y(y)             if (y < GRAPHICS_TOP  || y > GRAPHICS_BOTTOM) { return; }

// Clip coordinates out of range. Assumes signed coordinates for working correct also with values lesser than 0.
#define CLIP_COORDS(x, y)            { CLIP_COORD_X(x); CLIP_COORD_Y(y); }
#define CLIP_COORD_X(x)              { x = x < GRAPHICS_LEFT ? GRAPHICS_LEFT : x > GRAPHICS_RIGHT ? GRAPHICS_RIGHT : x; }
#define CLIP_COORD_Y(y)              { y = y < GRAPHICS_TOP ? GRAPHICS_TOP : y > GRAPHICS_BOTTOM ? GRAPHICS_BOTTOM : y; }

// Macro to swap two variables using XOR swap.
#define SWAP(a, b)                   { a ^= b; b ^= a; a ^= b; }


// Text dimension structures.
struct FontDimensions {
	int width, height;
};

// Structure for a point
typedef struct {
	int16_t x;
	int16_t y;
} point_t;

// OSD Colors
typedef enum {
    OSD_COLOR_TRANSP = 0x00,
    OSD_COLOR_BLACK = 0x01,
    OSD_COLOR_GRAY2 = 0x02,
    OSD_COLOR_WHITE = 0x03,
#if(VIDEO_BITS_PER_PIXEL == 4)
    OSD_COLOR_GRAY1 = 0x04,
    OSD_COLOR_GRAY3 = 0x05,
    OSD_COLOR_GRAY4 = 0x06,
    OSD_COLOR_GRAY5 = 0x07,
    OSD_COLOR_GRAY6 = 0x08,
    OSD_COLOR_STB = 0x09,
    OSD_COLOR_STW = 0x0A,
    OSD_COLOR_WHITE2 = 0x0B,
    OSD_COLOR_WHITE3 = 0x0C,
#endif
    OSD_NUM_COLORS
} OSDOSD_COLOR_t;

#if VIDEO_BITS_PER_PIXEL == 4
void set_text_color(OSDOSD_COLOR_t main_color, OSDOSD_COLOR_t outline_color);
#endif

void clearGraphics(void);
void draw_image(uint16_t x, uint16_t y, const struct Image * image);

void draw_pixel(int x, int y, OSDOSD_COLOR_t color);
void draw_hline(int x0, int x1, int y, OSDOSD_COLOR_t color);
void draw_hline_outlined(int x0, int x1, int y, int endcap0, int endcap1, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);

void draw_vline(int x, int y0, int y1, OSDOSD_COLOR_t color);
void draw_vline_outlined(int x, int y0, int y1, int endcap0, int endcap1, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);

void draw_filled_rectangle(int x, int y, int width, int height, OSDOSD_COLOR_t color);
void draw_rectangle_outlined(int x, int y, int width, int height, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);
void draw_circle_outlined(int cx, int cy, int r, int dashp, int bmode, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);

void draw_line(int x0, int y0, int x1, int y1, OSDOSD_COLOR_t color);
void draw_line_outlined(int x0, int y0, int x1, int y1,
						 __attribute__((unused)) int endcap0, __attribute__((unused)) int endcap1,
						 OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);
void draw_line_outlined_dashed(int x0, int y0, int x1, int y1,
								__attribute__((unused)) int endcap0, __attribute__((unused)) int endcap1,
								OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color, int dots);
const struct FontEntry* get_font_info(int font);
void calc_text_dimensions(const char *str, const struct FontEntry *font, int xs, int ys, struct FontDimensions *dim);
void draw_string(const char *str, int x, int y, int xs, int ys, int va, int ha, int font);
void draw_polygon(int16_t x, int16_t y, float angle, const point_t * points, uint8_t n_points, OSDOSD_COLOR_t line_color, OSDOSD_COLOR_t outline_color);

void osd_draw_vertical_scale(int v, int range, int halign, int x, int y, int height, int mintick_step, int majtick_step, int mintick_len,
                             int majtick_len, int boundtick_len, int flags);
#endif /* OSDUTILS_H */


