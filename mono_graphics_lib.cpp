/**
 * @file mono_graphics_lib.cpp
 * @brief This class is a C++ Raspberry Pi Pico port of the LibDriver 
 * C SSD1306 driver code found here: https://github.com/hepingood/ssd1306
 * 
 * Copyright (c) 2022 rppicomidi
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifdef NDEBUG
// Need to do this here for release builds or no CLI commands will be added
// All build variants except DEBUG define NDEBUG, which makes assert() macro generate
// no code at all, which prevents some required code in here from executing
#undef NDEBUG
#endif
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "mono_graphics_lib.h"
#include "ssd1306/src/driver_ssd1306_font.h"
#include "RPi-Pico-SSD1306-library/font.hpp"

rppicomidi::Mono_graphics::Mono_graphics(rppicomidi::Ssd1306* display_, Display_rotation initial_rotation_) :
    display{display_}, clip_rect{0,0,0,0},
    font8{8, 5, font_8x5, sizeof(font_8x5),' ','~',false},
    font12{12, 6, gsc_ssd1306_ascii_1206, sizeof(gsc_ssd1306_ascii_1206)},
    font16{16, 8, gsc_ssd1306_ascii_1608, sizeof(gsc_ssd1306_ascii_1608)},
    font24{24, 12, gsc_ssd1306_ascii_2412, sizeof(gsc_ssd1306_ascii_2412)},
    needs_render{0,0,0,0}
{
    canvas_nbytes = display->get_minimum_canvas_size();
    canvas = new uint8_t[canvas_nbytes];
    assert(canvas);
    memset(canvas, 0, canvas_nbytes);
    assert(display->init(initial_rotation_));
	set_clip_rect(0, 0, display->get_screen_width(), display->get_screen_height());
}

rppicomidi::Mono_graphics::~Mono_graphics()
{
    if (canvas)
        delete[] canvas;
}

void rppicomidi::Mono_graphics::draw_dot(uint8_t x, uint8_t y, Pixel_state fg_color, bool update)
{
	// only draw the dot if x and y are within the clipping rectangle
	if (x >= clip_rect.x_upper_left && x < clip_rect.width &&
		y >= clip_rect.y_upper_left && y < clip_rect.height) {
    	display->set_pixel_on_canvas(canvas, canvas_nbytes, x, y, fg_color);
	}
    if (update) {
        update_needs_render(x, y, x, y);
    }
}

void rppicomidi::Mono_graphics::draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, Pixel_state fg_color, bool update)
{
	// Uses Bresenham's line algorithm as described in Wikipedia
	int dx = abs(x1-x0);
	int sx = (x0<x1) ? 1 : -1;
	int dy = -abs(y1-y0);
	int sy = (y0<y1) ? 1 : -1;
	int err = dx+dy; // error value e_xy
	while(true) {
		draw_dot(x0, y0, fg_color);
		if (x0 == x1 && y0 == y1) {
			break; //done
		}
		int e2 = 2*err;
		if (e2 >= dy) { //e_exy+e_x > 0
			err += dy;
			x0 += sx;
		}
		if (e2 < dx) { // e_xy+e_y < 0
			err += dx;
			y0 += sy;
		}
	}
    if (update) {
        update_needs_render(x0, y0, x1, y1);
    }
}

void rppicomidi::Mono_graphics::draw_rectangle(uint8_t x0, uint8_t y0, uint8_t width, uint8_t height, Pixel_state fg_color, Pixel_state bg_color)
{
    update_needs_render(x0, y0, x0+width-1, y0+height-1);
    uint8_t x1 = x0 + width - 1;
    uint8_t y1 = y0 + height - 1;
    draw_line(x0,y0, x1, y0, fg_color); // top of the rectangle
    draw_line(x0,y1, x1, y1, fg_color); // bottom of the rectangle
    draw_line(x0,y0, x0, y1, fg_color); // left edge
    draw_line(x1,y0, x1, y1, fg_color); // right edge
    if (bg_color != Pixel_state::PIXEL_TRANSPARENT) {
	    // fill the rectangle using horizontal lines
	    ++x0;
	    --x1;
	    if (x0 <= x1) {
            ++y0;
            for (uint8_t y = y0; y < y1; y++) {
                draw_line(x0,y, x1, y, bg_color);
            }
        }
    }
}

void rppicomidi::Mono_graphics::draw_character(const Mono_mono_font& font, uint8_t x, uint8_t y, char chr,  Pixel_state fg_color, Pixel_state bg_color)
{
	if (chr > font.last_char || chr < font.first_char) {
        printf("warning illegal character received 0x%02x\r\n", chr);
        // not in this character set. Substitute a character that is
        chr = '?';
        if (chr > font.last_char || chr < font.first_char) {
            chr = ' ';
            if (chr > font.last_char || chr < font.first_char) {
                chr = font.first_char;
            }
        }
    }

	uint8_t nrows = font.height;
	uint8_t ncols = font.width;
	uint8_t nbytes_per_col = (nrows / 8) + (((nrows % 8)==0) ? 0:1);
	const uint8_t* pixels = font.font_bytes; // pixel array for the font starts after the nrows & ncols bytes
	// pixel array is stored columnwise in ASCII character order
	size_t idx = (size_t)(chr - font.first_char) * ncols * nbytes_per_col;	// the index of the first byte of pixel data for the character
    assert(idx < font.num_font_bytes);
	for (uint8_t col=0; col<ncols; col++) {
		uint8_t rowbits = pixels[idx++];
		uint8_t mask = font.msb_is_top ? 0x80 : 0x01;
		uint8_t xpixel = x+col;
		uint8_t ypixel = y;
		uint8_t column_byte = 0;
		for (uint8_t row = 0; row < nrows; row++) {
			draw_dot(xpixel, ypixel, (rowbits & mask)!= 0 ? fg_color : bg_color);
            if (font.msb_is_top)
			    mask >>= 1;
            else
                mask <<= 1;
			++ypixel;
			if (mask == 0) {
				// then displayed all 8 pixels for the byte in the font; 
				// If more bytes in this column, init mask again get &
				// get the next 8 pixels to display
				++column_byte;
				if (column_byte < nbytes_per_col) {
					mask = font.msb_is_top ? 0x80 : 0x01;
					rowbits = pixels[idx++];
				}
			}
		}
    }
    update_needs_render(x, y, x+font.width-1, y+font.height-1);

}

void rppicomidi::Mono_graphics::circle_points(int cx, int cy, int x, int y, Pixel_state fg_color, Pixel_state fill_color)
{
	if (x == 0) {
		draw_dot(cx, cy + y, fg_color);
		draw_dot(cx, cy - y, fg_color);
		draw_dot(cx + y, cy, fg_color);
		draw_dot(cx - y, cy, fg_color);
		draw_line(cx-y+1,cy, cx+y-1, cy, fill_color);
	}
	else if (x == y) {
		draw_dot(cx + x, cy + y, fg_color);
		draw_dot(cx - x, cy + y, fg_color);
		draw_line(cx-x+1,cy+y, cx+x-1, cy+y, fill_color);
		draw_dot(cx + x, cy - y, fg_color);
		draw_dot(cx - x, cy - y, fg_color);
		draw_line(cx-x+1,cy-y, cx+x-1, cy-y, fill_color);
	}
	else if (x < y) {
		draw_dot(cx + x, cy + y, fg_color);
		draw_dot(cx - x, cy + y, fg_color);
		draw_line(cx-x+1,cy+y, cx+x-1, cy+y, fill_color);
		draw_dot(cx + x, cy - y, fg_color);
		draw_dot(cx - x, cy - y, fg_color);
		draw_line(cx-x+1,cy-y, cx+x-1, cy-y, fill_color);
		draw_dot(cx + y, cy + x, fg_color);
		draw_dot(cx - y, cy + x, fg_color);
		draw_line(cx-y+1,cy+x, cx+y-1, cy+x, fill_color);
		draw_dot(cx + y, cy - x, fg_color);
		draw_dot(cx - y, cy - x, fg_color);
		draw_line(cx-y+1,cy-x, cx+y-1, cy-x, fill_color);
	}
}

void rppicomidi::Mono_graphics::draw_circle(uint8_t x0, uint8_t y0, uint8_t diameter, Pixel_state fg_color, Pixel_state fill_color)
{
	int radius = diameter / 2;
	int x_center = x0 + radius;
	int y_center = y0 + radius;
	draw_centered_circle(x_center, y_center, radius, fg_color, fill_color);
}

void rppicomidi::Mono_graphics::draw_centered_circle(uint8_t x_center, uint8_t y_center, uint8_t radius, Pixel_state fg_color, Pixel_state fill_color)
{
    update_needs_render(x_center-radius, y_center-radius, x_center+radius, y_center+radius);
	int x = 0;
	int y = radius;
	int p = (5 - radius*4)/4;

	circle_points(x_center, y_center, x, y, fg_color, fill_color);
	while (x < y) {
		x++;
		if (p < 0) {
			p += 2*x+1;
		} else {
			y--;
			p += 2*(x-y)+1;
		}
		circle_points(x_center, y_center, x, y, fg_color, fill_color);
	}
	if (fill_color == Pixel_state::PIXEL_TRANSPARENT) {
		radius = 0;
	}
	else {
		--radius;
	}
}

bool rppicomidi::Mono_graphics::task()
{
    return display->task();
}

void rppicomidi::Mono_graphics::update_needs_render(uint8_t x_min, uint8_t y_min, uint8_t x_max, uint8_t y_max)
{
    if (needs_render.width == 0 || needs_render.height == 0) {
        needs_render.x_upper_left = x_min;
        needs_render.y_upper_left = y_min;
        needs_render.width = x_max - x_min + 1;
        needs_render.height = y_max - y_min + 1;
    }
    else {
        uint8_t x_lower_right = needs_render.x_upper_left + needs_render.width - 1;
        uint8_t y_lower_right = needs_render.y_upper_left + needs_render.height - 1;
        if (x_min < needs_render.x_upper_left) {
            needs_render.x_upper_left = x_min;
        }
        if (y_min < needs_render.y_upper_left) {
            needs_render.y_upper_left = y_min;
        }
        if (x_max > x_lower_right) {
            needs_render.width = x_max - needs_render.x_upper_left + 1;
        }
        else {
            needs_render.width = x_lower_right - needs_render.x_upper_left + 1;
        }
        if (y_max > y_lower_right) {
            needs_render.height = y_max - needs_render.y_upper_left + 1;
        }
        else {
            needs_render.height = y_lower_right - needs_render.y_upper_left + 1;
        }
    }
}

void rppicomidi::Mono_graphics::render_non_blocking(void (*callback)(uint8_t id), uint8_t id)
{
    assert(display->render_non_blocking(canvas, needs_render.x_upper_left, needs_render.y_upper_left,
        needs_render.width,needs_render.height, callback, id));
    needs_render.height = 0;
    needs_render.width = 0;
}

void rppicomidi::Mono_graphics::center_string_on_two_lines(const Mono_mono_font& font, const char* str, uint8_t linenum)
{
    uint8_t max_line_length = get_screen_width() / font.width;
    uint8_t y = linenum * font.height;
    auto textlen = strlen(str);
    // clear the area to draw the lines of text
    draw_rectangle(0, y, get_screen_width(), 2*font.height,Pixel_state::PIXEL_ZERO, Pixel_state::PIXEL_ZERO);

    if (textlen <= max_line_length) {
        // Center the Produce String on the 2nd line of the screen
        center_string(font, str, y+font.height);
    }
    else {
        // Break the device_label string into two lines.
        char line1[max_line_length+1];
        char line2[max_line_length+1];
        // Copy as much of the text onto the first line as possible and copy the remaining
        // text to the next line
        strncpy(line1, str, max_line_length);
        line1[max_line_length] = '\0';
        strncpy(line2, str+max_line_length, max_line_length);
        line2[max_line_length] = '\0';

        // See if we can break the text at a space
        char* ptr = strrchr(line1, ' ');
        bool center = false;
        if (ptr != nullptr) {
            // Found the last space
            const char* remaining_text = str + (ptr - line1 + 1);
            if (strlen(remaining_text) <= max_line_length) {
                // Terminate line 1 at the last space
                *ptr = '\0';
                // copy the remaining text to line 2
                strncpy(line2, remaining_text, max_line_length);
                line2[max_line_length] = '\0';
                center = true; // center both lines of text for a cleaner look
            }
        }
        if (center) {
            center_string(font, line1, y);
            center_string(font, line2, y+font.height);
        }
        else {
            draw_string(font, 0, y, line1, strlen(line1), Pixel_state::PIXEL_ONE, Pixel_state::PIXEL_ZERO);
            draw_string(font, 0, y+font.height, line2, strlen(line2), Pixel_state::PIXEL_ONE, Pixel_state::PIXEL_ZERO);
        }
    }
}

// See https://en.wikipedia.org/wiki/BMP_file_format
// https://cdn.hackaday.io/files/274271173436768/Simplified%20Windows%20BMP%20Bitmap%20File%20Format%20Specification.htm
const uint8_t* rppicomidi::Mono_graphics::make_bmp_file_data()
{
    uint32_t filesize = get_bmp_file_data_size();
    uint8_t* bmp = new uint8_t[filesize];
    memset(bmp, 0, filesize);
    bmp[0] = 'B';
    bmp[1] = 'M';
    bmp[2] = filesize & 0xff;           // number of bytes in file
    bmp[3] = (filesize >> 8) & 0xff;
    bmp[4] = (filesize >> 16) & 0xff;
    bmp[5] = (filesize >> 24) & 0xff;
    // reserved 6-9
    bmp[10] = 62; // offset where pixel data starts, LSB. Upper bytes are 11-13 are 0
    bmp[14] = 40; // size of DIB header. Upper bytes 15-17 are 0
    bmp[18] = get_screen_width(); // number of horizontal pixels, upper byts 19-21 are 0
    int32_t neg_height = -get_screen_height();
    bmp[22] = neg_height & 0xFF; // number of vertical pixels, negative so pixels stored top to bottom
    bmp[23] = (neg_height >> 8) & 0xff;
    bmp[24] = (neg_height >> 16) & 0xff;
    bmp[25] = (neg_height >> 24) & 0xff;
    bmp[26] = 1; // number color planes, upper byte 27 is 0
    bmp[28] = 1; // bits per pixel, upper byte 29 is 0
    bmp[30] = 0; // compression = uncompressed, upper bytes 31-33 are also 0
    bmp[34] = canvas_nbytes & 0xff; // image size
    bmp[35] = (canvas_nbytes >> 8) & 0xff;
    bmp[36] = (canvas_nbytes >> 16) & 0xff;
    bmp[37] = (canvas_nbytes >> 24) & 0xff;
    bmp[38] = 0; // horizontal pixels per meter upper bytes 29-41 are 0 too
    bmp[42] = 0; // vertical pixels per meter uper byts 43-45 are 0 too
    bmp[46] = 0; // default number of colors in pallete = 2^1 = 2; upper bytes 47-49 are 0
    bmp[50] = 0; // number of important colors used, ignored, upper bytes 51-53 are 0
    bmp[54] = 0x00; // black, 0 pixel value in colors
    bmp[55] = 0;
    bmp[56] = 0;
    bmp[57] = 0;
    bmp[58] = 0xff; // white, 1 pixel value in colors
    bmp[59] = 0xff;
    bmp[60] = 0xff;
    bmp[61] = 0x00;


    // transform pixels in the canvas, which are stored 8 pixels per byte per column,
    // into BMP bitmaps which are stored in groups of 8 pixel rasters across a row.
    // Also canvas pixels are stored with an origin on the upper left. BMP pixels
    // are stored with origin on the upper left here because image height is negative
    // Transform the canvas into the bmp array starting at offset 64
    uint8_t* bmp_ptr = bmp+62;
    for (uint8_t y=0; y < get_screen_height(); y++) {
        for (uint8_t x_byte=0; x_byte < get_screen_width()/8; x_byte++) {
            uint8_t bits8 = 0;
            uint8_t mask = 0x80;
            for (uint8_t bit=0; bit < 8; bit++) {
                uint8_t x = x_byte*8 + bit;
                if (display->get_pixel_on_canvas(canvas, canvas_nbytes, x, y)) {
                    bits8 |= mask;
                }
                mask >>= 1;
            }
            *bmp_ptr++ = bits8;
        }
    }
    return bmp;
}
