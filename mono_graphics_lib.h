/**
 * @file mono_graphics_lib.h
 * @brief This class implements drawing monochrome graphics.
 * 
 * This class is a C++ Raspberry Pi Pico port of the LibDriver 
 * C SSD1306 driver code found here: https://github.com/hepingood/ssd1306
 * MIT License for those files is in the project directories
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
#pragma once
#include <cstdint>
#include <cstring>
#include "ssd1306.h"
#include "assert.h"
namespace rppicomidi {

/**
 * @brief define a rectangle by its upper left and lower right coordinates
 * 
 */
class Rectangle {
public:
    Rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
        x_upper_left = x;
        y_upper_left = y;
        width = w;
        height = h;
    }
    uint8_t x_upper_left, y_upper_left, width, height;
private:
    Rectangle()=delete;
};

/**
 * @brief Monospace Monochrome font
 *
 * Mono_mono_font is designed to display fixed width printable ASCII
 * characters. Each letter is represented by a bitmap image stored
 * in several consecutive bytes. The array font_bytes contains
 * the bitmaps required to render all ASCII characters from first_char
 * to last_char.
 *
 * Character bitmaps are stored one bit per pixel packed in bytes. A
 * byte can represent up to 8 pixels. The first pixel in a byte represents
 * the lowest vertical raster coordinate. After enough pixels for rendering
 * one vertical column of pixels are packed into successive bytes,
 * 
 */
class Mono_mono_font
{
public:
    Mono_mono_font(uint8_t height_, uint8_t width_, const uint8_t* font_bytes_, size_t num_font_bytes_, 
        char first_char_=' ', char last_char_='~', bool msb_is_top_=true) : height{height_}, 
        width{width_}, font_bytes{font_bytes_}, num_font_bytes{num_font_bytes_},
        first_char{first_char_}, last_char{last_char_}, msb_is_top{msb_is_top_}
    {}

    uint8_t height;         //!< the font height in pixels
    uint8_t width;          //!< the font width in pixels
    const uint8_t* font_bytes;    //!< font byte array formatted as described above
    const size_t num_font_bytes;  //!< number of bytes in the font_bytes array
    const char first_char;        //!< the first character in the font (often ' ')
    const char last_char;         //!< the last character in the font (often '~')
    bool msb_is_top;        //!< see description above
};

class Mono_graphics 
{
public:
    /**
     * @brief Construct a new Mono_graphics object
     * 
     * @param display the interface to the display
     */
    Mono_graphics(Ssd1306* display_, Display_rotation initial_rotation_);

    ~Mono_graphics();
    /**
     * @brief Set the clipping rectangle to the rectangle with the upper
     * left and lower right coordinates
     */
    inline void set_clip_rect(uint8_t x_upper_left, uint8_t y_upper_left, uint8_t width, uint8_t height)
    {
        assert(x_upper_left < display->get_screen_width());
        assert(width <= display->get_screen_width());
        assert(y_upper_left < display->get_screen_height());
        assert(height <= display->get_screen_height());
        clip_rect.x_upper_left = x_upper_left;
	    clip_rect.y_upper_left = y_upper_left;
	    clip_rect.width = width;
	    clip_rect.height = height;
    }

    inline void set_clip_rect(const Rectangle& rect) {
        set_clip_rect(rect.x_upper_left, rect.y_upper_left, rect.width, rect.height);
    }

    inline const Rectangle& get_clip_rect() const { return clip_rect; }

    /**
     * @brief Get the screen height object
     * 
     * @return the screen height
     */
    inline uint8_t get_screen_height() {
        return display->get_screen_height();
    }

    /**
     * @brief Get the screen width object
     * 
     * @return the screen width
     */
    inline uint8_t get_screen_width() {
        return display->get_screen_width();
    }

    /**
     * @brief set every byte in the canvas buffer to 0
     * 
     */
    inline void clear_canvas() {
        memset(canvas, 0, canvas_nbytes);
        update_needs_render(0, 0, get_screen_width()-1, get_screen_height()-1);
    }

    /**
     * @brief draw a single dot on the screen at raster coordinates (x,y)
     * using the writing mode described by fg_color.
     *
     * @param x
     * @param y
     * @param fg_color
     * @param update is true to update the needs_render rectangle
     */
    void draw_dot(uint8_t x, uint8_t y, Pixel_state fg_color, bool update=false);

    /**
     * @brief draw a line from (x0,y0) to (x1,y1)
     * using the writing mode described by fg_color.
     * 
     */
    void draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, Pixel_state fg_color, bool update=false);

    /**
     * @brief draw an empty rectangle width x height with upper left corner at (x0,y0)
     * using the writing mode described by fg_color.
     * 
     * To draw a hollow rectangle around existing drawing use bg_color=Pixel_state::PIXEL_TRANSPARENT.
     * To draw a filled rectangle over existing drawing use bg_color=Pixel_state::PIXEL_ONE or
     * Pixel_state::PIXEL_ZERO
     */
    void draw_rectangle(uint8_t x0, uint8_t y0, uint8_t width, uint8_t height, Pixel_state fg_color, Pixel_state bg_color);

    inline void draw_rectangle(const Rectangle& rect, Pixel_state fg_color, Pixel_state bg_color) {
        draw_rectangle(rect.x_upper_left, rect.y_upper_left, rect.width, rect.height, fg_color, bg_color);
    }

    /**
     * @brief draw a circle with upper left corner at (x0,y0) and the given diameter
     * using the writing mode described by fg_color and fill_color.
     * 
     * This function translates coordinates and calls draw_centered_circle(). See that
     * function for license comments.
     * 
     * @param x0 the horizontal coordiate of the upper left corner of the circle's square bounding box
     * @param y0 the vertical coordiate of the upper left corner of the circle's square bounding box
     * @param diameter is the circle diameter. The circle radius has to be an integer
     * number of pixels, so if diameter is odd, the radius will be smaller than expected.
     * @param fg_color is how to draw the pixel outside the circle
     * @param fill_color is how to draw the inside of the circle
     */
    void draw_circle(uint8_t x0, uint8_t y0, uint8_t diameter, Pixel_state fg_color, Pixel_state fill_color=Pixel_state::PIXEL_TRANSPARENT);

    /**
     * @brief draw a circle with center at (x_center, y_center) and the given radius
     * using the writing mode described by fg_color and fill_color.
     * 
     * This algorithm taken from https://groups.csail.mit.edu/graphics/classes/6.837/F98/Lecture6/circle.html
     * License for this function is not known. I modified the it to support filled circles too.
     * 
     * @param x_center the horizontal coordiate of the circle's center
     * @param y_center the vertical coordiate of the circle's center
     * @param radius is the circle's radius.
     * @param fg_color is how to draw the pixel outside the circle
     * @param fill_color is how to draw the inside of the circle
     */
    void draw_centered_circle(uint8_t x_center, uint8_t y_center, uint8_t radius, Pixel_state fg_color, Pixel_state fill_color=Pixel_state::PIXEL_TRANSPARENT);

    /**
     * @brief draw a single character to the screen based on the pixel_state.
     * 
     * @param font the font object to use when drawing the character
     * @param x horizontal position for current screen rotation of upper left pixel of the character
     * @param y vertical position for current screen rotation of upper left pixel of the character
     * @param chr the character to draw
     * @param fg_color how to render the "1" pixels of the font bitmap
     * @param bg_color how to render the "0" pixels of the font bitmap
     * @return true if successful, false otherwise
     */
    void draw_character(const Mono_mono_font& font, uint8_t x, uint8_t y, char chr, Pixel_state fg_color, Pixel_state bg_color);

    /**
     * @brief draw a character string to the screen based on the pixel_state.
     * 
     * @param font the font object to use when drawing the character
     * @param x horizontal position for current screen rotation of upper left pixel of the character
     * @param y vertical position for current screen rotation of upper left pixel of the character
     * @param str the NULL terminated C string to draw
     * @param len the number of characters to draw; must be <= strlen(str)
     * @param fg_color how to render the "1" pixels of the font bitmap
     * @param bg_color how to render the "0" pixels of the font bitmap
     * @return true if successful, false otherwise
     */
    void draw_string(const Mono_mono_font& font, uint8_t x, uint8_t y, const char* str, size_t len, Pixel_state fg_color, Pixel_state bg_color) {
        assert(strlen(str) <= len);
        while (len--) {
            draw_character(font, x, y, *str++, fg_color, bg_color);
            x+=font.width;
        }
    }

    /**
     * @brief draw a string centered on the screen (by the whole screen width)
     *
     * @param font the font to render the string
     * @param str the NULL terminated C string to draw
     * @param y vertical position for current screen rotation of upper left pixel of the character
     */
    void center_string(const Mono_mono_font& font, const char* str, uint8_t y)
    {
        auto text_len = strlen(str);
        int x = get_screen_width()/2 - (text_len*font.width)/2;
        draw_string(font, x, y, str, text_len, Pixel_state::PIXEL_ONE, Pixel_state::PIXEL_ZERO);
    }

    /**
     * @brief center str on the screen linenum+1 if str fits in one line. Otherwise,
     * break str into two lines and render the first line on linenum and the second
     * line on linenum+1
     *
     * @param font the font to render the string
     * @param str the string to display
     * @param linenum the line number of the screen, from 0 (top line of screen) to
     * display the text.
     */
    void center_string_on_two_lines(const Mono_mono_font& font, const char* str, uint8_t linenum);

    /**
     * @brief write the contents of the canvas buffer to the display memory
     * 
     */
    inline void render() {
        assert(display->write_display_mem(canvas, canvas_nbytes,0,0,display->get_num_cols()-1, display->get_num_pages()-1) );
        needs_render.height = 0;
        needs_render.width = 0;
    }

    /**
     * @brief write the contents of the canvas buffer to the display memory in the background
     *
     * @param callback the function to call when the display updates are done
     * @param id the display number (0-255) to identify the display that was updated
     */
    void render_non_blocking(void (*callback)(uint8_t id), uint8_t id);

    /**
     * @brief Get the display rotation object
     * 
     * @return Display_rotation 
     */
    inline Display_rotation get_display_rotation() {return display->get_display_rotation(); }

    /**
     * @brief Get the display number that this driver controls
     *
     * @return the display number (from 0)
     */
    inline uint8_t get_display_num() {return display->get_display_num(); }

    /**
     * @brief execute functions required to implement non-blocking function calls
     *
     * @return true if successful or false if there was an error
     */
    bool task();

    bool is_busy() { return display->is_busy(); }
    bool can_render() {return needs_render.width !=0 && needs_render.height != 0 && !display->is_busy();}

    /**
     * @brief Get the a read-only pointer to the canvas
     *
     * @return const uint8_t* canvas
     */
    const uint8_t* get_canvas() { return canvas; }

    /**
     * @brief Get the canvas nbytes object
     *
     * @return size_t the number of bytes in the canvas
     */
    size_t get_canvas_nbytes() { return canvas_nbytes; }

    /**
     * @brief create a bmp file format image of the canvas allocated by new
     *
     * @return const uint8_t* a pointer to a byte array containing bmp file
     * format image of the canvas
     * @note After the caller of this function is done with the data, free
     * the memory consumed by the image using delete[].
     */
    const uint8_t* make_bmp_file_data();

    size_t get_bmp_file_data_size() { return 62 + canvas_nbytes; }
    inline const Mono_mono_font& get_font_8() {return font8; }
    inline const Mono_mono_font& get_font_12() {return font12; }
    inline const Mono_mono_font& get_font_16() {return font16; }
    inline const Mono_mono_font& get_font_24() {return font24; }
private:
    Mono_graphics() = delete;
    Mono_graphics(Mono_graphics&) = delete;
    Ssd1306* display;
    uint8_t* canvas;
    size_t canvas_nbytes;
    void circle_points(int cx, int cy, int x, int y, Pixel_state bg_color, Pixel_state fill_color);
    void update_needs_render(uint8_t x_min, uint8_t y_min, uint8_t x_max, uint8_t y_max);
    Rectangle clip_rect;
    const Mono_mono_font font8;
    const Mono_mono_font font12;
    const Mono_mono_font font16;
    const Mono_mono_font font24;
    Rectangle needs_render;
};

}
