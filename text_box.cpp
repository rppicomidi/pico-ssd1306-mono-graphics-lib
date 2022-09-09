/**
 * @file button_led.h
 * @brief This class implements drawing centered text surrounded by a box
 * @todo rename it and make it describe what it does
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
 * SOFTWARE. * 
 */

#include "text_box.h"

rppicomidi::Text_box::Text_box(Mono_graphics& screen_, uint8_t x_, uint8_t y_, uint8_t width_, uint8_t height_,
        const char* text_, const Mono_mono_font& font_,  bool is_on_, Justify justify_) :
    screen(screen_), x{x_}, y{y_}, width{width_}, height{height_}, text{text_}, font{font_}, is_on{is_on_}
{
    set_justify(justify_);
    draw();
}

void rppicomidi::Text_box::set_state(bool is_on_)
{
    is_on = is_on_;
    draw();
}

void rppicomidi::Text_box::set_justify(Justify justify_)
{
    justify = justify_;
    size_t len = strlen(text);
    assert(len < 255);
    text_len = (uint8_t)(len);
    uint8_t pixel_padding = 2;
    int text_width = text_len * font.width + pixel_padding;
    if (text_width > (int)width) {
        text_width = width / 8;
        x_offset = x + pixel_padding;
    }
    else {
        switch (justify) {
            case JUSTIFY_LEFT:
                x_offset = x + pixel_padding;
                break;
            case JUSTIFY_CENTER:
                x_offset = x + width/2 - (text_len * font.width)/2;
                break;
            case JUSTIFY_RIGHT:
                x_offset = width - text_width + x - pixel_padding;
                break;
            default:
                break;
        }
    }
}

void rppicomidi::Text_box::draw()
{
    Pixel_state textbg = Pixel_state::PIXEL_ZERO;
    Pixel_state textfg = is_on ? Pixel_state::PIXEL_ONE:Pixel_state::PIXEL_ZERO;
    screen.draw_rectangle(x, y, width, height, textfg, textbg);
    screen.draw_string(font, x_offset, y+2, text, text_len, textfg, textbg);
}