/**
 * @file pixel_state.h
 * @brief This class describes how to draw a pixel on a monochrome graphics display
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
namespace rppicomidi {
/**
 * @brief this enum is more descriptive than color
 * 
 * When the memory bit is 0, then the pixel
 * will be off in normal display mode and on
 * in inverted display mode.
 * 
 * When the memory bit is 1, then the pixel
 * will be on in normal display mode and off
 * in inverted display mode.
 */
enum class Pixel_state {
    PIXEL_ZERO, //!< set the pixel memory bit to 0
    PIXEL_ONE,  //!< set the pixel memory bit to 1
    PIXEL_XOR,  //!< flip the pixel memory bit
    PIXEL_TRANSPARENT, //!< don't change the memory bit
};
}