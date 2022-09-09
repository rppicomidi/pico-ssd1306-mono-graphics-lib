/**
 * @file ssd1306hw.h
 * @brief This class describes the software interface for all hardware
 * types that use the SSD1306 chip
 * 
 * This class is a C++ Raspberry Pi Pico port of the LibDriver 
 * C SSD1306 driver code found here: https://github.com/hepingood/ssd1306
 * Functions copied based on the Raspberry Pi Pico SDK are noted in the
 * comments below. MIT License for those files are in their respective 
 * project directories
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
class Ssd1306hw {
public:
    Ssd1306hw() = default;
    virtual ~Ssd1306hw() = default;

    /**
     * @brief Write a command byte followed by 0 or more argument bytes to the SSD1306
     *
     * @param command a pointer to a uint8_t array containing the command and command data
     * @param nbytes the number of bytes in the command array.
     * @param display_num which of the displays on the same I2C port to update.
     * @return true if the write was successful
     * @return false if the write failed
     */
    virtual bool write_command(const uint8_t* command, uint8_t nbytes, uint8_t display_num)=0;

    /**
     * @brief Write a command byte followed by 0 or more argument bytes to the SSD1306 but do
     * not wait for the I2C transfers to complete.
     *
     * @param command a pointer to a uint8_t array containing the command and command data
     * @param nbytes the number of bytes in the command array.
     * @param display_num which of the displays on the same I2C port to update.
     * @param callback points to a function that gets called when the command write completes.
     * The instance parameter is the instance of the class that implements the callback function.
     * The result parameter of the callback is the number of bytes written (may be 0). Success
     * is parameter==nbytes+1
     * @return true if the write started with no error
     * @return false if the write start failed
     */
    virtual bool write_command_non_blocking(const uint8_t* command, uint8_t nbytes, uint8_t display_num, void (*callback)(void* instance, int result), void* instance)=0;

    /**
     * @brief
     *
     * @return true if the hardware interface is busy, false otherwise
     */
    virtual bool is_busy()=0;

    /**
     * @brief
     *
     * @return true if the hardware is in an ERROR state, false otherwise
     */
    virtual bool is_error_state()=0;

    /**
     * @brief execute the state machine for the hardware interface
     *
     * @return true if the state machine step was successful, false otherwise
     */
    virtual bool task()=0;

    /**
     * @brief Write display memory bytes to the SSD1306
     *
     * @param data a pointer to a uint8_t array containing the data
     * @param nbytes the number of bytes in the data array
     * @param display_num which of the displays on the same I2C port to update.
     * @return true if the write was successful
     * @return false if the write failed
     */
    virtual bool write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
            void (*callback)(void* instance, int result), void* instance)=0;

        /**
     * @brief Write display memory bytes to the SSD1306
     *
     * @param data a pointer to a uint8_t array containing the data
     * @param nbytes the number of bytes in the data array
     * @param display_num which of the displays on the same I2C port to update.
     * @return true if the write was successful
     * @return false if the write failed
     */
    virtual bool write_data(const uint8_t* data, size_t nbytes, uint8_t display_num)=0;
};
}