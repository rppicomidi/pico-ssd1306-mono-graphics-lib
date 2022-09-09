/**
 * @file ssd1306i2c.h
 * @brief This class implements I2C communication between the Raspberry Pi
 *  RP2040 chip and the SSD1306.
 *
 * This class is an enhanced C++ Raspberry Pi Pico port of the LibDriver 
 * C SSD1306 driver code found here: https://github.com/hepingood/ssd1306
 * Functions copied based on the Raspberry Pi Pico SDK are noted in the
 * comments below. MIT License for those files are in their respective 
 * project directories
 *
 * This driver supports several different configurations:
 *  - A single display wired to a single I2C port
 *  - Two displays wired to the same I2C port; one display has address 0x3C
 *    and the other 0x3D.
 * Implemented but not tested:
 *  - Up to 16 displays wired to a TCA9548A I2C mux (up to 2 displays per mux port)
 *  - Up to 16 displays wired to a TCA9548A I2C mux plus 1 or 2 displays
 *    wired directly to the I2C port.
 * Keep in mind that sharing the I2C port will slow down the update rate of
 * every display.
 *
 * This driver is not thread safe.
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
#include "hardware/i2c.h"
#include "pico/timeout_helper.h"
#include "ssd1306hw.h"
namespace rppicomidi {
class Ssd1306i2c : public Ssd1306hw
{
public:
    /**
     * @brief Construct a new i2c ssd1306 object
     * 
     * @param i2c_port the hardware handle for the I2C port in a struct
     * @param i2c_addr an array of I2C address for each display on the I2C bus.
     * @param sda_gpio the GPIO number of the I2C SDA signal
     * @param scl_gpio the GPIO number of the I2C SCL signal
     * @param ndisplays the number of displays wired on the same I2C bus
     * @param mux_addr the I2C address of the TCA9548A I2C mux chip or 0 if none is used
     * @param mux_map* an array of 8-bit bitmaps corresponding to the TCA9548A mux output
     * for each I2C addr. If the entry is 0, the port is not on a mux port but is instead
     * wired directly to the I2C port.
     */
    Ssd1306i2c(i2c_inst_t* i2c_port, const uint8_t* i2c_addr, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t ndisplays=1, uint8_t mux_addr=0, const uint8_t* mux_map=NULL);

    virtual ~Ssd1306i2c()=default;
    /**
     * @brief Write a command byte followed by 0 or more argument bytes to the SSD1306
     * 
     * @param command a pointer to a uint8_t array containing the command and command data
     * @param nbytes the number of bytes in the command array.
     * @return true if the write was successful
     * @return false if the write failed
     */
    bool write_command(const uint8_t* command, uint8_t nbytes, uint8_t display_num) final;

    /**
     * @brief Write a command byte followed by 0 or more argument bytes to the SSD1306 but do
     * not wait for the I2C transfers to complete.
     *
     * @param command a pointer to a uint8_t array containing the command and command data
     * @param nbytes the number of bytes in the command array.
     * @param display_num which of the displays on the same I2C port to update.
     * @param callback points to a function that gets called when the command write completes.
     * The result parameter of the callback is the number of bytes written (may be 0). Success
     * is parameter==nbytes+1
     * @return true if the write started with no error
     * @return false if the write start failed
     * @todo make this method support I2C mux chips
     */
    bool write_command_non_blocking(const uint8_t* command, uint8_t nbytes, uint8_t display_num, void (*callback)(void* instance, int result), void* instance_) final;

    /**
     * @brief Write display memory bytes to the SSD1306
     * 
     * @param data a pointer to a uint8_t array containing the data
     * @param nbytes the number of bytes in the data array.
     * @return true if the write was successful
     * @return false if the write failed
     */
    bool write_data(const uint8_t* data, size_t nbytes, uint8_t display_num) final;

    bool write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
        void (*callback)(void* instance, int result), void* instance);

    bool task() final;

    inline bool is_error_state() final {return task_state == ERROR;}
    inline bool is_busy() final { return task_state != IDLE && task_state != ERROR; }
private:
    Ssd1306i2c() = delete;
    Ssd1306i2c(Ssd1306i2c&) = delete;
    i2c_inst_t* i2c_port;
    const uint8_t* i2c_addr;
    uint8_t ndisplays;
    uint8_t mux_addr;
    const uint8_t* mux_map;
    uint8_t current_mux_map;

    enum task_state_e {IDLE, SRCBYTE, WAIT_LAST, ERROR} task_state;
    uint8_t regbyte;
    const uint8_t* srcbytes;
    int src_len; // number of source bytes remaining to send
    int src_bytes_sent;
    void (*done_callback)(void* instance, int result);
    void* cb_instance;

    /**
     * @brief test if the address is reserved (copied from pico-sdk)
     */
    inline bool i2c_reserved_addr(uint8_t addr) {
        return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
    }

    /**
     * @brief This is the same as pico-sdk i2c_write_blocking_internal except this function
     * sends the regbyte byte before sending all of the data and the return value is len+1 on success
     * 
     * @param regbyte 8-bit register byte; either 0 for SSD1306 commands or 0x40 for display data
     */
    int write_blocking_internal(i2c_inst_t *i2c, uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len, bool nostop,
                                       ::check_timeout_fn timeout_check, struct ::timeout_state *ts);

    /**
     * @brief This function is the same as pico-sdk i2c_write_blocking except this function
     * sends the regbyte byte before sending all of the data and the return value is len+1 on success
     * 
     * @param regbyte 8-bit register byte; either 0 for SSD1306 commands or 0x40 for display data
     */
    inline int write_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len, bool nostop) {
        return write_blocking_internal(i2c, addr, regbyte, src, len, nostop, NULL, NULL);
    }   

    /**
     * @brief This function is the same as pico-sdk i2c_write_blocking_until except this function
     * sends the regbyte byte before sending all of the data and the return value is len+1 on success
     * 
     * @param regbyte 8-bit register byte; either 0 for SSD1306 commands or 0x40 for display data
     */
    inline int write_blocking_until(i2c_inst_t *i2c, uint8_t regbyte, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                absolute_time_t until) {
        timeout_state_t ts;
        return write_blocking_internal(i2c, addr, regbyte, src, len, nostop, init_single_timeout_until(&ts, until), &ts);
    }

    bool write_non_blocking(uint8_t addr, uint8_t regbyte_, const uint8_t *src_, int len_, 
            void (*done_callback_)(void* instance, int result), void* instance_);

    bool send_byte_from_task(uint16_t data, bool is_last);

    inline bool is_tx_empty();
    bool is_i2c_error();
};
}