/**
 * @file ssd1306i2c.cpp
 * @brief This class implements I2C communication between the Raspberry Pi
 *  RP2040 chip and the SSD1306.
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

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "ssd1306i2c.h"
#include "pico/assert.h"
#include "pico/timeout_helper.h"

rppicomidi::Ssd1306i2c::Ssd1306i2c(i2c_inst_t* i2c_port_, const uint8_t* i2c_addr_, uint8_t sda_gpio_, uint8_t scl_gpio_, uint8_t ndisplays_, uint8_t mux_addr_, const uint8_t* mux_map_) :
    i2c_port{i2c_port_}, i2c_addr{i2c_addr_}, ndisplays{ndisplays_}, mux_addr{mux_addr_}, mux_map{mux_map_}, current_mux_map{0},
    task_state{IDLE}, regbyte{0}, srcbytes{nullptr}, src_len{0}, src_bytes_sent{0}, done_callback{nullptr}
{
    assert(ndisplays >0);
    assert((mux_addr == 0 && ndisplays <= 2) || (mux_addr !=0 && mux_map != NULL && ndisplays <= 18));

    i2c_init(i2c_port, 400000); // todo: do we need to make the i2c bit rate an argument?
    gpio_set_function(sda_gpio_, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio_);
    gpio_pull_up(scl_gpio_);
    //bi_decl(bi_2pins_with_func(sda_gpio_, scl_gpio_, GPIO_FUNC_I2C));
}

bool rppicomidi::Ssd1306i2c::write_command(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    uint8_t mux = mux_map == NULL ? 0:mux_map[display_num];
    bool success = false;
    if (mux == 0 || current_mux_map == mux) {
        success = (write_blocking(i2c_port, addr, 0x00, command_bytes, nbytes, false) == nbytes+1);
    }
    else {
        success = (::i2c_write_blocking(i2c_port, mux_addr, &mux, 1, false) == 1);
        if (success) {
            current_mux_map = mux;
            success = (write_blocking(i2c_port, addr, 0x00, command_bytes, nbytes, false) == nbytes+1);
        }
    }
    return success;
}

bool rppicomidi::Ssd1306i2c::write_command_non_blocking(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num,
    void (*callback)(void* instance, int result),void* instance_)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x00, command_bytes, nbytes, callback, instance_);
}

bool rppicomidi::Ssd1306i2c::write_data(const uint8_t* data, size_t nbytes, uint8_t display_num)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    uint8_t mux = mux_map == NULL ? 0:mux_map[display_num];
    bool success = false;
    if (mux == 0 || current_mux_map == mux) {
        success = (write_blocking(i2c_port, addr, 0x40, data, nbytes, false) == static_cast<int>(nbytes+1));
    }
    else {
        success = (::i2c_write_blocking(i2c_port, mux_addr, &mux, 1, false) == 1);
        if (success) {
            current_mux_map = mux;
            success = (write_blocking(i2c_port, addr, 0x40, data, nbytes, false) == static_cast<int>(nbytes+1));
        }
    }
    return success;
}

bool rppicomidi::Ssd1306i2c::write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
            void (*callback)(void* instance, int result), void* instance_)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x40, data, nbytes, callback, instance_);
}

// The following method slightly modified from the pico-sdk's i2c_write_blocking_internal function
int rppicomidi::Ssd1306i2c::write_blocking_internal(i2c_inst_t *i2c, uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len, bool nostop,
                                       check_timeout_fn timeout_check, struct timeout_state *ts) {
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    // Synopsys hw accepts start/stop flags alongside data items in the same
    // FIFO word, so no 0 byte transfers.
    invalid_params_if(I2C, len == 0);
    invalid_params_if(I2C, ((int)len) < 0);

    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    bool abort = false;
    bool timeout = false;

    uint32_t abort_reason = 0;
    int byte_ctr;

    int ilen = (int)len;
    bool first = true;
    for (byte_ctr = 0; byte_ctr <= ilen; ++byte_ctr) {
        /******************************************************
         * The pico-sdk code loop starts like this:
         * for (byte_ctr = 0; byte_ctr = ilen; ++byte_ctr) {
         * bool first = byte_ctr == 0;
         * bool last = byte_ctr == ilen - 1;
         *
         *  i2c->hw->data_cmd =
         *          bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
         *          bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
         *          *src++;
         * 
         * We have another byte, so the loop doesn't end until byte_ctr == ilen+1,
         * so now the stop condition is byte_ctr <=ilen
         * 
         * Now we know the first byte is always regbyte, and because 0-length
         * data array is not allowed, it is never the last byte.
         *******************************************************/
        bool last = byte_ctr == ilen;
        if (first) {
            i2c->hw->data_cmd =
                    bool_to_bit(i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                    regbyte;
            first = false;
        }
        else {
            i2c->hw->data_cmd =
                    bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                    *src++;
        }

        // Wait until the transmission of the address/data from the internal
        // shift register has completed. For this to function correctly, the
        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
        // was set in i2c_init.
        do {
            if (timeout_check) {
                timeout = timeout_check(ts, false);
                abort |= timeout;
            }
            tight_loop_contents();
        } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

        // If there was a timeout, don't attempt to do anything else.
        if (!timeout) {
            abort_reason = i2c->hw->tx_abrt_source;
            if (abort_reason) {
                // Note clearing the abort flag also clears the reason, and
                // this instance of flag is clear-on-read! Note also the
                // IC_CLR_TX_ABRT register always reads as 0.
                i2c->hw->clr_tx_abrt;
                abort = true;
            }

            if (abort || (last && !nostop)) {
                // If the transaction was aborted or if it completed
                // successfully wait until the STOP condition has occured.

                // TODO Could there be an abort while waiting for the STOP
                // condition here? If so, additional code would be needed here
                // to take care of the abort.
                do {
                    if (timeout_check) {
                        timeout = timeout_check(ts, false);
                        abort |= timeout;
                    }
                    tight_loop_contents();
                } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

                // If there was a timeout, don't attempt to do anything else.
                if (!timeout) {
                    i2c->hw->clr_stop_det;
                }
            }
        }

        // Note the hardware issues a STOP automatically on an abort condition.
        // Note also the hardware clears RX FIFO as well as TX on abort,
        // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
        if (abort)
            break;
    }

    int rval;

    // A lot of things could have just happened due to the ingenious and
    // creative design of I2C. Try to figure things out.
    if (abort) {
        if (timeout)
            rval = PICO_ERROR_TIMEOUT;
        else if (!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) {
            // No reported errors - seems to happen if there is nothing connected to the bus.
            // Address byte not acknowledged
            rval = PICO_ERROR_GENERIC;
        } else if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) {
            // Address acknowledged, some data not acknowledged
            rval = byte_ctr;
        } else {
            //panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
            rval = PICO_ERROR_GENERIC;
        }
    } else {
        rval = byte_ctr;
    }

    // nostop means we are now at the end of a *message* but not the end of a *transfer*
    i2c->restart_on_next = nostop;
    return rval;
}

bool rppicomidi::Ssd1306i2c::write_non_blocking(uint8_t addr, uint8_t regbyte_, const uint8_t *src_, int len_, 
            void (*done_callback_)(void* instance, int result), void* instance_)
{
    if (task_state != IDLE && task_state != ERROR) {
        // can't interrupt an ongoing transaction
        if (done_callback)
            (*done_callback)(cb_instance, 0);
        return false;
    }
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len_ > 0 && src_ != nullptr);
    regbyte = regbyte_;
    srcbytes = src_;
    src_len = len_;
    src_bytes_sent = 0;
    done_callback = done_callback_;
    cb_instance = instance_;

    /* start the tranaction */
    i2c_port->hw->enable = 0;
    i2c_port->hw->tar = addr;
    i2c_port->hw->enable = 1;
    i2c_port->hw->data_cmd = bool_to_bit(i2c_port->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
        regbyte;
    task_state = SRCBYTE;
    src_bytes_sent = 0;
    return true;
}

bool rppicomidi::Ssd1306i2c::is_tx_empty()
{
    return i2c_port->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
}

bool rppicomidi::Ssd1306i2c::is_i2c_error()
{
    bool is_error = false;
    if (i2c_port->hw->tx_abrt_source) {
        i2c_port->hw->clr_tx_abrt;
        task_state = ERROR;
        is_error = true;
    }
    return is_error;
}

bool rppicomidi::Ssd1306i2c::send_byte_from_task(uint16_t data, bool is_last)
{
    bool byte_sent = false;
    if (is_tx_empty()) {
        if (!is_i2c_error()) {
            i2c_port->hw->data_cmd =
                    bool_to_bit(is_last) << I2C_IC_DATA_CMD_STOP_LSB | data;
            byte_sent = true;
        }
    }
    return byte_sent;
}

bool rppicomidi::Ssd1306i2c::task()
{
    bool success = true;
    switch(task_state) {
        case IDLE:
            // Nothing to do
            break;
        case SRCBYTE:
        {
            uint16_t last = ((src_bytes_sent+1) == src_len);
            if (send_byte_from_task(*srcbytes, last)) {
                ++src_bytes_sent;
                ++srcbytes;
                if (last) {
                    task_state = WAIT_LAST;
                }
            }

        }
            break;
        case WAIT_LAST:
            if (is_tx_empty()) {
                if (is_i2c_error()) {
                    success = false;
                }
                else {
                    task_state = IDLE;
                }
                if (done_callback)
                    (*done_callback)(cb_instance, src_bytes_sent+1);
            }
            break;
        case ERROR:
            if (done_callback)
                (*done_callback)(cb_instance, src_bytes_sent+1);
            success = false;
            break;
        default:
            success = false;
            break;
    }
    return success;
}
