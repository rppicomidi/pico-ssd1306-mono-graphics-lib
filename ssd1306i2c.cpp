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
#include <stdio.h>

rppicomidi::Ssd1306i2c* rppicomidi::Ssd1306i2c::i2c0_irq_context = nullptr;
rppicomidi::Ssd1306i2c* rppicomidi::Ssd1306i2c::i2c1_irq_context = nullptr;

rppicomidi::Ssd1306i2c::Ssd1306i2c(i2c_inst_t* i2c_port_, uint32_t bps, const uint8_t* i2c_addr_, uint8_t sda_gpio_, uint8_t scl_gpio_, uint8_t ndisplays_, uint8_t mux_addr_, const uint8_t* mux_map_) :
    i2c_port{i2c_port_}, i2c_addr{i2c_addr_}, ndisplays{ndisplays_}, mux_addr{mux_addr_}, mux_map{mux_map_}, current_mux_map{0},
    regbyte{0}, srcbytes{nullptr}, src_len{0}, src_bytes_sent{0}, done_callback{nullptr}
{
    assert(ndisplays >0);
    assert((mux_addr == 0 && ndisplays <= 2) || (mux_addr !=0 && mux_map != NULL && ndisplays <= 18));

    i2c_init(i2c_port, bps);
    gpio_set_function(sda_gpio_, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio_);
    gpio_pull_up(scl_gpio_);
    i2c_port->hw->intr_mask = 0;
    //hw_clear_bits(&i2c_port->hw->con, I2C_IC_CON_TX_EMPTY_CTRL_BITS);
    if (i2c_port == i2c0) {
        i2c0_irq_context = this;
        irq_add_shared_handler(I2C0_IRQ, i2c0_irq_handler, PICO_DEFAULT_IRQ_PRIORITY);
        irq_set_enabled(I2C0_IRQ, true);
    }
    else {
        assert(i2c_port == i2c1);
        i2c1_irq_context = this;
        irq_add_shared_handler(I2C1_IRQ, i2c1_irq_handler, PICO_DEFAULT_IRQ_PRIORITY);
        irq_set_enabled(I2C1_IRQ, true);
    }
    //bi_decl(bi_2pins_with_func(sda_gpio_, scl_gpio_, GPIO_FUNC_I2C));
}

bool rppicomidi::Ssd1306i2c::write_command(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    xfer.display_num = display_num;
    bool success = false;
    success = (write_blocking(addr, 0x00, command_bytes, nbytes) == nbytes+1);
    return success;
}

bool rppicomidi::Ssd1306i2c::write_command_non_blocking(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num,
    void (*callback)(void* instance, int result),void* instance_)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    xfer.display_num = display_num;
    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x00, command_bytes, nbytes, callback, instance_);
}

bool rppicomidi::Ssd1306i2c::write_data(const uint8_t* data, size_t nbytes, uint8_t display_num)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    xfer.display_num = display_num;
    uint8_t addr = i2c_addr[display_num];
    bool success = false;
    success = write_blocking(addr, 0x40, data, nbytes) == static_cast<int>(nbytes+1);
    return success;
}

bool rppicomidi::Ssd1306i2c::write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
            void (*callback)(void* instance, int result), void* instance_)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    xfer.display_num = display_num;

    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x40, data, nbytes, callback, instance_);
}

int rppicomidi::Ssd1306i2c::write_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len)
{
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));

    while(is_busy()) {

    }
    if (is_error_state()) {
        return -3;
    }
    uint8_t mux = mux_map == NULL ? 0:mux_map[xfer.display_num];

    if (mux_map != 0 && current_mux_map != mux) {
        bool success = (::i2c_write_blocking(i2c_port, mux_addr, &mux, 1, false) == 1);
        if (success) {
            current_mux_map = mux;
        }
    }
    write_non_blocking(addr, regbyte, src, len, nullptr, nullptr);
    while(is_busy()) {

    }
    int nbytes = xfer.bytes_xferd;
    if (is_error_state()) {
        uint32_t flevel = i2c_port->hw->txflr;
        nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
    }
    return nbytes; // 1 for the command byte + nbytes if no error; -1 means even the address write failed.
}

bool rppicomidi::Ssd1306i2c::write_non_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, int len, 
            void (*done_callback)(void* instance, int result), void* instance)
{
    if (is_busy()) {
        // can't interrupt an ongoinxfer.stateion
        if (done_callback)
            (*done_callback)(instance, -2);
        return false;
    }
    // TODO: I broke the I2C MUX for non-blocking writes
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));
    assert(xfer.state == Xfer::IDLE);
    xfer.addr = addr; // the LSB is 0 for write
    xfer.regbyte = regbyte;
    xfer.bytes_xferd = 0;
    xfer.buffer = src;
    xfer.nbytes = len;
    xfer.done_callback = done_callback;
    xfer.cb_instance = instance;

    /* start the tranaction by enabling the TX-related interrupts; should fire the interrupt right away*/
    i2c_port->hw->intr_mask |= (I2C_IC_INTR_STAT_R_TX_ABRT_BITS |I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
    return true;
}

bool rppicomidi::Ssd1306i2c::is_tx_empty()
{
    return i2c_port->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
}

bool rppicomidi::Ssd1306i2c::is_i2c_error()
{
    return xfer.state == Xfer::ERROR;
}

bool rppicomidi::Ssd1306i2c::task()
{
    if (is_error_state()) {
        return false;
    }
    return true;
}

void rppicomidi::Ssd1306i2c::i2c_irq_handler()
{
    volatile uint32_t stat = i2c_port->hw->intr_stat;
#if 0
    // for some reason, this interrupt routine can trigger when the status is TX FIFO Empty and the TX FIFO Level is still 1
    // Weird timing issue?
    volatile uint32_t reg = i2c_port->hw->txflr;
    if (stat == I2C_IC_INTR_STAT_R_TX_EMPTY_BITS && reg != 0) {
        printf("stat=0x%08lx reg=0x%08lx\r\n", stat, reg);
        assert(reg == 0);
    }
#endif
    if (stat &  I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // Error
        io_ro_32 dummy = i2c_port->hw->clr_tx_abrt;
        (void)dummy;
        i2c_port->hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
        // Clear the stop detected IRQ if currently active // TODO is this necessary?
        if ((i2c_port->hw->intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) == 0) {
            io_ro_32 dummy = i2c_port->hw->clr_stop_det;
            (void)dummy;
        }
        xfer.state = Xfer::ERROR;
        return;
    }
    static io_ro_32 mask = (I2C_IC_INTR_STAT_R_TX_EMPTY_BITS);
    if ((stat & mask) != 0) {
        switch (xfer.state) {
            case Xfer::IDLE:
                // start a new transfer
                i2c_port->hw->enable = 0;
                i2c_port->hw->tar = xfer.addr;
                i2c_port->hw->enable = 1;
                i2c_port->hw->data_cmd = (1 << I2C_IC_DATA_CMD_RESTART_LSB) | xfer.regbyte;
                xfer.bytes_xferd++;
                // Fill the TX FIFO with as many data bytes as possible until data is exhausted or TX FIFO is full
                while(xfer.bytes_xferd <= xfer.nbytes && i2c_port->hw->txflr < 16)  {
                    uint32_t final = xfer.nbytes == xfer.bytes_xferd;
                    i2c_port->hw->data_cmd = (final << I2C_IC_DATA_CMD_STOP_LSB) | *xfer.buffer++;
                    xfer.bytes_xferd++;
                }
                xfer.state = Xfer::DATA;
                break;
            case Xfer::DATA:
                // TX FIFO is empty; check if any more data to send
                if (xfer.bytes_xferd > xfer.nbytes) {
                     // Disable the TX_EMPTY IRQ if currently active
                    if ((stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) != 0) {
                        i2c_port->hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
                    }
                    // Clear the stop detected IRQ if currently active // TODO is this necessary?
                    if ((stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) == 0) {
                        io_ro_32 dummy = i2c_port->hw->clr_stop_det;
                        (void)dummy;
                    }
                    xfer.state = Xfer::IDLE;
                    if (xfer.done_callback)
                        (*xfer.done_callback)(xfer.cb_instance, xfer.bytes_xferd);
                }
                else {
                    while(xfer.bytes_xferd <= xfer.nbytes && i2c_port->hw->txflr < 16)  {
                        uint32_t final = xfer.nbytes == xfer.bytes_xferd;
                        i2c_port->hw->data_cmd = (final << I2C_IC_DATA_CMD_STOP_LSB) | *xfer.buffer++;
                        xfer.bytes_xferd++;
                    }
                }
                break;
            default:
                assert(false);
                break;
        }
    }
}
