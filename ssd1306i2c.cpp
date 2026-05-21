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

rppicomidi::Ssd1306i2cPort* rppicomidi::Ssd1306i2cPort::i2c0_irq_context = nullptr;
rppicomidi::Ssd1306i2cPort* rppicomidi::Ssd1306i2cPort::i2c1_irq_context = nullptr;


rppicomidi::Ssd1306i2cPort::Ssd1306i2cPort(i2c_inst_t* i2c_port, uint32_t bps, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t mux_addr_) :
    i2c{i2c_port}, current_display{-1}, prev_display{-1}, ndisplays{0}, mux_addr{mux_addr_}
{
    i2c_init(i2c_port, bps);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);
    i2c_port->hw->intr_mask = 0;
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
#ifndef OS_FREERTOS
    critical_section_init(&crit_sec);
#endif
}


void rppicomidi::Ssd1306i2cPort::i2c0_irq_handler(void)
{
    i2c0_irq_context->lock_from_isr();
    if (i2c0_irq_context->current_display != -1) {
        auto display = i2c0_irq_context->displays[i2c0_irq_context->current_display];
        i2c0_irq_context->unlock_from_isr();
        display->i2c_irq_handler();
    }
    else {
        i2c0_irq_context->unlock_from_isr();
    }
}

void rppicomidi::Ssd1306i2cPort::i2c1_irq_handler(void)
{
    i2c1_irq_context->lock_from_isr();
    if (i2c1_irq_context->current_display != -1) {
        auto display = i2c1_irq_context->displays[i2c1_irq_context->current_display];
        i2c1_irq_context->unlock_from_isr();
        display->i2c_irq_handler();
    }
    else {
        i2c1_irq_context->unlock_from_isr();
    }
}

bool rppicomidi::Ssd1306i2cPort::begin_transfer(int8_t display_number)
{
    if ((i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS) == 0) {
        return false; // the TX FIFO needs to start out empty for this to work right
    }
    prev_display = current_display;
    current_display = display_number;
    if (is_mux_switch_reqired(display_number)) {
        // send the mux switch message first
        i2c->hw->enable = 0;
        i2c->hw->tar = mux_addr;
        i2c->hw->enable = 1;
        i2c->hw->data_cmd = (1 << I2C_IC_DATA_CMD_RESTART_LSB) | displays[current_display]->get_mux_map() | (1 << I2C_IC_DATA_CMD_STOP_LSB);
    }
    i2c->hw->intr_mask |= (I2C_IC_INTR_STAT_R_TX_ABRT_BITS |I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
    return true;
}

void rppicomidi::Ssd1306i2cPort::end_transfer()
{
    assert(is_busy());

    // Disable the TX_EMPTY IRQ
    i2c->hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
    // Clear the stop detected IRQ if currently active // TODO is this necessary?
    if ((i2c->hw->intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) == 0) {
        io_ro_32 dummy = i2c->hw->clr_stop_det;
        (void)dummy;
    }
    current_display = -1;
}

rppicomidi::Ssd1306i2c::Ssd1306i2c(Ssd1306i2cPort* i2c_port_, const uint8_t i2c_addr_, const uint8_t mux_map_) :
    i2c_port{i2c_port_}, i2c_addr{i2c_addr_}, mux_map{mux_map_}, 
    regbyte{0}, srcbytes{nullptr}, src_len{0}, src_bytes_sent{0}, done_callback{nullptr}
{
    xfer.display_num = i2c_port->register_display(this);
    assert(xfer.display_num >= 0);
    //bi_decl(bi_2pins_with_func(sda_gpio_, scl_gpio_, GPIO_FUNC_I2C));
}

bool rppicomidi::Ssd1306i2c::write_command(const uint8_t* command_bytes, uint8_t nbytes, uint8_t)
{
    assert(command_bytes);
    assert(nbytes);
    bool success = false;
    success = (write_blocking(i2c_addr, 0x00, command_bytes, nbytes) == nbytes+1);
    return success;
}

bool rppicomidi::Ssd1306i2c::write_command_non_blocking(const uint8_t* command_bytes, uint8_t nbytes, uint8_t,
    void (*callback)(void* instance, int result),void* instance_)
{
    assert(command_bytes);
    assert(nbytes);
    return write_non_blocking(i2c_addr, 0x00, command_bytes, nbytes, callback, instance_);
}

bool rppicomidi::Ssd1306i2c::write_data(const uint8_t* data, size_t nbytes, uint8_t)
{
    assert(data);
    assert(nbytes);
    bool success = false;
    success = write_blocking(i2c_addr, 0x40, data, nbytes) == static_cast<int>(nbytes+1);
    return success;
}

bool rppicomidi::Ssd1306i2c::write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t,
            void (*callback)(void* instance, int result), void* instance_)
{
    assert(data);
    assert(nbytes);

    return write_non_blocking(i2c_addr, 0x40, data, nbytes, callback, instance_);
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

    write_non_blocking(addr, regbyte, src, len, nullptr, nullptr);
    while(is_busy()) {

    }
    int nbytes = xfer.bytes_xferd;
    if (is_error_state()) {
        uint32_t flevel = i2c_port->get_i2c()->hw->txflr;
        nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
    }
    return nbytes; // 1 for the command byte + nbytes if no error; -1 means even the address write failed.
}

bool rppicomidi::Ssd1306i2c::write_non_blocking(uint8_t, uint8_t regbyte, const uint8_t *src, int len, 
            void (*done_callback)(void* instance, int result), void* instance)
{
    if (i2c_port->is_busy()) {
        // can't interrupt an ongoing transfer
        if (done_callback)
            (*done_callback)(instance, -2);
        return false;
    }
    assert(len == 0 || (len > 0 && src != nullptr));
    assert(xfer.state == Xfer::IDLE);
    i2c_port->lock();
    xfer.addr = i2c_addr; // the LSB is 0 for write
    xfer.regbyte = regbyte;
    xfer.bytes_xferd = 0;
    xfer.buffer = src;
    xfer.nbytes = len;
    xfer.done_callback = done_callback;
    xfer.cb_instance = instance;
    if (i2c_port->is_mux_switch_reqired(xfer.display_num)) {
        // So that the interrupt handler works correctly
        xfer.state = Xfer::MUX;
    }
    bool success = i2c_port->begin_transfer(xfer.display_num);
    i2c_port->unlock();
    return success;
}

bool rppicomidi::Ssd1306i2c::is_tx_empty()
{
    return i2c_port->get_i2c()->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
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

bool rppicomidi::Ssd1306i2c::send_byte_from_task(uint16_t, bool)
{
    assert(false);  // This code should not be called anywhere
    return false;
}

void rppicomidi::Ssd1306i2c::i2c_irq_handler()
{
    auto i2c = i2c_port->get_i2c();
    volatile uint32_t stat = i2c->hw->intr_stat;
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
        io_ro_32 dummy = i2c->hw->clr_tx_abrt;
        (void)dummy;
        i2c->hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
        // Clear the stop detected IRQ if currently active // TODO is this necessary?
        if ((i2c->hw->intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) == 0) {
            io_ro_32 dummy = i2c->hw->clr_stop_det;
            (void)dummy;
        }
        xfer.state = Xfer::ERROR;
        return;
    }
    assert((i2c->hw->raw_intr_stat & I2C_IC_INTR_STAT_R_TX_OVER_BITS) == 0);
    static io_ro_32 mask = (I2C_IC_INTR_STAT_R_TX_EMPTY_BITS);
    if ((stat & mask) != 0) {
        switch (xfer.state) {
            case Xfer::MUX:
            case Xfer::IDLE:
                // start a new transfer
                i2c->hw->enable = 0;
                i2c->hw->tar = xfer.addr;
                i2c->hw->enable = 1;
                i2c->hw->data_cmd = (1 << I2C_IC_DATA_CMD_RESTART_LSB) | xfer.regbyte;
                xfer.bytes_xferd++;
                // Fill the TX FIFO with as many data bytes as possible until data is exhausted or TX FIFO is full
                while(xfer.bytes_xferd <= xfer.nbytes && i2c->hw->txflr < 16)  {
                    uint32_t final = xfer.nbytes == xfer.bytes_xferd;
                    i2c->hw->data_cmd = (final << I2C_IC_DATA_CMD_STOP_LSB) | *xfer.buffer++;
                    xfer.bytes_xferd++;
                }
                xfer.state = Xfer::DATA;
                break;
            case Xfer::DATA:
                // TX FIFO is empty; check if any more data to send
                if (xfer.bytes_xferd > xfer.nbytes) {
                    // Disable the TX_EMPTY IRQ if currently active
                    if ((stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) != 0) {
                        i2c->hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
                    }
                    // Clear the stop detected IRQ if currently active // TODO is this necessary?
                    if ((stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) == 0) {
                        io_ro_32 dummy = i2c->hw->clr_stop_det;
                        (void)dummy;
                    }
                    i2c_port->lock_from_isr();
                    xfer.state = Xfer::IDLE;
                    i2c_port->end_transfer();
                    i2c_port->unlock_from_isr();
                    if (xfer.done_callback)
                        (*xfer.done_callback)(xfer.cb_instance, xfer.bytes_xferd);
                }
                else {
                    while(xfer.bytes_xferd <= xfer.nbytes && i2c->hw->txflr < 16)  {
                        uint32_t final = xfer.nbytes == xfer.bytes_xferd;
                        i2c->hw->data_cmd = (final << I2C_IC_DATA_CMD_STOP_LSB) | *xfer.buffer++;
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
