/**
 * @file ssd1306pioi2c.cpp
 * @brief This class implements I2C communication between the Raspberry Pi
 * RP2040 chip and the SSD1306 using a PIO module instead of one of the I2C
 * interface blocks. It is a C++ port of of the PIO I2C example in pico-examples.
 * Uses BSD 3-clause license from the code it uses. See
 * https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c for original
 * code and https://github.com/raspberrypi/pico-examples/blob/master/LICENSE.TXT
 * for the license.
 *
 * Copyright (c) 2022 rppicomidi
 */
#include "ssd1306pioi2c.h"
#include "assert.h"
#include "hardware/i2c.h" // for assertion check

rppicomidi::Ssd1306pio_i2c::Ssd1306pio_i2c(pio_hw_t* pio_instance_, uint state_machine_, uint offset_,
                uint8_t* i2c_addr_, uint8_t sda_gpio,
                uint8_t scl_gpio, uint8_t ndisplays_, uint8_t mux_addr_, uint8_t* mux_map_) :
    pio_instance{pio_instance_}, state_machine{state_machine_}, offset{offset_}, i2c_addr{i2c_addr_},
    ndisplays{ndisplays_}, mux_addr{mux_addr_}, mux_map{mux_map_}, current_mux_map{0},
    task_state{IDLE}, regbyte{0}, srcbytes{nullptr}, src_len{0}, src_bytes_sent{0}, done_callback{nullptr}
{
    assert(i2c_addr);
    assert(mux_addr == 0 || (mux_addr != 0 && mux_map != nullptr));
    if (state_machine == 0)
        offset = pio_add_program(pio_instance_, &i2c_program);
    i2c_program_init(pio_instance_, state_machine_, offset, sda_gpio, scl_gpio);
}

bool rppicomidi::Ssd1306pio_i2c::write_command(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    uint8_t mux = mux_map == NULL ? 0:mux_map[display_num];
    bool success = false;
    if (mux == 0 || current_mux_map == mux) {
        success = (write_blocking(addr, 0x00, command_bytes, nbytes) == nbytes+1);
    }
    else {
        success = (write_blocking(mux_addr, mux, nullptr, 0) == 1);
        if (success) {
            current_mux_map = mux;
            success = (write_blocking(addr, 0x00, command_bytes, nbytes) == nbytes+1);
        }
    }
    return success;
}

bool rppicomidi::Ssd1306pio_i2c::write_command_non_blocking(const uint8_t* command_bytes, uint8_t nbytes, uint8_t display_num,
    void (*callback)(void* instance, int result),void* instance_)
{
    assert(command_bytes);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x00, command_bytes, nbytes, callback, instance_);
}

bool rppicomidi::Ssd1306pio_i2c::write_data(const uint8_t* data, size_t nbytes, uint8_t display_num)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    uint8_t mux = mux_map == NULL ? 0:mux_map[display_num];
    bool success = false;
    if (mux == 0 || current_mux_map == mux) {
        success = (write_blocking(addr, 0x40, data, nbytes) == (int)(nbytes+1));
    }
    else {
        success = (write_blocking(mux_addr, mux, nullptr, 0) == 1);
        if (success) {
            current_mux_map = mux;
            success = (write_blocking(addr, 0x40, data, nbytes) == (int)(nbytes+1));
        }
    }
    return success;
}


bool rppicomidi::Ssd1306pio_i2c::write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
            void (*callback)(void* instance, int result), void* instance_)
{
    assert(data);
    assert(nbytes);
    assert(display_num < ndisplays);
    uint8_t addr = i2c_addr[display_num];
    return write_non_blocking(addr, 0x40, data, nbytes, callback, instance_);
}

// If I2C is ok, block and push data. Otherwise fall straight through.
void rppicomidi::Ssd1306pio_i2c::pio_i2c_put_or_err(uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio_instance, state_machine))
        if (pio_i2c_check_error())
            return;
    if (pio_i2c_check_error())
        return;
    // some versions of GCC dislike this
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(io_rw_16 *)&pio_instance->txf[state_machine] = data;
#pragma GCC diagnostic pop
}

uint8_t rppicomidi::Ssd1306pio_i2c::pio_i2c_get() {
    return (uint8_t)pio_sm_get(pio_instance, state_machine);
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_start() {
    pio_i2c_put_or_err(1u << PIO_I2C_ICOUNT_LSB); // Escape code for 2 instruction sequence
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // We are already in idle state, just pull SDA low
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // Also pull clock low so we can present data
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_stop() {
    pio_i2c_put_or_err(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // SDA is unknown; pull it down
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // Release clock
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);    // Release SDA to return to idle state
};

void rppicomidi::Ssd1306pio_i2c::pio_i2c_repstart() {
    pio_i2c_put_or_err(3u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
}

int rppicomidi::Ssd1306pio_i2c::write_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len)
{
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));

    int bytes_sent = 0;
    pio_i2c_start();
    pio_i2c_rx_enable(false);
    pio_i2c_put16((addr << 2) | 1u);
    
    while (!pio_i2c_check_error()) {
        if (!pio_sm_is_tx_fifo_full(pio_instance, state_machine)) {
            pio_i2c_put_or_err((regbyte << PIO_I2C_DATA_LSB) | 1u);
            ++bytes_sent;
            break;
        }
    }
    while (len && !pio_i2c_check_error()) {
        if (!pio_sm_is_tx_fifo_full(pio_instance, state_machine)) {
            --len;
            ++bytes_sent;
            pio_i2c_put_or_err((*src++ << PIO_I2C_DATA_LSB) | ((len == 0) << PIO_I2C_FINAL_LSB) | 1u);
        }
    }
    pio_i2c_stop();
    pio_i2c_wait_idle();
    if (pio_i2c_check_error()) {
        bytes_sent = -1; // signal an error
        pio_i2c_resume_after_error();
        pio_i2c_stop();
    }
    return bytes_sent;
}

bool rppicomidi::Ssd1306pio_i2c::pio_i2c_check_error() {
    return !!(pio_instance->irq & (1u << state_machine));
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_resume_after_error() {
    pio_sm_drain_tx_fifo(pio_instance, state_machine);
    pio_sm_exec(pio_instance, state_machine, (pio_instance->sm[state_machine].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_instance->irq = 1u << state_machine;
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_rx_enable(bool en) {
    if (en)
        hw_set_bits(&pio_instance->sm[state_machine].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    else
        hw_clear_bits(&pio_instance->sm[state_machine].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

bool rppicomidi::Ssd1306pio_i2c::write_non_blocking(uint8_t addr, uint8_t regbyte_, const uint8_t *src_, int len_, 
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
    task_state = REGBYTE;
    pio_i2c_start();
    pio_i2c_rx_enable(false);
    pio_i2c_put16((addr << 2) | 1u);
    return true;
}

void rppicomidi::Ssd1306pio_i2c::send_stop_from_task()
{
    pio_i2c_stop();
    pio_i2c_wait_idle();
    if (pio_i2c_check_error()) {
        task_state = ERROR;
        pio_i2c_resume_after_error();
        pio_i2c_stop();
    }
}

bool rppicomidi::Ssd1306pio_i2c::send_byte_from_task(uint16_t data)
{
    bool byte_sent = false;
    if (!pio_sm_is_tx_fifo_full(pio_instance, state_machine)) {
        if (pio_i2c_check_error()) {
            send_stop_from_task();
        }
        else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(io_rw_16 *)&pio_instance->txf[state_machine] = data;
#pragma GCC diagnostic pop
            byte_sent = true;
        }
    }
    return byte_sent;
}

bool rppicomidi::Ssd1306pio_i2c::task()
{
    bool success = true;
    switch(task_state) {
        case IDLE:
            // Nothing to do
            break;
        case REGBYTE:
            if (send_byte_from_task(((uint16_t)regbyte << PIO_I2C_DATA_LSB) | 1u)) {
                src_bytes_sent = 0;
                if (task_state != ERROR) {
                    task_state = SRCBYTE;
                }
                else {
                    success = false;
                    if (done_callback)
                        (*done_callback)(cb_instance, 0);
                }
            }
            break;
        case SRCBYTE:
            if (src_bytes_sent < src_len) {
                uint16_t last = ((src_bytes_sent+1) == src_len) << PIO_I2C_FINAL_LSB;
                if (send_byte_from_task(((uint16_t)*srcbytes<< PIO_I2C_DATA_LSB) | 1u | last)) {
                    ++src_bytes_sent;
                    ++srcbytes;
                }
            }
            else {
                send_stop_from_task();
                if (task_state == ERROR) {
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
            // Can't do anything
        default:
            success = false;
            break;
    }
    return success;
}