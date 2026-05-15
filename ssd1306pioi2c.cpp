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
#define PARAM_ASSERTIONS_ENABLED_I2C 1
#include "ssd1306pioi2c.h"
#include "assert.h"
#include "hardware/sync.h"
uint32_t rppicomidi::Ssd1306pio_i2c::pio_i2c_irq_mask = 0;
rppicomidi::Ssd1306pio_i2c* rppicomidi::Ssd1306pio_i2c::i2c_port[4]={nullptr, nullptr, nullptr, nullptr};
rppicomidi::Ssd1306pio_i2c::Ssd1306pio_i2c(pio_hw_t* pio_instance_, uint state_machine_, uint offset_,
                uint8_t* i2c_addr_, uint8_t sda_gpio,
                uint8_t scl_gpio, uint8_t ndisplays_, uint8_t mux_addr_, uint8_t* mux_map_) :
    pio_instance{pio_instance_}, state_machine{state_machine_}, offset{offset_}, i2c_addr{i2c_addr_},
    ndisplays{ndisplays_}, mux_addr{mux_addr_}, mux_map{mux_map_}, current_mux_map{0}//,
    //task_state{IDLE}, regbyte{0}, srcbytes{nullptr}, src_len{0}, src_bytes_sent{0}, done_callback{nullptr}
{
    assert(i2c_addr);
    assert(mux_addr == 0 || (mux_addr != 0 && mux_map != nullptr));
    if (state_machine == 0)
        offset = pio_add_program(pio_instance_, &i2c_irq_program);
    pio_sm_claim(pio_instance, state_machine);
    i2c_irq_program_init(pio_instance_, state_machine_, offset, sda_gpio, scl_gpio);
    if (pio_i2c_irq_mask == 0) {
        // then need to install the IRQ handler; do not enable it yet
        irq_add_shared_handler(PIO_IRQ_NUM(pio_instance, 0), pio_i2c_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(PIO_NUM(pio_instance)*2 + PIO0_IRQ_0, true);
    }
    i2c_port[state_machine] = this;
    pio_i2c_irq_mask = 1<<(state_machine + PIO_IRQ0_INTE_SM0_LSB);
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

void rppicomidi::Ssd1306pio_i2c::pio_i2c_start() {
    pio_i2c_put16(1u << PIO_I2C_ICOUNT_LSB); // Escape code for 2 instruction sequence
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // We are already in idle state, just pull SDA low
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // Also pull clock low so we can present data
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_stop() {
    pio_i2c_put16(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // SDA is unknown; pull it down
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // Release clock
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD1]);    // Release SDA to return to idle state
};

void rppicomidi::Ssd1306pio_i2c::pio_i2c_repstart() {
    pio_i2c_put16(3u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);
}

int rppicomidi::Ssd1306pio_i2c::write_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len)
{
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));

    while(is_busy()) {

    }
    write_non_blocking(addr, regbyte, src, len, nullptr, nullptr);
    while(is_busy()) {

    }
    int nbytes = xfer.bytes_xferd;
    if (is_error_state()) {
        uint32_t flevel = ((pio_instance->flevel) >> (8*state_machine)) & 0xF;
        nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
    }
    return nbytes; // 1 for the command byte + nbytes if no error; -1 means even the address write failed.
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_resume_after_error() {
    pio_sm_drain_tx_fifo(pio_instance, state_machine);
    pio_sm_exec(pio_instance, state_machine, (pio_instance->sm[state_machine].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_instance->irq = (0x10u << state_machine);
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_rx_enable(bool )
{
    assert(false); // TX FIFO is JOINED with RX FIFO to make TX FIFO bigger, so this routine should never be called
}

bool rppicomidi::Ssd1306pio_i2c::write_non_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, int len, 
    void (*done_callback)(void* instance, int result), void* instance)
{
    if (is_busy()) {
        // can't interrupt an ongoinxfer.stateion
        if (done_callback)
            (*done_callback)(instance, -2);
        return false;
    }
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));
    assert(xfer.state == Xfer::IDLE);
    xfer.addr = addr << 1; // the LSB is 0 for write
    xfer.regbyte = regbyte;
    xfer.bytes_xferd = 0;
    xfer.buffer = src;
    xfer.nbytes = len;
    xfer.done_callback = done_callback;
    xfer.cb_instance = instance;

    hw_set_bits(&pio_instance->inte0, 1<<(state_machine+PIO_IRQ0_INTE_SM0_LSB));

    return true;
}

bool rppicomidi::Ssd1306pio_i2c::task()
{
    if (is_error_state()) {
        return false;
    }
    return true;
}

void rppicomidi::Ssd1306pio_i2c::pio_sm_irq_handler()
{
    // Catch the error interrupt or an erroneous transfer start
    if ((pio_instance->irq & (0x10 << state_machine)) != 0 || (xfer.state == Xfer::IDLE && (xfer.nbytes >0 && xfer.bytes_xferd != 0))) {
        xfer.state = Xfer::ERROR;
        // prevent the interrupt from firing again; error must be cleared externally
        hw_clear_bits(&pio_instance->inte0, 1<<(state_machine+PIO_IRQ0_INTE_SM0_LSB));
        uint32_t flevel = ((pio_instance->flevel) >> (8*state_machine)) & 0xF;
        int nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
        if (xfer.done_callback)
            (*xfer.done_callback)(xfer.cb_instance, nbytes);
        return;
    }
    switch (xfer.state) {
        case Xfer::IDLE:
            pio_i2c_start(); // Only takes 3 words of the FIFO; keep going
            write16data(xfer.addr, 0);
            write16data(xfer.regbyte, xfer.nbytes == 0); // write the command register byte; might be the last transfer if only sending a command byte
            xfer.bytes_xferd++;
            // Fill the TX FIFO with as many data bytes as possible until data is exhausted or TX FIFO is full
            while(xfer.bytes_xferd <= xfer.nbytes && !pio_sm_is_tx_fifo_full(pio_instance, state_machine)) {
                uint16_t final = xfer.nbytes == xfer.bytes_xferd;
                write16data(*xfer.buffer++, final);
                xfer.bytes_xferd++;
            }
            xfer.state = Xfer::DATA;
            pio_instance->irq = 1 << state_machine; // clear the interrupt to keep transfer going
            break;
        case Xfer::DATA:
            // TX FIFO is empty; check if any more data to send
            if (xfer.bytes_xferd > xfer.nbytes) {
                pio_i2c_stop();
                xfer.state = Xfer::STOP;
            }
            else {
                while(xfer.bytes_xferd <= xfer.nbytes && !pio_sm_is_tx_fifo_full(pio_instance, state_machine)) {
                    uint16_t final = xfer.nbytes == xfer.bytes_xferd;
                    write16data(*xfer.buffer++, final);
                    xfer.bytes_xferd++;
                }
            }
            pio_instance->irq = 1 << state_machine;  // clear the interrupt to keep transfer going
            break;
        case Xfer::STOP:
            xfer.state = Xfer::IDLE;
            // xfer is complete; disable the PIOn INT0 interrupt request to prevent the transfer from restarting
            // The TX FIFO empty interrupt bit is still set to stall the state machine and trigger this
            // interrupt routine when a new transfer is ready to start
            hw_clear_bits(&pio_instance->inte0, 1<<(state_machine + PIO_IRQ0_INTE_SM0_LSB));
            if (xfer.done_callback)
                (*xfer.done_callback)(xfer.cb_instance, xfer.bytes_xferd);
            break;
        default:
            assert(false);
            break;
    }    
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_irq_handler()
{
    uint32_t mask = 1 << PIO_IRQ0_INTE_SM0_LSB;
    // process the interrupt for each state machine that are triggering it
    for (int sm = 0; sm < 4; sm++, mask <<= 1) {
        if (mask & pio_i2c_irq_mask) {
            // The state machine sm is running PIO I2C code
            if (i2c_port[sm]->pio_instance->ints0 & mask) {
                i2c_port[sm]->pio_sm_irq_handler();
            }
        }
    }
}