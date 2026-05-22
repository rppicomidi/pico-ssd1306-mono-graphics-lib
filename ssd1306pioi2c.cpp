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
#include <memory.h>

rppicomidi::Ssd1306pio_i2c_pio_manager* rppicomidi::Ssd1306pio_i2c_pio_manager::pio0_manager = nullptr;
rppicomidi::Ssd1306pio_i2c_pio_manager* rppicomidi::Ssd1306pio_i2c_pio_manager::pio1_manager = nullptr;
rppicomidi::Ssd1306pio_i2c_pio_manager* rppicomidi::Ssd1306pio_i2c_pio_manager::pio2_manager = nullptr;

rppicomidi::Ssd1306pio_i2c_pio_manager::Ssd1306pio_i2c_pio_manager(pio_hw_t* pio_instance_, uint offset_) :
    pio_instance{pio_instance_}, offset{offset_}
{
    uint ret = pio_add_program_at_offset(pio_instance, &i2c_irq_program, offset);
    assert(ret == offset);
    memset(ports, 0, sizeof(ports));
    if (pio_instance == pio0) {
        assert(pio0_manager == nullptr);
        pio0_manager = this;
        irq_add_shared_handler(PIO_IRQ_NUM(pio_instance, 0), pio0_i2c_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    }
    else if (pio_instance == pio1) {
        assert(pio1_manager == nullptr);
        pio1_manager = this;
        irq_add_shared_handler(PIO_IRQ_NUM(pio_instance, 0), pio1_i2c_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    }
#if NUM_PIOS > 2
    else {
        assert(pio_instance == pio2);
        assert(pio2_manager == nullptr);
        pio2_manager = this;
        irq_add_shared_handler(PIO_IRQ_NUM(pio_instance, 0), pio2_i2c_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    }
#endif
    irq_set_enabled(PIO_NUM(pio_instance)*2 + PIO0_IRQ_0, true);
}

pio_hw_t*  rppicomidi::Ssd1306pio_i2c_pio_manager::assign_sm_to_i2c(uint sm, Ssd1306pio_i2c_port* i2c_port, uint sda_gpio, uint scl_gpio)
{
    if (sm >= 4) {
        return nullptr;
    }
    pio_sm_claim(pio_instance, sm);
    ports[sm] = i2c_port;
    i2c_irq_program_init(pio_instance, sm, offset, sda_gpio, scl_gpio);

    return pio_instance;
}

void rppicomidi::Ssd1306pio_i2c_pio_manager::pio_i2c_irq_handler()
{
    uint32_t mask = 1 << PIO_IRQ0_INTE_SM0_LSB;
    // process the interrupt for each state machine that is triggering it
    for (int sm = 0; sm < 4; sm++, mask <<= 1) {
        if (ports[sm]) {
            // The state machine sm is running PIO I2C code
            if (pio_instance->ints0 & mask) {
                ports[sm]->irq_handler();
            }
        }
    }
}

void rppicomidi::Ssd1306pio_i2c_pio_manager::pio0_i2c_irq_handler()
{
    pio0_manager->pio_i2c_irq_handler();
}

void rppicomidi::Ssd1306pio_i2c_pio_manager::pio1_i2c_irq_handler()
{
    pio1_manager->pio_i2c_irq_handler();
}

void rppicomidi::Ssd1306pio_i2c_pio_manager::pio2_i2c_irq_handler()
{
    pio2_manager->pio_i2c_irq_handler();
}

rppicomidi::Ssd1306pio_i2c_port::Ssd1306pio_i2c_port(Ssd1306pio_i2c_pio_manager* pio_manager_, uint sm_, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t mux_addr_) :
    pio_manager{pio_manager_}, sm{sm_}, mux_addr{mux_addr_}, current_display{-1}, prev_display{-1}, ndisplays{0}
{
    memset(displays, 0, sizeof(displays));
    pio_instance = pio_manager->assign_sm_to_i2c(sm, this, sda_gpio, scl_gpio);
    assert(pio_instance != nullptr);
}

int8_t rppicomidi::Ssd1306pio_i2c_port::register_display(Ssd1306pio_i2c* display)
{
    if (ndisplays >= MAX_DISPLAYS) {
        return -1;
    }
    displays[ndisplays++] = display;
    return ndisplays - 1;
}

void rppicomidi::Ssd1306pio_i2c_port::irq_handler()
{
    lock_from_isr();
    if (current_display != -1) {
        auto display = displays[current_display];
        unlock_from_isr();
        display->pio_sm_irq_handler();
    }
    else {
        unlock_from_isr();
    }
}
void rppicomidi::Ssd1306pio_i2c_port::resume_after_error()
{
    pio_sm_drain_tx_fifo(pio_instance, sm);
    pio_sm_exec(pio_instance, sm, (pio_instance->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_instance->irq = (0x10u << sm);
}


void rppicomidi::Ssd1306pio_i2c_port::pio_i2c_start() {
    pio_i2c_put16(1u << PIO_I2C_ICOUNT_LSB); // Escape code for 2 instruction sequence
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // We are already in idle state, just pull SDA low
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // Also pull clock low so we can present data
}

void rppicomidi::Ssd1306pio_i2c_port::pio_i2c_stop() {
    pio_i2c_put16(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);    // SDA is unknown; pull it down
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);    // Release clock
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD1]);    // Release SDA to return to idle state
};

void rppicomidi::Ssd1306pio_i2c_port::pio_i2c_repstart() {
    pio_i2c_put16(3u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put16(set_scl_sda_program_instructions[I2C_SC0_SD0]);
}

bool rppicomidi::Ssd1306pio_i2c_port::begin_transfer(int8_t display_number)
{
    if (get_tx_fifo_level() != 0) {
        return false; // the TX FIFO needs to start out empty for this to work right
    }
    prev_display = current_display;
    current_display = display_number;
    if (is_mux_switch_reqired(display_number)) {
        // send the mux switch message first: takes 7 TX FIFO entries
        pio_i2c_start();
        write16data(mux_addr, 0);
        write16data(displays[display_number]->get_mux_map(), 1);
        pio_i2c_stop();
    }
    enable_irq();
    return true;
}

void rppicomidi::Ssd1306pio_i2c_port::end_transfer()
{
    assert(is_busy());

    // Disable the TX_EMPTY IRQ
    disable_irq();
    current_display = -1;
}

rppicomidi::Ssd1306pio_i2c::Ssd1306pio_i2c(Ssd1306pio_i2c_port* i2c_port_, uint8_t i2c_addr_, uint8_t mux_map_) :
    i2c_port{i2c_port_}, i2c_addr{i2c_addr_}, mux_map{mux_map_}
{
    assert(i2c_addr);
    i2c_port->lock();
    xfer.display_num = i2c_port->register_display(this);
    i2c_port->unlock();
}

bool rppicomidi::Ssd1306pio_i2c::write_command(const uint8_t* command_bytes, uint8_t nbytes, uint8_t)
{
    assert(command_bytes);
    assert(nbytes);
    bool success = false;
    success = (write_blocking(i2c_addr, 0x00, command_bytes, nbytes) == nbytes+1);
    return success;
}

bool rppicomidi::Ssd1306pio_i2c::write_command_non_blocking(const uint8_t* command_bytes, uint8_t nbytes, uint8_t,
    void (*callback)(void* instance, int result),void* instance_)
{
    assert(command_bytes);
    assert(nbytes);
    return write_non_blocking(i2c_addr, 0x00, command_bytes, nbytes, callback, instance_);
}

bool rppicomidi::Ssd1306pio_i2c::write_data(const uint8_t* data, size_t nbytes, uint8_t )
{
    assert(data);
    assert(nbytes);
    bool success = false;
    success = (write_blocking(i2c_addr, 0x40, data, nbytes) == (int)(nbytes+1));
    return success;
}


bool rppicomidi::Ssd1306pio_i2c::write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t ,
            void (*callback)(void* instance, int result), void* instance_)
{
    assert(data);
    assert(nbytes);
    return write_non_blocking(i2c_addr, 0x40, data, nbytes, callback, instance_);
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
        uint32_t flevel = i2c_port->get_tx_fifo_level();
        nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
    }
    return nbytes; // 1 for the command byte + nbytes if no error; -1 means even the address write failed.
}

void rppicomidi::Ssd1306pio_i2c::resume_after_error() {
    i2c_port->resume_after_error();
    xfer.state = Xfer::IDLE;
}

void rppicomidi::Ssd1306pio_i2c::pio_i2c_rx_enable(bool )
{
    assert(false); // TX FIFO is JOINED with RX FIFO to make TX FIFO bigger, so this routine should never be called
}


bool rppicomidi::Ssd1306pio_i2c::write_non_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, int len, 
    void (*done_callback)(void* instance, int result), void* instance)
{
    if (i2c_port->is_busy()) {
        // can't interrupt an ongoinxfer.stateion
        if (done_callback)
            (*done_callback)(instance, -2);
        return false;
    }
    invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
    invalid_params_if(I2C, i2c_reserved_addr(addr));
    assert(len == 0 || (len > 0 && src != nullptr));
    assert(xfer.state == Xfer::IDLE);
    i2c_port->lock();
    xfer.addr = addr << 1; // the LSB is 0 for write
    xfer.regbyte = regbyte;
    xfer.bytes_xferd = 0;
    xfer.buffer = src;
    xfer.nbytes = len;
    xfer.done_callback = done_callback;
    xfer.cb_instance = instance;
    if (i2c_port->is_mux_switch_reqired(xfer.display_num))
        xfer.state = Xfer::MUX;
    bool success = i2c_port->begin_transfer(xfer.display_num);
    i2c_port->unlock();
    return success;
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
    if (i2c_port->is_error_irq() || (xfer.state == Xfer::IDLE && (xfer.nbytes >0 && xfer.bytes_xferd != 0))) {
        xfer.state = Xfer::ERROR;
        // prevent the interrupt from firing again; error must be cleared externally
        i2c_port->disable_irq();
        uint32_t flevel = i2c_port->get_tx_fifo_level();
        int nbytes = xfer.bytes_xferd - flevel - 1; // The -1 is because the error occurred when one byte was already removed from the FIFO
        if (xfer.done_callback)
            (*xfer.done_callback)(xfer.cb_instance, nbytes);
        return;
    }
    switch (xfer.state) {
        case Xfer::MUX:
        case Xfer::IDLE:
            i2c_port->pio_i2c_start(); // Only takes 3 words of the FIFO; keep going
            i2c_port->write16data(xfer.addr, 0);
            i2c_port->write16data(xfer.regbyte, xfer.nbytes == 0); // write the command register byte; might be the last transfer if only sending a command byte
            xfer.bytes_xferd++;
            // Fill the TX FIFO with as many data bytes as possible until data is exhausted or TX FIFO is full
            while(xfer.bytes_xferd <= xfer.nbytes && !i2c_port->is_tx_fifo_full()) {
                uint16_t final = xfer.nbytes == xfer.bytes_xferd;
                i2c_port->write16data(*xfer.buffer++, final);
                xfer.bytes_xferd++;
            }
            xfer.state = Xfer::DATA;
            i2c_port->clr_irq(); // clear the interrupt to keep transfer going
            break;
        case Xfer::DATA:
            // TX FIFO is empty; check if any more data to send
            if (xfer.bytes_xferd > xfer.nbytes) {
                i2c_port->pio_i2c_stop();
                xfer.state = Xfer::STOP;
            }
            else {
                while(xfer.bytes_xferd <= xfer.nbytes && !i2c_port->is_tx_fifo_full()) {
                    uint16_t final = xfer.nbytes == xfer.bytes_xferd;
                    i2c_port->write16data(*xfer.buffer++, final);
                    xfer.bytes_xferd++;
                }
            }
            i2c_port->clr_irq();  // clear the interrupt to keep transfer going
            break;
        case Xfer::STOP:
            i2c_port->lock_from_isr();
            xfer.state = Xfer::IDLE;
            // xfer is complete; disable the PIOn INT0 interrupt request to prevent the transfer from restarting
            // The TX FIFO empty interrupt bit is still set to stall the state machine and trigger this
            // interrupt routine when a new transfer is ready to start
            i2c_port->end_transfer();
            i2c_port->unlock_from_isr();
            if (xfer.done_callback)
                (*xfer.done_callback)(xfer.cb_instance, xfer.bytes_xferd);
            break;
        default:
            assert(false);
            break;
    }    
}

#if 0
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
#endif