/**
 * @file ssd1306pioi2c.h
 * @brief This class implements I2C communication between the Raspberry Pi
 * RP2040 chip and the SSD1306 using a PIO module instead of one of the I2C
 * interface blocks. It is a C++ port of of the PIO I2C example in pico-examples.
 * Uses BSD 3-clause license from the code it uses. See
 * https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c for original
 * code and https://github.com/raspberrypi/pico-examples/blob/master/LICENSE.TXT
 * for the license.
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
 * Original source
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This source
 * Copyright (c) 2022 rppicomidi
 * same license
 */
#pragma once
#include <cstdint>
#include "i2c.pio.h"
#include "hardware/pio.h"
#include "ssd1306hw.h"
namespace rppicomidi {
class Ssd1306pio_i2c : public Ssd1306hw
{
public:
    /**
     * @brief Construct a new i2c ssd1306 object
     * 
     * @param pio_instance_ the hardware handle for the PIO
     * @param state_machine_ the state machine to execute the I2C
     * @param offset_ the program memory offset
     * @param i2c_addr the I2C address of the display
     * @param sda_gpio the GPIO number of the I2C SDA signal
     * @param scl_gpio //the GPIO number of the I2C SCL signal
     * @param mux_addr the I2C address of the TCA9548A I2C mux chip or 0 if none is used
     * @param mux_map* an array of 8-bit bitmaps corresponding to the TCA9548A mux output
     * for each I2C addr. If the entry is 0, the port is not on a mux port but is instead
     * wired directly to the I2C port.
     */
    Ssd1306pio_i2c(pio_hw_t* pio_instance_, uint state_machine_, uint offset_, uint8_t* i2c_addr, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t ndisplays=1, uint8_t mux_addr=0, uint8_t* mux_map=nullptr);
    virtual ~Ssd1306pio_i2c() = default;
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
    /**
     * @brief Get the PIO program offset
     *
     * This function is used to get the offset from the previously initialized
     * I2C PIO program on state machine 0 so that a new state machine can
     * run the same program for different IO pins.
     *
     * @return the offset value
     */
    uint get_offset() { return offset; }

    bool task() final;

    inline bool is_error_state() final {return task_state == ERROR;}
    inline bool is_busy() final { return task_state != IDLE && task_state != ERROR; }
private:
    Ssd1306pio_i2c() = delete;
    Ssd1306pio_i2c(Ssd1306pio_i2c&) = delete;

    pio_hw_t* pio_instance;
    uint state_machine;
    uint offset;
    uint8_t* i2c_addr;
    uint8_t ndisplays;
    uint8_t mux_addr;
    uint8_t* mux_map;
    uint8_t current_mux_map;
    enum task_state_e {IDLE, REGBYTE, SRCBYTE, ERROR} task_state;
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
     * @brief This is the similar to the pico-example for PIO I2C i2c_write_blocking_internal except
     *  this function sends the regbyte byte before sending all of the data and the return value is
     * len+1 on success, not 0.
     * 
     * @param regbyte 8-bit register byte; either 0 for SSD1306 commands or 0x40 for display data
     */
    int write_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len);

    void send_stop_from_task();
    bool send_byte_from_task(uint16_t data);
    bool write_non_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, int len, void (*done_callback)(void* instance, int result), void* instance_);
    // ----------------------------------------------------------------------------
    // Low-level functions and data copied from the PIO I2C example
    const int PIO_I2C_ICOUNT_LSB = 10;
    const int PIO_I2C_FINAL_LSB  = 9;
    const int PIO_I2C_DATA_LSB   = 1;
    const int PIO_I2C_NAK_LSB    = 0;
    void pio_i2c_rx_enable(bool en);

    void pio_i2c_start();
    void pio_i2c_stop();
    void pio_i2c_repstart();

    bool pio_i2c_check_error();
    void pio_i2c_resume_after_error();

    // If I2C is ok, block and push data. Otherwise fall straight through.
    void pio_i2c_put_or_err(uint16_t data);
    uint8_t pio_i2c_get();
    inline void pio_i2c_put16(uint16_t data) {
        while (pio_sm_is_tx_fifo_full(pio_instance, state_machine))
            ;
        // some versions of GCC dislike this
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *(io_rw_16 *)&pio_instance->txf[state_machine] = data;
    #pragma GCC diagnostic pop
    }

    void pio_i2c_wait_idle() {
        // Finished when TX runs dry or SM hits an IRQ
        pio_instance->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + state_machine);
        while (!(pio_instance->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + state_machine) || pio_i2c_check_error()))
            tight_loop_contents();
    }
};
}