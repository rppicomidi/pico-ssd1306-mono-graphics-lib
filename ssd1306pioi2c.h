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
 * Each PIO supports up to 4 I2C ports (1 per state machine). Each I2C port can control up to 18 displays
 * Possible ways to use the driver
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
 * Copyright (c) 2022,2026 rppicomidi
 * same license
 */
#pragma once
#include <cstdint>
#include "i2c-irq.pio.h"
#include "hardware/pio.h"
#include "ssd1306hw.h"
#ifndef OS_FREERTOS
#include "hardware/sync.h"
#include "pico/critical_section.h"
#define LOCK() critical_section_enter_blocking(&crit_sec)
#define UNLOCK() critical_section_exit(&crit_sec)
#define LOCK_FROM_ISR() critical_section_enter_blocking(&crit_sec)
#define UNLOCK_FROM_ISR() critical_section_exit(&crit_sec)
#else
#include "FreeRTOS.h"
#define LOCK() vTaskEnterCritical()
#define UNLOCK() vTaskExitCritical()
#define LOCK_FROM_ISR() uxSavedInterruptStatus = vTaskEnterCriticalFromISR()
#define UNLOCK_FROM_ISR() vTaskExitCriticalFromISR(uxSavedInterruptStatus)
#endif
#ifndef MAX_DISPLAYS
#define MAX_DISPLAYS 18
#endif
namespace rppicomidi {
class Ssd1306pio_i2c_port;
/**
 * @class manage the PIO allocation and the IRQ for each PIO used for SSD1306 control
 * @note there should be at most one instance of this for each PIO in PIO0-PIO2
 */
class Ssd1306pio_i2c_pio_manager
{
public:
    Ssd1306pio_i2c_pio_manager(pio_hw_t* pio_instance_, uint offset_);
    ~Ssd1306pio_i2c_pio_manager()=default;
    pio_hw_t* assign_sm_to_i2c(uint sm, Ssd1306pio_i2c_port* i2c_port, uint sda_gpio, uint scl_gpio);
    //inline void set_irq_enable(uint sm) { hw_set_bits(&pio_instance->inte0, 1<<(sm+PIO_IRQ0_INTE_SM0_LSB)); }
    //inline void clr_irq_enable(uint sm) { hw_clear_bits(&pio_instance->inte0, 1<<(sm+PIO_IRQ0_INTE_SM0_LSB)); }
    //void clear_irq(uint sm) { pio_instance->irq = 1 << sm; }
protected:
    pio_hw_t* pio_instance;
    uint offset;
    Ssd1306pio_i2c_port* ports[4];
    static void pio0_i2c_irq_handler();
    static void pio1_i2c_irq_handler();
    static void pio2_i2c_irq_handler();
    void pio_i2c_irq_handler();
    static Ssd1306pio_i2c_pio_manager* pio0_manager;
    static Ssd1306pio_i2c_pio_manager* pio1_manager;
    static Ssd1306pio_i2c_pio_manager* pio2_manager;
    Ssd1306pio_i2c_pio_manager() = delete;
    Ssd1306pio_i2c_pio_manager(Ssd1306pio_i2c_pio_manager&) = delete;
};

class Ssd1306pio_i2c;
/**
 * @class manage an individual PIO I2C port and up to 1 connected TCA9548A I2C Mux chip
 */
class Ssd1306pio_i2c_port
{
public:
    Ssd1306pio_i2c_port(Ssd1306pio_i2c_pio_manager* pio_manager, uint sm, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t mux_addr=0);
    int8_t register_display(Ssd1306pio_i2c* display);
    inline void irq_handler();

    // Write a 16-bit data word to the state machine TX FIFO
    inline void pio_i2c_put16(uint16_t data) {
        // some versions of GCC dislike this
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(io_rw_16 *)&pio_instance->txf[sm] = data;
        #pragma GCC diagnostic pop
    }
    // An I2C data byte in the TX FIFO upper 6 bits are 0;
    // bit 9 is set if it is the last word sent; bits 8:1 are data;
    // bit 0 is always 1 for reading the stop bit state
    inline void write16data(uint16_t data, uint16_t is_final) {
        pio_i2c_put16((data<<PIO_I2C_DATA_LSB) | (is_final << PIO_I2C_FINAL_LSB) | 1);
    }
    inline uint32_t get_tx_fifo_level() { return ((pio_instance->flevel) >> (8*sm)) & 0xF;}
    inline bool is_tx_fifo_full() {return pio_sm_is_tx_fifo_full(pio_instance, sm); }
    /**
     * @brief
     *
     * @return true if a mux switch is required
     */
    inline bool is_mux_switch_reqired(int8_t display_number) {return mux_addr != 0 && (display_number != prev_display || prev_display == -1); }
    void resume_after_error();
    /**
     * @brief attempt to start a transfer for the display specified by display_number
     *
     * As long as no transfer is in progress, this method enables interrupts that allow
     * the transfer to begin. This function must not be called from interrupt context.
     * If a TCA9548A I2C mux switch is required prior to the normal transfer, this function
     * will set up a mux switch prior to enabling interrupts.
     *
     * You must call this function from a critical section.
     *
     * @param display_number the display number of the display to write to
     */
    bool begin_transfer(int8_t display_number);

    /**
     * @brief ends a transfer for the display specified by display_number
     *
     * Disable I2C interrupts and update current_display and prev_display.
     *
     * This function must be called from interrupt context in a critical section when the transfer
     * is completed to let this class know to clear the transfer and disable interrupts
     *
     * This method does nothing if no transfer is ongoing.
     */
    void end_transfer();
    inline bool is_busy() {return current_display != -1; }
    inline void enable_irq() {hw_set_bits(&pio_instance->inte0, 1<<(sm+PIO_IRQ0_INTE_SM0_LSB)); }
    inline void disable_irq() {hw_clear_bits(&pio_instance->inte0, 1<<(sm+PIO_IRQ0_INTE_SM0_LSB)); }
    inline bool is_error_irq() {return (pio_instance->irq & (0x10 << sm)) != 0; }
    inline void clr_irq() {pio_instance->irq = 1 << sm;}
    inline void lock() { LOCK(); }
    inline void unlock() {UNLOCK(); }
    inline void lock_from_isr() {LOCK_FROM_ISR(); }
    inline void unlock_from_isr() {UNLOCK_FROM_ISR(); }

    void pio_i2c_start();
    void pio_i2c_stop();
    void pio_i2c_repstart();
protected:
    Ssd1306pio_i2c_pio_manager* pio_manager;
    uint sm;
    uint8_t mux_addr;       // The address of up to 1 TCA9548A I2C Mux, or 0 if no mux is attached.
    int8_t current_display; // -1 if no current transfer, otherwise the display number 0-ndisplays-1
    int8_t prev_display;    // initially -1, then the previous non-negative value of current_display
    Ssd1306pio_i2c* displays[MAX_DISPLAYS];   // A vector of pointers to all displays this Ssd1306i2cPort object manages
    int8_t ndisplays;
    pio_hw_t* pio_instance;
    // Low-level functions and data copied from the PIO I2C example
    const int PIO_I2C_ICOUNT_LSB = 10;
    const int PIO_I2C_FINAL_LSB  = 9;
    const int PIO_I2C_DATA_LSB   = 1;
    const int PIO_I2C_NAK_LSB    = 0;
    Ssd1306pio_i2c_port() = delete;
    Ssd1306pio_i2c_port(Ssd1306pio_i2c_port&) = delete;
#ifndef OS_FREERTOS
    critical_section_t crit_sec;
#else 
    UBaseType_t uxSavedInterruptStatus;
#endif
};

/**
 * @class manage I2C communication an individual display attached to an I2C port
 */
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
    Ssd1306pio_i2c(Ssd1306pio_i2c_port* i2c_port_, uint8_t i2c_addr, uint8_t mux_map=0);
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
     * is parameter==nbytes+1. A value of -1 indicates that address write failed. A value of -2
     * indicates an attempt to start a new write when another write was in progress.
     * @return true if the write started with no error
     * @return false if the write start failed
     * @todo make this method support I2C mux chips
     */
    bool write_command_non_blocking(const uint8_t* command, uint8_t nbytes, uint8_t display_num, void (*callback)(void* instance, int result), void* instance_) final;

    /**
     * @brief Write display memory bytes to the SSD1306
     * 
     * @param data a pointer to a uint8_t array containing the display memory data
     * @param nbytes the number of bytes in the data array.
     * @return true if the write was successful
     * @return false if the write failed
     */
    bool write_data(const uint8_t* data, size_t nbytes, uint8_t display_num) final;

    /**
     * @brief Write display memory bytes to the SSD1306 but do
     * not wait for the I2C transfers to complete.
     *
     * @param data a pointer to a uint8_t array containing the display memory data
     * @param nbytes the number of bytes in the data array.
     * @param display_num which of the displays on the same I2C port to update.
     * @param callback points to a function that gets called when the command write completes.
     * The result parameter of the callback is the number of bytes written (may be 0). Success
     * is parameter==nbytes+1. A value of -1 indicates that address write failed. A value of -2
     * indicates an attempt to start a new write when another write was in progress.
     * @return true if the write started with no error
     * @return false if the write start failed
     * @todo make this method support I2C mux chips
     */
    bool write_data_non_blocking(const uint8_t* data, size_t nbytes, uint8_t display_num,
            void (*callback)(void* instance, int result), void* instance);


    bool task() final;

    /**
     * @brief
     *
     * @return true if the PIO state machine is halted due to an error
     */
    inline bool is_error_state() final {return xfer.state == Xfer::ERROR;}

    /**
     * @brief
     *
     * @return true if there is a data or display command transfer in progress
     */
    inline bool is_busy() final { return i2c_port->is_busy() || (xfer.state != Xfer::IDLE && xfer.state != Xfer::ERROR); }

    /**
     * @brief Attempt to recover from an error
     *
     */
    inline void resume_after_error();
    void pio_sm_irq_handler();
    inline uint8_t get_mux_map() {return mux_map; }
protected:
    Ssd1306pio_i2c() = delete;
    Ssd1306pio_i2c(Ssd1306pio_i2c&) = delete;
    Ssd1306pio_i2c_port* i2c_port;
    uint8_t i2c_addr;
    uint8_t mux_map;
    uint8_t current_mux_map;

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

    //void send_stop_from_task();
    //bool send_byte_from_task(uint16_t data);
    bool write_non_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, int len, void (*done_callback)(void* instance, int result), void* instance_);
    // ----------------------------------------------------------------------------

    void pio_i2c_rx_enable(bool en);

    struct Xfer {
        Xfer() : buffer{nullptr}, done_callback{nullptr}, cb_instance{nullptr}, nbytes{0}, bytes_xferd{0}, state{IDLE}, addr{0}, regbyte{0} {}
        const uint8_t* buffer;
        void (*done_callback)(void* instance, int result);
        void* cb_instance;
        uint16_t nbytes;
        uint16_t bytes_xferd;
        enum Xfer_state_e {IDLE, MUX, DATA, STOP, ERROR} state;
        uint8_t addr;
        uint8_t regbyte;    // The register byte; separate from the data buffer
        int8_t display_num;
    } xfer;
    static void pio_i2c_irq_handler();
    static uint32_t pio_i2c_irq_mask;   // bit i is set if state machine i is running PIO I2C code
    //static Ssd1306pio_i2c* i2c_port[4]; // i2c_port[i] points to the Ssd1306pio_i2c object that the IRQ handler uses 
};
}
