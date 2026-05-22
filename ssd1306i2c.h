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
class Ssd1306i2c;

/**
 * @class Manage access to a RP2040 I2C port and manages the I2C ports of an attached TCA9548A I2C Mux, if any
 *
 * Each SSD1306 module can support one of 2 I2C addresses, so each I2C port can support
 * up to 2 displays. Each TCA9548A has up to 8 ports.
 * Total maximum displays per I2C0/I2C1 on the RP2040 is 2+(2*8)=18.
 *
 * An application needs to instantiate one instance of this class for each RP2040
 * I2C port it uses for mananaging SSD1306 display modules.
 */
class Ssd1306i2cPort
{
public:
    Ssd1306i2cPort(i2c_inst_t* i2c_port, uint32_t bps, uint8_t sda_gpio, uint8_t scl_gpio, uint8_t mux_addr=0);

    ~Ssd1306i2cPort()= default;

    /**
     * @brief add a Ssd1306i2c display interface object to the list of displays
     *
     * @param display a pointer to the display to register
     * @return the display number of the registered display or -1 if no room
     */
    inline int8_t register_display(Ssd1306i2c* display) {
        lock();
        if (ndisplays >= MAX_DISPLAYS) {
            unlock();
            return -1;
        }
        displays[ndisplays++] = display;
        int8_t ret = ndisplays - 1;
        unlock();
        return ret;
    }

    /**
     * @brief
     *
     * @return true if a mux switch is required
     */
    inline bool is_mux_switch_reqired(int8_t display_number) {return mux_addr != 0 && (display_number != prev_display || prev_display == -1); }

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
    bool is_busy() {return current_display != -1; }
    inline i2c_inst_t* get_i2c() {return i2c;}
    inline void lock() { LOCK(); }
    inline void unlock() {UNLOCK(); }
    inline void lock_from_isr() {LOCK_FROM_ISR(); }
    inline void unlock_from_isr() {UNLOCK_FROM_ISR(); }
protected:
    Ssd1306i2cPort() = delete;
    Ssd1306i2cPort(Ssd1306i2cPort&) = delete;
    i2c_inst_t* i2c;
    int8_t current_display; // -1 if no current transfer, otherwise the display number 0-ndisplays-1
    int8_t prev_display;    // initially -1, then the previous non-negative value of current_display
    Ssd1306i2c* displays[MAX_DISPLAYS];   // A vector of pointers to all displays this Ssd1306i2cPort object manages
    int8_t ndisplays;
    uint8_t mux_addr;       // The address of up to 1 TCA9548A I2C Mux, or 0 if no mux is attached.
    static Ssd1306i2cPort* i2c0_irq_context;
    static Ssd1306i2cPort* i2c1_irq_context;
    static inline void i2c0_irq_handler(void);
    static inline void i2c1_irq_handler(void);
#ifndef OS_FREERTOS
    critical_section_t crit_sec;
#else 
    UBaseType_t uxSavedInterruptStatus;
#endif
};

/**
 * @class manages I2C communication for each connected display
 *
 * This class uses an Ssd1306i2cPort to control access
 */
class Ssd1306i2c : public Ssd1306hw
{
public:
    /**
     * @brief Construct a new i2c ssd1306 object
     * 
     * @param i2c_port a pointer to  I2C port in a struct
     * @param i2c_addr an array of I2C address for each display on the I2C bus.
     * @param mux_map an array of 8-bit bitmaps corresponding to the TCA9548A mux output
     * for each I2C addr. If the entry is 0, the port is not on a mux port but is instead
     * wired directly to the I2C port.
     */
    Ssd1306i2c(Ssd1306i2cPort* i2c_port, const uint8_t i2c_addr, const uint8_t mux_map=0);

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

    inline bool is_error_state() final {return xfer.state == Xfer::ERROR;}
    inline bool is_busy() final { return xfer.state != Xfer::IDLE && xfer.state != Xfer::ERROR; }
    void i2c_irq_handler();
    inline uint8_t get_addr() { return i2c_addr; }
    inline uint8_t get_mux_map() { return mux_map; }
    inline int8_t get_display_num() {return xfer.display_num; }
protected:
    Ssd1306i2c() = delete;
    Ssd1306i2c(Ssd1306i2c&) = delete;
    Ssd1306i2cPort* i2c_port;
    uint8_t i2c_addr;
    const uint8_t mux_map;

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
     * @brief Call write_non_blocking but block until transfer completes
     * 
     * @param regbyte 8-bit register byte; either 0 for SSD1306 commands or 0x40 for display data
     */
    int write_blocking(uint8_t addr, uint8_t regbyte, const uint8_t *src, size_t len);

    bool write_non_blocking(uint8_t addr, uint8_t regbyte_, const uint8_t *src_, int len_, 
            void (*done_callback_)(void* instance, int result), void* instance_);

    bool send_byte_from_task(uint16_t data, bool is_last);

    inline bool is_tx_empty();
    bool is_i2c_error();
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
};
}