/* MIT License
 *
 * Copyright (c) 2026 rppicomidi
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
#include <cstdlib>
#include <cstdio>
#include <string>
#include "FreeRTOS.h"
#include "queue.h"
#include "pico/stdlib.h"
#include "ssd1306pioi2c.h"
#include "ssd1306.h"
#include "mono_graphics_lib.h"

#ifndef OLED_SCL_GPIO
#define OLED_SCL_GPIO 3
#endif
#ifndef OLED_SDA_GPIO
#define OLED_SDA_GPIO 2
#endif
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C
#endif
#ifndef MUX_ADDR
#define MUX_ADDR 0
#endif
#ifndef MUX_MAP
#if MUX_ADDR != 0
#define MUX_MAP 0x01
#else
#define MUX_MAP 0
#endif
#endif
namespace rppicomidi {

// Override the data_callback() so it can post a message to a queue instead
class Ssd1306_freertos : public Ssd1306 {
public:
    Ssd1306_freertos(Ssd1306hw* port_, uint8_t display_num_, Com_pin_cfg com_pin_config_=Com_pin_cfg::ALT_DIS,
            uint8_t landscape_x_max_ = 128, uint8_t landscape_y_max_=64,
            uint8_t first_column_=0, uint8_t first_page_=0) : Ssd1306(port_, display_num_, com_pin_config_,
                landscape_x_max_, landscape_y_max_, first_column_, first_page_) {}
    virtual void data_callback(int result) final;
private:
    Ssd1306_freertos() = delete;
    Ssd1306_freertos(Ssd1306_freertos&) = delete;
};

class Example {
public:
    // Singleton Pattern

    /**
     * @brief Get the Instance object
     *
     * @return the singleton instance
     */
    static Example& instance()
    {
        static Example _instance; // Guaranteed to be destroyed.
                                             // Instantiated on first use.
        return _instance;
    }
    Example(Example const&) = delete;
    void operator=(Example const&) = delete;

    Example();

    const uint8_t oled_addr=OLED_ADDR;   // the OLED I2C address as a constant
    uint8_t addr[1];                     // the OLED I2C address is stored here
    const uint8_t mux_addr=MUX_ADDR;     // no I2C mux
#if MUX_ADDR == 0
    uint8_t* mux_map=nullptr;            // no I2C mux
#else
    uint8_t mux_map[1] = MUX_MAP;
#endif
    // the i2c driver object
    Ssd1306pio_i2c i2c_driver_oled;

    Ssd1306_freertos ssd1306;    // the SSD1306 driver object
    Mono_graphics oled_screen; // the screen object

    QueueHandle_t display_queue;
    static const uint32_t display_queue_item_size{8};
    enum display_queue_commands{IDLE, ERROR, NEW_TIME};
    static void timer_task(void *param);
    static void display_task(void *param);
    static uint8_t to_bcd(uint8_t val) {
        if (val > 99) {
            return 0x99;
        }
        return ((val/10) << 4) | (val % 10);
    }
    static void int2hex(uint8_t val, char* buffer) {
        buffer[0] = (val >> 4) + '0';
        buffer[1] = (val & 0xF) + '0';
    }
};
}

// This function is called from interrupt context
void rppicomidi::Ssd1306_freertos::data_callback(int result)
{
    uint8_t buffer[rppicomidi::Example::display_queue_item_size];
    buffer[0] = rppicomidi::Example::ERROR;
    if (task_state != DATA) {
        task_state = ERROR; // should only call this from DATA state
    }
    else if (result == static_cast<int>(display_buffer_nbytes + 1)) {
        task_state = IDLE; // done and successful
        buffer[0] = rppicomidi::Example::IDLE;
    }
    else {
        task_state = ERROR; // done and not successful
    }
    xQueueSendToBackFromISR(rppicomidi::Example::instance().display_queue, buffer, nullptr);
}


void rppicomidi::Example::timer_task(void*)
{
    uint8_t buffer[display_queue_item_size];
    memset(buffer, 0, sizeof(buffer));
    buffer[0] = Example::NEW_TIME;
    while(1) {
        vTaskDelay(10);
        if (++buffer[1] > 99) {
            // fraction
            buffer[1] = 0;
            if (++buffer[2] > 59) {
                // seconds
                buffer[2] = 0;
                if (++buffer[3] > 59) {
                    // minutes
                    buffer[3] = 0;
                    if (++buffer[4] > 23) {
                        // hours
                        buffer[4] = 0;
                    }
                }
            }
        }
        auto& dq = Example::instance().display_queue;
        if (dq)
            xQueueSendToBack(dq, buffer, portMAX_DELAY);
    }
}

void rppicomidi::Example::display_task(void*)
{
    uint8_t hour, min, sec, frac;
    char time_str[]={'0','0',':','0','0',':','0','0','.','0','0','\0'};
    uint8_t buffer[display_queue_item_size];
    buffer[0] = IDLE;
    auto& me = Example::instance();
    me.display_queue = xQueueCreate(20, Example::display_queue_item_size);
    me.oled_screen.center_string(me.oled_screen.get_font_24(), "FreeRTOS", 0);
    me.oled_screen.center_string(me.oled_screen.get_font_16(), time_str, me.oled_screen.get_screen_height()/2 - 4);
    me.oled_screen.center_string(me.oled_screen.get_font_16(), "Timer Demo", me.oled_screen.get_screen_height()-17);
    xQueueSendToBack(me.display_queue, buffer, portMAX_DELAY); // render the initial screen
    while(1) {
        auto result = xQueueReceive(me.display_queue, buffer, portMAX_DELAY);
        if (result == pdPASS) {
            if (buffer[0] == NEW_TIME) {
                frac = to_bcd(buffer[1]);
                sec = to_bcd(buffer[2]);
                min = to_bcd(buffer[3]);
                hour = to_bcd(buffer[4]);
                // can render the display
                int2hex(hour, time_str);
                int2hex(min, time_str+3);
                int2hex(sec, time_str+6);
                int2hex(frac, time_str+9);
                me.oled_screen.center_string(me.oled_screen.get_font_16(), time_str, me.oled_screen.get_screen_height()/2 - 4);
                if (me.i2c_driver_oled.is_busy())
                    continue; // Display will render when the display notifies this task that it is idle
            }
            else {
                assert(buffer[0] == IDLE);
            }
            if (me.oled_screen.can_render())
                me.oled_screen.render_non_blocking(nullptr, 0);
        }
        else {
            assert(result);
        }
    }
}

rppicomidi::Example::Example()  : addr{oled_addr},
    i2c_driver_oled{pio0, 0, 400000, addr, OLED_SDA_GPIO, OLED_SCL_GPIO, sizeof(addr), mux_addr, mux_map},
    ssd1306{&i2c_driver_oled, 0, Ssd1306::Com_pin_cfg::ALT_DIS, 128, 64, 0, 0}, // set up the SSD1306 to drive at 128 x 64 oled
    oled_screen{&ssd1306, Display_rotation::Landscape0}, display_queue{nullptr}
{
    oled_screen.clear_canvas();
}

int main()
{
    using namespace rppicomidi;
    TaskHandle_t display;
    TaskHandle_t timer;
    stdio_init_all();
    xTaskCreate(Example::display_task, "display", configMINIMAL_STACK_SIZE, nullptr , configMAX_PRIORITIES-2, &display);
    xTaskCreate(Example::timer_task, "timer", configMINIMAL_STACK_SIZE, nullptr , configMAX_PRIORITIES-1, &timer);
    vTaskCoreAffinitySet(timer, 1);
    vTaskStartScheduler();

    return 0;
}


