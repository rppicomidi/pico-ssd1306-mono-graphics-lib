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

#include "pico/stdlib.h"
#include "ssd1306i2c.h"
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
    uint8_t addr[1];                // the OLED I2C address is stored here
    const uint8_t mux_addr=MUX_ADDR;       // no I2C mux
#if MUX_ADDR == 0
    uint8_t* mux_map = nullptr;       // no I2C mux
#else
    uint8_t mux_map[1];
#endif

    // the i2c driver object
    Ssd1306i2c i2c_driver_oled;

    Ssd1306 ssd1306;    // the SSD1306 driver object
    Mono_graphics oled_screen; // the screen object

    static uint16_t render_done_mask;
    static void callback(uint8_t display_num)
    {
        render_done_mask |= (1u << display_num);
    }
};
}

uint16_t rppicomidi::Example::render_done_mask = 0;

rppicomidi::Example::Example()  : addr{oled_addr}, mux_map{MUX_MAP},
    i2c_driver_oled{i2c1, 400000, addr, OLED_SDA_GPIO, OLED_SCL_GPIO, sizeof(addr), mux_addr, mux_map},
    ssd1306{&i2c_driver_oled, 0, Ssd1306::Com_pin_cfg::ALT_DIS, 128, 64, 0, 0}, // set up the SSD1306 to drive at 128 x 64 oled
    oled_screen{&ssd1306, Display_rotation::Landscape0}                       // set up the screen for rotated landscape orientation
{
    //sleep_ms(10); // Give the display some time to turn on 
    oled_screen.clear_canvas();
}

int main()
{
    using namespace rppicomidi;
    stdio_init_all();
    printf("1 display, built-in I2C port example\r\n");
    Example::instance().oled_screen.center_string(Example::instance().oled_screen.get_font_24(), "0123456789", 0);
    Example::instance().oled_screen.center_string(Example::instance().oled_screen.get_font_16(), "0123456789ABCDEF", 25);
    Example::instance().oled_screen.center_string(Example::instance().oled_screen.get_font_12(), "0123456789ABCDEF01234", 42);
    Example::instance().oled_screen.center_string(Example::instance().oled_screen.get_font_8(), "0123456789ABCDEF012345678", 55);
    Example::instance().render_done_mask = 0;
    Example::instance().oled_screen.render_non_blocking(Example::callback, 0);
    int num_displays = 1;
    uint16_t target_done_mask = (1<<(num_displays)) -1;
    bool success = true;
    while (success && Example::instance().render_done_mask != target_done_mask) {
        if (success) {
            success = Example::instance().oled_screen.task();
        }
    }
    if (!success) {
        printf("Error: raw 0x%08lx\r\n", i2c1->hw->raw_intr_stat);
        printf("tx abort: 0x%0lx\r\n", i2c1->hw->tx_abrt_source);
    }
    assert(success);
    printf("display initialized. Should show characters.\r\n");
    for(;;)
        ;
    return 0;
}
