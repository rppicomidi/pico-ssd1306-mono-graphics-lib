# pico-ssd1306-mono-graphics-lib
Raspberry Pi Pico SSD1306 graphics library that can manage multiple displays at the same time

There are a lot of SSD1306 mono graphics libraries out there. Why do we need another one? This
library supports driving more than one display at once. It supports multiple displays wired to
a TCA9548A I2C multiplexer chip. It also supports driving multiple displays concurrently at
using the RP2040's built-in I2C ports and the RP2040's PIOs configured as I2C ports. I2C
drivers are interrupt driven so display transfers can take place concurrently. I have tested
this using 9 displays at the same time. Because the I2C data rate is the main limiter of display
update rate, using interrupts and separate I2C ports per display makes updating 9 displays take
about the same time as updating only one.

This code is written in C++. Some of it is licensed under a BSD 3-clause license and some under
a MIT license. See the individual source files for more information.

This version builds with Pico SDK version 2.0. If you require support for older versions of
the Pico SDK, please use git version 3680b174e9e088eac6074573bf7aeb702568075e. For example:
```
git checkout 3680b174e9e088eac6074573bf7aeb702568075e
```
