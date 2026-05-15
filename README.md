# pico-ssd1306-mono-graphics-lib
Raspberry Pi Pico SSD1306 graphics library that can manage multiple displays at the same time

There are a lot of SSD1306 mono graphics libraries out there. Why do we need another one? This
library supports driving more than one display at once. It supports multiple displays wired to
a TCA9548A I2C multiplexer chip. It also supports driving multiple displays concurrently at
using the RP2040's built-in I2C ports and the RP2040's PIOs configured as I2C ports. I2C
drivers are interrupt driven so display transfers can take place concurrently. I have tested
this using 9 displays at the same time. Because the I2C data rate is the main limiter of display
update rate, using interrupts and separate I2C ports per display makes updating 9 displays take
about the same time as updating only one. This library runs without an RTOS but is relatively
easy to integrate with FreeRTOS.

This library contains examples in the `examples` directories that show how to use the library with
one display, 2 displays (built-in I2C), 4 displays (PIO), and TODO 9 displays (via an I/O expander).
Some use no RTOS, and some use FreeRTOS.

This code is written in C++. Some of it is licensed under a BSD 3-clause license and some under
a MIT license. See the individual source files for more information. The PIO I2C port firmware was
written in RP2xxx PIO assembly code based on the [PIO I2C example code](https://github.com/raspberrypi/pico-examples/blob/master/pio/i2c/i2c.pio)
modified for I2C transmit only and modified to signal an interrupt when the PIO state machine TX FIFO is empty.

This version was tested using with Pico SDK version 2.2 and the
[Raspberry Pi fork](https://github.com/raspberrypi/FreeRTOS-Kernel) of FreeRTOS
commit 4f7299d6ea746b27a9dd19e87af568e34bd65b15.
