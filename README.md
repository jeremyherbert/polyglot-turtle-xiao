# polyglot-turtle-xiao

This repository contains a firmware implementation of a composite USB device.

It is designed to be used with the [seeeduino xiao](http://wiki.seeedstudio.com/Seeeduino-XIAO/) (ATSAMD21G18A). When flashed with the correct firmware, this will be hereafter referred to as a *polyglot-turtleboard* or just *turtleboard*.

**Important notes:**

- The seeeduino xiao supports 0V to 3.3V digital signals. Connecting signals outside this range will likely cause permanent damage to the device.
- For simplicity, the USB to UART baudrate is fixed at 9600. The device will ignore requests from the PC to change this.

## Flashing the polyglot-turtle-xiao firmware

1. Download the firmware file `polygot-turtle-xiao.uf2` from the [releases](https://github.com/jeremyherbert/playful-turtle-xiao/releases) section of this repository.
2. Connect your seeeduino xiao to your computer with a USB C cable.
3. Force your device into bootloader mode. To do this, you need to take a wire and connect the RST and GND pads together **twice** in quick succession. These two pads are located next to the USB connector on the board. See the animation at the end of this list for an example.
4. A new USB storage device will appear connected to your computer (likely called "ARDUINO" or something similar)
5. Copy and paste the `polygot-turtle-xiao.uf2` file onto the new storage device
6. The device will reflash its own firmware and then disconnect the storage device. You're now ready to go! 
7. (Your device may need a power cycle to start correctly the first time; just unplug and replug the USB cable.)

![bootloader-animation](https://github.com/SeeedDocument/Seeeduino-XIAO/raw/master/img/XIAO-reset.gif)

## Usage

### USB to UART
To use the USB to UART converter, simply connect the device to a free USB port on your computer. Drivers should be automatically installed on all platforms that need them. The device will appear in your operating system as a serial port.

## LEDs

There are four LEDs next to the USB connector. 

- The green LED indicates that the board is powered on. 
- The two blue LEDs correspond to UART TX and RX activity.

## Connection information

TBD.