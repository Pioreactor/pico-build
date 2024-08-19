# pico-build


This repo is the source code, and provides instructions, for building the .elf file that is loaded to the RP2040 chip (its RAM) on the Pioreactor HAT.


### Building on a Raspberry Pi

 - on a Raspberry Pi, follow the setup instructions in Chapter 1. Quick Pico Setup of [the datasheet](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).
 - `mkdir ~/pico/pico-examples/pioreactor && cd ~/pico/pico-examples/pioreactor`
 - move `main.c` and `CMakeLists.txt` from this repo to the cwd
 - `cp ../../pico-sdk/external/pico_sdk_import.cmake .`
 - `mkdir pioreactor/build && cd build`
 - `export PICO_SDK_PATH=../../../pico-sdk`
 - `cmake .. && make`
 - if a RP2040 is connected over SWD, you can upload with `openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "init" -c "reset halt" -c "load_image main.elf" -c "resume 0x20000000" -c "exit"`


 ### Use

Implements the following i2c API

1. Write to any of the first 4 addresses (0, 1, 2, 3) to set the duty cycle of the corresponding PWM
2. Read from any of the next 4 addresses (4, 5, 6, 7) to get a 16 bit ADC value of the corresponding ADC
2. Read from address 8 to return the version of this code.

Here are some examples of using i2cset and i2cget to interact with this program:

To set the duty cycle of the PWM channel corresponding to address 0 to 75%, you could use the following i2cset command:

    i2cset -y 1 0x2C 0 0xC0

To read the 16-bit ADC value of the ADC channel corresponding to address 4, you could use the following i2cget command:

    i2cget -y 1 0x2C w 4

Note that the -y flag is used to automatically answer yes to any prompt from i2cset or i2cget.
The 1 after the -y flag specifies the i2c bus number to use, and the 0x2C specifies the i2c address of the peripheral.
The 0 and w in the i2cset and i2cget commands, respectively, specify the starting register address to be accessed.
The 4 in the i2cget command specifies the address of the ADC channel to read.
