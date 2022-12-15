# pico-build


This repo is the source code, and provides instructions, for building the .elf file that is loaded to the RP2040 chip on the Pioreactor hat.


### Building

 - on a Raspberry Pi, follow the setup instructions in Chapter 1. Quick Pico Setup of [the datasheet](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).
 - `mkdir ~/pico/pico-examples/pioreactor && cd ~/pico/pico-examples/pioreactor`
 - Move `main.c` and `CMakeLists.txt` from this repo to the cwd
 - `cp ../../pico-sdk/external/pico_sdk_import.cmake .`
 - `mkdir pioreactor/build && cd build`
 - `export PICO_SDK_PATH=../../../pico-sdk`
 - `cmake .. && make`
 - If a RP2040 is connected over SWD, you can upload with `openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program main.elf verify reset exit"`