# Phoenard Bootloader
The Phoenard bootloader is a stk500v2-based bootloader for the ATMEGA2560.
Supporting the key functionalities of uploading, verifying and signing on,
the bootloader specializes in the loading and saving of FLASH memory from/to
the Micro-SD card.

Using instructions stored in the back of the device's
EEPROM the bootloader can be told to load a given sketch HEX file in the root
directory of the Micro-SD card. 

Additional stk500-based commands have been added to read and write I/O registers,
read/write the Micro-SD, control SPI and control the ADC using the computer.
This allows for computer software to use and control the Phoenard with no
requirement to flash the chip.

All of this is done with a clear progress indicator on a ILI9325/ILI9328
TFT LCD screen.

## Added features
These features have been added to the stk500v2 protocol spec:
* Minimal Micro-SD library used to navigate FAT16/FAT32 filesystems
* Intel HEX/Binary parsing/saving logic
* ILI9325/ILI9328 LCD register initialization
* LCD command progress indicator
* SELECT-button pin used to load the default 'SKETCHES.HEX'
* EEPROM-based instruction bus for loading programs
  * Keeps track of current program name
  * Keeps track of current program size
  * Keeps track of current program modification state
  * Automatically saves previous modified program when loading
* New stk500v2 commands added for accessing Micro-SD:
  * CMD_READ_RAM_BYTE_ISP(0xE0): Read a single byte of RAM memory
  * CMD_PROGRAM_RAM_BYTE_ISP(0xE1): Write a single byte to RAM memory
  * CMD_READ_RAM_ISP(0xE2): Read a block of RAM memory
  * CMD_PROGRAM_RAM_ISP(0xE3): Write a block of RAM memory
  * CMD_INIT_SD_ISP(0xE6): Initialize Micro-SD Volume
  * CMD_PROGRAM_SD_ISP(0xE7): Write 512-byte block to M-SD
  * CMD_READ_SD_ISP(0xE8): Read 512-byte block from M-SD
  * CMD_PROGRAM_SD_FAT_ISP(0xE9): Write 512-byte FAT-table to M-SD
  * CMD_READ_ANALOG_ISP(0xEA): Setup and read ADC registers
  * CMD_TRANSFER_SPI_ISP(0xEB): Write to SPI and response full-duplex
  * CMD_MULTISERIAL_ISP(0xEC): Pipe communication between two Serial UART
* Under 8192 program flash size to fit in ATMEGA2560 bootloader area

## Removed features
These features from the stk500v2 protocol spec are not supported:
* Lock/Fuse bit programming not supported
* Monitor mode not available
* HVSP programming not supported
* Flash wiping not supported
* Not all STK parameter commands are supported

## Compilation notes
Please see the main phoenboot.cpp for important build parameters.
This project is developed and compiled using Atmel Studio (6.2).
The compiled HEX file of the bootloader can be found in the Debug folder.

For final release versions, we make use of the Arduino AVR GCC Compiler.
Compared to Atmel's toolchain, there is a significant difference in size.
You can make use of the Release folder's Makefile to build the project.
Unless the paths already match up, you will need to pass the AVR path.

To build the release firmware, use the following make command while inside
the Release folder:

```
make all AVR_PATH="C:/Program Files/Arduino/hardware/tools/avr/bin"
```

## Links
* [Our website](http://phoenard.com)
* [Phoenard Arduino library](https://github.com/Phoenard/Phoenard)
* [Phoenard Toolkit bootloader interface](https://github.com/Phoenard/Phoenard-Toolkit)
* [Basic operation instructions](http://phoenard.com/basic-operation/)
* [Latest build](http://builds.phoenard.com/firmware-latest.zip)
* [Past builds](http://builds.phoenard.com/build_firmware/)

## Versioning
Current version: 1.2

Past versions:
* v1.2 - Added multi-serial command and communication over WiFi/Bluetooth
* v1.1 - SPI transfer command fixed; SPI de-initialized on sketch boot
* v1.0 - Initial implementation

## License

The MIT License (MIT)

Copyright (c) 2015 Phoenard

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
