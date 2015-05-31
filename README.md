# Phoenard Bootloader
The Phoenard bootloader is a stk500v2-based bootloader for the ATMEGA2560.
Supporting the key functionalities of uploading, verifying and signing on,
the bootloader specializes in the loading and saving of FLASH memory from/to
the Micro-SD card.

Using instructions stored in the back of the device's
EEPROM the bootloader can be told to load a given sketch HEX file in the root
directory of the Micro-SD card. As well, the computer can be used to remotely
access the Micro-SD card through added stk500-based commands. 

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
  * CMD_INIT_SD_ISP(0xE6): Initialize Micro-SD Volume
  * CMD_PROGRAM_SD_ISP(0xE7): Write 512-byte block to M-SD
  * CMD_READ_SD_ISP(0xE8): Read 512-byte block from M-SD
  * CMD_PROGRAM_SD_FAT_ISP(0xE9): Write 512-byte FAT-table to M-SD
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

## Links
* [Our website](http://phoenard.com)
* [Phoenard Arduino library](https://github.com/Phoenard/Phoenard)
* [Phoenard Toolkit bootloader interface](https://github.com/Phoenard/Phoenard-Toolkit)
* [Basic operation instructions](http://phoenard.com/basic-operation/)

## Version history
Current version: v1.00

Version is updated with every major design or feature change.
* v1.00 - Initial version

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
