//**** ATMEL AVR - A P P L I C A T I O N   N O T E  ************************
//*
//* Title:		AVR068 - STK500 Communication Protocol
//* Filename:		command.h
//* Version:		1.0
//* Last updated:	31.01.2005
//*
//* Support E-mail:	avr@atmel.com
//*
//**************************************************************************

// *****************[ ST State message constants ]***********************

const unsigned char ST_START        = 0;
const unsigned char ST_GET_SEQ_NUM  = 1;
const unsigned char ST_MSG_SIZE_1   = 2;
const unsigned char ST_MSG_SIZE_2   = 3;
const unsigned char ST_GET_TOKEN    = 4;
const unsigned char ST_GET_DATA     = 5;
const unsigned char ST_GET_CHECK    = 6;
const unsigned char ST_PROCESS      = 7;

// *****************[ STK message constants ]***************************

const unsigned char MESSAGE_START                        = 0x1B;        //= ESC = 27 decimal
const unsigned char  TOKEN                               = 0x0E;

// *****************[ STK general command constants ]**************************

const unsigned char  CMD_SIGN_ON                         = 0x01;
const unsigned char  CMD_SET_PARAMETER                   = 0x02;
const unsigned char  CMD_GET_PARAMETER                   = 0x03;
const unsigned char  CMD_SET_DEVICE_PARAMETERS           = 0x04;
const unsigned char  CMD_OSCCAL                          = 0x05;
const unsigned char  CMD_LOAD_ADDRESS                    = 0x06;
const unsigned char  CMD_FIRMWARE_UPGRADE                = 0x07;


// *****************[ STK ISP command constants ]******************************

const unsigned char  CMD_ENTER_PROGMODE_ISP              = 0x10;
const unsigned char  CMD_LEAVE_PROGMODE_ISP              = 0x11;
const unsigned char  CMD_CHIP_ERASE_ISP                  = 0x12;
const unsigned char  CMD_PROGRAM_FLASH_ISP               = 0x13;
const unsigned char  CMD_READ_FLASH_ISP                  = 0x14;
const unsigned char  CMD_PROGRAM_EEPROM_ISP              = 0x15;
const unsigned char  CMD_READ_EEPROM_ISP                 = 0x16;
const unsigned char  CMD_PROGRAM_FUSE_ISP                = 0x17;
const unsigned char  CMD_READ_FUSE_ISP                   = 0x18;
const unsigned char  CMD_PROGRAM_LOCK_ISP                = 0x19;
const unsigned char  CMD_READ_LOCK_ISP                   = 0x1A;
const unsigned char  CMD_READ_SIGNATURE_ISP              = 0x1B;
const unsigned char  CMD_READ_OSCCAL_ISP                 = 0x1C;
const unsigned char  CMD_SPI_MULTI                       = 0x1D;

// *****************[ STK PP command constants ]*******************************

const unsigned char  CMD_ENTER_PROGMODE_PP               = 0x20;
const unsigned char  CMD_LEAVE_PROGMODE_PP               = 0x21;
const unsigned char  CMD_CHIP_ERASE_PP                   = 0x22;
const unsigned char  CMD_PROGRAM_FLASH_PP                = 0x23;
const unsigned char  CMD_READ_FLASH_PP                   = 0x24;
const unsigned char  CMD_PROGRAM_EEPROM_PP               = 0x25;
const unsigned char  CMD_READ_EEPROM_PP                  = 0x26;
const unsigned char  CMD_PROGRAM_FUSE_PP                 = 0x27;
const unsigned char  CMD_READ_FUSE_PP                    = 0x28;
const unsigned char  CMD_PROGRAM_LOCK_PP                 = 0x29;
const unsigned char  CMD_READ_LOCK_PP                    = 0x2A;
const unsigned char  CMD_READ_SIGNATURE_PP               = 0x2B;
const unsigned char  CMD_READ_OSCCAL_PP                  = 0x2C;   

const unsigned char  CMD_SET_CONTROL_STACK               = 0x2D;

// *****************[ STK HVSP command constants ]*****************************

const unsigned char  CMD_ENTER_PROGMODE_HVSP             = 0x30;
const unsigned char  CMD_LEAVE_PROGMODE_HVSP             = 0x31;
const unsigned char  CMD_CHIP_ERASE_HVSP                 = 0x32;
const unsigned char  CMD_PROGRAM_FLASH_HVSP              = 0x33;
const unsigned char  CMD_READ_FLASH_HVSP                 = 0x34;
const unsigned char  CMD_PROGRAM_EEPROM_HVSP             = 0x35;
const unsigned char  CMD_READ_EEPROM_HVSP                = 0x36;
const unsigned char  CMD_PROGRAM_FUSE_HVSP               = 0x37;
const unsigned char  CMD_READ_FUSE_HVSP                  = 0x38;
const unsigned char  CMD_PROGRAM_LOCK_HVSP               = 0x39;
const unsigned char  CMD_READ_LOCK_HVSP                  = 0x3A;
const unsigned char  CMD_READ_SIGNATURE_HVSP             = 0x3B;
const unsigned char  CMD_READ_OSCCAL_HVSP                = 0x3C;

// ***************[ Custom commands for Phoenard Bootloader ]***************

const unsigned char  CMD_READ_RAM_BYTE_ISP               = 0xE0;
const unsigned char  CMD_PROGRAM_RAM_BYTE_ISP            = 0xE1;
const unsigned char  CMD_READ_RAM_ISP                    = 0xE2;
const unsigned char  CMD_PROGRAM_RAM_ISP                 = 0xE3;
const unsigned char  CMD_INIT_SD_ISP                     = 0xE6;
const unsigned char  CMD_PROGRAM_SD_ISP                  = 0xE7;
const unsigned char  CMD_READ_SD_ISP                     = 0xE8;
const unsigned char  CMD_PROGRAM_SD_FAT_ISP              = 0xE9;
const unsigned char  CMD_READ_ANALOG_ISP                 = 0xEA;

// *****************[ STK status constants ]***************************

// Success
const unsigned char  STATUS_CMD_OK                       = 0x00;

// Warnings
const unsigned char  STATUS_CMD_TOUT                     = 0x80;
const unsigned char  STATUS_RDY_BSY_TOUT                 = 0x81;
const unsigned char  STATUS_SET_PARAM_MISSING            = 0x82;

// Errors
const unsigned char  STATUS_CMD_FAILED                   = 0xC0;
const unsigned char  STATUS_CKSUM_ERROR                  = 0xC1;
const unsigned char  STATUS_CMD_UNKNOWN                  = 0xC9;

// *****************[ STK parameter constants ]***************************
const unsigned char  PARAM_BUILD_NUMBER_LOW              = 0x80;
const unsigned char  PARAM_BUILD_NUMBER_HIGH             = 0x81;
const unsigned char  PARAM_HW_VER                        = 0x90;
const unsigned char  PARAM_SW_MAJOR                      = 0x91;
const unsigned char  PARAM_SW_MINOR                      = 0x92;
const unsigned char  PARAM_VTARGET                       = 0x94;
const unsigned char  PARAM_VADJUST                       = 0x95;
const unsigned char  PARAM_OSC_PSCALE                    = 0x96;
const unsigned char  PARAM_OSC_CMATCH                    = 0x97;
const unsigned char  PARAM_SCK_DURATION                  = 0x98;
const unsigned char  PARAM_TOPCARD_DETECT                = 0x9A;
const unsigned char  PARAM_STATUS                        = 0x9C;
const unsigned char  PARAM_DATA                          = 0x9D;
const unsigned char  PARAM_RESET_POLARITY                = 0x9E;
const unsigned char  PARAM_CONTROLLER_INIT               = 0x9F;

// *****************[ STK answer constants ]***************************

const unsigned char  ANSWER_CKSUM_ERROR                  = 0xB0;

