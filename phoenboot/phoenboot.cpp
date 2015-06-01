/*
The MIT License (MIT)

This file is part of the Phoenard Bootloader (phoenboot)
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
*/

/*****************************************************************************
* Created: 12/04/2014 10:14:22
* Author: Irmo van den Berge (Phoenard)
*
* Based upon the wiring stk500v2 bootloader.
*
* This is a bootloader following a tweaked version of the stk500v2 protocol.
* It maintains limited compatibility with the original full stk500v2 protocol.
*
* Important differences:
* - stk500v2 commands have been added to read/write the Micro-SD card.
* - no support for monitoring/fuse writing
* - command buffer size is 512 bytes instead of 256 bytes
* - initialization delay differs due to Micro-SD/Screen initialization
* - command response times will differ due to an altered process loop
* - automatic loading/saving of FLASH memory from/to Micro-SD using eeprom instructions
* - SELECT key input can be used to load the 'sketches.hex' file
* 
* What originally started as a modified WIRING bootloader ended up a complete rewrite.
* Here is a changelog of changes performed from the developer perspective.
*
* Code size optimizations have been performed:
* - Inlining single-use functions, moving multi-use code into functions
* - Repetitive sendchar calls when sending msg, replaced with msgBuffer_full padded with data at the start
* - Goto call to move to the main program, removed bootloader_status
* - Removal of the monitor and SET_LOCK_BITS commands
* - Read/writing flash moved to functions, used in more places than one
* - Removal of many #if(def) lines to make code more readable and manipulatable for our purposes
* - All code that was not meant to run on the ATMEGA2560 has been removed to simplify it
* - Heavy preference of code jumping over function calls to shrink code size to the absolute minimum
*
* Functionality has been added to fit the cleared space:
* - Loading a program from a HEX/BIN file on SD, using self-written minimal SD library
* - Saving a program to a HEX/BIN file on SD, using self-written minimal SD library
* - EEPROM settings at the back used to pass instructions to the bootloader (loaded file, to load file and status byte)
* - Display (port and register) initialization
*
* Compilation notes:
* - Pass -DF_CPU=16000000UL compiler option to set CPU frequency
* - Pass -fno-jump-tables compiler option so that goto functionality works correctly
* - Pass -finline-limit=9999 compiler option to make sure single-use large functions get inlined properly
* - Pass -Wl,--section-start=.text=3E000 linker option to write code into bootloader section of FLASH
* - Setup compiler options to optimize for size
*/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/common.h>
#include <stdlib.h>
#include "PHNSDMinimal.h"
#include "PHNSettings.h"
#include "display.h"
#include "command.h"

/*
 * UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */
#define BAUDRATE 115200

/*
 * The byte count stored in each individual HEX line in hex files produced when saving
 * For reading, any count 1-255 is supported.
 */
#define HEX_FORMAT_BYTECOUNT 16

/* Default Phoenard settings to be used by default */
#define SETTINGS_DEFAULT_SKETCH        {'S', 'K', 'E', 'T', 'C', 'H', 'E', 'S'}
#define SETTINGS_DEFAULT_FLAGS         (SETTINGS_LOAD | SETTINGS_CHANGED | SETTINGS_TOUCH_VER_INV)
#define SETTINGS_DEFAULT_TOUCH_HOR_A   138
#define SETTINGS_DEFAULT_TOUCH_HOR_B   171
#define SETTINGS_DEFAULT_TOUCH_VER_A   134
#define SETTINGS_DEFAULT_TOUCH_VER_B   115

/* Default Phoenard settings, used to restore corrupted or wiped EEPROM */
PHN_Settings SETTINGS_DEFAULT = {
  SETTINGS_DEFAULT_TOUCH_HOR_A, SETTINGS_DEFAULT_TOUCH_HOR_B,
  SETTINGS_DEFAULT_TOUCH_VER_A, SETTINGS_DEFAULT_TOUCH_VER_B,
  SETTINGS_DEFAULT_SKETCH, SETTINGS_DEFAULT_SKETCH, 0, SETTINGS_DEFAULT_FLAGS
};

/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0x00
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0x00
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           0x02
#define CONFIG_PARAM_SW_MINOR           0x0A

/* Hardware signature bytes for ATMEGA2560 */
static const unsigned char SIGNATURE_BYTES[3] = {0x1E, 0x98, 0x01};
  
/* Sign-on response identifier */
static const char SIGNATURE_NAME[] = "8AVRISP_2";

/*
 * Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
 */
#define BOOTSIZE   8192
#define APP_START  0x00000000
#define APP_END    (FLASHEND -(2*BOOTSIZE) + 1)

/* ATMega with two USART, use UART0 */
#define UART_BAUD_RATE_LOW       UBRR0L
#define UART_STATUS_REG          UCSR0A
#define UART_CONTROL_REG         UCSR0B
#define UART_ENABLE_TRANSMITTER  TXEN0
#define UART_ENABLE_RECEIVER     RXEN0
#define UART_TRANSMIT_COMPLETE   TXC0
#define UART_RECEIVE_COMPLETE    RXC0
#define UART_DATA_REG            UDR0
#define UART_DOUBLE_SPEED        U2X0

/*
 * Macro to calculate UBBR from XTAL and baudrate
 */
#define UART_BAUD_SELECT (((float)(F_CPU))/(((float)(BAUDRATE))*8.0)-1.0+0.5)

/*
 * How many cycles to wait before booting the program
 * Using a single set bit in the counter allows for size optimizations
 * Start multiplier must be a power of 2
 * Start divider can be any value
 */
#define BOOT_START_MULT (1UL << 20)
#define BOOT_START_DIV  1

/* Swaps two variables using an intermediate variable */
#define SWAP_VAR(c, a, b)   {c = a; a = b; b = c;}

/* Enables access to memory treated as an unsigned int */
#define PTR_TO_WORD(p) (*((unsigned int*) (p)))

/* Address type */
typedef uint32_t address_t;

/* Eases the transformation from two unsigned bytes to unsigned int */
typedef union {
  unsigned char bytes[2];
  uint16_t value;
} uint16_pack_t;

/* Eases the transformation from four unsigned bytes to unsigned long */
typedef union {
  unsigned char bytes[4];
  uint32_t value;
} uint32_pack_t;

/*
 * function prototypes
 */
void flash_write_page(address_t address, const unsigned char* p);
void loadSketch(PHN_Settings *boot_flags);
void saveSketch(PHN_Settings *boot_flags);
void saveBootflags(PHN_Settings *boot_flags);

//*****************************************************************************

void flash_write_page(address_t address, const unsigned char* p) {
  /* Memory protection */
  if (address >= APP_END || address & 0xFF) return;

  /* Erase memory; wait for erasing to complete */
  boot_page_erase(address);
  boot_spm_busy_wait();

  /* Write flash memory data */
  const unsigned char* p_end = p + SPM_PAGESIZE;
  do {
    boot_page_fill(address, PTR_TO_WORD(p));
    address  += 2;
    p        += 2;
  } while (p != p_end);

  /* Wait for writing to finish and re-enable RWW */
  boot_page_write(address - SPM_PAGESIZE);
  boot_spm_busy_wait();
  boot_rww_enable();
}

//************************************************************************

/* Waits for a byte to become available on UART */
static uint8_t waitForData() {
  unsigned long boot_count = 0;
  while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE))) {
    if ((boot_count+=BOOT_START_DIV) & BOOT_START_MULT) return 0;
  }
  return 1;
}

//*****************************************************************************

//*  for watch dog timer startup
void (*app_start)(void) = APP_START;

//*****************************************************************************

int main(void) {
  address_t address;
  unsigned char  msgParseState;
  unsigned int   msgLength_st1;
  unsigned char  checksum = 0;
  unsigned int   checksum_idx;
  unsigned int   msgLength = 0;
  unsigned char  msgBuffer_full[600];
  unsigned char* msgBuffer = (msgBuffer_full + 5);
  unsigned char  msgStatus;
  unsigned char  c, *p;
  unsigned char  isLeave;
  unsigned char  isCommandProcessed;
  unsigned char  iconFlags;
  uint16_pack_t  uint16_pack;
  uint32_pack_t  uint32_pack;
  PHN_Settings    boot_flags;

  //************************************************************************
  //*  Dec 29,  2011  <MLS> Issue #181, added watch dog timer support
  //*  handle the watch dog timer
  uint8_t  mcuStatusReg;
  mcuStatusReg  =  MCUSR;

  asm volatile ("cli");
  asm volatile ("wdr");
  MCUSR   =  0;
  WDTCSR |=  _BV(WDCE) | _BV(WDE);
  WDTCSR  =  0;
  asm volatile ("sei");
  // check if WDT generated the reset, if so, go straight to app
  if (mcuStatusReg & _BV(WDRF)) {
    app_start();
  }
  //************************************************************************
  /* Set home button to proper INPUT */
  SELECT_DDR &= ~SELECT_MASK;
  SELECT_PORT |= SELECT_MASK;

  /*
   * Init UART
   * set baudrate and enable USART receiver and transmitter without interrupts
   */
  UART_STATUS_REG    |=  (1 <<UART_DOUBLE_SPEED);
  UART_BAUD_RATE_LOW  =  UART_BAUD_SELECT;
  UART_CONTROL_REG    =  (1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

  /* Read boot settings from EEPROM */
  PHN_Settings_Load(boot_flags);

  /*
   * If the current filename is corrupted, reset
   * This prevents SD corruption issues caused by saving
   */
  for (unsigned char i = 0; i < 8; i++) {
    unsigned char c = boot_flags.sketch_current[i];
    if (c < 32 || c >= 127) {
      boot_flags = SETTINGS_DEFAULT;
      break;
    }
  }

  /* Initialize LCD Screen */
  LCD_init();

  /* Go here to start the bootloader, past the initialization */
bootloader:

  /* If HOME button is held down, reset sketch */
  if (!(SELECT_IN & SELECT_MASK)) {
    memcpy(boot_flags.sketch_toload, SETTINGS_DEFAULT.sketch_toload, 8);
    boot_flags.flags |= SETTINGS_LOAD | SETTINGS_CHANGED;
    saveBootflags(&boot_flags);
  }

  /* 0x1 = Sketch needs to be loaded from SD */
  if (boot_flags.flags & SETTINGS_LOAD) {
    /* Save the sketch if modified */
    saveSketch(&boot_flags);

    /* Load the sketch */
    loadSketch(&boot_flags);

    /* Directly go to the program when loaded - bootloader has timed out anyway */
    goto program;
  }

  /* Bootloader logic starts here */
  isLeave = 0;
  address = APP_START;
  while (!isLeave) {
    checksum = 0;
    msgParseState = ST_START;
    msgLength_st1 = 0;

    /* First stage: complete a full message */
    isCommandProcessed = 0;
    do {
      /* Wait for data to be available on UART, on timeout go to program */
      if (!waitForData()) goto program;

      /* Receive next byte of data */
      c = UART_DATA_REG;

      /* Update the checksum */
      checksum    ^=  c;

      switch (msgParseState) {
        case ST_START:
          if (c == MESSAGE_START) {
            msgParseState = ST_GET_SEQ_NUM;
          }
          break;

        case ST_GET_SEQ_NUM:
          /* Set sequence number in response buffer directly */
          msgBuffer_full[1] = c;
          msgParseState  =  ST_MSG_SIZE_1;
          break;

        case ST_MSG_SIZE_1:
          msgLength      =  c<<8;
          msgParseState  =  ST_MSG_SIZE_2;
          break;

        case ST_MSG_SIZE_2:
          msgLength     |=  c;
          msgParseState  =  ST_GET_TOKEN;
          break;

        case ST_GET_TOKEN:
          /* Restart if token check fails */
          if (c != TOKEN) {
            goto bootloader;
          }
          msgParseState = ST_GET_DATA;
          break;

        case ST_GET_DATA:
          msgBuffer[msgLength_st1++] = c;
          if (msgLength_st1 == msgLength) {
            msgParseState = ST_GET_CHECK;
          }
          break;

        case ST_GET_CHECK:
          /*
           * Check if the checksum matches
           * If checksum ^ expected == 0 it was correct
           * Nonzero indicates the checksum was incorrect
           * If this passes, continue processing the message data
           * If this fails, restart this stage from the beginning
           */
          if (checksum) {
            goto bootloader;
          } else {
            isCommandProcessed = 1;
          }
      }
    } while (!isCommandProcessed);

    /* 
     * Optimization: These values are used for almost every response
     * Put here and used before sending to save a few bytes of program memory
     */
    msgStatus = STATUS_CMD_OK;
    msgLength = 2;
    iconFlags = ICON_PCIDLE;

    /*
     * Now process the STK500 commands, see Atmel Appnote AVR068
     */    
    switch (msgBuffer[0]) {
      case CMD_SPI_MULTI:
        {
          unsigned char answerByte;
          unsigned char mode;

          if (msgBuffer[4]== 0x30) {
            answerByte = SIGNATURE_BYTES[msgBuffer[6]];
          } else if (msgBuffer[4] & 0x50) {
            if (msgBuffer[4] == 0x58) {
              mode = GET_HIGH_FUSE_BITS;
            } else {
              mode = GET_LOW_FUSE_BITS;
            }
            answerByte = boot_lock_fuse_bits_get(mode);
          } else {
            /* Default value for unknown requests */
            answerByte = 0;
          }
          /* Send a response with the requested byte */
          msgLength     =  7;
          msgBuffer[2]  =  0;
          msgBuffer[3]  =  msgBuffer[4];
          msgBuffer[4]  =  0;
          msgBuffer[5]  =  answerByte;
        }
        break;

      case CMD_SIGN_ON:
        msgLength = sizeof(SIGNATURE_NAME) + 2;
        memcpy(msgBuffer + 2, SIGNATURE_NAME, sizeof(SIGNATURE_NAME));
        break;

      case CMD_GET_PARAMETER:
        {
          unsigned char value;

          switch(msgBuffer[1]) {
            case PARAM_BUILD_NUMBER_LOW:
              value = CONFIG_PARAM_BUILD_NUMBER_LOW;
              break;
            case PARAM_BUILD_NUMBER_HIGH:
              value = CONFIG_PARAM_BUILD_NUMBER_HIGH;
              break;
            case PARAM_HW_VER:
              value = CONFIG_PARAM_HW_VER;
              break;
            case PARAM_SW_MAJOR:
              value = CONFIG_PARAM_SW_MAJOR;
              break;
            case PARAM_SW_MINOR:
              value = CONFIG_PARAM_SW_MINOR;
              break;
            default:
              value = 0;
              break;
          }
          msgLength    = 3;
          msgBuffer[2] = value;
        }
        break;

      case CMD_LEAVE_PROGMODE_ISP:
        msgLength = 2;
        isLeave   = 1;
        break;

      case CMD_SET_PARAMETER:
      case CMD_ENTER_PROGMODE_ISP:
        msgLength  =  2;
        break;

      case CMD_READ_SIGNATURE_ISP:
        msgLength     =  4;
        msgBuffer[2]  =  SIGNATURE_BYTES[msgBuffer[4]];
        msgBuffer[3]  =  STATUS_CMD_OK;
        break;

      case CMD_READ_LOCK_ISP:
      case CMD_READ_FUSE_ISP:
        {
          unsigned char mode;
          if (msgBuffer[0] == CMD_READ_LOCK_ISP) {
            mode = GET_LOCK_BITS;
          } else if (msgBuffer[2] == 0x50) {
            if (msgBuffer[3] == 0x08) {
              mode = GET_EXTENDED_FUSE_BITS;
            } else {
              mode = GET_LOW_FUSE_BITS;
            }
          } else {
            mode = GET_HIGH_FUSE_BITS;
          }
          msgLength     =  4;
          msgBuffer[2]  =  boot_lock_fuse_bits_get(mode);
          msgBuffer[3]  =  STATUS_CMD_OK;
        }
        break;

      case CMD_CHIP_ERASE_ISP:
        /* Chip erasing not supported */
        msgLength = 2;
        msgStatus = STATUS_CMD_FAILED;
        break;

      case CMD_LOAD_ADDRESS:
        /* Reverse the byte order, use intermediate 32-bit to 4 byte struct */
        uint32_pack = {msgBuffer[4], msgBuffer[3], msgBuffer[2], msgBuffer[1]};
        address = uint32_pack.value;
        msgLength = 2;
        break;

      case CMD_READ_SD_ISP:
      case CMD_PROGRAM_SD_ISP:
      case CMD_PROGRAM_SD_FAT_ISP:
      case CMD_READ_FLASH_ISP:
      case CMD_READ_EEPROM_ISP:
      case CMD_PROGRAM_FLASH_ISP:
      case CMD_PROGRAM_EEPROM_ISP:
        {
          /* Read the to read/write size by swapping the two bytes and copying the memory */
          uint16_pack = {msgBuffer[2], msgBuffer[1]};
          uint8_t sdAccessed = 0;
          unsigned int size = uint16_pack.value;
          msgLength = 2;

          /* Writing or reading? */
          if (msgBuffer[0] & 0x1) {
            unsigned char *p = (msgBuffer + 10);

            switch (msgBuffer[0]) {
              case CMD_PROGRAM_FLASH_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM;

                  /* Change address from word-space to byte-space */
                  address <<= 1;

                  /* Write next page of program memory */
                  flash_write_page(address, p);
                  p += size;

                  /* 
                   * Update program size, reset size when this is the first update
                   * On some programmers it pads the end of the program with 0xFFFF data
                   * We must make sure to exclude that data from the program size
                   */
                  if (!(boot_flags.flags & SETTINGS_CHANGED) || address >= boot_flags.sketch_size) {
                    /* Calculate program size excluding trailing 0xFFFF */
                    unsigned int size_tmp = size;
                    do {
                      p -= 2;
                      if (PTR_TO_WORD(p) != 0xFFFF) {
                        break;
                      }
                      size_tmp -= 2;
                    } while (size_tmp);

                    boot_flags.sketch_size = address + size_tmp;
                    boot_flags.flags |= SETTINGS_MODIFIED | SETTINGS_CHANGED;
                  }

                  /* Increment address by the amount of data written out */
                  address += size;

                  /* Change address back to word-space */
                  address >>= 1;
                }
                break;

              case CMD_PROGRAM_EEPROM_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM;

                  /* Write EEPROM, full block */
                  eeprom_write_block(p, (void*) ((uint16_t) address), size);
                  address += size;
                }
                break;

              case CMD_PROGRAM_SD_FAT_ISP:
                /* If writing FAT, it's the same logic, but handle mirror FAT */
                volume_cacheFATMirror = volume.isMultiFat;
              case CMD_PROGRAM_SD_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD;

                  /* Write Micro-SD, full block */
                  memcpy(volume_cacheBuffer.data, p, size);
                  volume_writeCache(address);
                  address++;
                  sdAccessed  =1;
                }
                break;
            }
          } else {
            unsigned char *p = (msgBuffer + 2);
            msgLength += size;
            switch (msgBuffer[0]) {
              case CMD_READ_FLASH_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM | ICON_PROGRESS_INVERT;
                                    
                  /* Read FLASH, half the size to convert from byte to word space */
                  size >>= 1;
                  do {
                    /* Read word in memory, copy to message buffer and select next word */
                    PTR_TO_WORD(p) = pgm_read_word_far(address << 1);
                    p += 2;
                    address++;
                  } while (--size);
                }
                break;

              case CMD_READ_EEPROM_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM | ICON_PROGRESS_INVERT;

                  /* Read EEPROM, full block */
                  eeprom_read_block(p, (void*) ((uint16_t) address), size);
                  address += size;
                }
                break;

              case CMD_READ_SD_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD | ICON_PROGRESS_INVERT;

                  /* Read Micro-SD, full block */
                  volume_readCache(address);
                  memcpy(p, volume_cacheBuffer.data, size);
                  address++;
                  sdAccessed = 1;
                }
                break;
            }
          }

          /* SD was accessed and something went wrong, return FAIL */
          if (sdAccessed && !volume.isInitialized) {
            msgLength = 2;
            msgStatus = STATUS_CMD_FAILED;
          }
        }
        break;

      case CMD_INIT_SD_ISP:
        {
            /* Set device icon */
            iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD | ICON_PROGRESS_INVERT;

            /* Ensure card is initialized by opening an arbitrary (non-existent) file */
            volume.isInitialized = 0;
            const char name_none[1] = {0};
            file_open(name_none, name_none, FILE_READ);

            /* Respond with all known volume variables */
            msgLength = 2 + sizeof(CardVolume);
            memcpy(msgBuffer + 2, &volume, sizeof(CardVolume));
        }
        break;

      default:
        msgLength = 2;
        msgStatus = STATUS_CMD_FAILED;
        break;
    }

    /*
     * Now send answer message back
     */
    /* Set the header of the message to send */
    msgBuffer_full[0] = MESSAGE_START;    
    msgBuffer_full[4] = TOKEN;
    msgBuffer_full[6] = msgStatus;

    /* Initialize the display, draw icon of current file */
    /* For some reason, putting this after the above 4 lines reduces binary size... */
    LCD_write_frame(iconFlags, boot_flags.sketch_current);

    /* Copy the message length data over, then swap the two bytes */
    memcpy(msgBuffer_full + 2, &msgLength, 2);
    SWAP_VAR(c, msgBuffer_full[2], msgBuffer_full[3]);

    /* Store the index in the msgBuffer where checksum is stored */
    checksum_idx = msgLength;

    /* Add header + checksum length to message length */
    msgLength += 6;

    /* Set initial checksum to 0 */
    msgBuffer[checksum_idx] = 0;

    /* Send the message, update checksum as we send */
    p = msgBuffer_full;
    do {
      UART_DATA_REG = *p;             /* Start transmission of next byte */
      msgBuffer[checksum_idx] ^= *p;  /* Update checksum */
      p++;                            /* Next byte */
      msgLength--;                    /* Update length to send */
      
      /* Complete transmission and clear TX flag */
      while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE)));
      UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);
    } while (msgLength);
  }

  /* Post-programming program label to jump to, skipping the main bootloader */
program:

  /* Save bootloader flags */
  saveBootflags(&boot_flags);   

  /* If no program available, go back to the bootloader */
  if (pgm_read_word_far(APP_START) == 0xFFFF) {
    LCD_write_frame(ICON_FROM_NONE | ICON_TO_CHIPROM, boot_flags.sketch_current);
    goto bootloader;
  }

  /* ===== Leaving the bootloader ===== */
  /* ================================== */

  /* Prepare the LCD screen for further program use */
  LCD_write_frame(ICON_FROM_NONE | ICON_TO_NONE, NULL);

  /* Turn off UART */
  UART_STATUS_REG &= 0xFD;

  /* Turn off Micro-SD SPI */
  SD_CS_PORT |= SD_CS_MASK;

  /* Jump to the program at address 0 */
  asm volatile("clr  r30 \n"
               "clr  r31 \n"
               "ijmp \n");

  /* This point is never reached, but to reduce size, infinite loop here */
  for(;;);
}

void saveBootflags(PHN_Settings *boot_flags) {
  /* Write bootloader flags to EEPROM when updated */
  if (boot_flags->flags & SETTINGS_CHANGED) {
    boot_flags->flags &= ~SETTINGS_CHANGED;
    PHN_Settings_Save(*boot_flags);
  }
}

void loadSketch(PHN_Settings *boot_flags) {
  /* Store old flags, then reset flags to remove loading flag */
  uint8_t oldFlags = boot_flags->flags;
  boot_flags->flags &= ~(SETTINGS_LOAD | SETTINGS_LOADWIPE);
  boot_flags->flags |= SETTINGS_CHANGED;

  /* Swap current sketch with the one to load, this way you can go 'back' at any time */
  unsigned char i;
  char c;
  for (i = 0; i < 8; i++) {
    SWAP_VAR(c, boot_flags->sketch_current[i], boot_flags->sketch_toload[i]);
  }

  /* Initialize the frame, draw icon to load. Force it. */
  LCD_write_frame(ICON_FROM_SD | ICON_TO_CHIPROM | ICON_DRAW_SKETCH, boot_flags->sketch_current);

  /* Open the file on SD */
  /* Don't load anything if WIPE flag is specified */
  address_t address = APP_START;
  if (!(oldFlags & SETTINGS_LOADWIPE) && file_open(boot_flags->sketch_current, "HEX", FILE_READ)) {
    /* Check the first 12 bytes to see if they are all HEX (0-9 A-F) data */
    /* If the file is empty either mode could be used since it reads random memory */
    /* This does not matter because in either mode empty data is handled properly */
    uint8_t* isHexData = volume_cacheCurrentBlock(0);
    for (i = 0; i < 12; i++) {
      if (isHexData[i] < 0x30 || isHexData[i] > 0x46) {
        isHexData = 0;
        break;
      }
    }

    /* Perform loading from SD */
    unsigned char length;
    unsigned char recordtype;
    unsigned char data_buff[SPM_PAGESIZE*2];
    unsigned char data_page_buffer[SPM_PAGESIZE*2];
    unsigned int data_page_buffer_len = 0;
    unsigned long file_size = file_available;
    while (file_position < file_size) {
      LCD_write_progress(file_position, file_size, STATUS_COLOR_SDLOAD);

      if (isHexData == 0) {
        /* Raw reading from the SD in blocks of 256 bytes at a time */
        flash_write_page(address, (unsigned char*) file_read(SPM_PAGESIZE));
        address += SPM_PAGESIZE;
        boot_flags->sketch_size = file_size;
      } else {
        /* Reading Intel HEX format */
        file_read_hex_line(data_buff);
        length = data_buff[0];
        recordtype = data_buff[3];

        /* Validate that the data is valid using the provided CRC */
        unsigned int crc_expected = data_buff[length + 4];
        unsigned int crc_read = 0;
        for (int i = 0; i < (length+4); i++) crc_read += data_buff[i];
        crc_read = (~crc_read + 1) & 0xFF;
        if (crc_expected != crc_read) {
          break;
        }

        /* Read the address part of data hex lines */
        if (!data_page_buffer_len && !recordtype) {
          address = (address & 0xFF0000) | (((address_t) data_buff[1] << 8) | (address_t) data_buff[2]);
        }

        /* Extended data segment, we ignore data_buff[5] (out of range) */
        if (recordtype == 0x2) {
          address = (address & 0xFFFF) | ((address_t) data_buff[4] << 12);
          continue;
        }

        /* Fill the page data buffer with the incoming data */
        memcpy(data_page_buffer + data_page_buffer_len, data_buff + 4, length);
        data_page_buffer_len += length;

        /* Write full page block when end */
        if ((data_page_buffer_len >= SPM_PAGESIZE) || (recordtype == 0x1)) {
          /* Write next page of flash memory */
          flash_write_page(address, data_page_buffer);

          /* Move buffered data back to the start */
          if (data_page_buffer_len >= SPM_PAGESIZE) {
            address += SPM_PAGESIZE;
            data_page_buffer_len -= SPM_PAGESIZE;
          } else {
            address += data_page_buffer_len;
            data_page_buffer_len = 0;
          }

          /* Copy the data after the PAGESIZE back to the start of the buffer */
          memcpy(data_page_buffer, data_page_buffer + SPM_PAGESIZE, data_page_buffer_len);

          /* Update sketch size with the updated address position */
          boot_flags->sketch_size = address;
        }

        if (recordtype == 0x1) {
          break;
        }
      }
    }
  }
  
  /*
   * Wipe the first word of program data after the last page written
   * This makes sure that it does not try to boot the previous program
   * It also fixes a bug where modulus 256 size programs fail loading
   * It writes a full page of program data to achieve this
   * The first WORD will be 0xFFFF to indicate no program, however
   * the words following it will contain 'random' RAM memory
   * If nothing was loaded the result is a 'wiped' program
   *
   * This is slightly unsafe, but since we are not writing RAM, we consider
   * this to be a safe approach to keep binary size below 8192 bytes.
   *
   * It is important this is only done when the address is a 256 modulus (or 0)
   * This means the start of a page is written without overwriting anything.
   * To guarantee that, the flash_write_page function checks for address & 0xFF.
   */
  unsigned int no_data = 0xFFFF;
  flash_write_page(address, (unsigned char*) &no_data);
}

void saveSketch(PHN_Settings *boot_flags) {
  /* 0x2 = Sketch modified, save first (uploaded new sketch to FLASH) */
  if (boot_flags->flags & SETTINGS_MODIFIED) {
    boot_flags->flags &= ~SETTINGS_MODIFIED;
    boot_flags->flags |= SETTINGS_CHANGED;

    /* Show saving frame */
    LCD_write_frame(ICON_FROM_CHIPROM | ICON_TO_SD | ICON_DRAW_SKETCH, boot_flags->sketch_current);

    /* Initiate the saving process to the SD */
    /* Note: always saves contents in Intel HEX format */
    if (file_open(boot_flags->sketch_current, "HEX", FILE_WIPE)) {

      address_t address_read  = APP_START;
      address_t address_write = APP_START;
      unsigned char buff[HEX_FORMAT_BYTECOUNT + 4];
      unsigned char buff_len = 0;
      char reached_end;
      do {
        LCD_write_progress(address_read, boot_flags->sketch_size, STATUS_COLOR_SDSAVE);

        /* Fill the buffer with data */
        PTR_TO_WORD(buff + buff_len + 4) = pgm_read_word_far(address_read);
        buff_len += 2;
        address_read += 2;
        reached_end = (address_read >= boot_flags->sketch_size);

        /* Write a chunk of data when end is reached, or bytecount is reached */
        if (reached_end || (buff_len == HEX_FORMAT_BYTECOUNT)) {
          file_write_hex_line(buff, buff_len, address_write, 0x0);
          address_write += buff_len;
          buff_len = 0;

          /* Increment extended data address as needed */
          if (!(address_write & 0xFFFF)) {
            buff[4] = address_write >> 12;
            buff[5] = 0;
            file_write_hex_line(buff, 2, 0x0000, 0x2);
          }
        }
      } while (!reached_end);

      /* Write data end line */
      file_write_hex_line(buff, 0, 0x0000, 0x1);

      /* Flush any buffered data to SD */
      file_flush();
    }

    /* Save boot flags right now to prevent saving again next time */
    saveBootflags(boot_flags);
  }
}