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
* - stk500v2 commands have been added to read/write RAM/SPI/ADC/Micro-SD
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
* - Computer interface for SPI, memory registers and ADC
* - Multi-serial for communication between UART devices
* - Operation over WiFi/Bluetooth
*
* Compiler options:
* -DF_CPU=16000000UL     set CPU frequency
* -Os                    optimize for size
* -fno-jump-tables       fixes goto functionality
* -finline-limit=9999    inline large functions
* -mno-interrupts        do not disable interrupts when changing stack pointer
*
* Linker options:
* -nostartfiles                     exclude vector table
* -Wl,--section-start=.text=3E000   write code into bootloader section of FLASH
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

/* Formats for sketch .HEX files */
#define SKETCH_FMT_NONE     0
#define SKETCH_FMT_HEX      1
#define SKETCH_FMT_BIN      2
#define SKETCH_FMT_DEFAULT  SKETCH_FMT_HEX

/* Turns extended stk500 Phoenard functions on or off */
#define BOOT_ENABLE_EXTENDED_FUNCTIONS 1

/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0x00
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0x00
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           0x02
#define CONFIG_PARAM_SW_MINOR           0x0A

/* Sign-on response identifier */
static const char SIGNATURE_NAME[] = "AVRISP_2";
static const int  SIGNATURE_LENGTH = strlen(SIGNATURE_NAME);

/*
 * Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
 */
#define BOOTSIZE   8192
#define APP_START  0x00000000
#define APP_END    (FLASHEND - BOOTSIZE + 1)

/* ATMega with two USART, use UART0 */
#define UART_BAUD_RATE_LOW       UBRR0L
#define UART_STATUS_REG          UCSR0A
#define UART_CONTROL_REG         UCSR0B
#define UART_ENABLE_TRANSMITTER  TXEN0
#define UART_ENABLE_RECEIVER     RXEN0
#define UART_REGISTER_EMPTY      UDRE0
#define UART_TRANSMIT_COMPLETE   TXC0
#define UART_RECEIVE_COMPLETE    RXC0
#define UART_DATA_REG            UDR0
#define UART_DOUBLE_SPEED        U2X0

/*
 * Macro to calculate UBBR from XTAL and baudrate
 */
#define UART_BAUD_SELECT (((float)(F_CPU))/(((float)(BAUDRATE))*8.0)-1.0+0.5)

/* Definitions for the pin 13 status LED */
#define STATUS_LED_DDR    DDRB
#define STATUS_LED_PORT   PORTB
#define STATUS_LED_MASK   _BV(PB7)
#define STATUS_LED_ON()   STATUS_LED_DDR |= STATUS_LED_MASK
#define STATUS_LED_OFF()  STATUS_LED_DDR &= ~STATUS_LED_MASK

/*
 * How many cycles to wait before booting the program
 * Using a single set bit in the counter allows for size optimizations
 * A longer timeout is used for wireless communication to account for
 * delays and losses over the network.
 */
#define BOOT_START_SERIAL    (1UL << 19) /* About 0.5 seconds */
#define BOOT_START_WIRELESS  (1UL << 26) /* About 63 seconds */

/* Enables access to memory treated as an unsigned int */
#define PTR_TO_WORD(p) (*((unsigned int*) (p)))

/* Eases the transformation from two unsigned bytes to unsigned int */
typedef union {
  unsigned char bytes[2];
  uint16_t value;
  uint8_t* ptr;
} uint16_pack_t;

/* Eases the transformation from four unsigned bytes to unsigned long */
typedef union {
  unsigned char bytes[4];
  uint16_t words[2];
  void* ptr[2];
  uint32_t value;
} uint32_pack_t;

/* Address type and start/constant */
typedef uint32_pack_t address_t;
static const address_t APP_START_ADDR = {APP_START};

/* Stores device settings synchronized with EEPROM */
PHN_Settings settings;

/*
 * function prototypes
 */
uint8_t readParameter(uint8_t b0, uint8_t b1, uint8_t b2);
void flash_write_page(address_t address, const char* p);
uint8_t openCurrentSketch(uint8_t mode);
void changeLoadedSketch();
void saveBootflags();

//*****************************************************************************

void flash_write_page(address_t address, const char* p) {
  /* Memory protection */
  if ((address.words[1] >= (APP_END>>16)) || address.bytes[0]) return;

  /* Erase the page before writing */
  boot_spm_busy_wait();
  boot_page_erase(address.value);
  boot_spm_busy_wait();

  /* Write flash memory data */
  const char* p_end = p + SPM_PAGESIZE;
  do {
    boot_page_fill(address.value, PTR_TO_WORD(p));
    address.value += 2;
    p += 2;
  } while (p != p_end);

  /* Write the page but don't wait; allow new data to be read */
  boot_page_write(address.value - SPM_PAGESIZE);
}

//************************************************************************

/* 
 * Parameters for the SPI_MULTI and READ_LOCK/FUSE/SIG_ISP commands.
 * The following specification was found during tests with AVRDude:
 * SIGNATURE: [0]==0x30  with addr = ([2] * 2) or addr = ([2] << 1)
 * LFUSE:     [0]==0x50 && [1]==0x00  with addr = GET_LOW_FUSE_BITS (0x00)
 * LOCK:      [0]==0x58 && [1]==0x00  with addr = GET_LOCK_BITS (0x01)
 * EFUSE:     [0]==0x50 && [1]==0x08  with addr = GET_EXTENDED_FUSE_BITS (0x02)
 * HFUSE:     [0]==0x58 && [1]==0x08  with addr = GET_HIGH_FUSE_BITS (0x03)
 *
 * From this table we can conclude that:
 * [0] bit 0x08 set: addr |= 1
 * [1] bit 0x08 set: addr |= 2
 *
 * As a result, this would logically translate to:
 * addr = ([0]==0x58) | (([1]==0x08) << 1);
 * addr = (([0] & 0x08) >> 3) | ([1] >> 2);
 * addr = ((([0] & 0x08) >> 1) | [1]) >> 2;
 */
uint8_t readParameter(uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t address;
  if (b0 == 0x30) {
    address = (b2 << 1);
    return boot_signature_byte_get(address);
  } else if (b0 & 0x50) {
    address = ((((b0 & 0x08) >> 1) | b1) >> 2);
    return boot_lock_fuse_bits_get(address);
  } else {
    return 0x00;
  }
}

/*************************************************************************
 * Main bootloader code starts here. Optimized jump to the main function,
 * so no vector (and main jump) table has to be stored.
 */
void __attribute__ ((naked)) __attribute__ ((section (".init9"))) __jumpMain(void) {
  asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
  asm volatile ( "clr __zero_reg__" );  // r1 set to 0
  asm volatile ( "cli");                // disable interrupts
  asm volatile ( "jmp main" );          // jump to main()
}
//************************************************************************

int main(void) {
  static unsigned char  msgBuffer_full[600];
  static unsigned char* msgBuffer = (msgBuffer_full + 5);
  address_t      address;
  unsigned long  input_timeoutCtr;
  uint16_pack_t  input_dataLength;
  unsigned int   input_dataIndex;
  unsigned char  input_checksum;
  unsigned char* checksum_byte;
  uint16_pack_t  msgLength;
  uint16_t       msgLength_tmp;
  unsigned char& msgCommand = msgBuffer[0];
  unsigned char  msgStatus;
  unsigned char  watchdogFlags;
  unsigned char  c, *p, *k;
  unsigned char  exitBootloader;
  unsigned char  iconFlags;
  unsigned char  wifiConnection = 0x00;
  unsigned char  isWirelessUART = (PORTL & (_BV(0) | _BV(2)));
  volatile uint8_t* uart_data_reg;
  volatile uint8_t* uart_stat_reg;

  /* Switch between using WiFi and USB UART */
  if (isWirelessUART) {
      uart_data_reg = &UDR2;
      uart_stat_reg = &UCSR2A;
  } else {
      uart_data_reg = &UDR0;
      uart_stat_reg = &UCSR0A;
  }

  /* Handle the watch dog timer */
  watchdogFlags = MCUSR;
  asm volatile ("wdr");
  MCUSR   =  0;
  WDTCSR |=  _BV(WDCE) | _BV(WDE);
  WDTCSR  =  0;

  /* Set home button to proper INPUT */
  SELECT_DDR &= ~SELECT_MASK;
  SELECT_PORT |= SELECT_MASK;

  /*
   * Initialize port L:
   * - TFTLCD_BL_MASK(5) to OUTPUT HIGH to turn on screen backlight
   * - VS1053B_POWER(6) set OUTPUT LOW to turn off chip power
   * - VS1053B_RESET(3) set OUTPUT LOW to put all chip pins in high impedance
   * - VS1053B_CS(4) set OUTPUT HIGH to disable chip SPI
   * - WIFI_PWR(0) set OUTPUT LOW to disable WiFi power
   * - BLUETOOTH_RST(2) set OUTPUT LOW to keep BlueTooth reset
   */
  DDRL = TFTLCD_BL_MASK | _BV(6)  | _BV(3)  | _BV(4) | _BV(0) | _BV(2);
  PORTL |= TFTLCD_BL_MASK | _BV(4);

  /*
   * Init UART
   * set baudrate and enable USART receiver and transmitter without interrupts
   */
  UART_STATUS_REG    |=  (1 <<UART_DOUBLE_SPEED);
  UART_BAUD_RATE_LOW  =  UART_BAUD_SELECT;
  UART_CONTROL_REG    =  (1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

  /* Read boot settings from EEPROM */
  PHN_Settings_Load(settings);

  /*
   * If the current filename is corrupted, reset
   * This prevents SD corruption issues caused by saving
   */
  for (unsigned char i = 0; i < 8; i++) {
    unsigned char c = settings.sketch_current[i];
    if (c < 32 || c >= 127) {
      settings = SETTINGS_DEFAULT;
      break;
    }
  }

  /* Reset address to 0 */
  address = APP_START_ADDR;

  /* Initialize LCD Screen */
  LCD_init();

  /* Go here to start the bootloader, past the initialization */
bootloader:
  exitBootloader = 0;

  /* Handle load instructions from EEPROM / SELECT-button */
  if (!(SELECT_IN & SELECT_MASK)) {
    memcpy(settings.sketch_toload, SETTINGS_DEFAULT.sketch_toload, 8);
    settings.flags |= SETTINGS_LOAD | SETTINGS_CHANGED;
    saveBootflags();
  }

  /* Handle loading of a sketch from Micro-SD. Go to program right away. */
  if (settings.flags & SETTINGS_LOAD) {
    changeLoadedSketch();
    exitBootloader = 1;
  }

  /* If this is a reset caused by the watchdog timer, skip bootloader */
  if (watchdogFlags & _BV(WDRF)) {
    // Goto is less size than exitBootloader = 1;
    goto program;
  }

  /* Bootloader logic starts here */
  while (!exitBootloader) {
    /* 
     * Read in a full message into the buffer
     * Increments first time - start -1
     */
    input_checksum = 0;
    input_dataIndex = ST_START-1;
    do {
      /* Wait until a new byte of data is available; on timeout run program */
      input_timeoutCtr = 0;
      do {
          input_timeoutCtr++;
          if (isWirelessUART) {
              if (input_timeoutCtr & BOOT_START_WIRELESS) goto program;
          } else {
              if (input_timeoutCtr & BOOT_START_SERIAL) goto program;
          }
      } while (!(*uart_stat_reg & (1 << UART_RECEIVE_COMPLETE)));

      /* Receive next byte of data, update the checksum and fill the buffer */
      c = *uart_data_reg;
      input_checksum ^= c;
      msgBuffer_full[++input_dataIndex] = c;
      
      /* Log wireless communication to serial */
      if (isWirelessUART) {
          UART_DATA_REG = c;
      }

      /* 
       * Character after the first comma received is our current WiFi connection.
       * This allows for multiple ongoing communications, as well running on both
       * UDP and TCP protocols and multiple open ports at the same time. Up to
       * 10 connections are theoretically possible in this implementation, however
       * the ESP8266 supports only up to 4 (id=0,1,2,3) connections at once.
       */
      if (wifiConnection & 0x80) wifiConnection = c;
      if (!(wifiConnection & 0x10) && (c == ',')) wifiConnection = 0x80;

      /* Update the input data length using the data currently inside the buffer */
      input_dataLength.bytes[1] = msgBuffer_full[ST_MSG_SIZE_1];
      input_dataLength.bytes[0] = msgBuffer_full[ST_MSG_SIZE_2];

      /* Verify the start and middle token of the current message */
      if ( (input_dataIndex == ST_GET_TOKEN && c != TOKEN) || 
           (  msgBuffer_full[ST_START] != MESSAGE_START  ) ) goto bootloader;

    } while (input_dataIndex < (input_dataLength.value+5));

    /* Send error message upon checksum failure */
    if (input_checksum) {
      msgCommand = ANSWER_CKSUM_ERROR;
    }

    /*
     * Optimization: These values are used for almost every response
     * Put here and used before sending to save a few bytes of program memory
     */
    msgStatus = STATUS_CMD_OK;
    msgLength.value = 2;
    iconFlags = ICON_PCIDLE;

    /* 
     * Finish flashing, enable RWW and save bootloader flags while performing non-flash operations
     * This ensures that settings after uploading are saved, and also ensures that the RWW
     * is enabled when verifying flash or reading EEPROM right after uploading.
     */
    if (msgCommand != CMD_PROGRAM_FLASH_ISP && msgCommand != CMD_LOAD_ADDRESS) {
      saveBootflags();
    }

    /*
     * Now process the STK500 commands, see Atmel Appnote AVR068
     */
    switch (msgCommand) {
      case CMD_SPI_MULTI:
        /* Send the requested parameter back */
        msgBuffer[2] = 0;
        msgBuffer[3] = msgBuffer[4];
        msgBuffer[4] = STATUS_CMD_OK;
        msgBuffer[5] = readParameter(msgBuffer[3], msgBuffer[5], msgBuffer[6]);
        msgBuffer[6] = STATUS_CMD_OK;
        msgLength.value = 7;
        break;

      case CMD_READ_SIGNATURE_ISP:
      case CMD_READ_LOCK_ISP:
      case CMD_READ_FUSE_ISP:
        /* Read parameter - similar arguments are used for signature, lock and fuse */
        msgBuffer[2] = readParameter(msgBuffer[2], msgBuffer[3], msgBuffer[4]);
        msgBuffer[3] = STATUS_CMD_OK;
        msgLength.value = 4;
        break;

      case CMD_GET_PARAMETER:
        switch(msgBuffer[1]) {
          case PARAM_BUILD_NUMBER_LOW:
             c = CONFIG_PARAM_BUILD_NUMBER_LOW; break;
          case PARAM_BUILD_NUMBER_HIGH:
             c = CONFIG_PARAM_BUILD_NUMBER_HIGH; break;
          case PARAM_HW_VER:
             c = CONFIG_PARAM_HW_VER; break;
          case PARAM_SW_MAJOR:
             c = CONFIG_PARAM_SW_MAJOR; break;
          case PARAM_SW_MINOR:
             c = CONFIG_PARAM_SW_MINOR; break;
          default:
             c = 0; break;
        }
        msgBuffer[2] = c;
        msgLength.value = 3;
        break;

      case CMD_SIGN_ON:
        msgBuffer[2] = SIGNATURE_LENGTH;
        memcpy(msgBuffer + 3, SIGNATURE_NAME, SIGNATURE_LENGTH);
        msgLength.value = SIGNATURE_LENGTH + 3;
        break;

      case CMD_LEAVE_PROGMODE_ISP:
        exitBootloader = 1;
        /* Fall-through */
      case CMD_SET_PARAMETER:
      case CMD_ENTER_PROGMODE_ISP:
        msgLength.value = 2;
        break;

      case CMD_LOAD_ADDRESS:
        /* Reverse the byte order */
        address.bytes[0] = msgBuffer[4];
        address.bytes[1] = msgBuffer[3];
        address.bytes[2] = msgBuffer[2];
        address.bytes[3] = msgBuffer[1];
        msgLength.value = 2;
        break;

#if BOOT_ENABLE_EXTENDED_FUNCTIONS
      case CMD_READ_RAM_BYTE_ISP:
        /* Clear mask byte so output is not affected */
        msgBuffer[3] = 0x00;
      case CMD_PROGRAM_RAM_BYTE_ISP:
        {
          /* Access single RAM byte using an address, mask and value */
          uint8_t* address    = ((uint16_pack_t) {msgBuffer[2], msgBuffer[1]}).ptr;
          uint8_t  data_mask  = msgBuffer[3];
          uint8_t  data_value = msgBuffer[4];

          /* Send back the updated data at the address */
          msgBuffer[2] = (*address & ~data_mask) | (data_value & data_mask);
          msgLength.value = 3;

          /* If specified, write to the address */
          if (msgCommand == CMD_PROGRAM_RAM_BYTE_ISP) {
            *address = msgBuffer[2];
          }
          break;
        }

      case CMD_READ_ANALOG_ISP:
        ADMUX = msgBuffer[3];
        ADCSRB = msgBuffer[2];
        ADCSRA = msgBuffer[1];
        while (ADCSRA & (1<<ADSC));
        msgBuffer[3] = ADCL;
        msgBuffer[2] = ADCH;
        msgLength.value = 4;
        break;

      case CMD_TRANSFER_SPI_ISP:
        {
          /* 
           * Write data to SPI and read the received data.  This logic
           * supports full-duplex mode; every byte transmitted results
           * in one byte received. If you only want to receive data,
           * simply pad the transmit buffer with 0xFF.
           */
          msgLength.value = 2;
          uint8_t *p = msgBuffer + 2;
          while (msgLength.value < input_dataLength.value) {
            SPDR = *p;
            while (!(SPSR & (1 << SPIF)));
            *(p++) = SPDR;
            msgLength.value++;
          }
          break;
        }        

      case CMD_MULTISERIAL_ISP:
        {
          /*
           * Transfer bytes received between two UART serial ports. This allows someone
           * to directly communicate with WiFi/Bluetooth/GPS through USB. It is even
           * possible to communicate with the Sim908 using Bluetooth. This also allows
           * you to perform Bluetooth Serial communication using a computer.
           */
          uint8_t* reg_a = (uint8_t*) *reinterpret_cast<uint16_t*>(msgBuffer+1);
          uint8_t* reg_b = (uint8_t*) *reinterpret_cast<uint16_t*>(msgBuffer+3);
          uint8_t* reg_c;
          for (;;) {
            if (reg_a[0] & (1 << UART_RECEIVE_COMPLETE)) {
              reg_b[6] = reg_a[6];
            }
            reg_c = reg_a;
            reg_a = reg_b;
            reg_b = reg_c;
          }
          break;
        }

      case CMD_INIT_SD_ISP:
        {
          /* Set device icon */
          iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD | ICON_PROGRESS_INVERT;

          /* Ensure card is initialized by opening an arbitrary (non-existent) file */
          volume.isInitialized = 0;

          /* Initialize volume by supplying a guaranteed-invalid name */
          char* buff = (char*) msgBuffer+2;
          *buff = 0;
          file_open(buff, buff, SDMIN_FILE_READ);

          /* Respond with all known volume variables */
          msgLength.value = 2 + sizeof(CardVolume);
          memcpy(buff, &volume, sizeof(CardVolume));
          break;
        }

#endif

#if BOOT_ENABLE_EXTENDED_FUNCTIONS
      case CMD_READ_RAM_ISP:
      case CMD_PROGRAM_RAM_ISP:
      case CMD_READ_SD_ISP:
      case CMD_PROGRAM_SD_ISP:
      case CMD_PROGRAM_SD_FAT_ISP:
#endif
      case CMD_READ_FLASH_ISP:
      case CMD_READ_EEPROM_ISP:
      case CMD_PROGRAM_FLASH_ISP:
      case CMD_PROGRAM_EEPROM_ISP:
        {
          /* Read the size by using the msgLength 16-pack temporarily */
          unsigned int size = ((uint16_pack_t) {msgBuffer[2], msgBuffer[1]}).value;
          msgLength.value = 2;

          /* Track whether the Micro-SD is accessed */
          uint8_t sdAccessed = 0;

          /* Writing or reading? */
          if (msgCommand & 0x1) {
            char *p = (char*) (msgBuffer + 10);

            switch (msgCommand) {
              case CMD_PROGRAM_FLASH_ISP:
                {
                  /* Set device icon */
                  iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM;

                  /* Change address from word-space to byte-space */
                  address.value <<= 1;

                  /* Write next page of program memory */
                  flash_write_page(address, p);
                  p += size;

                  /* 
                   * Update program size, reset size when this is the first update
                   * On some programmers it pads the end of the program with 0xFFFF data
                   * We must make sure to exclude that data from the program size
                   */
                  if (!(settings.flags & SETTINGS_CHANGED) || address.value >= settings.sketch_size) {
                    /* Calculate program size excluding trailing 0xFFFF */
                    unsigned int size_tmp = size;
                    do {
                      p -= 2;
                      if (PTR_TO_WORD(p) != 0xFFFF) {
                        break;
                      }
                      size_tmp -= 2;
                    } while (size_tmp);

                    settings.sketch_size = address.value + size_tmp;
                    settings.flags |= SETTINGS_MODIFIED | SETTINGS_CHANGED;
                  }

                  /* Increment address by the amount of data written out */
                  address.value += size;

                  /* Change address back to word-space */
                  address.value >>= 1;
                }
                break;

              case CMD_PROGRAM_EEPROM_ISP:
                /* Set device icon */
                iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM;

                /* Write EEPROM, full block */
                eeprom_write_block(p, address.ptr[0], size);
                address.value += size;
                break;

              case CMD_PROGRAM_SD_FAT_ISP:
                /* If writing FAT, it's the same logic, but handle mirror FAT */
                volume_cacheFATMirror = volume.isMultiFat;
              case CMD_PROGRAM_SD_ISP:
                /* Set device icon */
                iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD;

                /* Write Micro-SD, full block */
                memcpy(volume_cacheBuffer.data, p, size);
                volume_writeCache(address.value);
                address.value++;
                sdAccessed = 1;
                break;

              case CMD_PROGRAM_RAM_ISP:
                memcpy(address.ptr[0], p, size);
                address.value += size;
                break;
            }
          } else {
            unsigned char *p = (msgBuffer + 2);
            msgLength.value += size;
            switch (msgCommand) {
              case CMD_READ_FLASH_ISP:
                /* Set device icon */
                iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM | ICON_PROGRESS_INVERT;

                /* Read FLASH, where size is in WORD space; reading 2 bytes at a time */
                do {
                  /* Read word in memory, copy to message buffer and select next word */
                  PTR_TO_WORD(p) = pgm_read_word_far(address.value << 1);
                  p += 2;
                  address.value++;
                } while (size -= 2);
                break;

              case CMD_READ_EEPROM_ISP:
                /* Set device icon */
                iconFlags = ICON_FROM_COMPUTER | ICON_TO_CHIPROM | ICON_PROGRESS_INVERT;

                /* Read EEPROM, full block */
                eeprom_read_block(p, address.ptr[0], size);
                address.value += size;
                break;

              case CMD_READ_SD_ISP:
                /* Set device icon */
                iconFlags = ICON_FROM_COMPUTER | ICON_TO_SD | ICON_PROGRESS_INVERT;

                /* Read Micro-SD, full block */
                volume_readCache(address.value);
                memcpy(p, volume_cacheBuffer.data, size);
                address.value++;
                sdAccessed = 1;
                break;

              case CMD_READ_RAM_ISP:
                memcpy(p, (uint8_t*) address.ptr[0], size);
                address.value += size;
                break;
            }
          }

          /* SD was accessed and something went wrong, return FAIL */
          if (sdAccessed && !volume.isInitialized) {
            msgLength.value = 2;
            msgStatus = STATUS_CMD_FAILED;
          }
        }
        break;

      default:
        msgLength.value = 2;
        msgStatus = STATUS_CMD_FAILED;
        break;
    }

    /*
     * Now send answer message back
     */
    /* Set the header of the message to send */
    /*msgBuffer_full[0] = MESSAGE_START;*/       /* Start token */
    /*msgBuffer_full[1] = msgBuffer_full[1];*/   /* Sequence number */
    msgBuffer_full[2] = msgLength.bytes[1];      /* Length high */
    msgBuffer_full[3] = msgLength.bytes[0];      /* Length low */
    /*msgBuffer_full[4] = TOKEN;*/               /* Token */
    /*msgBuffer_full[5] = msgCommand;*/          /* Command ID */
    msgBuffer_full[6] = msgStatus;               /* Command status */

    /*
     * Initialize the display, draw icon of current file
     * For some reason, putting this after the above lines reduces binary size...
     * Do not run this for the SIGN_OUT command so the bootloader can be silent skipped
     */
    if (msgCommand != CMD_LEAVE_PROGMODE_ISP) {
      LCD_write_frame(iconFlags);
    }

    /* Store the pointer of the checksum byte and reset it to 0 */
    checksum_byte = msgBuffer + msgLength.value;
    *checksum_byte = 0;

    /* Add header + checksum length to message length */
    msgLength.value += 6;

    /* For WiFi Serial, execute the CIPSEND command in bursts */
    if (PORTL & _BV(0)) {

      /* Generate command with message length parameter 00-64 */
      unsigned char wifi_cmd[] = "AT+CIPSEND=0,000\r\n";
      wifi_cmd[11] = wifiConnection;
      wifi_cmd[13] = '0';
      wifi_cmd[14] = '0';

      k = wifi_cmd+15;
      msgLength_tmp = msgLength.value;
      do {
        *(k--) = '0' + (msgLength_tmp % 10);
      } while (msgLength_tmp/=10);

      /* Write out the command to UART */
      k = wifi_cmd;
      do {
        *uart_stat_reg |= (1 << UART_TRANSMIT_COMPLETE);
        *uart_data_reg = *k;
        while (!(*uart_stat_reg & (1 << UART_TRANSMIT_COMPLETE)));
      } while (*(++k));

      /* Wait until OK is received with timeout */
      input_timeoutCtr = 0;
      while (*uart_data_reg != 'K') {
        if (++input_timeoutCtr & BOOT_START_WIRELESS) goto bootloader;
      }
    }

    /* Transmit the data */
    p = msgBuffer_full;
    do {
      *uart_stat_reg |= (1 << UART_TRANSMIT_COMPLETE); /* Clear TX flag */
      *uart_data_reg = *p;   /* Start transmission of next byte */
      *checksum_byte ^= *p;  /* Update checksum */
      p++;                   /* Next byte */

      /* Wait for the transmission to complete */
      while (!(*uart_stat_reg & (1 << UART_TRANSMIT_COMPLETE)));
    } while (--msgLength.value);

    /* Reset WiFi connection ID for future command execution */
    wifiConnection = 0x00;
  }

  /* Post-programming program label to jump to, skipping the main bootloader */
program:

  /* Reset watchdog flags so we don't end up rapidly looping */
  watchdogFlags = 0;

  /* Finish flashing, enable RWW and save bootloader flags */
  saveBootflags();

  /* If no program available, go back to the bootloader */
  if (pgm_read_word_far(APP_START) == 0xFFFF) {
    LCD_write_frame(ICON_FROM_NONE | ICON_TO_CHIPROM);
    goto bootloader;
  }

  /* ===== Leaving the bootloader ===== */
  /* ================================== */

  /* Prepare the LCD screen for further program use */
  *settings.sketch_current = 0;
  LCD_write_frame(ICON_PCIDLE);

  /* Turn off UART */
  UART_STATUS_REG &= 0xFD;

  /* Turn off SPI and put Micro-SD CS High, all on Port B */
  DDRB = SD_CS_MASK;
  PORTB = SD_CS_MASK;
  SPCR = 0;

  /* Jump to the program at address 0 */
  asm volatile ("jmp %0" :: "i" (APP_START));

  /* This point is never reached, but to reduce size, infinite loop here */
  for(;;);
}

void saveBootflags() {
  /* Wait for any operations to finish and enable RWW */
  boot_spm_busy_wait();
  boot_rww_enable();

  /* Write bootloader flags to EEPROM when updated */
  if (settings.flags & SETTINGS_CHANGED) {
    settings.flags &= ~SETTINGS_CHANGED;
    PHN_Settings_Save(settings);
    eeprom_busy_wait();
  }
}

uint8_t openCurrentSketch(uint8_t mode) {
  uint8_t fmt;
  if (!file_open(settings.sketch_current, "HEX", mode)) {
    fmt = SKETCH_FMT_NONE;
  } else if (!file_size) {
    fmt = SKETCH_FMT_DEFAULT;
  } else {
    fmt = SKETCH_FMT_HEX;

    /* Read out the format of the file using the first 12 bytes */
    uint8_t* data = volume_cacheCurrentBlock(0);
    for (unsigned char i = 0; i < 12; i++) {
      if (data[i] < 0x30 || data[i] > 0x46) {
        fmt = SKETCH_FMT_BIN;
      }
    }
  }
  return fmt;
}

void changeLoadedSketch() {
  uint8_t oldFlags = settings.flags;
  uint8_t file_format;

  /* Turn on LED to indicate we are saving and loading */
  STATUS_LED_ON();

  /* 0x2 = Sketch modified, save first (uploaded new sketch to FLASH) */
  if (settings.flags & SETTINGS_MODIFIED) {

    /* Show saving frame */
    LCD_write_frame(ICON_FROM_CHIPROM | ICON_TO_SD | ICON_DRAW_SKETCH);

    /* Initiate the saving process to the SD */
    file_format = openCurrentSketch(SDMIN_FILE_CREATE);
    if (file_format) {

      /* Delete old file contents */
      file_truncate();

      address_t address_read = APP_START_ADDR;
      address_t address_write = APP_START_ADDR;
      static char buff[HEX_FORMAT_BYTECOUNT + 4];
      unsigned char buff_len = 0;
      char reached_end;
      do {
        LCD_write_progress(address_read.value, settings.sketch_size, STATUS_COLOR_SDSAVE);

        /* Fill the buffer with data */
        PTR_TO_WORD(buff + buff_len + 4) = pgm_read_word_far(address_read.value);
        buff_len += 2;
        address_read.value += 2;
        reached_end = (address_read.value >= settings.sketch_size);

        /* Write a chunk of data when end is reached, or bytecount is reached */
        if (reached_end || (buff_len == HEX_FORMAT_BYTECOUNT)) {
          if (file_format == SKETCH_FMT_BIN) {
            file_write(buff+4, buff_len);
            
          } else {
            file_append_hex_line(buff, buff_len, address_write.words[0], 0x0);
            address_write.value += buff_len;

            /* Increment extended data address as needed */
            if (!address_write.words[0]) {
              buff[4] = address_write.value >> 12;
              buff[5] = 0;
              file_append_hex_line(buff, 2, 0x0000, 0x2);
            }

            /* Write end delimiter */
            if (reached_end) {
              file_append_hex_line(buff, 0, 0x0000, 0x1);
            }
          }
          buff_len = 0;
        }
      } while (!reached_end);

      /* Flush any buffered data to SD */
      file_flush();
    }
  }

  /* Store old flags, then reset flags to remove loading flag */
  settings.flags &= ~(SETTINGS_LOAD | SETTINGS_LOADWIPE | SETTINGS_MODIFIED);
  settings.flags |= SETTINGS_CHANGED;

  /* Swap current sketch with the one to load, this way you can go 'back' at any time */
  char c;
  char* toload = settings.sketch_toload;
  char* current = settings.sketch_current;
  for (unsigned char i = 0; i < 8; i++) {
    c = toload[i];
    toload[i] = current[i];
    current[i] = c;
  }

  /* Save boot flags right now so when resets happen, it won't end up looping trying to load */
  saveBootflags();

  /* Initialize the frame, draw icon to load. Force it. */
  LCD_write_frame(ICON_FROM_SD | ICON_TO_CHIPROM | ICON_DRAW_SKETCH);

  /* Open the file on SD */
  /* Don't load anything if WIPE flag is specified */
  address_t address = APP_START_ADDR;
  static char data_buff[SPM_PAGESIZE*2];
  static char data_page_buffer[SPM_PAGESIZE*2];
  if (!(oldFlags & SETTINGS_LOADWIPE) && 
       (file_format = openCurrentSketch(SDMIN_FILE_READ)) ) {

    /* Perform loading from SD */
    unsigned char length;
    unsigned char recordtype;
    unsigned int data_page_buffer_len = 0;
    while (file_position < file_size) {
      LCD_write_progress(file_position, file_size, STATUS_COLOR_SDLOAD);

      if (file_format == SKETCH_FMT_BIN) {
        /* Raw reading from the SD in blocks of 256 bytes at a time */
        flash_write_page(address, file_read(SPM_PAGESIZE));
        address.value += SPM_PAGESIZE;
        settings.sketch_size = file_size;
        
      } else {
        /* Reading Intel HEX format */
        file_read_hex_line(data_buff);
        length = data_buff[0];
        recordtype = data_buff[3];

        /* Validate that the data is valid using the provided CRC */
        unsigned char crc_expected = data_buff[length + 4];
        unsigned char crc_read = 0;
        for (int i = 0; i < (length+4); i++) crc_read += data_buff[i];
        crc_read = (~crc_read + 1) & 0xFF;
        if (crc_expected != crc_read) {
          break;
        }

        /* Read the address part of data hex lines */
        if (!data_page_buffer_len && !recordtype) {
          address.bytes[0] = data_buff[2];
          address.bytes[1] = data_buff[1];
        }

        /* Extended data segment, we ignore data_buff[5] (out of range) */
        if (recordtype == 0x2) {
          address.words[1] = ((uint16_t) data_buff[4] >> 4);
          continue;
        }

        /* Extended Linear Address, found to exist sometimes */
        if (recordtype == 0x4) {
          address.bytes[2] = data_buff[5];
        /*address.bytes[3] = data_buff[4]; (out of range) */
          continue;
        }

        /* Fill the page data buffer with the incoming data */
        memcpy(data_page_buffer + data_page_buffer_len, data_buff + 4, length);
        data_page_buffer_len += length;

        /* Write full page block when end */
        if ((data_page_buffer_len >= SPM_PAGESIZE) || (recordtype == 0x1)) {

          /* Make sure the buffer is padded with 0xFFFF */
          for (int i = data_page_buffer_len; i < SPM_PAGESIZE; i++) {
            data_page_buffer[i] = 0xFF;
          }

          /* Write next page of flash memory */
          flash_write_page(address, data_page_buffer);

          /* Move buffered data back to the start */
          if (data_page_buffer_len >= SPM_PAGESIZE) {
            address.value += SPM_PAGESIZE;
            data_page_buffer_len -= SPM_PAGESIZE;
          } else {
            address.value += data_page_buffer_len;
            data_page_buffer_len = 0;
          }

          /* Copy the data after the PAGESIZE back to the start of the buffer */
          memcpy(data_page_buffer, data_page_buffer + SPM_PAGESIZE, data_page_buffer_len);

          /* Update sketch size with the updated address position */
          settings.sketch_size = address.value;
          settings.flags |= SETTINGS_CHANGED;
        }

        if (recordtype == 0x1) {
          break;
        }
      }
    }
  }

  /* Turn off LED to indicate we are done saving and loading */
  STATUS_LED_OFF();

  /*
   * Wipe the first page of program data after the last page written
   * This makes sure that it does not try to boot the previous program
   * It also fixes a bug where modulus 256 size programs fail loading
   * It writes a full page of program data to achieve this
   * All data will contain 0xFFFF to indicate no program
   * If nothing was loaded the result is a 'wiped' program
   *
   * It is important this is only done when the address is a 256 modulus (or 0)
   * This means the start of a page is written without overwriting anything.
   * To guarantee that, the flash_write_page function checks for address & 0xFF.
   */
  for (int i = 0; i < SPM_PAGESIZE; i++) data_buff[i] = 0xFF;
  flash_write_page(address, data_buff);
}