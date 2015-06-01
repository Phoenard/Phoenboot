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
#include "display.h"

/* Splits a 16-bit argument into two 8-bit bytes in memory */
#define ARG(value)   ((value) & 0xFF), ((value) >> 8)

/* Initial port states */
#define INIT_DDR_MASK   (TFTLCD_CS_MASK | TFTLCD_RS_MASK | TFTLCD_WR_MASK | TFTLCD_RD_MASK | TFTLCD_RESET_MASK | EXSRAM_HOLD_MASK)
#define INIT_PORT_MASK  (TFTLCD_RS_MASK | TFTLCD_WR_MASK | TFTLCD_RD_MASK | TFTLCD_RESET_MASK | EXSRAM_HOLD_MASK)

/* Reset on/off toggle logic */
#define RESET_A  (INIT_PORT_MASK & ~TFTLCD_RESET_MASK)
#define RESET_B  (INIT_PORT_MASK |  TFTLCD_RESET_MASK) 

/* Constants for fast reading/writing of data - not to be used for commands */
#define WR_WRITE_A  (INIT_PORT_MASK & ~TFTLCD_WR_MASK)
#define WR_WRITE_B  (INIT_PORT_MASK |  TFTLCD_WR_MASK)
#define WR_READ_A   (INIT_PORT_MASK & ~TFTLCD_RD_MASK)
#define WR_READ_B   (INIT_PORT_MASK |  TFTLCD_RD_MASK)

/* Specific reading/writing constants for usage by commands */
#define WR_COMMAND_WRITE_A  (WR_WRITE_A & ~TFTLCD_RS_MASK)
#define WR_COMMAND_WRITE_B  (WR_WRITE_B & ~TFTLCD_RS_MASK)

/* LCD Initialization register values */
const unsigned char LCD_REG_DATA[] = {
  LCD_CMD_START_OSC,          ARG(0x0001),
  LCD_CMD_DRIV_OUT_CTRL,      ARG(0x0100),
  LCD_CMD_DRIV_WAV_CTRL,      ARG(0x0700),
  LCD_CMD_RESIZE_CTRL,        ARG(0x0000),
  LCD_CMD_DISP_CTRL2,         ARG(0x0202),
  LCD_CMD_DISP_CTRL3,         ARG(0x0000),
  LCD_CMD_DISP_CTRL4,         ARG(0x0000),
  LCD_CMD_RGB_DISP_IF_CTRL1,  ARG(0x0000),
  LCD_CMD_FRM_MARKER_POS,     ARG(0x0000),
  LCD_CMD_RGB_DISP_IF_CTRL2,  ARG(0x0000),
  LCD_CMD_POW_CTRL1,          ARG(0x0000),
  LCD_CMD_POW_CTRL2,          ARG(0x0000),
  LCD_CMD_POW_CTRL3,          ARG(0x0000),
  LCD_CMD_POW_CTRL4,          ARG(0x0000),
  LCD_CMD_POW_CTRL1,          ARG(0x17b0),
  LCD_CMD_POW_CTRL2,          ARG(0x0037),
  LCD_CMD_POW_CTRL3,          ARG(0x0138),
  LCD_CMD_POW_CTRL4,          ARG(0x1700),
  LCD_CMD_POW_CTRL7,          ARG(0x000d),
  LCD_CMD_GAMMA_CTRL1,        ARG(0x0001),
  LCD_CMD_GAMMA_CTRL2,        ARG(0x0606),
  LCD_CMD_GAMMA_CTRL3,        ARG(0x0304),
  LCD_CMD_GAMMA_CTRL4,        ARG(0x0103),
  LCD_CMD_GAMMA_CTRL5,        ARG(0x011d),
  LCD_CMD_GAMMA_CTRL6,        ARG(0x0404),
  LCD_CMD_GAMMA_CTRL7,        ARG(0x0404),
  LCD_CMD_GAMMA_CTRL8,        ARG(0x0404),
  LCD_CMD_GAMMA_CTRL9,        ARG(0x0700),
  LCD_CMD_GAMMA_CTRL10,       ARG(0x0a1f),
  LCD_CMD_HOR_START_AD,       ARG(0x0000),
  LCD_CMD_HOR_END_AD,         ARG(0x00ef),
  LCD_CMD_VER_START_AD,       ARG(0x0000),
  LCD_CMD_VER_END_AD,         ARG(0x013f),
  LCD_CMD_GATE_SCAN_CTRL1,    ARG(0x2700),
  LCD_CMD_GATE_SCAN_CTRL2,    ARG(0x0003),
  LCD_CMD_GATE_SCAN_CTRL3,    ARG(0x0000),
  LCD_CMD_PANEL_IF_CTRL1,     ARG(0x0010),
  LCD_CMD_PANEL_IF_CTRL2,     ARG(0x0000),
  LCD_CMD_PANEL_IF_CTRL3,     ARG(0x0003),
  LCD_CMD_PANEL_IF_CTRL4,     ARG(0x0101),
  LCD_CMD_PANEL_IF_CTRL5,     ARG(0x0000),
  LCD_CMD_PANEL_IF_CTRL6,     ARG(0x0000),
  LCD_CMD_DISP_CTRL1,         ARG(0x0133)
};

/* 32x32 1-bit headerless icons for COMPUTER, CHIPROM and SD memories */
const unsigned char ICONS[3][96] = {{
  0x00, 0x00, 0x80, 0x3F, 0xFC, 0xFF, 0xC7, 0x7F,
  0xFE, 0xFF, 0xCF, 0x60, 0x06, 0x00, 0xCC, 0x60,
  0x06, 0x00, 0xCC, 0x7F, 0x06, 0x00, 0xCC, 0x60,
  0x06, 0x00, 0xCC, 0x60, 0x06, 0x00, 0xCC, 0x7F,
  0x06, 0x00, 0xCC, 0x7F, 0x06, 0x00, 0xCC, 0x7F,
  0x06, 0x00, 0xCC, 0x7B, 0x06, 0x00, 0xCC, 0x7F,
  0xFE, 0xFF, 0xCF, 0x7F, 0xFC, 0xFF, 0xC7, 0x7B,
  0x00, 0x0E, 0xC0, 0x7F, 0x00, 0x0E, 0xC0, 0x7F,
  0xE0, 0xFF, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x76, 0x77, 0x75, 0x3B, 0x52, 0x57, 0x25, 0x29,
  0x52, 0x75, 0x25, 0x3B, 0x52, 0x15, 0x25, 0x19,
  0x76, 0x15, 0x27, 0x2B, 0x00, 0x00, 0x00, 0x00
}, {
  0x00, 0x00, 0x00, 0x00, 0x50, 0x55, 0x55, 0x05,
  0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0x07,
  0x18, 0x00, 0x00, 0x0C, 0xCD, 0xFF, 0xFF, 0x59,
  0xE4, 0xFF, 0xFF, 0x13, 0xF5, 0xFF, 0xFF, 0x57,
  0xF4, 0xFF, 0xFF, 0x17, 0x15, 0x9D, 0x63, 0x57,
  0xD4, 0x6D, 0x5D, 0x17, 0xD5, 0x6D, 0x73, 0x54,
  0x94, 0x0D, 0x6F, 0x15, 0xD5, 0x6D, 0x5D, 0x55,
  0xD4, 0x69, 0x63, 0x15, 0xF5, 0xFF, 0xFF, 0x57,
  0x14, 0x00, 0x00, 0x14, 0xF5, 0xFF, 0xFF, 0x57,
  0xE4, 0xFF, 0xFF, 0x13, 0xCD, 0xFF, 0xFF, 0x59,
  0x18, 0x00, 0x00, 0x0C, 0xF0, 0xFF, 0xFF, 0x07,
  0x00, 0x00, 0x00, 0x00, 0x50, 0x55, 0x55, 0x05
}, {
  0xFC, 0xFF, 0xFF, 0x3F, 0x06, 0x00, 0x00, 0x60,
  0xF3, 0xFF, 0xFF, 0xCF, 0x99, 0x59, 0x44, 0x98,
  0x0D, 0x70, 0x77, 0xBB, 0x0D, 0x50, 0x77, 0xBB,
  0x4D, 0x52, 0x77, 0xBB, 0x4D, 0x52, 0x77, 0xBB,
  0x4D, 0x52, 0x74, 0xB8, 0x4D, 0xF2, 0xFF, 0xBF,
  0x4D, 0xF2, 0x18, 0xBC, 0x4D, 0x72, 0x18, 0xB8,
  0x4D, 0x32, 0xFF, 0xB8, 0xFD, 0x7F, 0xF8, 0xB9,
  0xFD, 0xFF, 0xF3, 0xB8, 0xFD, 0x3F, 0x10, 0x9C,
  0xFD, 0x3F, 0x18, 0xDE, 0xFD, 0xFF, 0xFF, 0x4F,
  0xFD, 0xFF, 0x3F, 0x60, 0xFD, 0x0F, 0x9C, 0x3F,
  0xF9, 0xE7, 0xCD, 0x00, 0xF3, 0x33, 0x61, 0x00,
  0x06, 0x18, 0x3F, 0x00, 0xFC, 0x0F, 0x1C, 0x00
}};

unsigned char lcd_progress = LCD_PROG_CNT - LCD_PROG_MARQ_BATCH;
unsigned char lcd_progress_color = LCD_BLACK;
unsigned char lcd_icon_flags = 0; // Default state of 0 indicates not initialized!

void LCD_init(void) {
  /* Initialize backlight and data pin to output high */
  TFTLCD_DATA_DDR = 0xFF;
  TFTLCD_BL_DDR  = TFTLCD_BL_MASK;
  TFTLCD_BL_PORT = TFTLCD_BL_MASK;

  /* Initialize LCD control port register */
  DDRK  = INIT_DDR_MASK;
  PORTK = INIT_PORT_MASK;

  /* Reset screen */
  TFTLCD_RESET_PORT = RESET_A;
  _delay_ms(2.0);
  TFTLCD_RESET_PORT = RESET_B;
}

void LCD_write_byte(unsigned char data, unsigned long count) {
  /* Push the data */
  TFTLCD_DATA_PORT = data;
  while (count) {
    TFTLCD_WR_PORT = WR_WRITE_A;
    count--;
    TFTLCD_WR_PORT = WR_WRITE_B;
  }
}

void LCD_write_register(unsigned char cmd, unsigned char arg0, unsigned char arg1) {
  LCD_write_command(cmd);
  LCD_write_byte(arg1, 1);
  LCD_write_byte(arg0, 1);
}

void LCD_write_register(unsigned char cmd, unsigned int arg) {
  LCD_write_register(cmd, arg & 0xFF, arg >> 8);
}

void LCD_write_command(unsigned char cmd) {
  TFTLCD_WR_PORT = WR_COMMAND_WRITE_A;
  TFTLCD_DATA_PORT = 0;
  TFTLCD_WR_PORT = WR_COMMAND_WRITE_B;
  TFTLCD_WR_PORT = WR_COMMAND_WRITE_A;
  TFTLCD_DATA_PORT = cmd;
  TFTLCD_WR_PORT = WR_COMMAND_WRITE_B;
  TFTLCD_WR_PORT = WR_WRITE_B;
}

void LCD_write_line_raw(unsigned int x, unsigned int y, unsigned long length, unsigned char mode, unsigned char color) {
  /* Setup the CGRAM position to the specified x/y coordinate */
  LCD_write_register(LCD_CMD_GRAM_VER_AD, (LCD_WIDTH - 1) - x);
  LCD_write_register(LCD_CMD_GRAM_HOR_AD, (LCD_HEIGHT - 1) - y);

  /* Write the ENTRY_MOD, use the definition to save 2 bytes */
  LCD_write_register(LCD_CMD_ENTRY_MOD, mode, 0x10);

  /* Write CGRAM command, then push out the pixels */
  LCD_write_command(LCD_CMD_RW_GRAM);
  LCD_write_byte(color, length);
}

void drawIcon(uint16_t x, unsigned char idx) {
  uint8_t width, height;
  const unsigned char* data;
  uint16_t y;

  idx &= ICON_DATA_MASK;
  if (idx == ICON_FROM_NONE) {
    y = LCD_STATUSICON_Y;
    width = LCD_STATUSICON_W;
    height = LCD_STATUSICON_H;
    data = NULL;
  } else if (idx == ICON_SKETCH) {
    y = LCD_ICON_Y;
    width = LCD_ICON_W;
    height = LCD_ICON_H;
    data = volume_cacheBuffer.data;
  } else {
    y = LCD_STATUSICON_Y;
    width = LCD_STATUSICON_W;
    height = LCD_STATUSICON_H;
    data = ICONS[idx];
  }
  unsigned char pix_dat = 0, width_ctr;
  while (height--) {
    /* Set LCD cursor to start of line by drawing 0-length line */
    LCD_write_line(x, y++, 0, LCD_MODE_HOR, LCD_BLACK);

    width_ctr = width;
    do {
      /* Refresh pixel data every 8 pixels */
      if ((width_ctr & 0x7) == 0 && data) pix_dat = *data++;

      /* Push the pixel */
      LCD_write_byte((pix_dat & 0x1) ? LCD_ICON_COLOR : LCD_BLACK, 2);

      /* Next pixel data bit */
      pix_dat >>= 1;
    } while (--width_ctr);
  }
}

/* Clear the screen and write the frame, use NULL filename to wipe the screen only */
void LCD_write_frame(unsigned char iconFlags, char* sketchIconFile) {
  /* Clear the screen (frame) as needed */
  if (!lcd_icon_flags) {

    /* Delay needed after reset to give LCD time to initialize */
    _delay_ms(25.0);

    /* Initialize the LCD registers */
    const uint8_t *data = LCD_REG_DATA;
    const uint8_t *data_end = LCD_REG_DATA + sizeof(LCD_REG_DATA);
    do {
      LCD_write_register(data[0], data[1], data[2]);
    } while ((data += 3) != data_end);

    /* Fill with BLACK */
    LCD_write_line(0, 0, LCD_PIXELS, LCD_MODE_HOR, LCD_BLACK);

    /* If clearing, do not write a frame */
    if (sketchIconFile == NULL) return;

    /* Draw the box frame */
    LCD_write_rect(LCD_PROG_X, LCD_PROG_Y, LCD_PROG_W + LCD_PROG_STEP, LCD_PROG_H, LCD_FRAMECOLOR);

    /* Draw icon next */
    iconFlags |= ICON_FORCEDRAW;
  }

  if (iconFlags != ICON_PCIDLE) {
    unsigned char new_progress;
    if (lcd_icon_flags != iconFlags) {
      lcd_icon_flags = iconFlags;

      /* Update sketch icon if needed */
      if ((lcd_icon_flags & ICON_FORCEDRAW)) {
        if (file_open(sketchIconFile, "SKI", FILE_READ)) {
          volume_cacheCurrentBlock(0);
        } else {          
          /* Generate a default icon if we failed to open one */
          const unsigned int buff_len = 512;
          const unsigned int width = 3;
          const unsigned char mask_l = (1<<width)-1;
          const unsigned char mask_r = mask_l<<(8-width);
          const unsigned int buff_end_offset = buff_len - 8*(width+1);
          const uint8_t* data_end = volume_cacheBuffer.data + buff_end_offset;

          /* Set to block 0 which is never written and always re-read */
          volume_cacheBlockNumber = 0;
          memset(volume_cacheBuffer.data, 0xFF, 512);

          uint8_t* data = volume_cacheBuffer.data + 8*width;;
          uint8_t i = 32;
          do {
            *(data++) = (i==32) ? mask_l : (i==0) ? mask_r : 0x00;
            i += 32;
          } while (data < data_end);
        }
        drawIcon(LCD_ICON_X, ICON_SKETCH);
      }

      /* Update icons */
      // 0/1/2 -> Sidebar icons
      // 3 -> Sketch icon
      // 4 -> Sidebar no icon
      drawIcon(LCD_STATUSICON_X_A, iconFlags >> ICON_DATA_SHIFT);
      drawIcon(LCD_STATUSICON_X_B, iconFlags);

      /* Wipe current progress, reset marquee */
      lcd_progress = 0;
      lcd_progress_color = LCD_BLACK;
      new_progress = LCD_PROG_CNT;
    } else {
      /* Draw and update the marquee bar */
      uint8_t marqueeColor;

      /* FROM=NONE: no data, show NO_PROG indication */
      /* FROM/TO=SD: SD-card is accessed, show PC_SD indication */
      /* Any other case: Chip ROM is accessed, show PC_ROM indication */
      if (lcd_icon_flags & ICON_FROM_NONE) {
        marqueeColor = STATUS_COLOR_NOPROG;
      } else if (lcd_icon_flags & (ICON_FROM_SD | ICON_TO_SD)) {
        marqueeColor = STATUS_COLOR_PC_SD;
      } else {
        marqueeColor = STATUS_COLOR_PC_ROM;
      }

      if (lcd_progress >= LCD_PROG_CNT) {
        lcd_progress = 0;
        lcd_progress_color = (lcd_progress_color ? LCD_BLACK : marqueeColor);
      }
      new_progress = lcd_progress + LCD_PROG_MARQ_BATCH;
    }
    LCD_write_progress(new_progress);
  }

  /* No sketch: clear screen by wiping from top-left pixel to bottom-right pixel */
  if (sketchIconFile == NULL) {
    LCD_write_line(LCD_WIPE_X, LCD_WIPE_Y, LCD_WIPE_LENGTH, LCD_MODE_HOR, LCD_BLACK);
  }
}

void LCD_write_progress(unsigned long current, unsigned long total, unsigned char color) {
  lcd_progress_color = color;
  LCD_write_progress(LCD_PROG_CNT * current / (total + LCD_PROG_CNT));
}

void LCD_write_progress(unsigned char progress) {
  /* Reset progress as needed */
  if (progress < lcd_progress) {
    lcd_progress = 0;
  }

  /* Draw each progress bar line */
  unsigned char bar_x;
  while (lcd_progress < progress) {
    lcd_progress++;
    if (lcd_icon_flags & ICON_PROGRESS_INVERT) {
      bar_x = LCD_PROG_W + 1 - lcd_progress * LCD_PROG_STEP;
    } else {
      bar_x = lcd_progress * LCD_PROG_STEP;
    }
    LCD_write_line(LCD_PROG_X + bar_x, LCD_PROG_Y + 2, LCD_PROG_H - 4, LCD_MODE_VER, lcd_progress_color);
  }
}