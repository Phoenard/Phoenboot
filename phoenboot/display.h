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

/*
 * The display module is a minimalistic library for the ILI9325/ILI9328 TFT LCD.
 * Its main importance in the Phoenard OE is initializing the ports before use.
 * Display initialization and the drawing of icons/progress is implemented.
 * Functions are heavily optimized for size and may be slower than can be.
 *
 * Created: 05/06/2014 14:55:28
 * Author: Irmo van den Berge
 */ 

#include  <util/delay.h>
#include "PHNSDMinimal.h"
#include "PHNCore.h"

#ifndef LCDMINIMAL_H_
#define LCDMINIMAL_H_

#define LCD_WIDTH   320
#define LCD_HEIGHT  240
#define LCD_PIXELS  ((unsigned long) LCD_WIDTH * (unsigned long) LCD_HEIGHT)

#define LCD_MODE_HOR  0x8
#define LCD_MODE_VER  0x0
#define LCD_BLACK     0x00
#define LCD_WHITE     0xFF
#define LCD_GRAY      0xD5
#define LCD_YELLOW    0xEE
#define LCD_ORANGE    0xE4
#define LCD_RED       0xE0
#define LCD_GREEN     0xC7
#define LCD_BLUE      0x9E
#define LCD_CYAN      0xDF
#define LCD_PURPLE    0xDC

/* LCD progress status colors */
#define STATUS_COLOR_NOPROG  LCD_RED
#define STATUS_COLOR_SDLOAD  LCD_BLUE
#define STATUS_COLOR_SDSAVE  LCD_PURPLE
#define STATUS_COLOR_PC_ROM  LCD_GREEN
#define STATUS_COLOR_PC_SD   LCD_CYAN

/* Icon settings */
#define LCD_ICON_W      64
#define LCD_ICON_H      64
#define LCD_ICON_X      ((LCD_WIDTH - LCD_ICON_W) / 2)
#define LCD_ICON_Y      20
#define LCD_ICON_COLOR  LCD_WHITE
#define LCD_ICON_RIGHT  (LCD_WIDTH - LCD_ICON_W - LCD_ICON_X - 1)
#define LCD_ICON_LEFT   LCD_ICON_X

/* Progress bar settings */
#define LCD_PROG_STEP   2
#define LCD_PROG_MARQ_BATCH  4
#define LCD_PROG_CNT    (LCD_PROG_MARQ_BATCH * 27)
#define LCD_PROG_W      (LCD_PROG_CNT * LCD_PROG_STEP + 1)
#define LCD_PROG_H      32
#define LCD_PROG_X      ((LCD_WIDTH - LCD_PROG_W) / 2)
#define LCD_PROG_Y      ((LCD_HEIGHT - LCD_PROG_H) / 2)
#define LCD_PROG_LEFT   LCD_PROG_X
#define LCD_PROG_RIGHT  (LCD_WIDTH - LCD_PROG_W - LCD_PROG_LEFT - LCD_PROG_STEP - 1)
#define LCD_PROG_CLEAR  0xFF
#define LCD_FRAMECOLOR  LCD_WHITE

/* Device icon settings */
#define LCD_STATUSICON_W    32
#define LCD_STATUSICON_H    24
#define LCD_STATUSICON_X_A  (LCD_PROG_X + LCD_PROG_W + 5)
#define LCD_STATUSICON_X_B  (LCD_PROG_X - LCD_STATUSICON_W - 5)
#define LCD_STATUSICON_Y    LCD_PROG_Y + 3

/* LCD Register commands */
#define LCD_CMD_START_OSC          0x00
#define LCD_CMD_DRIV_OUT_CTRL      0x01
#define LCD_CMD_DRIV_WAV_CTRL      0x02
#define LCD_CMD_ENTRY_MOD          0x03
#define LCD_CMD_RESIZE_CTRL        0x04
#define LCD_CMD_DISP_CTRL1         0x07
#define LCD_CMD_DISP_CTRL2         0x08
#define LCD_CMD_DISP_CTRL3         0x09
#define LCD_CMD_DISP_CTRL4         0x0A
#define LCD_CMD_RGB_DISP_IF_CTRL1  0x0C
#define LCD_CMD_FRM_MARKER_POS     0x0D
#define LCD_CMD_RGB_DISP_IF_CTRL2  0x0F
#define LCD_CMD_POW_CTRL1          0x10
#define LCD_CMD_POW_CTRL2          0x11
#define LCD_CMD_POW_CTRL3          0x12
#define LCD_CMD_POW_CTRL4          0x13
#define LCD_CMD_GRAM_HOR_AD        0x20
#define LCD_CMD_GRAM_VER_AD        0x21
#define LCD_CMD_RW_GRAM            0x22
#define LCD_CMD_POW_CTRL7          0x29
#define LCD_CMD_FRM_RATE_COL_CTRL  0x2B
#define LCD_CMD_GAMMA_CTRL1        0x30
#define LCD_CMD_GAMMA_CTRL2        0x31
#define LCD_CMD_GAMMA_CTRL3        0x32
#define LCD_CMD_GAMMA_CTRL4        0x35
#define LCD_CMD_GAMMA_CTRL5        0x36
#define LCD_CMD_GAMMA_CTRL6        0x37
#define LCD_CMD_GAMMA_CTRL7        0x38
#define LCD_CMD_GAMMA_CTRL8        0x39
#define LCD_CMD_GAMMA_CTRL9        0x3C
#define LCD_CMD_GAMMA_CTRL10       0x3D
#define LCD_CMD_HOR_START_AD       0x50
#define LCD_CMD_HOR_END_AD         0x51
#define LCD_CMD_VER_START_AD       0x52
#define LCD_CMD_VER_END_AD         0x53
#define LCD_CMD_GATE_SCAN_CTRL1    0x60
#define LCD_CMD_GATE_SCAN_CTRL2    0x61
#define LCD_CMD_GATE_SCAN_CTRL3    0x6A
#define LCD_CMD_PART_IMG1_DISP_POS 0x80
#define LCD_CMD_PART_IMG1_START_AD 0x81
#define LCD_CMD_PART_IMG1_END_AD   0x82
#define LCD_CMD_PART_IMG2_DISP_POS 0x83
#define LCD_CMD_PART_IMG2_START_AD 0x84
#define LCD_CMD_PART_IMG2_END_AD   0x85
#define LCD_CMD_PANEL_IF_CTRL1     0x90
#define LCD_CMD_PANEL_IF_CTRL2     0x92
#define LCD_CMD_PANEL_IF_CTRL3     0x93
#define LCD_CMD_PANEL_IF_CTRL4     0x95
#define LCD_CMD_PANEL_IF_CTRL5     0x97
#define LCD_CMD_PANEL_IF_CTRL6     0x98

/* Status transferring icons */
#define ICON_DATA_MASK         0x7
#define ICON_DATA_SHIFT        3
#define ICON_FROM_COMPUTER     (0 << 0)
#define ICON_FROM_CHIPROM      (1 << 0)
#define ICON_FROM_SD           (2 << 0)
#define ICON_SKETCH            (3 << 0)
#define ICON_FROM_NONE         (4 << 0)
#define ICON_TO_COMPUTER       (0 << ICON_DATA_SHIFT)
#define ICON_TO_CHIPROM        (1 << ICON_DATA_SHIFT)
#define ICON_TO_SD             (2 << ICON_DATA_SHIFT)
#define ICON_TO_NONE           (4 << ICON_DATA_SHIFT)
#define ICON_PCIDLE            (ICON_FROM_COMPUTER | ICON_TO_NONE)
#define ICON_FORCEDRAW         (1 << 6)
#define ICON_PROGRESS_INVERT   (1 << 7)

void LCD_init(void);
void LCD_write_register(unsigned char cmd, unsigned char arg0, unsigned char arg1);
void LCD_write_register(unsigned char cmd, unsigned int arg);
void LCD_write_command(unsigned char cmd);
void LCD_write_byte(unsigned char data, unsigned long count);
void LCD_write_line_raw(unsigned int x, unsigned int y, unsigned long length, unsigned char mode, unsigned char color);

void LCD_write_frame(unsigned char iconFlags, char* sketchIconFile);
void LCD_write_progress(unsigned long current, unsigned long total, unsigned char color);
void LCD_write_progress(unsigned char progress);
void LCD_write_icon(const char* filename);

// Macro to perform line drawing, which includes calculations in the value
// Since the length is known at compile-time, a definition is used instead of a function
// This reduces the final binary size
#define LCD_write_line(x, y, length, mode, color)  LCD_write_line_raw(x, y, ((unsigned long) (length) << 1), mode, color)

// Macro to perform rectangle drawing, which includes calculations in the value
// Since the parameters are known at compile-time, a definition is used instead of a function
// This reduces the final binary size
#define LCD_write_rect(x, y, w, h, color) { \
  LCD_write_line(x, y, w, LCD_MODE_HOR, color); \
  LCD_write_line(x, y + h - 1, w, LCD_MODE_HOR, color); \
  LCD_write_line(x, y, h, LCD_MODE_VER, color); \
  LCD_write_line(x + w - 1, y, h, LCD_MODE_VER, color); \
}

#endif /* LCDMINIMAL_H_ */