/*
	This file is part of VGA_t4 library.

	VGA_t4 library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Copyright (C) 2020 J-M Harvengt

	Inspired from the original Teensy3 uVGA library of Eric PREVOTEAU.
	QTIMER/FlexIO code based on Teensy4 examples of KurtE, Manitou and easone 
	from the Teensy4 forum (https://forum.pjrc.com)
*/

#ifndef _VGA_T4_H
#define _VGA_T4_H

#include <Arduino.h>
#include <avr_emulation.h>
#include <DMAChannel.h>
#include <math.h>


// Enable debug info (requires serial initialization)
//#define DEBUG

// Enable 12bits mode
// Default is 8bits RRRGGGBB (332) 
// But 12bits GBB0RRRRGGGBB (444) feasible BUT NOT TESTED !!!!
//#define BITS12



#ifdef BITS12
typedef uint16_t vga_pixel;
#define VGA_RGB(r,g,b)  ( (((r>>3)&0x1f)<<11) | (((g>>2)&0x3f)<<5) | (((b>>3)&0x1f)<<0) )
#else
typedef uint8_t vga_pixel;
#define VGA_RGB(r,g,b)   ( (((r>>5)&0x07)<<5) | (((g>>5)&0x07)<<2) | (((b>>6)&0x3)<<0) )
#endif

typedef enum vga_mode_t
{
  VGA_MODE_320x240 = 0,
  VGA_MODE_320x480 = 1,
  VGA_MODE_352x240 = 2,
  VGA_MODE_352x480 = 3,
  VGA_MODE_512x240 = 4,
  VGA_MODE_512x480 = 5,
  VGA_MODE_640x240 = 6,
  VGA_MODE_640x480 = 7,
  VGA_MODE_544x240 = 8,
  VGA_MODE_544x480 = 9
} vga_mode_t;


typedef enum vga_error_t
{
	VGA_OK = 0,
	VGA_ERROR = -1
} vga_error_t;


#define DEFAULT_VSYNC_PIN 8


class VGA_T4
{
public:

  VGA_T4(int vsync_pin = DEFAULT_VSYNC_PIN);

  // display VGA image
  vga_error_t begin(vga_mode_t mode);
  void end();
  void debug();
  
  // retrieve real size of the frame buffer
  void get_frame_buffer_size(int *width, int *height);

  // wait next Vsync
  void waitSync();


  // =========================================================
  // graphic primitives
  // =========================================================

  void clear(vga_pixel col) ; 
  void drawPixel(int x, int y, vga_pixel color);
  vga_pixel getPixel(int x, int y);
  vga_pixel * getLineBuffer(int j);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, vga_pixel color);
  void drawText(int16_t x, int16_t y, const char * text, vga_pixel fgcolor, vga_pixel bgcolor, bool doublesize);
  void drawSprite(int16_t x, int16_t y, const vga_pixel *bitmap);
  void drawSprite(int16_t x, int16_t y, const vga_pixel *bitmap, uint16_t croparx, uint16_t cropary, uint16_t croparw, uint16_t croparh);
  void writeScreen(const vga_pixel *pcolors);  
  void writeLine(int width, int height, int y, vga_pixel *buf);
  void writeLine(int width, int height, int stride, uint8_t *buffer, vga_pixel *palette16);
  void writeScreen(int width, int height, int stride, uint8_t *buffer, vga_pixel *palette16);

  // ************************************** GFX API extension from darthvader ******************************************************
  void drawline(int16_t x1, int16_t y1, int16_t x2, int16_t y2, vga_pixel color);
  void drawcircle(int16_t x, int16_t y, uint16_t radius, vga_pixel color);
  void drawfilledcircle(int16_t x, int16_t y, int16_t radius, vga_pixel fillcolor, vga_pixel bordercolor);
  void drawellipse(int16_t cx, int16_t cy, uint16_t radius1, uint16_t radius2, vga_pixel color);
  void drawfilledellipse(int16_t cx, int16_t cy, uint16_t radius1, uint16_t radius2, vga_pixel fillcolor, vga_pixel bordercolor);
  void drawtriangle(int16_t ax, int16_t ay, int16_t bx, int16_t by, int16_t cx, int16_t cy, vga_pixel color);
  void drawfilledtriangle(int16_t ax, int16_t ay, int16_t bx, int16_t by, int16_t cx, int16_t cy, vga_pixel fillcolor, vga_pixel bordercolor);
  void drawquad(int16_t centerx, int16_t centery, int16_t w, int16_t h, int16_t angle, vga_pixel color);
  void drawfilledquad(int16_t centerx, int16_t centery, int16_t w, int16_t h, int16_t angle, vga_pixel fillcolor, vga_pixel bordercolor);
  // *******************************************************************************************************************************

private:
  static uint8_t _vsync_pin;
  static uint32_t currentLine;
  static DMAChannel flexio1DMA;
  static DMAChannel flexio2DMA; 

  static void QT3_isr(void);
};


#endif


