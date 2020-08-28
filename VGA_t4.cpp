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

#include "VGA_t4.h"
#include "VGA_font8x8.h"

// Objective:
// generates VGA signal fully in hardware with as little as possible CPU help

// Principle:
// QTimer3 (timer3) used to generate H-PUSE and line interrupt (and V-PULSE)
// 2 FlexIO shift registers (1 and 2) and 2 DMA channels used to generate
// RGB out, combined to create 8bits(/12bits) output.

// Note:
// - supported resolutions of 352x240,352x480,512x240 and 512x480 pixels
// - video memory is allocated using malloc in T4 heap
// - as the 2 DMA transfers are not started exactly at same time, there is a bit of color smearing 
//   but tried to be compensated by pixel shifting 
// - Default is 8bits RRRGGGBB (332) 
//   But 12bits GBB0RRRRGGGBB (444) feasible BUT NOT TESTED !!!!
// - Only ok at 600MHz else some disturbances visible
// - I did not tested on an HDMI display with VGA adapter


#define USE_VIDEO_PLL 1

#define TOP_BORDER    40
#define PIN_HBLANK    15

#ifdef BITS12
#define PIN_R_B3      5
#endif
#define PIN_R_B2      33
#define PIN_R_B1      4
#define PIN_R_B0      3
#define PIN_G_B2      2
#define PIN_G_B1      13
#define PIN_G_B0      11
#define PIN_B_B1      12
#define PIN_B_B0      10
#ifdef BITS12
#define PIN_B_B2      6
#define PIN_B_B3      9
#define PIN_G_B3      32
#endif

#define DMA_HACK      0x80

#define ABS(X)  ((X) > 0 ? (X) : -(X))

// Precomputed sinus and cosinus table from 0 to 359 degrees
// The tables are in Degrees not in Radian !
const float calcsi[360]={
		0.000001 ,                                                                                                                            //  0
		0.01745239 , 0.03489947 , 0.05233591 , 0.06975641 , 0.08715567 , 0.1045284 , 0.1218692 , 0.139173 , 0.1564343 , 0.173648 ,            // 1 à  10
		0.1908088 , 0.2079115 , 0.2249509 , 0.2419217 , 0.2588188 , 0.2756371 , 0.2923715 , 0.3090167 , 0.3255679 , 0.3420198 ,               // 11 à  20
		0.3583677 , 0.3746063 , 0.3907308 , 0.4067363 , 0.4226179 , 0.4383708 , 0.4539901 , 0.4694712 , 0.4848093 , 0.4999996 ,               // 21 à  30
		0.5150377 , 0.5299189 , 0.5446386 , 0.5591925 , 0.573576 , 0.5877848 , 0.6018146 , 0.615661 , 0.62932 , 0.6427872 ,                   // 31 à  40
		0.6560586 , 0.6691301 , 0.6819978 , 0.6946579 , 0.7071063 , 0.7193394 , 0.7313532 , 0.7431444 , 0.7547091 , 0.7660439 ,               // 41 à  50
		0.7771455 , 0.7880103 , 0.798635 , 0.8090165 , 0.8191515 , 0.8290371 , 0.8386701 , 0.8480476 , 0.8571668 , 0.8660249 ,                // 51 à  60
		0.8746193 , 0.8829472 , 0.8910061 , 0.8987936 , 0.9063074 , 0.913545 , 0.9205045 , 0.9271835 , 0.9335801 , 0.9396922 ,                // 61 à  70
		0.9455183 , 0.9510562 , 0.9563044 , 0.9612614 , 0.9659255 , 0.9702954 , 0.9743698 , 0.9781474 , 0.981627 , 0.9848075 ,                // 71 à  80
		0.9876881 , 0.9902679 , 0.992546 , 0.9945218 , 0.9961946 , 0.9975639 , 0.9986295 , 0.9993908 , 0.9998476 , 0.99999 ,                  // 81 à  90
		0.9998477 , 0.9993909 , 0.9986296 , 0.9975642 , 0.9961948 , 0.994522 , 0.9925463 , 0.9902682 , 0.9876886 , 0.984808 ,                 // 91 à  100
		0.9816275 , 0.9781479 , 0.9743704 , 0.9702961 , 0.9659262 , 0.9612621 , 0.9563052 , 0.9510571 , 0.9455191 , 0.9396932 ,               // 101 à  110
		0.933581 , 0.9271844 , 0.9205055 , 0.9135461 , 0.9063086 , 0.8987948 , 0.8910073 , 0.8829485 , 0.8746206 , 0.8660263 ,                // 111 à  120
		0.8571682 , 0.8480491 , 0.8386716 , 0.8290385 , 0.8191531 , 0.8090182 , 0.7986366 , 0.7880119 , 0.7771472 , 0.7660457 ,               // 121 à  130
		0.7547108 , 0.7431462 , 0.7313551 , 0.7193412 , 0.7071083 , 0.6946598 , 0.6819999 , 0.6691321 , 0.6560606 , 0.6427892 ,               // 131 à  140
		0.629322 , 0.6156631 , 0.6018168 , 0.5877869 , 0.5735782 , 0.5591948 , 0.5446408 , 0.5299212 , 0.5150401 , 0.5000019 ,                // 141 à  150
		0.4848116 , 0.4694737 , 0.4539925 , 0.4383733 , 0.4226205 , 0.4067387 , 0.3907333 , 0.3746087 , 0.3583702 , 0.3420225 ,               // 151 à  160
		0.3255703 , 0.3090193 , 0.2923741 , 0.2756396 , 0.2588214 , 0.2419244 , 0.2249534 , 0.2079142 , 0.1908116 , 0.1736506 ,               // 161 à  170
		0.156437 , 0.1391758 , 0.1218719 , 0.1045311 , 0.08715825 , 0.06975908 , 0.05233867 , 0.03490207 , 0.01745508 , 0.0277 ,              // 171 à  180
		-0.01744977 , -0.03489676 , -0.05233313 , -0.06975379 , -0.08715296 , -0.1045256 , -0.1218666 , -0.1391703 , -0.1564316 , -0.1736454 ,// 181 à  190
		-0.1908061 , -0.207909 , -0.2249483 , -0.241919 , -0.2588163 , -0.2756345 , -0.2923688 , -0.3090142 , -0.3255653 , -0.3420173 ,       // 191 à  200
		-0.3583652 , -0.3746038 , -0.3907282 , -0.4067339 , -0.4226155 , -0.4383683 , -0.4539878 , -0.4694688 , -0.4848068 , -0.4999973 ,     // 201 à  210
		-0.5150353 , -0.5299166 , -0.5446364 , -0.5591902 , -0.5735739 , -0.5877826 , -0.6018124 , -0.615659 , -0.6293178 , -0.642785 ,       // 211 à  220
		-0.6560566 , -0.6691281 , -0.6819958 , -0.694656 , -0.7071043 , -0.7193374 , -0.7313514 , -0.7431425 , -0.7547074 , -0.7660421 ,      // 221 à  230
		-0.7771439 , -0.7880087 , -0.7986334 , -0.8090149 , -0.8191499 , -0.8290355 , -0.8386687 , -0.8480463 , -0.8571655 , -0.8660236 ,     // 231 à  240
		-0.8746178 , -0.882946 , -0.8910049 , -0.8987925 , -0.9063062 , -0.9135439 , -0.9205033 , -0.9271825 , -0.9335791 , -0.9396913 ,      // 241 à  250
		-0.9455173 , -0.9510553 , -0.9563036 , -0.9612607 , -0.9659248 , -0.9702948 , -0.9743692 , -0.9781467 , -0.9816265 , -0.9848071 ,     // 251 à  260
		-0.9876878 , -0.9902675 , -0.9925456 , -0.9945215 , -0.9961944 , -0.9975638 , -0.9986293 , -0.9993907 , -0.9998476 , -0.99999 ,       // 261 à  270
		-0.9998478 , -0.9993909 , -0.9986298 , -0.9975643 , -0.9961951 , -0.9945223 , -0.9925466 , -0.9902686 , -0.987689 , -0.9848085 ,      // 271 à  280
		-0.981628 , -0.9781484 , -0.974371 , -0.9702968 , -0.965927 , -0.9612629 , -0.9563061 , -0.9510578 , -0.9455199 , -0.9396941 ,        // 281 à  290
		-0.933582 , -0.9271856 , -0.9205065 , -0.9135472 , -0.9063097 , -0.898796 , -0.8910086 , -0.8829498 , -0.8746218 , -0.8660276 ,       // 291 à  300
		-0.8571696 , -0.8480505 , -0.8386731 , -0.8290402 , -0.8191546 , -0.8090196 , -0.7986383 , -0.7880136 , -0.777149 , -0.7660476 ,      // 301 à  310
		-0.7547125 , -0.7431479 , -0.7313569 , -0.7193431 , -0.7071103 , -0.6946616 , -0.6820017 , -0.6691341 , -0.6560627 , -0.6427914 ,     // 311 à  320
		-0.6293243 , -0.6156651 , -0.6018188 , -0.5877892 , -0.5735805 , -0.5591971 , -0.5446434 , -0.5299233 , -0.5150422 , -0.5000043 ,     // 321 à  330
		-0.484814 , -0.4694761 , -0.4539948 , -0.4383755 , -0.4226228 , -0.4067413 , -0.3907359 , -0.3746115 , -0.3583725 , -0.3420248 ,      // 331 à  340
		-0.325573 , -0.3090219 , -0.2923768 , -0.2756425 , -0.2588239 , -0.2419269 , -0.2249561 , -0.2079169 , -0.1908143 , -0.1736531 ,      // 341 à  350
		-0.1564395 , -0.1391783 , -0.1218746 , -0.1045339 , -0.08716125 , -0.06976161 , -0.0523412 , -0.03490484 , -0.01745785 };             // 351 à  359

const float calcco[360]={
		0.99999 ,                                                                                                                            //  0
		0.9998477 , 0.9993908 , 0.9986295 , 0.9975641 , 0.9961947 , 0.9945219 , 0.9925462 , 0.9902681 , 0.9876884 , 0.9848078 ,              // 1 à  10
		0.9816272 , 0.9781477 , 0.9743701 , 0.9702958 , 0.9659259 , 0.9612617 , 0.9563049 , 0.9510566 , 0.9455186 , 0.9396928 ,              // 11 à  20
		0.9335806 , 0.927184 , 0.920505 , 0.9135456 , 0.906308 , 0.8987943 , 0.8910067 , 0.8829478 , 0.8746199 , 0.8660256 ,                 // 21 à  30
		0.8571675 , 0.8480483 , 0.8386709 , 0.8290379 , 0.8191524 , 0.8090173 , 0.7986359 , 0.7880111 , 0.7771463 , 0.7660448 ,              // 31 à  40
		0.75471 , 0.7431452 , 0.7313541 , 0.7193403 , 0.7071072 , 0.6946589 , 0.6819989 , 0.6691311 , 0.6560596 , 0.6427882 ,                // 41 à  50
		0.629321 , 0.6156621 , 0.6018156 , 0.5877859 , 0.5735771 , 0.5591936 , 0.5446398 , 0.52992 , 0.5150389 , 0.5000008 ,                 // 51 à  60
		0.4848104 , 0.4694724 , 0.4539914 , 0.438372 , 0.4226191 , 0.4067376 , 0.3907321 , 0.3746075 , 0.3583689 , 0.3420211 ,               // 61 à  70
		0.3255692 , 0.309018 , 0.2923728 , 0.2756384 , 0.2588201 , 0.241923 , 0.2249522 , 0.2079128 , 0.1908101 , 0.1736494 ,                // 71 à  80
		0.1564357 , 0.1391743 , 0.1218706 , 0.1045297 , 0.08715699 , 0.06975782 , 0.05233728 , 0.0349008 , 0.01745369 , 0.0138 ,             // 81 à  90
		-0.01745104 , -0.03489815 , -0.05233451 , -0.06975505 , -0.08715434 , -0.1045271 , -0.1218679 , -0.1391717 , -0.156433 , -0.1736467 ,// 91 à  100
		-0.1908075 , -0.2079102 , -0.2249495 , -0.2419204 , -0.2588175 , -0.2756359 , -0.2923701 , -0.3090155 , -0.3255666 , -0.3420185 ,    // 101 à  110
		-0.3583664 , -0.3746051 , -0.3907295 , -0.4067351 , -0.4226166 , -0.4383696 , -0.4539889 , -0.4694699 , -0.4848081 , -0.4999984 ,    // 111 à  120
		-0.5150366 , -0.5299177 , -0.5446375 , -0.5591914 , -0.5735749 , -0.5877837 , -0.6018136 , -0.6156599 , -0.6293188 , -0.6427862 ,    // 121 à  130
		-0.6560575 , -0.669129 , -0.6819969 , -0.6946569 , -0.7071053 , -0.7193384 , -0.7313522 , -0.7431435 , -0.7547083 , -0.7660431 ,     // 131 à  140
		-0.7771447 , -0.7880094 , -0.7986342 , -0.8090158 , -0.8191508 , -0.8290363 , -0.8386694 , -0.8480469 , -0.8571661 , -0.8660243 ,    // 141 à  150
		-0.8746186 , -0.8829465 , -0.8910055 , -0.898793 , -0.9063068 , -0.9135445 , -0.9205039 , -0.927183 , -0.9335796 , -0.9396918 ,      // 151 à  160
		-0.9455178 , -0.9510558 , -0.956304 , -0.9612611 , -0.9659252 , -0.9702951 , -0.9743695 , -0.978147 , -0.9816267 , -0.9848073 ,      // 161 à  170
		-0.9876879 , -0.9902677 , -0.9925459 , -0.9945216 , -0.9961945 , -0.9975639 , -0.9986294 , -0.9993907 , -0.9998476 , -0.99999 ,      // 171 à  180
		-0.9998477 , -0.9993909 , -0.9986297 , -0.9975642 , -0.9961949 , -0.9945222 , -0.9925465 , -0.9902685 , -0.9876888 , -0.9848083 ,    // 181 à  190
		-0.9816277 , -0.9781482 , -0.9743707 , -0.9702965 , -0.9659266 , -0.9612625 , -0.9563056 , -0.9510574 , -0.9455196 , -0.9396937 ,    // 191 à  200
		-0.9335815 , -0.927185 , -0.9205061 , -0.9135467 , -0.9063091 , -0.8987955 , -0.8910079 , -0.8829491 , -0.8746213 , -0.866027 ,      // 201 à  210
		-0.857169 , -0.8480497 , -0.8386723 , -0.8290394 , -0.8191538 , -0.8090189 , -0.7986375 , -0.7880127 , -0.7771481 , -0.7660466 ,     // 211 à  220
		-0.7547117 , -0.743147 , -0.731356 , -0.7193421 , -0.7071092 , -0.6946609 , -0.6820008 , -0.6691331 , -0.6560616 , -0.6427905 ,      // 221 à  230
		-0.6293229 , -0.6156641 , -0.6018178 , -0.5877882 , -0.5735794 , -0.5591961 , -0.5446419 , -0.5299222 , -0.5150412 , -0.5000032 ,    // 231 à  240
		-0.4848129 , -0.4694746 , -0.4539936 , -0.4383744 , -0.4226216 , -0.4067401 , -0.3907347 , -0.3746099 , -0.3583714 , -0.3420237 ,    // 241 à  250
		-0.3255718 , -0.3090207 , -0.2923756 , -0.2756409 , -0.2588227 , -0.2419256 , -0.2249549 , -0.2079156 , -0.1908126 , -0.1736519 ,    // 251 à  260
		-0.1564383 , -0.139177 , -0.1218734 , -0.1045326 , -0.08715951 , -0.06976035 , -0.05233994 , -0.03490358 , -0.01745659 , -0.0427 ,   // 261 à  270
		0.01744851 , 0.0348955 , 0.05233186 , 0.06975229 , 0.08715146 , 0.1045246 , 0.1218654 , 0.139169 , 0.1564303 , 0.1736439 ,           // 271 à  280
		0.1908047 , 0.2079078 , 0.224947 , 0.2419178 , 0.2588149 , 0.2756331 , 0.2923674 , 0.309013 , 0.3255641 , 0.3420161 ,                // 281 à  290
		0.3583638 , 0.3746024 , 0.3907273 , 0.4067327 , 0.4226143 , 0.4383671 , 0.4539864 , 0.4694674 , 0.4848059 , 0.4999962 ,              // 291 à  300
		0.5150342 , 0.5299154 , 0.5446351 , 0.559189 , 0.5735728 , 0.5877816 , 0.6018113 , 0.6156578 , 0.6293167 , 0.6427839 ,               // 301 à  310
		0.6560556 , 0.6691272 , 0.6819949 , 0.6946549 , 0.7071033 , 0.7193366 , 0.7313506 , 0.7431416 , 0.7547064 , 0.7660413 ,              // 311 à  320
		0.7771428 , 0.7880079 , 0.7986327 , 0.8090141 , 0.8191492 , 0.8290347 , 0.8386678 , 0.8480456 , 0.8571648 , 0.8660229 ,              // 321 à  330
		0.8746172 , 0.8829452 , 0.8910043 , 0.8987919 , 0.9063057 , 0.9135434 , 0.9205029 , 0.9271819 , 0.9335786 , 0.9396909 ,              // 331 à  340
		0.9455169 , 0.9510549 , 0.9563032 , 0.9612602 , 0.9659245 , 0.9702945 , 0.9743689 , 0.9781465 , 0.9816261 , 0.9848069 ,              // 341 à  350
		0.9876875 , 0.9902673 , 0.9925455 , 0.9945213 , 0.9961942 , 0.9975637 , 0.9986292 , 0.9993906 , 0.9998476 };                         // 351 à  359


// Full buffer including back/front porch 
static vga_pixel * gfxbuffer __attribute__((aligned(32)));
static uint32_t dstbuffer __attribute__((aligned(32)));

// Visible vuffer
static vga_pixel * framebuffer;
static int  fb_width;
static int  fb_height;
static int  fb_stride;
static int  maxpixperline;
static int  left_border;
static int  right_border;
static int  line_double;
static int  pix_shift;

#ifdef DEBUG
static uint32_t   ISRTicks_prev = 0;
volatile uint32_t ISRTicks = 0;
#endif 

uint8_t    VGA_T4::_vsync_pin = -1;
uint32_t   VGA_T4::currentLine=0;
DMAChannel VGA_T4::flexio1DMA;
DMAChannel VGA_T4::flexio2DMA; 
static volatile uint32_t VSYNC = 0;
#define NOP asm volatile("nop\n\t");


FASTRUN void VGA_T4::QT3_isr(void) {
  TMR3_SCTRL3 &= ~(TMR_SCTRL_TCF);
  TMR3_CSCTRL3 &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);

  cli();
  
  // V-PULSE
  if (currentLine > 1) {
    digitalWrite(_vsync_pin, 1);
    VSYNC = 0;
  } else {
    digitalWrite(_vsync_pin, 0);
    VSYNC = 1;
  }
  
  currentLine++;
  currentLine = currentLine % 525;

  uint32_t y = (currentLine - TOP_BORDER) >> line_double;
  // Visible area  
  if (y >= 0 && y < fb_height) {                          
    /*
    flexio1DMA.disable();
    flexio2DMA.disable();
    flexio1DMA.sourceBuffer((uint32_t *)&gfxbuffer[fb_stride*y], fb_stride); 
    flexio1DMA.destination(FLEXIO1_SHIFTBUFNBS0);
    flexio1DMA.disableOnCompletion();
    flexio2DMA.sourceBuffer((uint32_t *)&gfxbuffer[fb_stride*y], fb_stride); 
    flexio2DMA.destination(FLEXIO2_SHIFTBUF0);
    flexio2DMA.disableOnCompletion();
    flexio2DMA.enable();
    flexio1DMA.enable();
    */

    // Disable DMAs
    DMA_CERQ = flexio2DMA.channel;
    DMA_CERQ = flexio1DMA.channel; 
    // Setup source adress
    // Aligned 32 bits copy
#ifdef USE_VIDEO_PLL 
    // DMA delay is different in this case...
    unsigned long * p=(uint32_t *)&gfxbuffer[fb_stride*y+4];
#else
    unsigned long * p=(uint32_t *)&gfxbuffer[fb_stride*y];
#endif    
    flexio2DMA.TCD->SADDR = p;
    if (pix_shift & DMA_HACK) 
    {
      // Unaligned copy
      uint8_t * p2=(uint8_t *)&gfxbuffer[fb_stride*y+(pix_shift&0xf)];
      flexio1DMA.TCD->SADDR = p2;
    }
    else  {
      p=(uint32_t *)&gfxbuffer[fb_stride*y+(pix_shift&0xf)];
      flexio1DMA.TCD->SADDR = p;
    }
    // Enable DMAs
    DMA_SERQ = flexio2DMA.channel; 
    if (fb_width <= 544)
      DMA_SERQ = flexio1DMA.channel; 
    arm_dcache_flush((void*)((uint32_t *)&gfxbuffer[fb_stride*y]), fb_stride);
  }
  sei();  

#ifdef DEBUG
  ISRTicks++; 
#endif  
  asm volatile("dsb");
}


VGA_T4::VGA_T4(int vsync_pin = DEFAULT_VSYNC_PIN)
{
  _vsync_pin = vsync_pin;
}

// VGA 640x480@60Hz
// Screen refresh rate 60 Hz
// Vertical refresh    31.46875 kHz
// Pixel freq.         25.175 MHz
//
// Visible area        640  25.422045680238 us
// Front porch         16   0.63555114200596 us
// Sync pulse          96   3.8133068520357 us
// Back porch          48   1.9066534260179 us
// Whole line          800  31.777557100298 us

#define frame_freq     60.0     // Hz
#define line_freq      31.46875 // KHz
#define pix_freq       (line_freq*800) // KHz (25.175 MHz)

// pix_period = 39.7ns
// H-PULSE is 3.8133us = 3813.3ns => 96 pixels (see above for the rest)
#define frontporch_pix  16
#define backporch_pix   48 //16 //48

// Flexio Clock
// PLL3 SW CLOCK    (3) => 480 MHz
// PLL5 VIDEO CLOCK (2) => See formula for clock (we take 604200 KHz as /24 it gives 25175)
#define FLEXIO_CLK_SEL_PLL3 3
#define FLEXIO_CLK_SEL_PLL5 2


/* Set video PLL */
// There are /1, /2, /4, /8, /16 post dividers for the Video PLL. 
// The output frequency can be set by programming the fields in the CCM_ANALOG_PLL_VIDEO, 
// and CCM_ANALOG_MISC2 register sets according to the following equation.
// PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM)

// nfact: 
// This field controls the PLL loop divider. 
// Valid range for DIV_SELECT divider value: 27~54.

#define POST_DIV_SELECT 2
static void set_videoClock(int nfact, int32_t nmult, uint32_t ndiv, bool force) // sets PLL5
{
if (!force && (CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_ENABLE)) return;

    CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE
 			             | CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(1) // 2: 1/1; 1: 1/2; 0: 1/4
 			             | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(nfact);  
 	CCM_ANALOG_PLL_VIDEO_NUM   = nmult /*& CCM_ANALOG_PLL_VIDEO_NUM_MASK*/;
 	CCM_ANALOG_PLL_VIDEO_DENOM = ndiv /*& CCM_ANALOG_PLL_VIDEO_DENOM_MASK*/;  	
 	CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_POWERDOWN;//Switch on PLL
 	while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK)) {}; //Wait for pll-lock  	
   	
   	const int div_post_pll = 1; // other values: 2,4
  
  	if(div_post_pll>3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;  	
  	CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_BYPASS;//Disable Bypass
}


// display VGA image
vga_error_t VGA_T4::begin(vga_mode_t mode)
{
  uint32_t flexio_clock_div;

  // Default PLL3
  int flexio_clk_sel = FLEXIO_CLK_SEL_PLL3;
  int flexio_freq = 480000;
  int div_select;
  int num ;
  int denom;  
  switch(mode) {

#ifdef USE_VIDEO_PLL
    case VGA_MODE_320x240:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/2;
      right_border = frontporch_pix/2;
      fb_width = 320;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/2);
      line_double = 1;
      pix_shift = 2+DMA_HACK;
      break;
    case VGA_MODE_320x480:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/2;
      right_border = frontporch_pix/2;
      fb_width = 320;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/2); 
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;   

    case VGA_MODE_640x240:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 640;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/pix_freq;
      line_double = 1;
      pix_shift = 0; //1+DMA_HACK;
      break;
    case VGA_MODE_640x480:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 640;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = (flexio_freq/pix_freq); 
      line_double = 0;
      pix_shift = 0; //1+DMA_HACK;
      break;   

    case VGA_MODE_544x240:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 544;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.2)+2;
      line_double = 1;
      pix_shift = 0;//+DMA_HACK;
      break;
    case VGA_MODE_544x480:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 544;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.2)+2;
      line_double = 0;
      pix_shift = 0;//+DMA_HACK;
      break;   

    case VGA_MODE_512x240:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3); 
      line_double = 1;
      pix_shift = 0;//+DMA_HACK;
      break;
    case VGA_MODE_512x480:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3); 
      line_double = 0;
      pix_shift = 0;//+DMA_HACK;
      break; 

    case VGA_MODE_352x240:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/1.75;
      right_border = frontporch_pix/1.75;
      fb_width = 352;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.75); 
      line_double = 1;
      pix_shift = 2+DMA_HACK;
      break;
    case VGA_MODE_352x480:
      flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
      div_select = 49;
      num = 135;
      denom = 100;
      flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
      set_videoClock(div_select,num,denom,true);     
      left_border = backporch_pix/1.75;
      right_border = frontporch_pix/1.75;
      fb_width = 352;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.75); 
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;       
#else
    case VGA_MODE_320x240:
      left_border = backporch_pix/2;
      right_border = frontporch_pix/2;
      fb_width = 320;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/2);
      line_double = 1;
      pix_shift = 2+DMA_HACK;
      break;
    case VGA_MODE_320x480:
      left_border = backporch_pix/2;
      right_border = frontporch_pix/2;
      fb_width = 320;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/2);
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;

    case VGA_MODE_640x240:
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 640;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/pix_freq;
      line_double = 1;
      pix_shift = 2+DMA_HACK;
      break;
    case VGA_MODE_640x480:
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 640;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/pix_freq;
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;     

    case VGA_MODE_544x240:
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 544;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.1)+4; //1.2???
      line_double = 1;
      pix_shift = 0; //2+DMA_HACK;
      break;
    case VGA_MODE_544x480:
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 544;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.1)+4; //1.2???
      line_double = 0;
      pix_shift = 0; //2+DMA_HACK;
      break;

   case VGA_MODE_512x240:
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3);
      line_double = 1;
      pix_shift = 0; //2+DMA_HACK;
      break;
    case VGA_MODE_512x480:
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3);
      line_double = 0;
      pix_shift = 0; //2+DMA_HACK;
      break;

   case VGA_MODE_352x240:
      left_border = backporch_pix/1.75;
      right_border = frontporch_pix/1.75;
      fb_width = 352;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.75);
      line_double = 1;
      pix_shift = 2+DMA_HACK;
      break;
    case VGA_MODE_352x480:
      left_border = backporch_pix/1.75;
      right_border = frontporch_pix/1.75;
      fb_width = 352;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.75);
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;
#endif   
  }	

  Serial.println("frequency");
  Serial.println(flexio_freq);
  Serial.println("div");
  Serial.println(flexio_freq/pix_freq);

  pinMode(_vsync_pin, OUTPUT);
  pinMode(PIN_HBLANK, OUTPUT);

  /* Basic pin setup FlexIO1 */
  pinMode(PIN_G_B2, OUTPUT);  // FlexIO1:4 = 0x10
  pinMode(PIN_R_B0, OUTPUT);  // FlexIO1:5 = 0x20
  pinMode(PIN_R_B1, OUTPUT);  // FlexIO1:6 = 0x40
  pinMode(PIN_R_B2, OUTPUT);  // FlexIO1:7 = 0x80
#ifdef BITS12
  pinMode(PIN_R_B3, OUTPUT);  // FlexIO1:8 = 0x100
#endif
  /* Basic pin setup FlexIO2 */
  pinMode(PIN_B_B0, OUTPUT);  // FlexIO2:0 = 0x00001
  pinMode(PIN_B_B1, OUTPUT);  // FlexIO2:1 = 0x00002
  pinMode(PIN_G_B0, OUTPUT);  // FlexIO2:2 = 0x00004
  pinMode(PIN_G_B1, OUTPUT);  // FlexIO2:3 = 0x00008
#ifdef BITS12
  pinMode(PIN_B_B2, OUTPUT);  // FlexIO2:10 = 0x00400
  pinMode(PIN_B_B3, OUTPUT);  // FlexIO2:11 = 0x00800
  pinMode(PIN_G_B3, OUTPUT);  // FlexIO2:12 = 0x01000
#endif

  /* High speed and drive strength configuration */
  *(portControlRegister(PIN_G_B2)) = 0xFF; 
  *(portControlRegister(PIN_R_B0)) = 0xFF;
  *(portControlRegister(PIN_R_B1)) = 0xFF;
  *(portControlRegister(PIN_R_B2)) = 0xFF;
#ifdef BITS12
  *(portControlRegister(PIN_R_B3)) = 0xFF;
#endif
  *(portControlRegister(PIN_B_B0)) = 0xFF; 
  *(portControlRegister(PIN_B_B1)) = 0xFF;
  *(portControlRegister(PIN_G_B0)) = 0xFF;
  *(portControlRegister(PIN_G_B1)) = 0xFF;
#ifdef BITS12  
  *(portControlRegister(PIN_B_B2))  = 0xFF;
  *(portControlRegister(PIN_B_B3))  = 0xFF;
  *(portControlRegister(PIN_G_B3)) = 0xFF;
#endif


  /* Set clock for FlexIO1 and FlexIO2 */
  CCM_CCGR5 &= ~CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  CCM_CDCDR = (CCM_CDCDR & ~(CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(7) | CCM_CDCDR_FLEXIO1_CLK_PODF(7))) 
    | CCM_CDCDR_FLEXIO1_CLK_SEL(flexio_clk_sel) | CCM_CDCDR_FLEXIO1_CLK_PRED(0) | CCM_CDCDR_FLEXIO1_CLK_PODF(0);
  CCM_CCGR3 &= ~CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  CCM_CSCMR2 = (CCM_CSCMR2 & ~(CCM_CSCMR2_FLEXIO2_CLK_SEL(3))) | CCM_CSCMR2_FLEXIO2_CLK_SEL(flexio_clk_sel);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7)|CCM_CS1CDR_FLEXIO2_CLK_PODF(7)) )
    | CCM_CS1CDR_FLEXIO2_CLK_PRED(0) | CCM_CS1CDR_FLEXIO2_CLK_PODF(0);


 /* Set up pin mux FlexIO1 */
  *(portConfigRegister(PIN_G_B2)) = 0x14;
  *(portConfigRegister(PIN_R_B0)) = 0x14;
  *(portConfigRegister(PIN_R_B1)) = 0x14;
  *(portConfigRegister(PIN_R_B2)) = 0x14;
#ifdef BITS12
  *(portConfigRegister(PIN_R_B3)) = 0x14;
#endif
  /* Set up pin mux FlexIO2 */
  *(portConfigRegister(PIN_B_B0)) = 0x14;
  *(portConfigRegister(PIN_B_B1)) = 0x14;
  *(portConfigRegister(PIN_G_B0)) = 0x14;
  *(portConfigRegister(PIN_G_B1)) = 0x14;
#ifdef BITS12
  *(portConfigRegister(PIN_B_B2)) = 0x14;
  *(portConfigRegister(PIN_B_B3)) = 0x14;
  *(portConfigRegister(PIN_G_B3)) = 0x14;
#endif

  /* Enable the clock */
  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  /* Enable the FlexIO with fast access */
  FLEXIO1_CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;
  FLEXIO2_CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;

  uint32_t timerSelect, timerPolarity, pinConfig, pinSelect, pinPolarity, shifterMode, parallelWidth, inputSource, stopBit, startBit;
  uint32_t triggerSelect, triggerPolarity, triggerSource, timerMode, timerOutput, timerDecrement, timerReset, timerDisable, timerEnable;

  /* Shifter 0 registers for FlexIO2 */ 
#ifdef BITS12
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(16); // 16-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(0);      // Select pins FXIO_D0 through FXIO_D12
#else
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(4);  // 8-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(0);      // Select pins FXIO_D0 through FXIO_D3
#endif
  inputSource = FLEXIO_SHIFTCFG_INSRC*(0);    // Input source from Shifter 1
  stopBit = FLEXIO_SHIFTCFG_SSTOP(0);         // Stop bit disabled
  startBit = FLEXIO_SHIFTCFG_SSTART(0);       // Start bit disabled, transmitter loads data on enable 
  timerSelect = FLEXIO_SHIFTCTL_TIMSEL(0);    // Use timer 0
  timerPolarity = FLEXIO_SHIFTCTL_TIMPOL*(1); // Shift on negedge of clock 
  pinConfig = FLEXIO_SHIFTCTL_PINCFG(3);      // Shifter pin output
  pinPolarity = FLEXIO_SHIFTCTL_PINPOL*(0);   // Shifter pin active high polarity
  shifterMode = FLEXIO_SHIFTCTL_SMOD(2);      // Shifter transmit mode
  /* Shifter 0 registers for FlexIO1 */
  FLEXIO2_SHIFTCFG0 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO2_SHIFTCTL0 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  FLEXIO2_SHIFTCFG1 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO2_SHIFTCTL1 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  FLEXIO2_SHIFTCFG2 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO2_SHIFTCTL2 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  FLEXIO2_SHIFTCFG3 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO2_SHIFTCTL3 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
#ifdef BITS12
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(5);  // 5-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D8
#else
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(4);  // 8-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D7
#endif  
  FLEXIO1_SHIFTCFG0 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL0 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
#ifdef COMBINED_SHIFTREGISTERS  
  FLEXIO1_SHIFTCFG1 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL1 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  FLEXIO1_SHIFTCFG2 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL2 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  FLEXIO1_SHIFTCFG3 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL3 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
#endif  
  /* Timer 0 registers for FlexIO2 */ 
  timerOutput = FLEXIO_TIMCFG_TIMOUT(1);      // Timer output is logic zero when enabled and is not affected by the Timer reset
  timerDecrement = FLEXIO_TIMCFG_TIMDEC(0);   // Timer decrements on FlexIO clock, shift clock equals timer output
  timerReset = FLEXIO_TIMCFG_TIMRST(0);       // Timer never reset
  timerDisable = FLEXIO_TIMCFG_TIMDIS(2);     // Timer disabled on Timer compare
  timerEnable = FLEXIO_TIMCFG_TIMENA(2);      // Timer enabled on Trigger assert
  stopBit = FLEXIO_TIMCFG_TSTOP(0);           // Stop bit disabled
  startBit = FLEXIO_TIMCFG_TSTART*(0);        // Start bit disabled
  triggerSelect = FLEXIO_TIMCTL_TRGSEL(1+4*(0)); // Trigger select Shifter 0 status flag
  triggerPolarity = FLEXIO_TIMCTL_TRGPOL*(1); // Trigger active low
  triggerSource = FLEXIO_TIMCTL_TRGSRC*(1);   // Internal trigger selected
  pinConfig = FLEXIO_TIMCTL_PINCFG(0);        // Timer pin output //3??
  pinSelect = FLEXIO_TIMCTL_PINSEL(0);        // Select pin FXIO_D0
  pinPolarity = FLEXIO_TIMCTL_PINPOL*(0);     // Timer pin polarity active high
  timerMode = FLEXIO_TIMCTL_TIMOD(1);         // Dual 8-bit counters baud mode
  // flexio_clock_div : Output clock frequency is N times slower than FlexIO clock (41.7 ns period) (23.980MHz?)
#ifdef BITS12
  #define SHIFTS_PER_TRANSFER 8 // Shift out 8 times with every transfer = two 32-bit words = contents of Shifter 0 (16bits)
#else
#ifdef COMBINED_SHIFTREGISTERS
#endif  
  #define SHIFTS_PER_TRANSFER 4 // Shift out 4 times with every transfer = two 32-bit words = contents of Shifter 0 
#endif
  FLEXIO2_TIMCFG0 = timerOutput | timerDecrement | timerReset | timerDisable | timerEnable | stopBit | startBit;
  FLEXIO2_TIMCTL0 = triggerSelect | triggerPolarity | triggerSource | pinConfig | pinSelect | pinPolarity | timerMode;
  FLEXIO2_TIMCMP0 = ((SHIFTS_PER_TRANSFER*2-1)<<8) | ((flexio_clock_div/2-1)<<0);
  /* Timer 0 registers for FlexIO1 */
  FLEXIO1_TIMCFG0 = timerOutput | timerDecrement | timerReset | timerDisable | timerEnable | stopBit | startBit;
  FLEXIO1_TIMCTL0 = triggerSelect | triggerPolarity | triggerSource | pinConfig | pinSelect | pinPolarity | timerMode;
  FLEXIO1_TIMCMP0 = ((SHIFTS_PER_TRANSFER*2-1)<<8) | ((flexio_clock_div/2-1)<<0);
#ifdef DEBUG
  Serial.println("FlexIO setup complete");
#endif

  /* Enable DMA trigger on Shifter0, DMA request is generated when data is transferred from buffer0 to shifter0 */
  FLEXIO2_SHIFTSDEN |= (1<<0);
  FLEXIO1_SHIFTSDEN |= (1<<0);
  /* Disable DMA channel so it doesn't start transferring yet */
  flexio1DMA.disable();
  flexio2DMA.disable();
  /* Set up DMA channel to use Shifter 0 trigger */
  flexio1DMA.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO1_REQUEST0);
  flexio2DMA.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);

#ifdef COMBINED_SHIFTREGISTERS
  flexio2DMA.TCD->NBYTES = 4;
  flexio2DMA.TCD->SOFF = 4;
  flexio2DMA.TCD->SLAST = 0;
  flexio2DMA.TCD->BITER = 64 / 4;
  flexio2DMA.TCD->CITER = 64 / 4;
  flexio2DMA.TCD->DADDR = &FLEXIO2_SHIFTBUF0;
  flexio2DMA.TCD->DOFF = 0;
  flexio2DMA.TCD->DLASTSGA = 0;
  flexio2DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
  flexio2DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;

  flexio1DMA.TCD->NBYTES = 4;
  flexio1DMA.TCD->SOFF = 4;
  flexio1DMA.TCD->SLAST = 0;
  flexio1DMA.TCD->BITER = 64 / 4;;
  flexio1DMA.TCD->CITER = 64 / 4;;
  flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
  flexio1DMA.TCD->DOFF = 0;
  flexio1DMA.TCD->DLASTSGA = 0;
  flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
  flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
#else 
  // Setup DMA2 Flexio2 copy
  flexio2DMA.TCD->NBYTES = 4;
  flexio2DMA.TCD->SOFF = 4;
  flexio2DMA.TCD->SLAST = -maxpixperline;
  flexio2DMA.TCD->BITER = maxpixperline / 4;
  flexio2DMA.TCD->CITER = maxpixperline / 4;
  flexio2DMA.TCD->DADDR = &FLEXIO2_SHIFTBUF0;
  flexio2DMA.TCD->DOFF = 0;
  flexio2DMA.TCD->DLASTSGA = 0;
  flexio2DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
  flexio2DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;

  // Setup DMA1 Flexio1 copy
  // Use pixel shift to avoid color smearing?
  if (pix_shift & DMA_HACK)
  {
    if (pix_shift & 0x3 == 0) {
      // Aligned 32 bits copy (32bits to 32bits)
      flexio1DMA.TCD->NBYTES = 4;
      flexio1DMA.TCD->SOFF = 4;
      flexio1DMA.TCD->SLAST = -maxpixperline;
      flexio1DMA.TCD->BITER = maxpixperline / 4;
      flexio1DMA.TCD->CITER = maxpixperline / 4;
      flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
      flexio1DMA.TCD->DOFF = 0;
      flexio1DMA.TCD->DLASTSGA = 0;
      flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits to 32bits
      flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
    }
    else {
      // Unaligned (source) 32 bits copy (8bits to 32bits)
      flexio1DMA.TCD->NBYTES = 4;
      flexio1DMA.TCD->SOFF = 1;
      flexio1DMA.TCD->SLAST = -maxpixperline;
      flexio1DMA.TCD->BITER = maxpixperline / 4;
      flexio1DMA.TCD->CITER = maxpixperline / 4;
      flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
      flexio1DMA.TCD->DOFF = 0;
      flexio1DMA.TCD->DLASTSGA = 0;
      flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(2); // 8bits to 32bits
      flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ; // disable on completion
    }	
  }
  else 
  {
      // Aligned 32 bits copy
      flexio1DMA.TCD->NBYTES = 4;
      flexio1DMA.TCD->SOFF = 4;
      flexio1DMA.TCD->SLAST = -maxpixperline;
      flexio1DMA.TCD->BITER = maxpixperline / 4;
      flexio1DMA.TCD->CITER = maxpixperline / 4;
      flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
      flexio1DMA.TCD->DOFF = 0;
      flexio1DMA.TCD->DLASTSGA = 0;
      flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
      flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
  } 
#endif


#ifdef DEBUG
  Serial.println("DMA setup complete");
#endif

  // enable clocks for QTIMER3: generates the 15KHz for hsync
  // Pulse:
  // low  : 3.8133 us => 569x6.7ns
  // total: 31.777 us => 4743x6.7ns (high = 4174x6.7ns)
  // (OLD TEST)
  // (4us low, 28us high => 32us)
  // (597x6.7ns for 4us)
  // (4179x6.7ns for 28us) 
  CCM_CCGR6 |= 0xC0000000;              //enable clocks to CG15 of CGR6 for QT3
  //configure QTIMER3 Timer3 for test of alternating Compare1 and Compare2
  
  #define MARGIN_N 1005 //8
  #define MARGIN_D 1000

  TMR3_CTRL3 = 0b0000000000100000;      //stop all functions of timer 
  // Invert output pin as we want the interupt on rising edge
  TMR3_SCTRL3 = 0b0000000000000011;     //0(TimerCompareFlag),0(TimerCompareIntEnable),00(TimerOverflow)0000(NoCapture),0000(Capture Disabled),00, 1(INV output),1(OFLAG to Ext Pin)
  TMR3_CNTR3 = 0;
  TMR3_LOAD3 = 0;
  /* Inverted timings */
  TMR3_COMP13 = ((4174*MARGIN_N)/MARGIN_D)-1;
  TMR3_CMPLD13 = ((4174*MARGIN_N)/MARGIN_D)-1;
  TMR3_COMP23 = ((569*MARGIN_N)/MARGIN_D)-1;
  TMR3_CMPLD23 = ((569*MARGIN_N)/MARGIN_D)-1;

  TMR3_CSCTRL3 = 0b0000000010000101;    //Compare1 only enabled - Compare Load1 control and Compare Load2 control both on
  TMR3_CTRL3 = 0b0011000000100100;      // 001(Count rising edges Primary Source),1000(IP Bus Clock),00 (Secondary Source), 
                                        // 0(Count Once),1(Count up to Compare),0(Count Up),0(Co Channel Init),100(Toggle OFLAG on alternating Compare1/Compare2)
  //configure Teensy pin Compare output
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 1;      // QT3 Timer3 is now on pin 15
  attachInterruptVector(IRQ_QTIMER3, QT3_isr);  //declare which routine performs the ISR function
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);  
#ifdef DEBUG
  Serial.println("QTIMER3 setup complete");
  Serial.print("V-PIN is ");
  Serial.println(_vsync_pin);
#endif

  /* initialize gfx buffer */
  gfxbuffer = (vga_pixel*)malloc(fb_stride*fb_height*sizeof(vga_pixel)+4); // 4bytes for pixel shift 
  if (gfxbuffer == NULL) return(VGA_ERROR);

  memset((void*)&gfxbuffer[0],0, fb_stride*fb_height*sizeof(vga_pixel)+4);
  framebuffer = (vga_pixel*)&gfxbuffer[left_border];

  return(VGA_OK);
}

void VGA_T4::end()
{
  cli(); 
  /* Disable DMA channel so it doesn't start transferring yet */
  flexio1DMA.disable();
  flexio2DMA.disable(); 
  FLEXIO2_SHIFTSDEN &= ~(1<<0);
  FLEXIO1_SHIFTSDEN &= ~(1<<0);
  /* disable clocks for flexio and qtimer */
  CCM_CCGR5 &= ~CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  CCM_CCGR3 &= ~CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  CCM_CCGR6 &= ~0xC0000000;
  sei(); 
  delay(50);
  if (gfxbuffer != NULL) free(gfxbuffer); 
}

void VGA_T4::debug()
{
#ifdef DEBUG
  delay(1000);
  uint32_t t=ISRTicks;
  if (ISRTicks_prev != 0) Serial.println(t-ISRTicks_prev);
  ISRTicks_prev = t;
#endif  
}

// retrieve size of the frame buffer
void VGA_T4::get_frame_buffer_size(int *width, int *height)
{
  *width = fb_width;
  *height = fb_height;
}

void VGA_T4::waitSync()
{
  while (VSYNC == 0) {};
}


void VGA_T4::clear(vga_pixel color) {
  int i,j;
  for (j=0; j<fb_height; j++)
  {
    vga_pixel * dst=&framebuffer[j*fb_stride];
    for (i=0; i<fb_width; i++)
    {
      *dst++ = color;
    }
  }
}


void VGA_T4::drawPixel(int x, int y, vga_pixel color){
  framebuffer[y*fb_stride+x] = color;
}

vga_pixel VGA_T4::getPixel(int x, int y){
  return(framebuffer[y*fb_stride+x]);
}

vga_pixel * VGA_T4::getLineBuffer(int j) {
  return (&framebuffer[j*fb_stride]);
}

void VGA_T4::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, vga_pixel color) {
  int i,j,l=y;
  for (j=0; j<h; j++)
  {
    vga_pixel * dst=&framebuffer[l*fb_stride+x];
    for (i=0; i<w; i++)
    {
      *dst++ = color;
    }
    l++;
  }
}

void VGA_T4::drawText(int16_t x, int16_t y, const char * text, vga_pixel fgcolor, vga_pixel bgcolor, bool doublesize) {
  vga_pixel c;
  vga_pixel * dst;
  
  while ((c = *text++)) {
    const unsigned char * charpt=&font8x8[c][0];

    int l=y;
    for (int i=0;i<8;i++)
    {     
      unsigned char bits;
      if (doublesize) {
        dst=&framebuffer[l*fb_stride+x];
        bits = *charpt;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        l++;       
      }
      dst=&framebuffer[l*fb_stride+x]; 
      bits = *charpt++;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else *dst++=bgcolor;
      l++;
    }
    x +=8;
  } 
}

void VGA_T4::drawSprite(int16_t x, int16_t y, const vga_pixel *bitmap) {
    drawSprite(x,y,bitmap, 0,0,0,0);
}

void VGA_T4::drawSprite(int16_t x, int16_t y, const vga_pixel *bitmap, uint16_t arx, uint16_t ary, uint16_t arw, uint16_t arh)
{
  int bmp_offx = 0;
  int bmp_offy = 0;
  vga_pixel *bmp_ptr;
    
  int w =*bitmap++;
  int h = *bitmap++;


  if ( (arw == 0) || (arh == 0) ) {
    // no crop window
    arx = x;
    ary = y;
    arw = w;
    arh = h;
  }
  else {
    if ( (x>(arx+arw)) || ((x+w)<arx) || (y>(ary+arh)) || ((y+h)<ary)   ) {
      return;
    }
    
    // crop area
    if ( (x > arx) && (x<(arx+arw)) ) { 
      arw = arw - (x-arx);
      arx = arx + (x-arx);
    } else {
      bmp_offx = arx;
    }
    if ( ((x+w) > arx) && ((x+w)<(arx+arw)) ) {
      arw -= (arx+arw-x-w);
    }  
    if ( (y > ary) && (y<(ary+arh)) ) {
      arh = arh - (y-ary);
      ary = ary + (y-ary);
    } else {
      bmp_offy = ary;
    }
    if ( ((y+h) > ary) && ((y+h)<(ary+arh)) ) {
      arh -= (ary+arh-y-h);
    }     
  }

   
  int l=ary;
  bitmap = bitmap + bmp_offy*w + bmp_offx;
  for (int row=0;row<arh; row++)
  {
    vga_pixel * dst=&framebuffer[l*fb_stride+arx];  
    bmp_ptr = (vga_pixel*)bitmap;
    for (int col=0;col<arw; col++)
    {
        *dst++ = *bmp_ptr++;            
    } 
    bitmap +=  w;
    l++;
  } 
}

void VGA_T4::writeLine(int width, int height, int y, uint8_t *buf, vga_pixel *palette16) {
  vga_pixel * dst=&framebuffer[y*fb_stride];
  if (width > fb_width) {
#ifdef TFT_LINEARINT    
    int delta = (width/(width-fb_width))-1;
    int pos = delta;
    for (int i=0; i<fb_width; i++)
    {
      uint16_t val = palette16[*buf++];
      pos--;      
      if (pos == 0) {
#ifdef LINEARINT_HACK        
        val  = ((uint32_t)palette16[*buf++] + val)/2;
#else
        uint16_t val2 = *buf++;
        val = RGBVAL16((R16(val)+R16(val2))/2,(G16(val)+G16(val2))/2,(B16(val)+B16(val2))/2);
#endif        
        pos = delta;
      }
      *dst++=val;
    }
#else
    int step = ((width << 8)/fb_width);
    int pos = 0;
    for (int i=0; i<fb_width; i++)
    {
      *dst++=palette16[buf[pos >> 8]];
      pos +=step;
    }  
#endif       
  }
  else if ((width*2) == fb_width) {
    for (int i=0; i<width; i++)
    {
      *dst++=palette16[*buf];
      *dst++=palette16[*buf++];
    }       
  }
  else {
    if (width <= fb_width) {
      dst += (fb_width-width)/2;
    }
    for (int i=0; i<width; i++)
    {
      *dst++=palette16[*buf++];
    }       
  }
}

void VGA_T4::writeLine(int width, int height, int y, vga_pixel *buf) {
  uint8_t * dst=&framebuffer[y*fb_stride];    
  if (width > fb_width) {

    int step = ((width << 8)/fb_width);
    int pos = 0;
    vga_pixel plo=0;
    uint16_t pr,pg,pb;
    int ppos=0;
    for (int i=0; i<fb_width; i++)
    {
      uint16_t r,g,b;
      vga_pixel lo,hi;
      b = buf[ppos];
      r = b & 0xe0;
      g = b & 0x1c;
      b = b & 0x03;
      pos +=step;
      if ( ((pos >> 8)-ppos) == 1 ) {
        lo = (r+pr)/2 | (g+pg)/2 | (b+pb)/2;
      }
      else {
        lo = r | g | b;
      }
      hi = lo & 0xf0;
      lo &= 0xf;
      *dst++ = plo| hi;
      plo=lo;
      pr = r;
      pg = g;
      pb = b;
      ppos = pos >> 8;
      /*
      vga_pixel lo,hi;
      lo = buf[pos >> 8];
      hi = lo & 0xf0;
      lo &= 0xf;
      *dst++ = plo| hi;
      plo=lo;
      pos +=step;
       */
    }        
  }
  else if ((width*2) == fb_width) {
    for (int i=0; i<width; i++)
    {
      *dst++=*buf;
      *dst++=*buf++;
    }       
  }
  else {
    if (width <= fb_width) {
      dst += (fb_width-width)/2;
    }
    if (pix_shift&DMA_HACK) {
      for (int i=0; i<width; i++)
      {
        *dst++=*buf++;
      }      
    }
    else {
      vga_pixel plo=0,pplo=0;
      for (int i=0; i<width; i++)
      {
        //*dst++=*buf++;
        vga_pixel lo,hi;
        lo = *buf++;
        hi = lo & 0xf0;
        lo &= 0xf;
        *dst++ = plo| hi;
        //pplo=plo;
        plo=lo;
      }
    }
  }
}

void VGA_T4::writeScreen(int width, int height, int stride, uint8_t *buf, vga_pixel *palette16) {
  uint8_t *buffer=buf;
  uint8_t *src; 

  int i,j,y=0;
  if (width*2 <= fb_width) {
    for (j=0; j<height; j++)
    {
      vga_pixel * dst=&framebuffer[y*fb_stride];        
      src=buffer;
      for (i=0; i<width; i++)
      {
        vga_pixel val = palette16[*src++];
        *dst++ = val;
        *dst++ = val;
      }
      y++;
      if (height*2 <= fb_height) {
        dst=&framebuffer[y*fb_stride];           
        src=buffer;
        for (i=0; i<width; i++)
        {
          vga_pixel val = palette16[*src++];
          *dst++ = val;
          *dst++ = val;
        }
        y++;      
      } 
      buffer += stride;      
    }
  }
  else if (width <= fb_width) {
    //dst += (fb_width-width)/2;
    for (j=0; j<height; j++)
    {
      vga_pixel * dst=&framebuffer[y*fb_stride+(fb_width-width)/2];          
      src=buffer;
      for (i=0; i<width; i++)
      {
        vga_pixel val = palette16[*src++];
        *dst++ = val;
      }
      y++;
      if (height*2 <= fb_height) {
        dst=&framebuffer[y*fb_stride+(fb_width-width)/2];          
        src=buffer;
        for (i=0; i<width; i++)
        {
          vga_pixel val = palette16[*src++];
          *dst++ = val;
        }
        y++;
      }      
      buffer += stride;  
    }
  }   
}



//--------------------------------------------------------------
// Draw a line between 2 points
// x1,y1   : 1st point
// x2,y2   : 2nd point
// Color   : 16bits color
//--------------------------------------------------------------
void VGA_T4::drawline(int16_t x1, int16_t y1, int16_t x2, int16_t y2, vga_pixel color){
  uint8_t yLonger = 0;
  int incrementVal, endVal;
  int shortLen = y2-y1;
  int longLen = x2-x1;
  int decInc;
  int j = 0, i = 0;

  if(ABS(shortLen) > ABS(longLen)) {
    int swap = shortLen;
    shortLen = longLen;
    longLen = swap;
    yLonger = 1;
  }

  endVal = longLen;

  if(longLen < 0) {
    incrementVal = -1;
    longLen = -longLen;
    endVal--;
  } else {
    incrementVal = 1;
    endVal++;
  }

  if(longLen == 0)
    decInc = 0;
  else
    decInc = (shortLen << 16) / longLen;

  if(yLonger) {
    for(i = 0;i != endVal;i += incrementVal) {
      drawPixel(x1 + (j >> 16),y1 + i,color);
      j += decInc;
    }
  } else {
    for(i = 0;i != endVal;i += incrementVal) {
      drawPixel(x1 + i,y1 + (j >> 16),color);
      j += decInc;
    }
  }
}
  
//--------------------------------------------------------------
// Draw a circle.
// x, y - center of circle.
// r - radius.
// color - color of the circle.
//--------------------------------------------------------------
void VGA_T4::drawcircle(int16_t x, int16_t y, uint16_t radius, vga_pixel color){
  int16_t a, b, P;

  a = 0;
  b = radius;
  P = 1 - radius;

  do {
    if(((a+x) >= 0) && ((b+x) >= 0))
      drawPixel(a+x, b+y, color);
    if(((b+x) >= 0) && ((a+y) >= 0))
      drawPixel(b+x, a+y, color);
    if(((x-a) >= 0) && ((b+y) >= 0))
      drawPixel(x-a, b+y, color);
    if(((x-b) >= 0) && ((a+y) >= 0))
      drawPixel(x-b, a+y, color);
    if(((b+x) >= 0) && ((y-a) >= 0))
      drawPixel(b+x, y-a, color);
    if(((a+x) >= 0) && ((y-b) >= 0))
      drawPixel(a+x, y-b, color);
    if(((x-a) >= 0) && ((y-b) >= 0))
      drawPixel(x-a, y-b, color);
    if(((x-b) >= 0) && ((y-a) >= 0))
      drawPixel(x-b, y-a, color);

    if(P < 0)
      P+= 3 + 2*a++;
    else
      P+= 5 + 2*(a++ - b--);
  } while(a <= b);    
}
  
//--------------------------------------------------------------
// Displays a full circle.
// x          : specifies the X position
// y          : specifies the Y position
// radius     : specifies the Circle Radius
// fillcolor  : specifies the Circle Fill Color
// bordercolor: specifies the Circle Border Color
//--------------------------------------------------------------
void VGA_T4::drawfilledcircle(int16_t x, int16_t y, int16_t radius, vga_pixel fillcolor, vga_pixel bordercolor){
  int32_t  D;    /* Decision Variable */
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */

  D = 3 - (radius << 1);

  CurX = 0;
  CurY = radius;

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      drawline(x - CurX, y - CurY , x - CurX, 2*CurY + y - CurY,fillcolor);
      drawline(x + CurX, y - CurY , x + CurX, 2*CurY + y - CurY,fillcolor);
    }

    if(CurX > 0)
    {
      drawline(x - CurY, y - CurX , x - CurY, 2*CurX + y - CurX,fillcolor);
      drawline(x + CurY, y - CurX , x + CurY, 2*CurX + y - CurX,fillcolor);
    }
    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  drawcircle(x, y, radius,bordercolor);
}
  
//--------------------------------------------------------------
// Displays an Ellipse.
// cx: specifies the X position
// cy: specifies the Y position
// radius1: minor radius of ellipse.
// radius2: major radius of ellipse.
// color: specifies the Color to use for draw the Border from the Ellipse.
//--------------------------------------------------------------
void VGA_T4::drawellipse(int16_t cx, int16_t cy, uint16_t radius1, uint16_t radius2, vga_pixel color){
  int x = -radius1, y = 0, err = 2-2*radius1, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = radius1;
  rad2 = radius1;

  if (radius1 > radius2)
  {
    do {
      K = (float)(rad1/rad2);
      drawPixel(cx-x,cy+(uint16_t)(y/K),color);
      drawPixel(cx+x,cy+(uint16_t)(y/K),color);
      drawPixel(cx+x,cy-(uint16_t)(y/K),color);
      drawPixel(cx-x,cy-(uint16_t)(y/K),color);

      e2 = err;
      if (e2 <= y) {
        err += ++y*2+1;
        if (-x == y && e2 <= x) e2 = 0;
      }
      if (e2 > x) err += ++x*2+1;
    }
    while (x <= 0);
  }
  else
  {
    y = -radius2;
    x = 0;
    do {
      K = (float)(rad2/rad1);
      drawPixel(cx-(uint16_t)(x/K),cy+y,color);
      drawPixel(cx+(uint16_t)(x/K),cy+y,color);
      drawPixel(cx+(uint16_t)(x/K),cy-y,color);
      drawPixel(cx-(uint16_t)(x/K),cy-y,color);

      e2 = err;
      if (e2 <= x) {
        err += ++x*2+1;
        if (-y == x && e2 <= y) e2 = 0;
      }
      if (e2 > y) err += ++y*2+1;
    }
    while (y <= 0);
  }
}
  
// Draw a filled ellipse.
// cx: specifies the X position
// cy: specifies the Y position
// radius1: minor radius of ellipse.
// radius2: major radius of ellipse.
// fillcolor  : specifies the Color to use for Fill the Ellipse.
// bordercolor: specifies the Color to use for draw the Border from the Ellipse.
void VGA_T4::drawfilledellipse(int16_t cx, int16_t cy, uint16_t radius1, uint16_t radius2, vga_pixel fillcolor, vga_pixel bordercolor){
  int x = -radius1, y = 0, err = 2-2*radius1, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = radius1;
  rad2 = radius2;

  if (radius1 > radius2)
  {
    do
    {
      K = (float)(rad1/rad2);
      drawline((cx+x), (cy-(uint16_t)(y/K)), (cx+x), (cy-(uint16_t)(y/K)) + (2*(uint16_t)(y/K) + 1) , fillcolor);
      drawline((cx-x), (cy-(uint16_t)(y/K)), (cx-x), (cy-(uint16_t)(y/K)) + (2*(uint16_t)(y/K) + 1) , fillcolor);

      e2 = err;
      if (e2 <= y)
      {
        err += ++y*2+1;
        if (-x == y && e2 <= x) e2 = 0;
      }
      if (e2 > x) err += ++x*2+1;

    }
    while (x <= 0);
  }
  else
  {
    y = -radius2;
    x = 0;
    do
    {
      K = (float)(rad2/rad1);
      drawline((cx-(uint16_t)(x/K)), (cy+y), (cx-(uint16_t)(x/K)) + (2*(uint16_t)(x/K) + 1), (cy+y) , fillcolor);
      drawline((cx-(uint16_t)(x/K)), (cy-y), (cx-(uint16_t)(x/K)) + (2*(uint16_t)(x/K) + 1), (cy-y) , fillcolor);

      e2 = err;
      if (e2 <= x)
      {
        err += ++x*2+1;
        if (-y == x && e2 <= y) e2 = 0;
      }
      if (e2 > y) err += ++y*2+1;
    }
    while (y <= 0);
  }
  drawellipse(cx,cy,radius1,radius2,bordercolor);
}
  
//--------------------------------------------------------------
// Draw a Triangle.
// ax,ay, bx,by, cx,cy - the triangle points.
// color    - color of the triangle.
//--------------------------------------------------------------
void VGA_T4::drawtriangle(int16_t ax, int16_t ay, int16_t bx, int16_t by, int16_t cx, int16_t cy, vga_pixel color){
  drawline(ax , ay , bx , by , color);
  drawline(bx , by , cx , cy , color);
  drawline(cx , cy , ax , ay , color);
}
  
//--------------------------------------------------------------
// Draw a Filled Triangle.
// ax,ay, bx,by, cx,cy - the triangle points.
// fillcolor - specifies the Color to use for Fill the triangle.
// bordercolor - specifies the Color to use for draw the Border from the triangle.
//--------------------------------------------------------------
void VGA_T4::drawfilledtriangle(int16_t ax, int16_t ay, int16_t bx, int16_t by, int16_t cx, int16_t cy, vga_pixel fillcolor, vga_pixel bordercolor){
  float ma, mb, mc    ; //'gradient of the lines
  float start, finish ; //'draw a line from start to finish!
  float tempspace     ; //'temporary storage for swapping values...
  double x1,x2,x3      ;
  double y1,y2,y3      ;
  int16_t n           ;

  //' need to sort out ay, by and cy into order.. highest to lowest
  //'
  if(ay < by)
  {
    //'swap x's
    tempspace = ax;
    ax = bx;
    bx = tempspace;

    //'swap y's
    tempspace = ay;
    ay = by;
    by = tempspace;
  }

  if(ay < cy)
  {
    //'swap x's
    tempspace = ax;
    ax = cx;
    cx = tempspace;

    //'swap y's
    tempspace = ay;
    ay = cy;
    cy = tempspace;
  }

  if(by < cy)
  {
    //'swap x's
    tempspace = bx;
    bx = cx;
    cx = tempspace;

    //'swap y's
    tempspace = by;
    by = cy;
    cy = tempspace;
  }

  //' Finally - copy the values in order...

  x1 = ax; x2 = bx; x3 = cx;
  y1 = ay; y2 = by; y3 = cy;

  //'bodge if y coordinates are the same
  if(y1 == y2)  y2 = y2 + 0.01;
  if(y2 == y3)  y3 = y3 + 0.01;
  if(y1 == y3)  y3 = y3 + 0.01;

  ma = (x1 - x2) / (y1 - y2);
  mb = (x3 - x2) / (y2 - y3);
  mc = (x3 - x1) / (y1 - y3);

  //'from y1 to y2
  for(n = 0;n >= (y2 - y1);n--)
  {
    start = n * mc;
    finish = n * ma;
    drawline((int16_t)(x1 - start), (int16_t)(n + y1), (int16_t)(x1 + finish), (int16_t)(n + y1), fillcolor);
  }


  //'and from y2 to y3

  for(n = 0;n >= (y3 - y2);n--)
  {
    start = n * mc;
    finish = n * mb;
    drawline((int16_t)(x1 - start - ((y2 - y1) * mc)), (int16_t)(n + y2), (int16_t)(x2 - finish), (int16_t)(n + y2), fillcolor);
  }

  // draw the border color triangle
  drawtriangle(ax,ay,bx,by,cx,cy,bordercolor);
}


//--------------------------------------------------------------
//  Displays a Rectangle at a given Angle.
//  centerx			: specifies the center of the Rectangle.
//	centery
//  w,h 	: specifies the size of the Rectangle.
//	angle			: specifies the angle for drawing the rectangle
//  color	    	: specifies the Color to use for Fill the Rectangle.
//--------------------------------------------------------------
void VGA_T4::drawquad(int16_t centerx, int16_t centery, int16_t w, int16_t h, int16_t angle, vga_pixel color){
	int16_t	px[4],py[4];
	float	l;
	float	raddeg = 3.14159 / 180;
	float	w2 = w / 2.0;
	float	h2 = h / 2.0;
	float	vec = (w2*w2)+(h2*h2);
	float	w2l;
	float	pangle[4];

	l = sqrtf(vec);
	w2l = w2 / l;
	pangle[0] = acosf(w2l) / raddeg;
	pangle[1] = 180.0 - (acosf(w2l) / raddeg);
	pangle[2] = 180.0 + (acosf(w2l) / raddeg);
	pangle[3] = 360.0 - (acosf(w2l) / raddeg);
	px[0] = (int16_t)(calcco[((int16_t)(pangle[0]) + angle) % 360] * l + centerx);
	py[0] = (int16_t)(calcsi[((int16_t)(pangle[0]) + angle) % 360] * l + centery);
	px[1] = (int16_t)(calcco[((int16_t)(pangle[1]) + angle) % 360] * l + centerx);
	py[1] = (int16_t)(calcsi[((int16_t)(pangle[1]) + angle) % 360] * l + centery);
	px[2] = (int16_t)(calcco[((int16_t)(pangle[2]) + angle) % 360] * l + centerx);
	py[2] = (int16_t)(calcsi[((int16_t)(pangle[2]) + angle) % 360] * l + centery);
	px[3] = (int16_t)(calcco[((int16_t)(pangle[3]) + angle) % 360] * l + centerx);
	py[3] = (int16_t)(calcsi[((int16_t)(pangle[3]) + angle) % 360] * l + centery);
	// here we draw the quad
	drawline(px[0],py[0],px[1],py[1],color);
	drawline(px[1],py[1],px[2],py[2],color);
	drawline(px[2],py[2],px[3],py[3],color);
	drawline(px[3],py[3],px[0],py[0],color);
}
  
//--------------------------------------------------------------
//  Displays a filled Rectangle at a given Angle.
//  centerx			: specifies the center of the Rectangle.
//	centery
//  w,h 	: specifies the size of the Rectangle.
//	angle			: specifies the angle for drawing the rectangle
//  fillcolor    	: specifies the Color to use for Fill the Rectangle.
//  bordercolor  	: specifies the Color to use for draw the Border from the Rectangle.
//--------------------------------------------------------------
void VGA_T4::drawfilledquad(int16_t centerx, int16_t centery, int16_t w, int16_t h, int16_t angle, vga_pixel fillcolor, vga_pixel bordercolor){
	int16_t	px[4],py[4];
	float	l;
	float	raddeg = 3.14159 / 180;
	float	w2 = w / 2.0;
	float	h2 = h / 2.0;
	float	vec = (w2*w2)+(h2*h2);
	float	w2l;
	float	pangle[4];

	l = sqrtf(vec);
	w2l = w2 / l;
	pangle[0] = acosf(w2l) / raddeg;
	pangle[1] = 180.0 - (acosf(w2l) / raddeg);
	pangle[2] = 180.0 + (acosf(w2l) / raddeg);
	pangle[3] = 360.0 - (acosf(w2l) / raddeg);
	px[0] = (int16_t)(calcco[((int16_t)(pangle[0]) + angle) % 360] * l + centerx);
	py[0] = (int16_t)(calcsi[((int16_t)(pangle[0]) + angle) % 360] * l + centery);
	px[1] = (int16_t)(calcco[((int16_t)(pangle[1]) + angle) % 360] * l + centerx);
	py[1] = (int16_t)(calcsi[((int16_t)(pangle[1]) + angle) % 360] * l + centery);
	px[2] = (int16_t)(calcco[((int16_t)(pangle[2]) + angle) % 360] * l + centerx);
	py[2] = (int16_t)(calcsi[((int16_t)(pangle[2]) + angle) % 360] * l + centery);
	px[3] = (int16_t)(calcco[((int16_t)(pangle[3]) + angle) % 360] * l + centerx);
	py[3] = (int16_t)(calcsi[((int16_t)(pangle[3]) + angle) % 360] * l + centery);
	// We draw 2 filled triangle for made the quad
	// To be uniform we have to use only the Fillcolor
	drawfilledtriangle(px[0],py[0],px[1],py[1],px[2],py[2],fillcolor,fillcolor);
	drawfilledtriangle(px[2],py[2],px[3],py[3],px[0],py[0],fillcolor,fillcolor);
	// here we draw the BorderColor from the quad
	drawline(px[0],py[0],px[1],py[1],bordercolor);
	drawline(px[1],py[1],px[2],py[2],bordercolor);
	drawline(px[2],py[2],px[3],py[3],bordercolor);
	drawline(px[3],py[3],px[0],py[0],bordercolor);
}

