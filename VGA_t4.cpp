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
// - supported resolutions: 320x240,320x480,640x240 and 640x480 pixels
// - experimental resolution: 352x240,352x480
// - experimental resolution: 512x240,512x480 (not stable)
// - video memory is allocated using malloc in T4 heap
// - as the 2 DMA transfers are not started exactly at same time, there is a bit of color smearing 
//   but tried to be compensated by pixel shifting 
// - Default is 8bits RRRGGGBB (332) 
//   But 12bits GBB0RRRRGGGBB (444) feasible BUT NOT TESTED !!!!
// - Only ok at 600MHz else some disturbances visible



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

#define R16(rgb) ((rgb>>8)&0xf8) 
#define G16(rgb) ((rgb>>3)&0xfc) 
#define B16(rgb) ((rgb<<3)&0xf8) 

// Full buffer including back/front porch 
static vga_pixel *gfxbuffer;
static void *gfxbufferP;

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
static int  ref_div_select;
static int  ref_freq_num;
static int  ref_freq_denom;
static int  ref_pix_shift;
static int  combine_shiftreg;

#ifdef DEBUG
static uint32_t   ISRTicks_prev = 0;
volatile uint32_t ISRTicks = 0;
#endif 

uint8_t    VGA_T4::_vsync_pin = -1;
DMAChannel VGA_T4::flexio1DMA;
DMAChannel VGA_T4::flexio2DMA; 
static volatile uint32_t VSYNC = 0;
static volatile uint32_t currentLine=0;
//#define NOP asm volatile("nop\n\t");



PolyDef	PolySet;  // will contain a polygon data


FASTRUN void VGA_T4::QT3_isr(void) {
  TMR3_SCTRL3 &= ~(TMR_SCTRL_TCF);
  TMR3_CSCTRL3 &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);
  
  // V-PULSE
  if (currentLine > 0) {
    digitalWrite(_vsync_pin, 1);
    VSYNC = 0;
  } else {
    digitalWrite(_vsync_pin, 0);
    VSYNC = 1;
  }
  
  currentLine++;
  currentLine = currentLine % 525;


  int y = (currentLine - TOP_BORDER) >> line_double;
  // Visible area  
  if (y >= 0 && y < fb_height) {  
    // Disable DMAs
    //DMA_CERQ = flexio2DMA.channel;
    //DMA_CERQ = flexio1DMA.channel; 

    // Setup source adress
    // Aligned 32 bits copy
    unsigned long * p=(uint32_t *)&gfxbuffer[fb_stride*y];  
    flexio2DMA.TCD->SADDR = p;
    if (pix_shift & DMA_HACK) 
    {
      // Unaligned copy
      uint8_t * p2=(uint8_t *)&gfxbuffer[fb_stride*y+(pix_shift&0xf)];
      flexio1DMA.TCD->SADDR = p2;
    }
    else  {
      p=(uint32_t *)&gfxbuffer[fb_stride*y+(pix_shift&0xc)]; // multiple of 4
      flexio1DMA.TCD->SADDR = p;
    }

    // Enable DMAs
    DMA_SERQ = flexio2DMA.channel; 
    DMA_SERQ = flexio1DMA.channel; 
    //arm_dcache_flush_delete((void*)((uint32_t *)&gfxbuffer[fb_stride*y]), fb_stride);
    arm_dcache_flush((void*)((uint32_t *)&gfxbuffer[fb_stride*y]), fb_stride);
  }  else {
    asm volatile("dsb");
  }

#ifdef DEBUG
  ISRTicks++; 
#endif  

}


VGA_T4::VGA_T4(int vsync_pin)
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
#define frontporch_pix  20 //16
#define backporch_pix   44 //48

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
FLASHMEM
static void set_videoClock(int nfact, int32_t nmult, uint32_t ndiv, bool force) // sets PLL5
{
//if (!force && (CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_ENABLE)) return;

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

void VGA_T4::tweak_video(int shiftdelta, int numdelta, int denomdelta)
{
  if ( (numdelta != 0) || (denomdelta != 0) )   {
    set_videoClock(ref_div_select,ref_freq_num+numdelta,ref_freq_denom+denomdelta,true);  
  }  
  if (shiftdelta != 0) {
    pix_shift = ref_pix_shift + shiftdelta;
  }
}

// display VGA image
FLASHMEM
vga_error_t VGA_T4::begin(vga_mode_t mode)
{
  uint32_t flexio_clock_div;
  combine_shiftreg = 0;
//  int div_select = 49;
//  int num = 135;  
//  int denom = 100;  
  int div_select = 20;
  int num = 9800;
  int denom = 10000;  
  int flexio_clk_sel = FLEXIO_CLK_SEL_PLL5;   
  int flexio_freq = ( 24000*div_select + (num*24000)/denom )/POST_DIV_SELECT;
  set_videoClock(div_select,num,denom,true);
  switch(mode) {
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
      pix_shift = 4;
      combine_shiftreg = 1;
      break;
    case VGA_MODE_640x480:
      left_border = backporch_pix;
      right_border = frontporch_pix;
      fb_width = 640;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = (flexio_freq/pix_freq); 
      line_double = 0;
      pix_shift = 4;
      combine_shiftreg = 1;
      break;   
    case VGA_MODE_512x240:
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3)+2; 
      line_double = 1;
      pix_shift = 0;
      break;
    case VGA_MODE_512x480:
      left_border = backporch_pix/1.3;
      right_border = frontporch_pix/1.3;
      fb_width = 512;
      fb_height = 480 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.3)+2; 
      line_double = 0;
      pix_shift = 0;
      break; 
    case VGA_MODE_352x240:
      left_border = backporch_pix/1.75;
      right_border = frontporch_pix/1.75;
      fb_width = 352;
      fb_height = 240 ;
      fb_stride = left_border+fb_width+right_border;
      maxpixperline = fb_stride;
      flexio_clock_div = flexio_freq/(pix_freq/1.75)+2; 
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
      flexio_clock_div = flexio_freq/(pix_freq/1.75)+2; 
      line_double = 0;
      pix_shift = 2+DMA_HACK;
      break;         
  }	

  // Save param for tweek adjustment
  ref_div_select = div_select;
  ref_freq_num = num;
  ref_freq_denom = denom;
  ref_pix_shift = pix_shift;

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
  inputSource = FLEXIO_SHIFTCFG_INSRC*(1);    // Input source from next shifter
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

  /* Shifter 0 registers for FlexIO1 */ 
#ifdef BITS12
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(5);  // 5-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D8
#else
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(4);  // 8-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D7
#endif  
  FLEXIO1_SHIFTCFG0 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL0 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  
  if (combine_shiftreg) {
    pinConfig = FLEXIO_SHIFTCTL_PINCFG(0);    // Shifter pin output disabled
    FLEXIO2_SHIFTCFG1 = parallelWidth | inputSource | stopBit | startBit;
    FLEXIO2_SHIFTCTL1 = timerSelect | timerPolarity | pinConfig | shifterMode;
    FLEXIO1_SHIFTCFG1 = parallelWidth | inputSource | stopBit | startBit;
    FLEXIO1_SHIFTCTL1 = timerSelect | timerPolarity | pinConfig | shifterMode;
  }
  /* Timer 0 registers for FlexIO2 */ 
  timerOutput = FLEXIO_TIMCFG_TIMOUT(1);      // Timer output is logic zero when enabled and is not affected by the Timer reset
  timerDecrement = FLEXIO_TIMCFG_TIMDEC(0);   // Timer decrements on FlexIO clock, shift clock equals timer output
  timerReset = FLEXIO_TIMCFG_TIMRST(0);       // Timer never reset
  timerDisable = FLEXIO_TIMCFG_TIMDIS(2);     // Timer disabled on Timer compare
  timerEnable = FLEXIO_TIMCFG_TIMENA(2);      // Timer enabled on Trigger assert
  stopBit = FLEXIO_TIMCFG_TSTOP(0);           // Stop bit disabled
  startBit = FLEXIO_TIMCFG_TSTART*(0);        // Start bit disabled
  if (combine_shiftreg) {
    triggerSelect = FLEXIO_TIMCTL_TRGSEL(1+4*(1)); // Trigger select Shifter 1 status flag
  }
  else {
    triggerSelect = FLEXIO_TIMCTL_TRGSEL(1+4*(0)); // Trigger select Shifter 0 status flag
  }    
  triggerPolarity = FLEXIO_TIMCTL_TRGPOL*(1); // Trigger active low
  triggerSource = FLEXIO_TIMCTL_TRGSRC*(1);   // Internal trigger selected
  pinConfig = FLEXIO_TIMCTL_PINCFG(0);        // Timer pin output disabled
  //pinSelect = FLEXIO_TIMCTL_PINSEL(0);        // Select pin FXIO_D0
  //pinPolarity = FLEXIO_TIMCTL_PINPOL*(0);     // Timer pin polarity active high
  timerMode = FLEXIO_TIMCTL_TIMOD(1);         // Dual 8-bit counters baud mode
  // flexio_clock_div : Output clock frequency is N times slower than FlexIO clock (41.7 ns period) (23.980MHz?)

  int shifts_per_transfer;
#ifdef BITS12
  shifts_per_transfer = 8;
#else
  if (combine_shiftreg) {
    shifts_per_transfer = 8; // Shift out 8 times with every transfer = 64-bit word = contents of Shifter 0+1
  }
  else {
    shifts_per_transfer = 4; // Shift out 4 times with every transfer = 32-bit word = contents of Shifter 0 
  }
#endif
  FLEXIO2_TIMCFG0 = timerOutput | timerDecrement | timerReset | timerDisable | timerEnable | stopBit | startBit;
  FLEXIO2_TIMCTL0 = triggerSelect | triggerPolarity | triggerSource | pinConfig /*| pinSelect | pinPolarity*/ | timerMode;
  FLEXIO2_TIMCMP0 = ((shifts_per_transfer*2-1)<<8) | ((flexio_clock_div/2-1)<<0);
  /* Timer 0 registers for FlexIO1 */
  FLEXIO1_TIMCFG0 = timerOutput | timerDecrement | timerReset | timerDisable | timerEnable | stopBit | startBit;
  FLEXIO1_TIMCTL0 = triggerSelect | triggerPolarity | triggerSource | pinConfig /*| pinSelect | pinPolarity*/ | timerMode;
  FLEXIO1_TIMCMP0 = ((shifts_per_transfer*2-1)<<8) | ((flexio_clock_div/2-1)<<0);
#ifdef DEBUG
  Serial.println("FlexIO setup complete");
#endif

  /* Enable DMA trigger on Shifter0, DMA request is generated when data is transferred from buffer0 to shifter0 */ 
  if (combine_shiftreg) {
    FLEXIO2_SHIFTSDEN |= (1<<1); 
    FLEXIO1_SHIFTSDEN |= (1<<1);
  }
  else {
    FLEXIO2_SHIFTSDEN |= (1<<0); 
    FLEXIO1_SHIFTSDEN |= (1<<0);
  }
  /* Disable DMA channel so it doesn't start transferring yet */
  flexio1DMA.disable();
  flexio2DMA.disable();
  /* Set up DMA channel to use Shifter 0 trigger */
  flexio1DMA.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO1_REQUEST0);
  flexio2DMA.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);


  if (combine_shiftreg) {
    flexio2DMA.TCD->NBYTES = 8;
    flexio2DMA.TCD->SOFF = 4;
    flexio2DMA.TCD->SLAST = -maxpixperline;
    flexio2DMA.TCD->BITER = maxpixperline / 8;
    flexio2DMA.TCD->CITER = maxpixperline / 8;
    flexio2DMA.TCD->DADDR = &FLEXIO2_SHIFTBUF0;
    flexio2DMA.TCD->DOFF = 0;
    flexio2DMA.TCD->DLASTSGA = 0;
    flexio2DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(3); // 32bits => 64bits
    flexio2DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
  
    flexio1DMA.TCD->NBYTES = 8;
    flexio1DMA.TCD->SOFF = 4;
    flexio1DMA.TCD->SLAST = -maxpixperline;
    flexio1DMA.TCD->BITER = maxpixperline / 8;
    flexio1DMA.TCD->CITER = maxpixperline / 8;
    flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
    flexio1DMA.TCD->DOFF = 0;
    flexio1DMA.TCD->DLASTSGA = 0;
    flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(3); // 32bits => 64bits
    flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;     
  }
  else {
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
      if ( (pix_shift & 0x3) == 0) {
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
  }

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
  
  #define MARGIN_N 1005 // 1206 at 720MHz //1005 at 600MHz
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
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0); 
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);  
#ifdef DEBUG
  Serial.println("QTIMER3 setup complete");
  Serial.print("V-PIN is ");
  Serial.println(_vsync_pin);
#endif

  /* initialize gfx buffer */
  #define ALIGNDMA 32
  if (gfxbufferP == NULL) {
	  gfxbufferP = malloc(fb_stride*fb_height*sizeof(vga_pixel)+4+(ALIGNDMA-1) ); // 4bytes for pixel shift 
	  gfxbuffer = (vga_pixel*) ((void*)((intptr_t)gfxbufferP & ~ALIGNDMA)); //Align buffer;
  }	  
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
  if (gfxbufferP != NULL) free(gfxbufferP); 
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

void VGA_T4::waitLine(int line)
{
  while (currentLine != (unsigned)line) {};
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
	if((x>=0) && (x<=fb_width) && (y>=0) && (y<=fb_height))
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

void VGA_T4::drawSprite(int16_t x, int16_t y, const int16_t *bitmap) {
    drawSprite(x,y,bitmap, 0,0,0,0);
}

void VGA_T4::drawSprite(int16_t x, int16_t y, const int16_t *bitmap, uint16_t arx, uint16_t ary, uint16_t arw, uint16_t arh)
{
  int bmp_offx = 0;
  int bmp_offy = 0;
  int16_t *bmp_ptr;
    
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
    bmp_ptr = bitmap;
    for (int col=0;col<arw; col++)
    {
        uint16_t pix= *bmp_ptr++;
        *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix));
    } 
    bitmap +=  w;
    l++;
  } 
}

void VGA_T4::writeLine(int width, int height, int y, uint8_t *buf, vga_pixel *palette) {
  if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
  vga_pixel * dst=&framebuffer[y*fb_stride];
  if (width > fb_width) {
#ifdef TFT_LINEARINT    
    int delta = (width/(width-fb_width))-1;
    int pos = delta;
    for (int i=0; i<fb_width; i++)
    {
      uint16_t val = palette[*buf++];
      pos--;
      if (pos == 0) {
#ifdef LINEARINT_HACK
        val  = ((uint32_t)palette[*buf++] + val)/2;
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
      *dst++=palette[buf[pos >> 8]];
      pos +=step;
    }  
#endif
  }
  else if ((width*2) == fb_width) {
    for (int i=0; i<width; i++)
    {
      *dst++=palette[*buf];
      *dst++=palette[*buf++];
    } 
  }
  else {
    if (width <= fb_width) {
      dst += (fb_width-width)/2;
    }
    for (int i=0; i<width; i++)
    {
      *dst++=palette[*buf++];
    } 
  }
}

void VGA_T4::writeLine(int width, int height, int y, vga_pixel *buf) {
  if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
  uint8_t * dst=&framebuffer[y*fb_stride];    
  if (width > fb_width) {
    int step = ((width << 8)/fb_width);
    int pos = 0;
    for (int i=0; i<fb_width; i++)
    {
      *dst++ = buf[pos >> 8];
      pos +=step;
    }        
  }
  else if ((width*2) == fb_width) {
    if ( ( !(pix_shift & DMA_HACK) ) && (pix_shift & 0x3) ) {
      vga_pixel *buf2 = buf + (pix_shift & 0x3);
      for (int i=0; i<width; i++)
      {
        *dst++ = (*buf & 0xf0) | (*buf2 & 0x0f);
        *dst++ = (*buf++ & 0xf0) | (*buf2++ & 0x0f);
      }
    }  
    else {
      for (int i=0; i<width; i++)
      {
        *dst++=*buf;
        *dst++=*buf++;
      }
    }      
  }
  else {
    if (width <= fb_width) {
      dst += (fb_width-width)/2;
    }
    if ( ( !(pix_shift & DMA_HACK) ) && (pix_shift & 0x3) ) {
      vga_pixel *buf2 = buf + (pix_shift & 0x3);
      for (int i=0; i<width; i++)
      {
        *dst++ = (*buf++ & 0xf0) | (*buf2++ & 0x0f);
      }
    }  
    else {
      for (int i=0; i<width; i++)
      {
        *dst++=*buf++;
      }
    }
  }
}

void VGA_T4::writeLine16(int width, int height, int y, uint16_t *buf) {
  if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
  uint8_t * dst=&framebuffer[y*fb_stride];    
  if (width > fb_width) {
    int step = ((width << 8)/fb_width);
    int pos = 0;
    for (int i=0; i<fb_width; i++)
    {
      uint16_t pix = buf[pos >> 8];
      *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix)); 
      pos +=step;
    }        
  }
  else if ((width*2) == fb_width) {
    for (int i=0; i<width; i++)
    {
      uint16_t pix = *buf++;
      uint8_t col = VGA_RGB(R16(pix),G16(pix),B16(pix));
      *dst++= col;
      *dst++= col;
    }       
  }
  else {
    if (width <= fb_width) {
      dst += (fb_width-width)/2;
    }
    for (int i=0; i<width; i++)
    {
      uint16_t pix = *buf++;
      *dst++=VGA_RGB(R16(pix),G16(pix),B16(pix));
    }      
  }
}

void VGA_T4::writeScreen(int width, int height, int stride, uint8_t *buf, vga_pixel *palette) {
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
        vga_pixel val = palette[*src++];
        *dst++ = val;
        *dst++ = val;
      }
      y++;
      if (height*2 <= fb_height) {
        dst=&framebuffer[y*fb_stride];    
        src=buffer;
        for (i=0; i<width; i++)
        {
          vga_pixel val = palette[*src++];
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
        vga_pixel val = palette[*src++];
        *dst++ = val;
      }
      y++;
      if (height*2 <= fb_height) {
        dst=&framebuffer[y*fb_stride+(fb_width-width)/2];   
        src=buffer;
        for (i=0; i<width; i++)
        {
          vga_pixel val = palette[*src++];
          *dst++ = val;
        }
        y++;
      }
      buffer += stride;  
    }
  }   
}

void VGA_T4::copyLine(int width, int height, int ysrc, int ydst) {
  if ( (height<fb_height) && (height > 2) ) {
    ysrc += (fb_height-height)/2;
    ydst += (fb_height-height)/2;
  }    
  uint8_t * src=&framebuffer[ysrc*fb_stride];    
  uint8_t * dst=&framebuffer[ydst*fb_stride]; 
  memcpy(dst,src,width);   
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
// Draw a horizontal line
// x1,y1   : starting point
// lenght  : lenght in pixels
// color   : 16bits color
//--------------------------------------------------------------
void VGA_T4::draw_h_line(int16_t x, int16_t y, int16_t lenght, vga_pixel color){
	drawline(x , y , x + lenght , y , color);
}

//--------------------------------------------------------------
// Draw a vertical line
// x1,y1   : starting point
// lenght  : lenght in pixels
// color   : 16bits color
//--------------------------------------------------------------
void VGA_T4::draw_v_line(int16_t x, int16_t y, int16_t lenght, vga_pixel color){
	drawline(x , y , x , y + lenght , color);
}

//--------------------------------------------------------------
// Draw a circle.
// x, y - center of circle.
// r - radius.
// color - color of the circle.
//--------------------------------------------------------------
void VGA_T4::drawcircle(int16_t x, int16_t y, int16_t radius, vga_pixel color){
  int16_t a, b, P;

  a = 0;
  b = radius;
  P = 1 - radius;

  do {
      drawPixel(a+x, b+y, color);
      drawPixel(b+x, a+y, color);
      drawPixel(x-a, b+y, color);
      drawPixel(x-b, a+y, color);
      drawPixel(b+x, y-a, color);
      drawPixel(a+x, y-b, color);
      drawPixel(x-a, y-b, color);
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
      draw_v_line(x - CurX, y - CurY, 2*CurY, fillcolor);
      draw_v_line(x + CurX, y - CurY, 2*CurY, fillcolor);
    }

    if(CurX > 0)
    {
      draw_v_line(x - CurY, y - CurX, 2*CurX, fillcolor);
      draw_v_line(x + CurY, y - CurX, 2*CurX, fillcolor);
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
void VGA_T4::drawellipse(int16_t cx, int16_t cy, int16_t radius1, int16_t radius2, vga_pixel color){
  int x = -radius1, y = 0, err = 2-2*radius1, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = radius1;
  rad2 = radius2;

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
void VGA_T4::drawfilledellipse(int16_t cx, int16_t cy, int16_t radius1, int16_t radius2, vga_pixel fillcolor, vga_pixel bordercolor){
  int x = -radius1, y = 0, err = 2-2*radius1, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = radius1;
  rad2 = radius2;

  if (radius1 > radius2)
  {
    do
    {
      K = (float)(rad1/rad2);
      draw_v_line((cx+x), (cy-(uint16_t)(y/K)), (2*(uint16_t)(y/K) + 1) , fillcolor);
      draw_v_line((cx-x), (cy-(uint16_t)(y/K)), (2*(uint16_t)(y/K) + 1) , fillcolor);

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
      draw_h_line((cx-(uint16_t)(x/K)), (cy+y), (2*(uint16_t)(x/K) + 1) , fillcolor);
      draw_h_line((cx-(uint16_t)(x/K)), (cy-y), (2*(uint16_t)(x/K) + 1) , fillcolor);

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

//--------------------------------------------------------------
//  Displays a Polygon.
//  centerx			: are specified with PolySet.Center.x and y.
//	centery
//  cx              : Translate the polygon in x direction
//  cy              : Translate the polygon in y direction
//  bordercolor  	: specifies the Color to use for draw the Border from the polygon.
//  polygon points  : are specified with PolySet.Pts[n].x and y 
//  After the last polygon point , set PolySet.Pts[n + 1].x to 10000
//  Max number of point for the polygon is set by MaxPolyPoint previously defined.
//--------------------------------------------------------------
void VGA_T4::drawpolygon(int16_t cx, int16_t cy, vga_pixel bordercolor){
	uint8_t n = 1;
	while((PolySet.Pts[n].x < 10000) && (n < MaxPolyPoint)){
		drawline(PolySet.Pts[n].x + cx, 
	             PolySet.Pts[n].y + cy, 
				 PolySet.Pts[n - 1].x + cx , 
				 PolySet.Pts[n - 1].y + cy, 
				 bordercolor);
		n++;		
	}
	// close the polygon
	drawline(PolySet.Pts[0].x + cx, 
	         PolySet.Pts[0].y + cy, 
			 PolySet.Pts[n - 1].x + cx, 
			 PolySet.Pts[n - 1].y + cy, 
			 bordercolor);
}

//--------------------------------------------------------------
//  Displays a filled Polygon.
//  centerx			: are specified with PolySet.Center.x and y.
//	centery
//  cx              : Translate the polygon in x direction
//  cy              : Translate the polygon in y direction
//  fillcolor  	    : specifies the Color to use for filling the polygon.
//  bordercolor  	: specifies the Color to use for draw the Border from the polygon.
//  polygon points  : are specified with PolySet.Pts[n].x and y 
//  After the last polygon point , set PolySet.Pts[n + 1].x to 10000
//  Max number of point for the polygon is set by MaxPolyPoint previously defined.
//--------------------------------------------------------------
void VGA_T4::drawfullpolygon(int16_t cx, int16_t cy, vga_pixel fillcolor, vga_pixel bordercolor){
	int n,i,j,k,dy,dx;
	int y,temp;
	int a[MaxPolyPoint][2],xi[MaxPolyPoint];
	float slope[MaxPolyPoint];

    n = 0;

	while((PolySet.Pts[n].x < 10000) && (n < MaxPolyPoint)){
		a[n][0] = PolySet.Pts[n].x;
		a[n][1] = PolySet.Pts[n].y;
		n++;
	}

	a[n][0]=PolySet.Pts[0].x;
	a[n][1]=PolySet.Pts[0].y;

	for(i=0;i<n;i++)
	{
		dy=a[i+1][1]-a[i][1];
		dx=a[i+1][0]-a[i][0];

		if(dy==0) slope[i]=1.0;
		if(dx==0) slope[i]=0.0;

		if((dy!=0)&&(dx!=0)) /*- calculate inverse slope -*/
		{
			slope[i]=(float) dx/dy;
		}
	}

	for(y=0;y< 480;y++)
	{
		k=0;
		for(i=0;i<n;i++)
		{

			if( ((a[i][1]<=y)&&(a[i+1][1]>y))||
					((a[i][1]>y)&&(a[i+1][1]<=y)))
			{
				xi[k]=(int)(a[i][0]+slope[i]*(y-a[i][1]));
				k++;
			}
		}

		for(j=0;j<k-1;j++) /*- Arrange x-intersections in order -*/
			for(i=0;i<k-1;i++)
			{
				if(xi[i]>xi[i+1])
				{
					temp=xi[i];
					xi[i]=xi[i+1];
					xi[i+1]=temp;
				}
			}

		for(i=0;i<k;i+=2)
		{
			drawline(xi[i] + cx,y + cy,xi[i+1]+1 + cx,y + cy, fillcolor);
		}

	}

	// Draw the polygon outline
	drawpolygon(cx , cy , bordercolor);
}

//--------------------------------------------------------------
//  Displays a rotated Polygon.
//  centerx			: are specified with PolySet.Center.x and y.
//	centery
//  cx              : Translate the polygon in x direction
//  ct              : Translate the polygon in y direction
//  bordercolor  	: specifies the Color to use for draw the Border from the polygon.
//  polygon points  : are specified with PolySet.Pts[n].x and y 
//  After the last polygon point , set PolySet.Pts[n + 1].x to 10000
//  Max number of point for the polygon is set by MaxPolyPoint previously defined.
//--------------------------------------------------------------
void VGA_T4::drawrotatepolygon(int16_t cx, int16_t cy, int16_t Angle, vga_pixel fillcolor, vga_pixel bordercolor, uint8_t filled)
{
	Point2D 	SavePts[MaxPolyPoint];
	uint16_t	n = 0;
	int16_t		ctx,cty;
	float		raddeg = 3.14159 / 180;
	float		angletmp;
	float		tosquare;
	float		ptsdist;

	ctx = PolySet.Center.x;
	cty = PolySet.Center.y;
	
	while((PolySet.Pts[n].x < 10000) && (n < MaxPolyPoint)){
		// Save Original points coordinates
		SavePts[n] = PolySet.Pts[n];
		// Rotate and save all points
		tosquare = ((PolySet.Pts[n].x - ctx) * (PolySet.Pts[n].x - ctx)) + ((PolySet.Pts[n].y - cty) * (PolySet.Pts[n].y - cty));
		ptsdist  = sqrtf(tosquare);
		angletmp = atan2f(PolySet.Pts[n].y - cty,PolySet.Pts[n].x - ctx) / raddeg;
		PolySet.Pts[n].x = (int16_t)((cosf((angletmp + Angle) * raddeg) * ptsdist) + ctx);
		PolySet.Pts[n].y = (int16_t)((sinf((angletmp + Angle) * raddeg) * ptsdist) + cty);
		n++;
	}	
	
	if(filled != 0)
	  drawfullpolygon(cx , cy , fillcolor , bordercolor);
    else
	  drawpolygon(cx , cy , bordercolor);

	// Get the original points back;
	n=0;
	while((PolySet.Pts[n].x < 10000) && (n < MaxPolyPoint)){
		PolySet.Pts[n] = SavePts[n];
		n++;
	}	
}





/*******************************************************************
 Experimental GAME engine supporting:
 - Multiple tiles layers with independent scrolling
 - Sprites (MAX_SPRITES)
 - up to 256 redefinable tiles
 - up to 256 redefinable sprites
*******************************************************************/
struct Sprite_t {
  int x;
  int y;
  unsigned char index;
};

static vga_pixel * tilesbuffer __attribute__((aligned(32))) = NULL;
static vga_pixel * spritesbuffer __attribute__((aligned(32))) = NULL;
static unsigned char * tilesram __attribute__((aligned(32))) = NULL;
static Sprite_t * spritesdata __attribute__((aligned(32))) = NULL;
static int nb_layers = 0;
static int nb_tiles = 0;
static int nb_sprites = 0;
static int hscr[TILES_MAX_LAYERS];
static int vscr[TILES_MAX_LAYERS];
static int hscr_beg[TILES_MAX_LAYERS]={0,0};
static int hscr_end[TILES_MAX_LAYERS]={TILES_ROWS-1, TILES_ROWS-1};
static int hscr_mask=0;


static void drawSpr(unsigned char index, int x, int y) {
  if ((x + SPRITES_W) <= 0) return;
  if (x >= (fb_width-hscr_mask)) return;
  if ((y + SPRITES_H) <= 0) return;
  if (y >= fb_height) return;
  
  vga_pixel * src=&spritesbuffer[index*SPRITES_W*SPRITES_H];
  int i,j;
  vga_pixel pix;  
  for (j=0; j<SPRITES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<SPRITES_W; i++)
    { 
      pix=*src++;
      if ( (!pix) || ((x+i) < 0) || ((x+i) > (fb_width-hscr_mask)) || ((y+j) < 0) || ((y+j) >= fb_height) ) dst++;
      else *dst++ = pix;
    }
  }
}


static void drawTile(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  int i,j;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      *dst++ = *src++;
    }
  }
}

static void drawTileCropL(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  int i,j;
  vga_pixel pix;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      pix=*src++;
      if ((x+i) < 0) dst++;
      else
        *dst++ = pix;
    }
  }
}

static void drawTileCropR(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  int i,j;
  vga_pixel pix;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      pix = *src++;
      if ((x+i) > (fb_width-hscr_mask)) *dst++=0;
      else
        *dst++ = pix;
    }
  }
}

static void drawTransTile(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  vga_pixel pix;
  int i,j;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      if ((pix=*src++)) *dst++ = pix;
      else dst++;
    }
  }
}

static void drawTransTileCropL(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  vga_pixel pix;
  int i,j;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      pix=*src++;
      if ((x+i) < 0) dst++;
      else
        if (pix) *dst++ = pix;
        else dst++;
    }
  }
}

static void drawTransTileCropR(unsigned char tile, int x, int y) {
  vga_pixel * src=&tilesbuffer[tile*TILES_W*TILES_H];
  vga_pixel pix;
  int i,j;
  for (j=0; j<TILES_H; j++)
  {
    vga_pixel * dst=&framebuffer[((j+y)*fb_stride)+x];
    for (i=0; i<TILES_W; i++)
    { 
      if ((x+i) > (fb_width-hscr_mask)) src++;
      else
        if ((pix=*src++)) *dst++ = pix;
        else *dst++;
    }
  }
}



static void tileText(unsigned char index, int16_t x, int16_t y, const char * text, vga_pixel fgcolor, vga_pixel bgcolor, vga_pixel *dstbuffer, int dstwidth, int dstheight) {
  vga_pixel c;
  vga_pixel * dst;
  
  while ((c = *text++)) {
    const unsigned char * charpt=&font8x8[c][0];
    int l=y;
    for (int i=0;i<8;i++)
    {     
      unsigned char bits;
      dst=&dstbuffer[(index*dstheight+l)*dstwidth+x]; 
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

static void tileTextOverlay(int16_t x, int16_t y, const char * text, vga_pixel fgcolor) {
  vga_pixel c;
  vga_pixel * dst;
  
  while ((c = *text++)) {
    const unsigned char * charpt=&font8x8[c][0];
    int l=y;
    for (int i=0;i<8;i++)
    {     
      unsigned char bits;
      dst=&framebuffer[+l*fb_stride+x]; 
      bits = *charpt++;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      bits = bits >> 1;     
      if (bits&0x01) *dst++=fgcolor;
      else dst++;
      l++;
    }
    x +=8;
  } 
}
static const char * hex = "0123456789ABCDEF";

void VGA_T4::begin_gfxengine(int nblayers, int nbtiles, int nbsprites)
{
  nb_layers = nblayers;
  nb_tiles = nbtiles;
  nb_sprites = nbsprites;

  if (spritesbuffer == NULL) spritesbuffer = (vga_pixel*)malloc(SPRITES_W*SPRITES_H*sizeof(vga_pixel)*nb_sprites);
  if (tilesbuffer == NULL) tilesbuffer = (vga_pixel*)malloc(TILES_W*TILES_H*sizeof(vga_pixel)*nb_tiles); 
  if (tilesram == NULL) tilesram = malloc(TILES_COLS*TILES_ROWS*nb_layers); 
  if (spritesdata == NULL) spritesdata = malloc(SPRITES_MAX*sizeof(Sprite_t)); 

  memset((void*)spritesbuffer,0, SPRITES_W*SPRITES_H*sizeof(vga_pixel)*nb_sprites); 
  memset((void*)tilesbuffer,0, TILES_W*TILES_H*sizeof(vga_pixel)*nb_tiles); 
  memset((void*)tilesram,0,TILES_COLS*TILES_ROWS*nb_layers); 

  /* Random test tiles */
  char numhex[3];
  for (int i=0; i<nb_tiles; i++)
  {
    int r = random(0x40,0xff);
    int g = random(0x40,0xff);
    int b = random(0x40,0xff);
    if (i==0) {
      memset((void*)&tilesbuffer[TILES_W*TILES_H*sizeof(vga_pixel)*i],0, TILES_W*TILES_H*sizeof(vga_pixel));     
    }
    else {
      memset((void*)&tilesbuffer[TILES_W*TILES_H*sizeof(vga_pixel)*i],VGA_RGB(r,g,b), TILES_W*TILES_H*sizeof(vga_pixel)); 
      numhex[0] = hex[(i>>4) & 0xf];
      numhex[1] = hex[i & 0xf];
      numhex[2] = 0;
      if (TILES_W == 16 )tileText(i, 0, 0, numhex, VGA_RGB(0xff,0xff,0xff), VGA_RGB(0x40,0x40,0x40), tilesbuffer,TILES_W,TILES_H); 
    }
  }
  /* Random test sprites */
  for (int i=0; i<nb_sprites; i++)
  {
    int r = random(0x40,0xff);
    int g = random(0x40,0xff);
    int b = random(0x40,0xff);
    if (i==0) {
      memset((void*)&spritesbuffer[SPRITES_W*SPRITES_H*sizeof(vga_pixel)*i],0, SPRITES_W*SPRITES_H*sizeof(vga_pixel));     
    }
    else {
      memset((void*)&spritesbuffer[SPRITES_W*SPRITES_H*sizeof(vga_pixel)*i],VGA_RGB(r,g,b), SPRITES_W*SPRITES_H*sizeof(vga_pixel)); 
      numhex[0] = hex[(i>>4) & 0xf];
      numhex[1] = hex[i & 0xf];
      numhex[2] = 0;
      tileText(i, 0, 0, numhex, VGA_RGB(0xff,0xff,0x00), VGA_RGB(0x00,0x00,0x00),spritesbuffer,SPRITES_W,SPRITES_H); 
    }
  }
}


void VGA_T4::run_gfxengine()
{
  waitLine(480+40);

  unsigned char * tilept;

  // Layer 0
  for (int j=0; j<TILES_ROWS; j++)
  {
    tilept = &tilesram[j*TILES_COLS];
    if ( (j>=hscr_beg[0]) && (j<=hscr_end[0]) ) {     
      int modcol = (hscr[0] >> TILES_HBITS) % TILES_COLS;
      for (int i=0; i<TILES_COLS; i++)
      {
        (i == 0) ? drawTileCropL(tilept[modcol], (i<<TILES_HBITS) - (hscr[0] & TILES_HMASK), j*TILES_H) : 
          (i == (TILES_COLS-1))?drawTileCropR(tilept[modcol], (i<<TILES_HBITS) - (hscr[0] & TILES_HMASK), j*TILES_H) : 
            drawTile(tilept[modcol], (i<<TILES_HBITS) - (hscr[0] & TILES_HMASK), j*TILES_H);
        modcol++;
        modcol = modcol % TILES_COLS; 
      }
    }  
    else {
      for (int i=0; i<TILES_COLS; i++)
      {
        (i == (TILES_COLS-1)) ? drawTileCropR(tilept[i], (i<<TILES_HBITS), j*TILES_H) :
          drawTile(tilept[i], (i<<TILES_HBITS), j*TILES_H);
      }      
    }
  }

  // Other layers
  if (nb_layers > 1) {
    int lcount = 1;
    while (lcount < nb_layers) {
      for (int j=0; j<TILES_ROWS; j++)
      {
        tilept = &tilesram[(j+lcount*TILES_ROWS)*TILES_COLS];
        if ( (j>=hscr_beg[lcount]) && (j<=hscr_end[lcount]) ) {     
          int modcol = (hscr[lcount] >> TILES_HBITS) % TILES_COLS;
          for (int i=0; i<TILES_COLS; i++)
          {
            (i == 0) ? drawTransTileCropL(tilept[modcol], (i<<TILES_HBITS) - (hscr[lcount] & TILES_HMASK), j*TILES_H) : 
              (i == (TILES_COLS-1))?drawTransTileCropR(tilept[modcol], (i<<TILES_HBITS) - (hscr[lcount] & TILES_HMASK), j*TILES_H) : 
                drawTransTile(tilept[modcol], (i<<TILES_HBITS) - (hscr[lcount] & TILES_HMASK), j*TILES_H);
            modcol++;
            modcol = modcol % TILES_COLS; 
          }
        }          
        else {
          for (int i=0; i<TILES_COLS; i++)
          {
            drawTransTile(tilept[i], (i<<TILES_HBITS), j*TILES_H);
          }      
        }
      }
      lcount++;
    }  
  } 

/*
 static char * ro="01234567890123456789012345678901234567";
 for (int i=0; i<TILES_ROWS*2; i++)
 {
  tileTextOverlay(0, i*8, ro, VGA_RGB(0x00,0xff,0x00)); 
 }
*/

 for (int i=0; i<SPRITES_MAX; i++)
 {
   drawSpr(spritesdata[i].index, spritesdata[i].x, spritesdata[i].y);
 }
}

void tile_data(unsigned char index, vga_pixel * data, int len)
{
  memcpy((void*)&tilesbuffer[index*TILES_W*TILES_H],(void*)data,len); 
}

void VGA_T4::sprite_data(unsigned char index, vga_pixel * data, int len)
{ 
  memcpy((void*)&spritesbuffer[index*SPRITES_W*SPRITES_H],(void*)data,len); 
}

void VGA_T4::sprite(int id , int x, int y, unsigned char index)
{
  if (id < SPRITES_MAX) {
    spritesdata[id].x = x;  
    spritesdata[id].y = y;  
    spritesdata[id].index = index;  
  }
}


void VGA_T4::sprite_hide(int id)
{
  if (id < SPRITES_MAX) {
    spritesdata[id].x = -16;  
    spritesdata[id].y = -16;  
    spritesdata[id].index = 0;  
  }  
}

void VGA_T4::tile_draw(int layer, int x, int y, unsigned char index)
{
  tilesram[(y+layer*TILES_ROWS)*TILES_COLS+x] = index;
}

void VGA_T4::tile_draw_row(int layer, int x, int y, unsigned char * data, int len)
{
  while (len--)
  {
    tilesram[(y+layer*TILES_ROWS)*TILES_COLS+x++] = *data++;
  }
}

void VGA_T4::tile_draw_col(int layer, int x, int y, unsigned char * data, int len)
{
  while (len--)
  {
    tilesram[(y++ +layer*TILES_ROWS)*TILES_COLS+x] = *data++;
  }
}

void VGA_T4::hscroll(int layer, int value)
{
  hscr[layer] = value;
}

void VGA_T4::vscroll(int layer, int value)
{
  vscr[layer] = value;
}

void VGA_T4::set_hscroll(int layer, int rowbeg, int rowend, int mask)
{
  hscr_beg[layer] = rowbeg;
  hscr_end[layer] = rowend;
  hscr_mask = mask+1;
}

void VGA_T4::set_vscroll(int layer, int colbeg, int colend, int mask)
{
  hscr_beg[layer] = colbeg;
  hscr_end[layer] = colend;
  hscr_mask = mask+1;
}

/*******************************************************************
 Experimental I2S interrupt based sound driver for PCM51xx !!!
*******************************************************************/

FLASHMEM static void set_audioClock(int nfact, int32_t nmult, uint32_t ndiv, bool force) // sets PLL4
{
  if (!force && (CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_ENABLE)) return;

  CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE
           | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) // 2: 1/4; 1: 1/2; 0: 1/1
           | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

  CCM_ANALOG_PLL_AUDIO_NUM   = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
  CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;
  
  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) {}; //Wait for pll-lock
  
  const int div_post_pll = 1; // other values: 2,4
  CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
  if(div_post_pll>1) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
  if(div_post_pll>3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;
  
  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;//Disable Bypass
}

#define AUDIO_SAMPLE_RATE_EXACT  11025.0 //44117.64706 //11025.0 //22050.0 //44117.64706 //31778.0

FLASHMEM static void config_sai1()
{
  CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);
  double fs = AUDIO_SAMPLE_RATE_EXACT;
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);
  double C = (fs * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);

  set_audioClock(c0, c1, c2, true);
  // clear SAI1_CLK register locations
  CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
               | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4

  n1 = n1 / 2; //Double Speed for TDM

  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
               | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) // &0x07
               | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1); // &0x3f

  IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
                    | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));  //Select MCLK


  // configure transmitter
  int rsync = 0;
  int tsync = 1;

  I2S1_TMR = 0;
  I2S1_TCR1 = I2S_TCR1_RFW(1);
  I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP // sync=0; tx is async;
        | (I2S_TCR2_BCD | I2S_TCR2_DIV((1)) | I2S_TCR2_MSEL(1));
  I2S1_TCR3 = I2S_TCR3_TCE;
  I2S1_TCR4 = I2S_TCR4_FRSZ((2-1)) | I2S_TCR4_SYWD((32-1)) | I2S_TCR4_MF
        | I2S_TCR4_FSD | I2S_TCR4_FSE | I2S_TCR4_FSP;
  I2S1_TCR5 = I2S_TCR5_WNW((32-1)) | I2S_TCR5_W0W((32-1)) | I2S_TCR5_FBT((32-1));


  I2S1_RMR = 0;
  I2S1_RCR1 = I2S_RCR1_RFW(1);
  I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_RCR2_BCP  // sync=0; rx is async;
        | (I2S_RCR2_BCD | I2S_RCR2_DIV((1)) | I2S_RCR2_MSEL(1));
  I2S1_RCR3 = I2S_RCR3_RCE;
  I2S1_RCR4 = I2S_RCR4_FRSZ((2-1)) | I2S_RCR4_SYWD((32-1)) | I2S_RCR4_MF
        | I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
  I2S1_RCR5 = I2S_RCR5_WNW((32-1)) | I2S_RCR5_W0W((32-1)) | I2S_RCR5_FBT((32-1));

  //CORE_PIN23_CONFIG = 3;  // MCLK
  CORE_PIN21_CONFIG = 3;  // RX_BCLK
  CORE_PIN20_CONFIG = 3;  // RX_SYNC
  CORE_PIN7_CONFIG  = 3;  // TX_DATA0
  I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
  I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE  | I2S_TCSR_FRDE ;//<-- not using DMA */;
}



//DMAMEM __attribute__((aligned(32))) static uint32_t i2s_tx[1024];

static bool fillfirsthalf = true;
static uint16_t cnt = 0;
static uint16_t sampleBufferSize = 0;

static void (*fillsamples)(short * stream, int len) = nullptr;

static uint32_t * i2s_tx_buffer __attribute__((aligned(32)));
static uint16_t * i2s_tx_buffer16;
static uint16_t * txreg = (uint16_t *)((uint32_t)&I2S1_TDR0 + 2);


FASTRUN void VGA_T4::AUDIO_isr() {
  
  *txreg = i2s_tx_buffer16[cnt]; 
  cnt = cnt + 1;
  cnt = cnt & (sampleBufferSize*2-1);

  if (cnt == 0) {
    fillfirsthalf = false;
    NVIC_SET_PENDING(IRQ_SOFTWARE);
  } 
  else if (cnt == sampleBufferSize) {
    fillfirsthalf = true;
    NVIC_SET_PENDING(IRQ_SOFTWARE);
  }
/*
  I2S1_TDR0 = i2s_tx_buffer[cnt]; 
  cnt = cnt + 1;
  cnt = cnt & (sampleBufferSize-1);
  if (cnt == 0) {
    fillfirsthalf = false;
    NVIC_SET_PENDING(IRQ_SOFTWARE);
  } 
  else if (cnt == sampleBufferSize/2) {
    fillfirsthalf = true;
    NVIC_SET_PENDING(IRQ_SOFTWARE);
  }
*/
}

FASTRUN void VGA_T4::SOFTWARE_isr() {
  //Serial.println("x");
  if (fillfirsthalf) {
    fillsamples((short *)i2s_tx_buffer, sampleBufferSize);
    arm_dcache_flush_delete((void*)i2s_tx_buffer, (sampleBufferSize/2)*sizeof(uint32_t));
  }  
  else { 
    fillsamples((short *)&i2s_tx_buffer[sampleBufferSize/2], sampleBufferSize);
    arm_dcache_flush_delete((void*)&i2s_tx_buffer[sampleBufferSize/2], (sampleBufferSize/2)*sizeof(uint32_t));
  }
}

// display VGA image
FLASHMEM void VGA_T4::begin_audio(int samplesize, void (*callback)(short * stream, int len))
{
  fillsamples = callback;
  i2s_tx_buffer =  (uint32_t*)malloc(samplesize*sizeof(uint32_t)); //&i2s_tx[0];

  if (i2s_tx_buffer == NULL) {
    Serial.println("could not allocate audio samples");
    return;  
  }
  memset((void*)i2s_tx_buffer,0, samplesize*sizeof(uint32_t));
  arm_dcache_flush_delete((void*)i2s_tx_buffer, samplesize*sizeof(uint32_t));
  i2s_tx_buffer16 = (uint16_t*)i2s_tx_buffer;

  sampleBufferSize = samplesize;

  config_sai1();
  attachInterruptVector(IRQ_SAI1, AUDIO_isr);
  NVIC_ENABLE_IRQ(IRQ_SAI1);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);  // 0 highest priority, 255 = lowest priority 
  NVIC_SET_PRIORITY(IRQ_SAI1, 127);
  attachInterruptVector(IRQ_SOFTWARE, SOFTWARE_isr);
  NVIC_SET_PRIORITY(IRQ_SOFTWARE, 208);
  NVIC_ENABLE_IRQ(IRQ_SOFTWARE);

  I2S1_TCSR |= 1<<8;  // start generating TX FIFO interrupts

  Serial.print("Audio sample buffer = ");
  Serial.println(samplesize);
}
 
FLASHMEM void VGA_T4::end_audio()
{
  if (i2s_tx_buffer != NULL) {
  	free(i2s_tx_buffer);
  }
}

