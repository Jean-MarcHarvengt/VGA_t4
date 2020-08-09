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

// Full buffer including back/front porch 
static vga_pixel * gfxbuffer __attribute__((aligned(32)));
// Visible vuffer
static vga_pixel * framebuffer;
static int fb_width;
static int fb_height;
static int fb_stride;
static int left_border;
static int line_double;
static int pix_shift;


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

    DMA_CERQ = flexio2DMA.channel; // Disable DMAs
    DMA_CERQ = flexio1DMA.channel; 
    
    // Aligned 32 bits copy
    unsigned long * p=(uint32_t *)&gfxbuffer[fb_stride*y];
    flexio2DMA.TCD->NBYTES = 4;
    flexio2DMA.TCD->SADDR = p;
    flexio2DMA.TCD->SOFF = 4;
    flexio2DMA.TCD->SLAST = -fb_stride;
    flexio2DMA.TCD->BITER = fb_stride / 4;
    flexio2DMA.TCD->CITER = fb_stride / 4;
    flexio2DMA.TCD->DADDR = &FLEXIO2_SHIFTBUF0;
    flexio2DMA.TCD->DOFF = 0;
    flexio2DMA.TCD->DLASTSGA = 0;
    flexio2DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
    flexio2DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
    if (pix_shift) 
    {
      // Unaligned (source) 32 bits copy
      uint8_t * p2=(uint8_t *)&gfxbuffer[fb_stride*y+pix_shift];
      flexio1DMA.TCD->CITER = fb_stride / 4;
      flexio1DMA.TCD->BITER = fb_stride / 4;
      flexio1DMA.TCD->SADDR = p2;
      flexio1DMA.TCD->NBYTES = 4;
      flexio1DMA.TCD->SOFF = 1;
      flexio1DMA.TCD->SLAST = -fb_stride;
      flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
      flexio1DMA.TCD->DOFF = 0;
      flexio1DMA.TCD->DLASTSGA = 0;
      flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(2); // 8bits to 32bits
      flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ; // disable on completion
    }
    else 
    {
      // Aligned 32 bits copy
      p=(uint32_t *)&gfxbuffer[fb_stride*y];
      flexio1DMA.TCD->NBYTES = 4;
      flexio1DMA.TCD->SADDR = p;
      flexio1DMA.TCD->SOFF = 4;
      flexio1DMA.TCD->SLAST = -fb_stride;
      flexio1DMA.TCD->BITER = fb_stride / 4;
      flexio1DMA.TCD->CITER = fb_stride / 4;
      flexio1DMA.TCD->DADDR = &FLEXIO1_SHIFTBUFNBS0;
      flexio1DMA.TCD->DOFF = 0;
      flexio1DMA.TCD->DLASTSGA = 0;
      flexio1DMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32bits
      flexio1DMA.TCD->CSR |= DMA_TCD_CSR_DREQ;
    }      
    DMA_SERQ = flexio2DMA.channel; 
    DMA_SERQ = flexio1DMA.channel; // Enable DMAs

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



// display VGA image
vga_error_t VGA_T4::begin(vga_mode_t mode)
{
  uint32_t flexio_clock_div;

  switch(mode) {
    case VGA_MODE_352x240:
      left_border = 60;
      fb_width = 352;
      fb_height = 240 ;
      fb_stride = fb_width+left_border;
      flexio_clock_div = 35;
      line_double = 1;
      pix_shift = 2;
      break;
    case VGA_MODE_352x480:
      left_border = 60;
      fb_width = 352;
      fb_height = 480 ;
      fb_stride = fb_width+left_border;
      flexio_clock_div = 35;
      line_double = 0;
      pix_shift = 2;
      break;
    case VGA_MODE_512x240:
      left_border = 80;
      fb_width = 512;
      fb_height = 240 ;
      fb_stride = fb_width+left_border;
      flexio_clock_div = 24;
      line_double = 1;
      pix_shift = 0;
      break;
    case VGA_MODE_512x480:
      left_border = 80;
      fb_width = 512;
      fb_height = 480 ;
      fb_stride = fb_width+left_border;
      flexio_clock_div = 24;
      line_double = 0;
      pix_shift = 0;
      break;
  }	

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
  pinMode(PIN_B_B2,  OUTPUT); // FlexIO2:10 = 0x00400
  pinMode(PIN_B_B3,  OUTPUT); // FlexIO2:11 = 0x00800
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

  /* Set clock to 480 MHz for FlexIO1 and FlexIO2 */
  CCM_CCGR5 &= ~CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  CCM_CDCDR = (CCM_CDCDR & ~(CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(7) | CCM_CDCDR_FLEXIO1_CLK_PODF(7))) 
    | CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(0) | CCM_CDCDR_FLEXIO1_CLK_PODF(0);
  CCM_CCGR3 &= ~CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  CCM_CSCMR2 = (CCM_CSCMR2 & ~(CCM_CSCMR2_FLEXIO2_CLK_SEL(3))) | CCM_CSCMR2_FLEXIO2_CLK_SEL(3);
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
  inputSource = FLEXIO_SHIFTCFG_INSRC*(1);    // Input source from Shifter 1
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
#ifdef BITS12
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(5);  // 5-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D8
#else
  parallelWidth = FLEXIO_SHIFTCFG_PWIDTH(4);  // 8-bit parallel shift width
  pinSelect = FLEXIO_SHIFTCTL_PINSEL(4);      // Select pins FXIO_D4 through FXIO_D7
#endif  
  FLEXIO1_SHIFTCFG0 = parallelWidth | inputSource | stopBit | startBit;
  FLEXIO1_SHIFTCTL0 = timerSelect | timerPolarity | pinConfig | pinSelect | pinPolarity | shifterMode;
  
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
  // flexio_clock_div : Output clock frequency is N times slower than FlexIO clock (41.7 ns period);
#ifdef BITS12
  #define SHIFTS_PER_TRANSFER 8 // Shift out 8 times with every transfer = two 32-bit words = contents of Shifter 0 (16bits)
#else
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
  TMR3_CTRL3 = 0b0000000000100000;      //stop all functions of timer 
  TMR3_SCTRL3 = 0b0000000000000001;     //0(TimerCompareFlag),0(TimerCompareIntEnable),00(TimerOverflow)0000(NoCapture),0000(Capture Disabled),00, 0,1(OFLAG to Ext Pin)
  TMR3_CNTR3 = 0;
  TMR3_LOAD3 = 0;
  TMR3_COMP13 = 569-1;
  TMR3_CMPLD13 = 569-1;
  TMR3_COMP23 = 4174-1;
  TMR3_CMPLD23 = 4174-1;
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
    for (int i=0; i<fb_width; i++)
    {
      *dst++=buf[pos >> 8];
      pos +=step;
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
    for (int i=0; i<width; i++)
    {
      *dst++=*buf++;
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
