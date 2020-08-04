#include <VGA_t4.h>

static VGA_T4 vga;
static int fb_width, fb_height;

#define BLUE       VGA_RGB(0, 0, 170)
#define LIGHT_BLUE VGA_RGB(0, 136, 255)

 
void setup() {
  vga_error_t err = vga.begin(VGA_MODE_352x240);
  if(err != 0)
  {
    Serial.println("fatal error");
    while(1);
  }
  
  vga.clear(LIGHT_BLUE);
  vga.get_frame_buffer_size(&fb_width, &fb_height);
  vga.drawRect((fb_width-320)/2,(fb_height-200)/2, 320,200, BLUE);
  vga.drawText((fb_width-320)/2,(fb_height-200)/2+1*8,"    **** COMMODORE 64 BASIC V2 ****     ",LIGHT_BLUE,BLUE,false);
  vga.drawText((fb_width-320)/2,(fb_height-200)/2+3*8," 64K RAM SYSTEM  38911 BASIC BYTES FREE ",LIGHT_BLUE,BLUE,false);
  vga.drawText((fb_width-320)/2,(fb_height-200)/2+5*8,"READY.",LIGHT_BLUE,BLUE,false);
}


void loop() {
  vga.drawText((fb_width-320)/2,(fb_height-200)/2+6*8," ",BLUE,LIGHT_BLUE,false);
  delay(500);
  vga.drawText((fb_width-320)/2,(fb_height-200)/2+6*8," ",BLUE,BLUE,false);
  delay(500);
  //vga.debug();
}