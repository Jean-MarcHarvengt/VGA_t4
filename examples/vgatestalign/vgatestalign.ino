#include <VGA_t4.h>

static VGA_T4 vga;
static int fb_width, fb_height;



#define RED(x)        VGA_RGB(x, 0, 0)
#define GREEN(x)      VGA_RGB(0, x, 0)
#define BLUE(x)       VGA_RGB(0, 0, x)
#define WHITE(x)      VGA_RGB(x, x, x)

#define LIGHT_BLUE VGA_RGB(0, 136, 255)

 
void setup() {
  vga_error_t err = vga.begin(VGA_MODE_352x480);
  if(err != 0)
  {
    Serial.println("fatal error");
    while(1);
  }
  
  vga.clear(WHITE(0xff));
  vga.drawRect(120,0,230,240,LIGHT_BLUE);
  vga.drawRect(130,10,50,60,RED(0xff));
}

int count = 0;

void loop() {
   count += 1;
   count &= 255;
 //vga.clear(count<<5);
  delay(50);
  vga.waitSync();
  vga.drawRect(10,10+0  ,100,60,RED(count));
  vga.drawRect(10,10+60 ,100,60,GREEN(count));
  vga.drawRect(10,10+120,100,60,BLUE(count));
  
}
