#include <VGA_t4.h>

static VGA_T4 vga;
static int fb_width, fb_height;

uint16_t screenx = 352;
uint16_t screeny = 480;

#define RED(x)        VGA_RGB(x, 0, 0)
#define GREEN(x)      VGA_RGB(0, x, 0)
#define BLUE(x)       VGA_RGB(0, 0, x)
#define WHITE(x)      VGA_RGB(x, x, x)

#define LIGHT_BLUE VGA_RGB(0, 136, 255)

 
void setup() {
  vga_error_t err = vga.begin(VGA_MODE_640x480);
  if(err != 0)
  {
    Serial.println("fatal error");
    while(1);
  }
  
  vga.clear(VGA_RGB(0xff,0xff,0xff));
  vga.drawRect(120,0,230,240,LIGHT_BLUE);
  vga.drawRect(130,10,50,60,RED(0xff));

  vga.drawRect(120,350,50,50,RED(0xff));
  vga.drawRect(180,350,50,50,GREEN(0xff));
  vga.drawRect(240,350,50,50,BLUE(0xff));

  vga.drawline(50 , 200 , 300 , 400, BLUE(0xff));
  vga.drawcircle(176 , 240 , 170 , VGA_RGB(0x00,0x00,0x00));
  vga.drawfilledcircle(176 , 240 , 50 , RED(0xff) , GREEN(0xff));
  vga.drawellipse(100 , 300 , 50 , 30 , BLUE(0xff));
  vga.drawfilledellipse(100 , 300 , 40 , 20 , GREEN(0xff) , RED(0xff));
  vga.drawtriangle(200 ,100, 250, 400, 300, 250, BLUE(0xff));
  vga.drawfilledtriangle(205 ,110, 250, 390, 290, 240, LIGHT_BLUE , RED(0xff));
  for (int i = 0;i<screeny;i+=10){
    vga.drawline(300 , i , 350 , i, VGA_RGB(0x00,0x00,0x00));
  }
  for (int i = 0;i<screenx;i+=10){
    vga.drawline(i , 420 , i , 470, VGA_RGB(0x00,0x00,0x00));
  }
  vga.drawquad(80 , 400 , 50 , 30 , 45 , BLUE(0xff));
  vga.drawfilledquad(130 , 400 , 50 , 30 , 195 , GREEN(0xff) , RED(0xff));
}

int count = 0;

void loop() {
   //count += 1;
   //count &= 255;
 //vga.clear(count<<5);
  //delay(1);
  //vga.waitSync();
  vga.drawRect(10,10+0  ,100,60,RED(0xff));
  vga.drawRect(10,10+60 ,100,60,GREEN(0xff));
  vga.drawRect(10,10+120,100,60,BLUE(0xff));
  
}
