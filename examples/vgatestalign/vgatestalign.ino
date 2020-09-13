#include <VGA_t4.h>

static VGA_T4 vga;
static int fb_width, fb_height;

uint16_t screenx = 640;
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

  // Polygon data
  PolySet.Center.x = 40      ; PolySet.Center.y = 40 ; 
  PolySet.Pts[0].x = 30      ; PolySet.Pts[0].y = 10  ;
  PolySet.Pts[1].x = 40     ; PolySet.Pts[1].y = 20 ;
  PolySet.Pts[2].x = 40     ; PolySet.Pts[2].y = 40 ;
  PolySet.Pts[3].x = 60     ; PolySet.Pts[3].y = 40 ;
  PolySet.Pts[4].x = 60     ; PolySet.Pts[4].y = 70 ;
  PolySet.Pts[5].x = 50      ; PolySet.Pts[5].y = 70 ;
  PolySet.Pts[6].x = 50    ; PolySet.Pts[6].y = 50 ;
  PolySet.Pts[7].x = 20    ; PolySet.Pts[7].y = 50 ;
  PolySet.Pts[8].x = 20    ; PolySet.Pts[8].y = 40 ;
  PolySet.Pts[9].x = 10    ; PolySet.Pts[9].y = 40 ;
  PolySet.Pts[10].x = 10   ; PolySet.Pts[10].y = 30 ;
  PolySet.Pts[11].x = 20   ; PolySet.Pts[11].y = 30 ;
  PolySet.Pts[12].x = 10000 ; // ****** last point.x >= 10000 ******
}

void loop() {
  
  vga.clear(VGA_RGB(0xff,0xff,0xff));
  delay(1000);

  for (int i = 0;i<100000;i++){
    vga.drawPixel(random(screenx) , random(screeny), VGA_RGB(random(256),random(256),random(256)));
  }
  
  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<screeny;i++){
    if((i % 2) == 0)
      vga.draw_h_line(320 , i, (int16_t)(i/2), VGA_RGB(random(256),random(256),random(256)));
    else
      vga.draw_h_line(320 , i, -((int16_t)(i/2)), VGA_RGB(random(256),random(256),random(256)));
  }
  
  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<screenx;i++){
    if((i % 2) == 0)
      vga.draw_v_line(i , 240, (int16_t)(i/3), VGA_RGB(random(256),random(256),random(256)));
    else
      vga.draw_v_line(i , 240, -((int16_t)(i/3)), VGA_RGB(random(256),random(256),random(256)));
  }
  
  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (uint16_t i = 0;i<100;i++){
    vga.drawcircle(random(screenx) , random(screeny), random(100)+20 , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (uint16_t i = 0;i<1000;i++){
    vga.drawfilledcircle(random(screenx) , random(screeny), random(100)+20 , VGA_RGB(random(256),random(256),random(256)) , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (uint16_t i = 0;i<100;i++){
    vga.drawellipse(random(screenx) , random(screeny) , random(100)+20 ,random(100)+20 , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (uint16_t i = 0;i<1000;i++){
    vga.drawfilledellipse(random(screenx) , random(screeny) , random(100)+20 ,random(100)+20 , VGA_RGB(random(256),random(256),random(256)) , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<1000;i++){
    vga.drawfilledtriangle(random(screenx) , random(screeny) ,random(screenx) , random(screeny) ,random(screenx) , random(screeny) , VGA_RGB(random(256),random(256),random(256)) , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawtriangle(random(screenx) , random(screeny) ,random(screenx) , random(screeny) ,random(screenx) , random(screeny) , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<1000;i++){
    vga.drawfilledquad(random(screenx) , random(screeny) ,random(150)+10 , random(150)+10 ,random(360) , VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawquad(random(screenx) , random(screeny) ,random(150)+10 , random(150)+10,random(360) , VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<200;i++){
    vga.drawline(random(screenx) , random(screeny) , random(screenx) , random(screeny), VGA_RGB(random(256),random(256),random(256)));
  }
  
  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawpolygon(random(screenx) , random(screeny), VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawfullpolygon(random(screenx) , random(screeny), VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawrotatepolygon(random(screenx) , random(screeny) , random(360), VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)),0);
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<100;i++){
    vga.drawrotatepolygon(random(screenx) , random(screeny) , random(360), VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)),1);
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

  for (int i = 0;i<1000;i++){
    vga.drawrotatepolygon(200 , 240 , i % 360, VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)),0);
    vga.drawrotatepolygon(400 , 240 , i % 360, VGA_RGB(random(256),random(256),random(256)), VGA_RGB(random(256),random(256),random(256)),1);
    delay(10);
    vga.clear(VGA_RGB(0xff,0xff,0xff));
  }

  delay(1000);
  vga.clear(VGA_RGB(0xff,0xff,0xff));

}
