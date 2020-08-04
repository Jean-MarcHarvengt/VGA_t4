// Mandelbrot fractal (code from from the teensy uVGA library)

#include <VGA_t4.h>

static VGA_T4 vga;

const byte cmap[] =
{
	0b00000000, 0b11100000, 0b11100100, 0b11101000, 0b11101100, 0b11110000, 0b11110100, 0b11111000, 0b11111100,
	0b11011100, 0b10111100, 0b10011100, 0b01111100, 0b01011100, 0b00111100, 0b00011100, 0b00011101, 0b00011110,
	0b00011111, 0b00011011, 0b00010111, 0b00010011, 0b00001111, 0b00001011, 0b00000111, 0b00000011, 0b00100011,
	0b01000011, 0b01100011, 0b10000011, 0b10100011, 0b11000011, 0b11100011, 0b11100010, 0b11100001, 0b11100000, 0b00000000
};

void setup()
{
	int ret;
	ret = vga.begin(VGA_MODE_512x240);

	Serial.println(ret);

	if(ret != 0)
	{
		Serial.println("fatal error");
		while(1);
	}
  vga.clear(VGA_RGB(0x00,0x00,0x00));
}

void loop()
{
	int fb_width, fb_height;
	int max = 256;
	int row, col;
	float c_re, c_im, x, y, x_new;
	int iteration;

	vga.get_frame_buffer_size(&fb_width, &fb_height);

	for (row = 0; row < fb_height; row++)
	{
		for (col = 0; col < fb_width; col++)
		{
			c_re = (col - fb_width / 1.5) * 4.0 / fb_width;
			c_im = (row - fb_height / 2.0) * 4.0 / fb_width;
			x = 0;
			y = 0;
			iteration = 0;

			while (((x * x + y * y) <= 4) && (iteration < max))
			{
				x_new = x * x - y * y + c_re;
				y = 2 * x * y + c_im;
				x = x_new;
				iteration++;
			}
			if (iteration < max)
				vga.drawPixel(col, row, cmap[iteration]);
			else
				vga.drawPixel(col, row, 0);
		}
	}
	for(;;);
}
