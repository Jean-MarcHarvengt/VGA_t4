# VGA_t4 library by J-M Harvengt

---
## 1. Description

This library offers VGA output to the Teensy4/4.1 MCU familly.<br>
It is inspired from the uVGA library for Teensy3.X by Eric Prevoteau and so, uses only few CPU resources.<br>
It currently support 8bits RGB(RRRGGGBB) as uVGA does but in theory could be extended to 12bits (not tested!)<br>
The implementation is quite different from the uVGA due to the different infrastructure of the T4 family:<br>

- QTimer3 (timer3) is used to generate the H-PUSE and the line interrupt (so also the V-PULSE)
- 2 FlexIO registers (1 and 2) and 2 DMA channels are used to generate RGB out, combining 2x4pins to create 8bits output.
- the DMA transfers are initiated from the line interrupt to generate pixels so the front/back porch pixels are also part of each line buffer.

It currently supports 352x240 and 512x240 resolutions<br>

See code and examples for more details:
- Mandlebrot example was taken from the uVGA library to illustrate close compatibility.
- Vgatest make use of the very limited GFX api offered.


---
## 2. Wiring

* Hsync pin (fixed: **15**) -> 82R resistor -> VGA 13
    *depend on QTIMER Timer3 output*

* Vsync pin (default: **8**) -> 82R resistor -> VGA 14
    *can be changed to any pin*

* the rest is very much dependent of the FlexIO 1 and 2 IO mapping:

* Teensy __pin 33__ (FlexIO1:7 = 0x80) -> 470R resistor -> VGA pin 1 (red)
* Teensy __pin 4__ (FlexIO1:6 = 0x40) -> 1k resistor -> VGA pin 1 (red)
* Teensy __pin 3__ (FlexIO1:5 = 0x20) -> 2k2 resistor -> VGA pin 1 (red)

* Teensy __pin 2__  (FlexIO1:4 = 0x10) -> 470R resistor -> VGA pin 2 (green)
* Teensy __pin 13__  (FlexIO2:3 = 0x08) -> 1k resistor -> VGA pin 2 (green)
* Teensy __pin 11__  (FlexIO2:2 = 0x04) -> 2k2 resistor -> VGA pin 2(green)

* Teensy __pin 12__ (FlexIO2:1 = 0x02) -> 390R resistor ->VGA Pin 3 (blue)
* Teensy __pin 10__ (FlexIO2:0 = 0x01) -> 820R resistor ->VGA Pin 3 (blue)

* Teensy pin GND -> VGA pins 5,6,7,8,10

for more accurate colors, replace 2k2 by 2k and 470R by 510R

---
## 3. Known issues

- currently fixed resolution of 352x240 or 512x240 pixels
- video memory is allocated using malloc in T4 heap
- as the 2 DMA transfers are not started exactly at same time, there is a bit of color smearing
- Some gitter (may be can be improved?) 
- Default is 8bits RRRGGGBB (332) but 12bits GBB0RRRRGGGBB (444) feasible BUT NOT TESTED !!!!
- I did not tested on an HDMI display with a VGA adapter
