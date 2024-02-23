

// Simple I2C test for 128x64 oled.
// Use smaller faster AvrI2c class in place of Wire.
// Edit AVRI2C_FASTMODE in SSD1306Ascii.h to change the default I2C frequency.
//
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// IC2 addresses
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

SSD1306AsciiAvrI2c oled;

int a; // Position in array - Don't change - an array larger than 256 will need to use "int = a"

// 'bluetooth', 16x16px
const unsigned char epd_bitmap_bluetooth [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xc0, 0xfe, 0x6c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x03, 0x3f, 0x13, 0x0e, 0x04, 0x00, 0x00, 0x00, 0x00
};

// 'audio', 16x16px
const unsigned char epd_bitmap_audio [] PROGMEM = {
0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xe0, 0xf0, 0xf0, 0x00, 0x40, 0x80, 0x20, 0xc8, 0x30, 0xe0, 0x00, 
0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x0f, 0x1f, 0x00, 0x06, 0x03, 0x0c, 0x13, 0x18, 0x07, 0x00
};

// 'battery', 32x16px
const unsigned char epd_bitmap_battery [] PROGMEM = {
	0x00, 0x00, 0x00, 0xf8, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 
	0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0xf8, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0x0f, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 
	0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0f, 0x07, 0x03, 0x00, 0x00, 0x00
};

// 'battery_percent', 29x16px
const unsigned char epd_bitmap_battery_percent [] PROGMEM = {
	0xfe, 0xff, 0xff, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 
	0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 
	0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 
	0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xff, 0xff, 0x7f
};

const unsigned char epd_bitmap_welcome [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf0, 0xf8, 
	0xf8, 0xfc, 0xfc, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfc, 0xfc, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 
	0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 
	0x3f, 0x1f, 0x1f, 0x0f, 0x0f, 0x07, 0x07, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 
	0x07, 0x0f, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x3c, 0x3f, 0x7f, 0x3f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0x0f, 0x1f, 0x1f, 0x0f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0xe0, 0xe0, 0xf0, 0x70, 0x60, 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x20, 0x58, 0x3f, 
	0x1f, 0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};



float vario = -2.2;
float altitude    = 0.4;
float vcc = 4200.0;
int temp = 22;
bool beep_on = 1;
bool bluetooth_on = 1;

//------------------------------------------------------------------------------
void setup() {

  oled.begin(&Adafruit128x64, SCREEN_ADDRESS);
  // Call oled.setI2cClock(frequency) to change from the default frequency.

  oled.setFont(Adafruit5x7);

  welcomedisplay();
}

//------------------------------------------------------------------------------
void loop() {

    updateoled(altitude, temp, vario, vcc);
    vario = vario + 0.5;
    altitude = altitude + 300;
    vcc = vcc - 100;
    temp = temp - 2;
    delay(4000);
}


void welcomeDisplay(){
  oled.set2X();
  oled.setCursor(2,2);
  oled.println("Kaki");
  oled.set1X();
  oled.println("v 0.1");

  draw_image(0,60,64,64,epd_bitmap_welcome);

  delay(2000);
  oled.clear();
}

void updateDisplay(int altitude, int temp, float vario, float vcc){
    oled.clear();
    oled.set2X();
    
    // altitude
    oled.setCursor(0, 0);
    oled.print(altitude);
    oled.println("m");
    
    // vario
    oled.setCursor(0, 6);
    if (vario>0) {
      oled.print("+");
      oled.print(vario);
    }
    else oled.print(vario);
    oled.set1X();
    oled.print("m/s");

    // temp
    oled.setCursor(96, 7);
    oled.print(temp);
    oled.print(" C");

    // draw bluetooth icon if bluetooth is on
    if (bluetooth_on){
      draw_image(3,90,16,16, epd_bitmap_bluetooth);
    };

    // draw audio icon if audio vario is on    
    if (beep_on){
      draw_image(3,110,16,16, epd_bitmap_audio);
    };

    // battery - draw rect.
    draw_image(0,90,29,16, epd_bitmap_battery_percent);
    oled.setCursor(93, 0);
    int b_percent = round((1-(4200.0-vcc)/(4200.0 - 3500.0))*100.0);
    oled.print(b_percent);
    oled.print("%");
  
}



void draw_image(byte r, byte c, byte l, byte h, const unsigned char  image_name[]){
  // r and c are positions of the image, rows(8 pixel high each) and column(x axes)
  // l and h are size of image in pixels (ex. 32x16). 
  // h needs to be multiple of 8.

  a = 0; // resets position in array - Don't change 

  for (byte b = 0; b < h/8; b++) {
    oled.setCursor (c,(r+b)); 
    for (byte i = 0; i < l; i++) {
      oled.ssd1306WriteRam(pgm_read_byte(&image_name[a]));
      a++;
    }  
  }
}

