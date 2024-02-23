/*
Kaki V.0

pin diagram:

BMP
  BMP VCC - Nano 3v3
  BMP GND - Nano GND
  BMP SCL - Nano A5
  BMP SDA - Nano A4

HC-05
  HC-05 5v - Nano 5v // module is 3.3v but voltage divider on board. Better to provide 3v3 instead maybe? 
  HC-05 GND - Nano GND
  HC-05 RX - Nano MISO ?? // or D11 (soft TX)

OLED
  OLED VCC - Nano 3v3
  OLED GND - Nano GND
  OLED SCL - Nano A5
  OLED SDA - Nano A4  

BUZZER
  BUZZER - Nano D9
  BUZZER - Nano D10

*/


// hardcoded variables
#define BLUETOOTH_SPEED 9600      //bluetooth speed (9600 by default)
#define FREQUENCY 20  // freq output in Hz : uncomment this line and adjust value for frequency mode
#define RX 11                     // not used (TX module bluetooth) // 0r 10
#define TX 12                     // MISO on ISCP (RX module bluetooth) // Or 11?
#define BUZZER_PIN 9
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// IC2 addresses
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define BMP_ADDRESS 0x76 // Default address, if it doesn't work try 0x77

// libraries 
// #include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include "SSD1306AsciiAvrI2c.h"
#include <VoltageReference.h>
#include <Adafruit_BMP280.h>
#include <toneAC.h>

// bluetooth - RX not connected
SoftwareSerial BTserial(RX, TX); 

// oled display
SSD1306AsciiAvrI2c oled;

// Pressure and Temp sensor
Adafruit_BMP280 bmp;

// Battery voltage
VoltageReference vRef;

// Define pin for the button
const int buttonPin = 4; 
// Variable to store the previous state of the button
int prevButtonState = HIGH;

// standard settings - will be changed by user through button
boolean bluetooth_on = true;  // Bluetooth on / off
boolean beep_on = false;      // Audio on / off


// CODE FROM ANDREI's Project https://www.instructables.com/id/DIY-Arduino-Variometer-for-Paragliding
float vario = 0;    
byte samples = 40;    
byte maxsamples = 50;
float alt[51];                                                       
float tim[51];  
float tempo = millis();             // Beschreibung einer Tempovariablen vom Typ float und Zuweisung eines Wertes mit der Funktion millis () - Zählt die Zeit ab Programmstart in Millisekunden
float N1 = 0;                       // Variable zur Mittelwertbildung                                 
float N2 = 0;                       // Variable zur Mittelwertbildung                                
float N3 = 0;                       // Variable zur Mittelwertbildung                                 
float D1 = 0;                       // Variable zur Mittelwertbildung                               
float D2 = 0;                       // Variable zur Mittelwertbildung       

// To count time between beeps
float beep = 10.0;                                  
float Beep_period = 10.0;


// DISPLAY
float last_display_update = 10.0;  // To count time beteween display updates
int refresh_rate = 1000; // refreshes every X ms


// images and symbols 

int a; // init position in array for draw_image function

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


void setup()   {                
  Serial.begin(9600);

  //Voltage Measurement
  vRef.begin();

  // Initialize BMP Pressure and Temp Sensor. 
  bmp.begin(0x76);

  // Initialize Bluetooth
  BTserial.begin(BLUETOOTH_SPEED); 

  // Initialize 128x64 OLED Screen
  oled.begin(&Adafruit128x64, SCREEN_ADDRESS);
  oled.setFont(Adafruit5x7);

  // Buzzer output on Pin 9 for toneAC
  pinMode(BUZZER_PIN, OUTPUT);

  // Set the Button pin as an input
  pinMode(buttonPin, INPUT);

  // wait until the Sensor is ready
  welcomeDisplay();

}

/*
TODO:
  * round climb rate to the first decimal, like +1.33 m/s -> 1.30 m/s 
  * test bluetooth - DONE
  * pinout -- DONE
  * test audio -- DONE works when bluetooth is off? weird. Need to investigate
  * add button to switch between modes = DONE -check state changes as expected
  * profits??
*/

void loop() {
/*
  Objectives: 
    * P0: keep reading the raw values pressure from the sensor and send them to the phone via bluetooth -> for XC Track
    * P1: at each read from the sensor we also compute the average climb rate based on a sliding window of the last 50 sensor readings -> for audio
    * P1: if the climbing rate goes over 2m/s (or TreshUp) initiate the beep. Beep length varies based on climb rate, but when it's reached it will delay readings for up to 400ms. Problem for sensor data bt trasmission
    * P1: every x operations update the screen with what's going on: we can either take the avergae of the past x operations or just take current values

*/
  // read Pressure, Temperature and Altitude
  uint32_t pressure = bmp.readPressure();
  int8_t temperature = bmp.readTemperature();
  float altitude = bmp.readAltitude();

  // Calculate vario (climb rate)
  vario = VarioMSCalculation(altitude);

  // Read the current state of the button and cycle through bluetooth and audio mode
  int buttonState = digitalRead(buttonPin);
  // Check if the button is pressed (LOW) and the previous state was not pressed
  if (buttonState == LOW && prevButtonState == HIGH) {
    // Toggle the value of bluetooth_mode
    bluetooth_on = !bluetooth_on;
    beep_on = !beep_on;
  }
  // Update the previous button state
  prevButtonState = buttonState;

  // bluetooth and audio
  if (bluetooth_on){
    // Create LK8000 sentences for communication XC Track app
    LK8000_sentences(pressure, temperature);
  };

  // Beep (Treshold up/Treshold Down
  if (beep_on){             // only when beep is on
    VarioBeep(1,-1);  // low values for testing
  }

  
  // Every 600 ms update display:
  if ((tempo - last_display_update) > refresh_rate) {   //  tempo is updated in VarioMSCalculation(). updates every 600ms
    last_display_update = tempo;

    // read battery
    int vcc = 3900; // vRef.readVcc();
    vcc = abs(vcc - 50); // FOR TESTING

    updateDisplay(altitude, temperature, vario, vcc);

    // testing
    // Serial.print("altitude: ");
    // Serial.print(altitude);
    // Serial.print("temp: ");
    // Serial.print(temperature);
    // Serial.print("vario: ");
    // Serial.println(vario);
  }
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



float VarioMSCalculation(float altitude){
  /*
  This is a moving window averager for the climb rate in m/s.
  Every time it runs it updates the last value of two arrays of altitude (alt) and time (tim). 
  */
  tempo = millis();                                                      
  N1 = 0;                              
  N2 = 0;                             
  N3 = 0;                              
  D1 = 0;                                       
  D2 = 0;                                
  
  vario = 0;       
  
  // shift every previous measurement stored in the arrays back of one position 
  for(int cc=1; cc<=maxsamples; cc++)
  {
    alt[(cc-1)] = alt[cc];
    tim[(cc-1)] = tim[cc];
  }

  // updates the last value in the array with the measured altitude and time
  alt[maxsamples] = altitude;
  tim[maxsamples] = tempo;
  
  // defines starting time. maxsamples is 50. samples is 40. So sets the stime to the tenth value of time array: tim[10]
  float stime = tim[maxsamples-samples];

  for(int cc=(maxsamples-samples); cc<maxsamples; cc++)
  { 
    N1+=(tim[cc]-stime)*alt[cc]; // time since start time per altitude
    N2+=(tim[cc]-stime); // time since start time
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  }
  
  vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);

  return vario;

}

void VarioBeep(float TreshUp, float TreshDown) {
  // enforces beeps are separate between eachother.
  if ((tempo - beep) > Beep_period + 100) {   //  From the previous loop till now at least Beep_period+100 must have passed (400-200ms depending on vario) 
    beep = tempo;
    if (vario > TreshUp && vario < 15) { // if the climb rate is more than threshold and less than 15 play
      Beep_period = 400 - (vario * 80); // brekes at very high clib rates
      toneAC(500 + (250 * vario),5);
      delay(abs(Beep_period)); // shit way to force Beep_period to be positive
      toneAC();
    } else if (vario < TreshDown) {
      Beep_period = 200;
      toneAC(200, 5);
      delay(400);
      toneAC();
    }
  }
}


void LK8000_sentences(uint32_t pressure, int8_t temperature) {
  //https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt
  //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  char s[128];
  char format[]="LK8EX1,%lu,0,9999,%d,999,";
  snprintf(s,sizeof(s),format,pressure,temperature);
  // Calculating checksum for data string
  uint16_t checksum = 0, bi;
  for (uint8_t ai = 0; ai < strlen(s); ai++) {
    bi = (uint8_t)s[ai];
    checksum ^= bi;
  }
  char s_checksum[128];
  snprintf(s_checksum,sizeof(s_checksum),"*%X",checksum);
  strcat(s,s_checksum);
  char str_out[128]="$";
  strcat(str_out,s);
  
  // now that we have str_out we need to pass it to the bluetooth module
  BTserial.println(str_out);
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
