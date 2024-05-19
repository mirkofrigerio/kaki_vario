/*************************
  This is the developement script that powers Kaki's Vario. The components
  required are as follow
  - Arduino Nano clone
  - MS5611 pressure sensor
  - Bluetooth sensor HC-05 or HC-06

  The main purpose of the Kaki vario is to send high-frqeuancy barometer 
  sensor data to a smartphone, where it' used by XCTrack to compute variational
  height and determine the ratio of ascent/descent. XCTrack vario pre-processes the 
  sensor data through a filter, so there is no need to treat outliers here. We'll
  just send the raw data over bluetooth to our smartphone. The data is sent over
  strings with lk8000 format:
   LK8000 sentence : 32 chars to send
   $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

pin diagram:

  MS5611
    MS5611 VCC - Nano 3v3
    MS5611 GND - Nano GND
    MS5611 SCL - Nano A5
    MS5611 SDA - Nano A4
    MS5611 CSB - Nano GND

  HC-05
    HC-05 5v - Nano 5v // module is 3.3v but voltage divider on board. Better to provide 3v3 instead maybe? 
    HC-05 GND - Nano GND
    HC-05 RX - Nano TX // alternatively use MISO or D pin. Need softwareSerial for those tho

  OLED
    OLED VCC - Nano 3v3
    OLED GND - Nano GND
    OLED SCL - Nano A5
    OLED SDA - Nano A4  

  BUZZER
    BUZZER - Nano D9
    BUZZER - Nano D10

  Double Toggle SLIDE SWITCH (3 positions, 8 pins)

  First row: controls On - OFF

  I   I   I    I
  vin vin O+

  O- ---> Nano GND
    

 *************************/

// #include <Wire.h>
#include <MS5611.h>  // Library for MS5611 pressure sensor
#include <SoftwareSerial.h>

// Define the HC-05 serial connection
SoftwareSerial BTserial(10, 11);  // RX, TX

// Create an instance of the MS5611 sensor
MS5611 MS5611(0x77);

void setup() {

  // Initialize BMP Pressure and Temp Sensor. 
  MS5611.begin();
  MS5611.setOversampling(OSR_STANDARD);  
  MS5611.reset(1); // corrects pressure measurements by factor of 2. Not needed on all MS5611 chips

  // Initialize Bluetooth communication
  BTserial.begin(9600);

}

void loop() {
  // Read MS5611
  int result = MS5611.read();

  // Read pressure and temperature from MS5611
  float pressure = MS5611.getPressure()*100;
  float temperature = MS5611.getTemperature();

  // Create LK8000 sentences for communication XC Track app
  LK8000_sentences(pressure, temperature);

  delay(100);  // Adjust the delay as needed for your desired frequency
}

void LK8000_sentences(uint32_t pressure, int8_t temperature) {
  //https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt
  //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  //$LK8EX1,pressure,99999,9999,temp,999,*checksum
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