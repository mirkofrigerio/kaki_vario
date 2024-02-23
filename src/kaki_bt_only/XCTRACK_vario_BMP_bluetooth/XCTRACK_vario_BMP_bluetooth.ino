/***************************************************************************
  This is the developement script that powers Kaki's Vario. The components
  required are as follow
  - Arduino Nano clone
  - BMP280 pressure sensor
  - 128x64 I2C Display
  - Buzzer
  - Bluetooth sensor HC-05 or HC-06

  The main purpose of the Kaki vario is to send high-frqeuancy barometer 
  sensor data to a smartphone, where it' used by XCTrack to compute variational
  height and determine the ratio of ascent/descent. XCTrack vario pre-processes the 
  sensor data through a filter, so there is no need to treat outliers here. We'll
  just send the raw data over bluetooth to our smartphone. The data is sent over
  strings with lk8000 format:
   LK8000 sentence : 32 chars to send
   $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

 ***************************************************************************/

#define FREQUENCY 20  // freq output in Hz : uncomment this line and adjust value for frequency mode

#include <avr/wdt.h>
#define RX 11                     // not used (TX module bluetooth)
#define TX 12                     // MISO on ISCP (RX module bluetooth)
#include <SoftwareSerial.h>
SoftwareSerial BTserial(RX, TX);  // RX not connected

#include <Wire.h>
#include <SPI.h> //needed?
#include <Adafruit_BMP280.h>

// // Pressure and Temp sensor
Adafruit_BMP280 bmp; // I2C


#define BMP_ADDRESS 0x76 // Default address, if it doesn't work try 0x77
#define BLUETOOTH_SPEED 9600      //bluetooth speed (9600 by default)


void setup() {
  Serial.begin(9600);

  // Initialize BMP Pressure and Temp Sensor. 
  bmp280_initialize();

  // Initialize Bluetooth
  BTserial.begin(BLUETOOTH_SPEED); 

  // Initialize 128x64 OLED Screen

}

void loop() {

  // read Pressure and Temperature
  uint32_t pressure = bmp.readPressure();
  int8_t temperature = bmp.readTemperature();

  // Create LK8000 sentences for communication XC Track app
  LK8000_sentences(pressure, temperature);

  // VarioMSCalculation();

  // Serial.println();
  delay(100);
}


void bmp280_initialize() {
  while ( !Serial ) delay(100);   // wait for native usb
  unsigned status;
  status = bmp.begin(BMP_ADDRESS);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */  
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

  // debugging 
  Serial.println(str_out);

}

// // Variofunktion FROM ANDREI's Project https://www.instructables.com/id/DIY-Arduino-Variometer-for-Paragliding
// void VarioMSCalculation(){
//   tempo = millis();                                                       // Beschreibung einer Tempovariablen vom Typ float und Zuweisung eines Wertes mit der Funktion millis () - Zählt die Zeit ab Programmstart in Millisekunden
//   N1 = 0;                                                                 // Variable zur Mittelwertbildung
//   N2 = 0;                                                                 // Variable zur Mittelwertbildung
//   N3 = 0;                                                                 // Variable zur Mittelwertbildung
//   D1 = 0;                                                                 // Variable zur Mittelwertbildung
//   D2 = 0;                                                                 // Variable zur Mittelwertbildung
  
//   ///// ОБНУЛЕНИЕ ВАРИОМЕТРА / ZERO VARIO /////
//   vario = 0;       
  
//   for(int cc=1; cc<=maxsamples; cc++)                                     // Averager 
//   {                                                                       // 
//     alt[(cc-1)] = alt[cc];                                                // http://www.instructables.com/id/GoFly-paraglidinghanglidinggliding-altimeter-v/?ALLSTEPS
//     tim[(cc-1)] = tim[cc];                                                // http://redhats.ru/variometer-arduino-2015/
//   }                                                                       //
//                                                                           //
//   alt[maxsamples] = bmp.readAltitude();                                   //
//   tim[maxsamples] = tempo;                                                //
//   float stime = tim[maxsamples-samples];                                  //
//                                                                           //
//   for(int cc=(maxsamples-samples); cc<maxsamples; cc++)                   //
//   {                                                                       //
//     N1+=(tim[cc]-stime)*alt[cc];                                          //
//     N2+=(tim[cc]-stime);                                                  //
//     N3+=(alt[cc]);                                                        //
//     D1+=(tim[cc]-stime)*(tim[cc]-stime);                                  //
//     D2+=(tim[cc]-stime);                                                  //
//   }                                                                       // Durchschnittliches Körperende 
  
//   /////VARIO VALUES CALCULATING /////
//   vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);                     // BERECHNUNG VON VARIOMETERWERTEN

//   //Serial.print("Vario Russ: ");
//   Serial.println(vario);
// }



// void VarioBeep(float TreshUp, float TreshDown){

//     if ((tempo - beep) > Beep_period + 100)                         
//   {                                                                
//     beep=tempo;                                                    
//     if (vario > TreshUp && vario<15)                                
//     {                                                              
      
//       Beep_period = 400-(vario*80);       
      
//       tone( buzzer , (500 + (250 * vario)), abs(Beep_period));
//     }                                             
//     else if (vario < TreshDown)                      // Sinking Tone
//     {                                              
//       Beep_period=200;                                           
//       tone(buzzer,200,400);                                    
//     }                                                          
//   }           
  
// }