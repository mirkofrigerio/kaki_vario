#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <toneAC.h>

#define BUZZER_PIN 9

Adafruit_BMP280 bmp;  // Assuming you have initialized the BMP sensor


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

//BEEP
float beep = 10.0;                                  
float Beep_period = 10.0;  

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  delay(2000);
}

void loop() {

  // read Pressure, Temperature and Altitude
  uint32_t pressure = bmp.readPressure();
  int8_t temperature = bmp.readTemperature();
  float altitude = bmp.readAltitude();
  // int vcc = vRef.readVcc();

  // float climbRate = calculateClimbRate(altitude);
  float climbRate = VarioMSCalculation(altitude);

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.print(" meters | Climb Rate: ");
  Serial.print(climbRate);
  Serial.println(" m/s");

  VarioBeep(0.5, -0.5);


  delay(39);  // Adjust the delay based on your desired update frequency
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
      Beep_period = 400 - (vario * 80);
      toneAC(500 + (250 * vario),10);
      delay(abs(Beep_period)); // shit way to force Beep_period to be positive
      toneAC();
    } else if (vario < TreshDown) {
      Beep_period = 200;
      toneAC(200, 10);
      delay(400);
      toneAC();
    }
  }
}