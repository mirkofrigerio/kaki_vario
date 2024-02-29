#define SWITCH_PIN 3   // Change this to the actual pin number you have connected the switch to

/*
pinout

First: controls On - OFF

I   I   I    I
Vin Vin B+

B- ---> Nano GND

Second: controls mode

I  I  I     I
  GND D

Remember INPUT_PULLUP when using pinMode



*/
void setup() {
  Serial.begin(9600);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  // Check the switch position and update the mode accordingly
  int switchState = digitalRead(SWITCH_PIN);
  Serial.println(switchState);
  // if (switchState == HIGH) {
  //   // Bluetooth mode code
  //   Serial.println("AUDIO on!");
  // }

  // else {
  //   // Audio mode code
  //   Serial.println("BT on!");
    
  // }

  delay(200);
}
  