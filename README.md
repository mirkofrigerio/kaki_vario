# Kaki Vario - a wallet friendly paragliding variometer
Repository containing the Arduino code behind the Kaki paragliding vario.

## References:
This project is based on several similar projects found online:
    -  XCTRACK_vario -> excellent instructions on how to build a bluetooth vario that connects to XC Track. https://gitlab.com/dvarrel/XCTRACK_vario 
    -  Variometer for Paragliding by laisch -> not on github but https://www.instructables.com/Variometer-for-Paragliding/ excellent result, hoping to achieve something similar but with Bluetooth. Note his code is too big

## Product
The Kaki vario should be a small, lightweight and affordable vario. We are building it out of Arduino components, as they are easy and cheap to come across.

## Components:
- Arduino Nano
- BMP280 or MS5611 pressure sensors
- HC-05 bluetooth module
- buzzer
- push button
- 126x64 OLED screen
- on-off switch
- battery and 1s charging module

## Functions:
Most importantly, the vario should be good at working as an external barometer sensor for XC track. XC track will use it to determine the climb rate through more complex averaging and calculation, and using phone accellerometer and other phone sensors.
Secondly, the vario should display battery, climb rate, altitude (impossible to get right without adding the daily pressure at sea level value), temperature and active functions (bt/audio).
Thirdly, the vario should serve as a stand-alone audio vario. However we might want to switch this function on and off, such that when bt is on and we are using XC Track the vario is silent. 


