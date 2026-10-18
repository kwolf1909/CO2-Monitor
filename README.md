This project realizes a CO2-monitor based on Sensirion-sensor SCD41.<br>
The hardware-components are: MCU ATtiny1614 (16 KB flash), SCD41, Adafruit alphanumeric 4-digit display, PEC11-based rotary encoder.<br>
It shows the current CO2-value, the temperature or humidity.<br>
The user interface is rotary-knob based, to select between display-modes CO2, TEMP, HUM, VCC and CAL-Mode.<br><br>
-In CO2-Mode, pushing the button enters altitide set mode.<br>
-In Temp-Mode, pushing the button enters temperature calibration mode.<br>
-In VCC-Mode, the supply-voltage is displayed.<br>
-Selecting CAL enters the CO2 calibration-mode. This requires putting the sensor to outside air with ~400 PPM for 3 minutes.<br><br>
The software should be compatible with all modern AVRs like ATtiny 0/1/2-series, ATmega 0-series (Arduino Nano Every) and AVR Dx-series.<br>
External libraries used: SensirionI2cScd4x, RTClib<br>
Compiles in Arduino IDE using MegaTinyCore, MegaAVR-core, DxCore - depending on MCU used.
