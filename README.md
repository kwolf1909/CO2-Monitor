This project realizes a CO2-monitor based on Sensirion-sensor SCD41.<br>
The hardware-components are: MCU ATtiny1614 (16 KB flash), SCD41, Adafruit alphanumeric 4-digit display, PEC11-based rotary encoder.<br>
It shows the current CO2-value, the temperature or humidity.<br>
The user interface is rotary-knob based, to select between display-modes CO2, TEMP, HUM, VCC and CAL-Mode.<br><br>
-In CO2-Mode, pushing the button enters altitide set mode.<br>
-In TEMP-Mode, pushing the button enters temperature calibration mode.<br>
-In HUM-Mode, the humidity is displayed.<br>
-In VCC-Mode, the supply-voltage is displayed.<br>
-Selecting CAL enters manual CO2 calibration-mode. This requires putting the sensor to outside air (~400 PPM) for 3 minutes.<br><br>
A seperate LED indicates several warning-levels by blink-signals.<br>
1 blink: CO2-level above 800PPM, 2 blinks: CO2-level above 1500PPM, 3 blinks: battery low,<br>
4 blinks: No sensor data<br><br>
The software should be compatible with all modern AVRs like ATtiny 0/1/2-series, ATmega 0-series (Arduino Nano Every) and AVR Dx-series.<br>
External libraries used: SensirionI2cScd4x, RTClib<br>
Compiles in Arduino IDE using MegaTinyCore, MegaAVR-core, DxCore - depending on MCU used.<br>
