#include <Wire.h>
#include <SensirionI2cScd4x.h>
#include <RTClib.h>
#include "RotaryEncoder.h"
#define SEG14
#include "AlphaDisplay.h"

//#define DEBUGSERIAL

#define VERSION             "1.000"

#define DISPLAY_ADDRESS     0x70
#define DISPLAY_DIGITS      4
#define DISPLAY_BRIGHTNESS  4

#define ENCA_PIN            8
#define ENCB_PIN            9
#define BUTTON_PIN          10
#define STATUS_LED          0

#define WAIT_DELAY          3000
#define HALFSEC_DELAY       500
#define FLASH_DELAY         250
#define END_DELAY           10000
#define ANIMATE_DELAY       100
#define COUNT_DELAY         250
#define SENSOR_DELAY        1000
#define VCC_DELAY           10000

#define SENSOR_ALTITUDE     50
#define SENSOR_TARGETCO2    400
#define SENSOR_CYCLES       60
#define CAL_MINUTES         3
#define CAL_SECONDS         0
#define TEMPOFFSET_MAX      99
#define VCC_LOWLIMIT        29

#define CO2LEVEL_800        800
#define CO2LEVEL_1500       1500

#define IND_MEDIUM          0x01
#define IND_HIGH            0x02
#define IND_NODATA          0x04
#define IND_VCCLOW          0x08

// make sure that we use the proper definition of NO_ERROR
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cScd4x sensor;

DateTime dt;

AlphaDisplay alpha;

RotaryEncoder *rotary = nullptr;

bool      sens, update, updateFailed, buttonPressed, bVcc, rotaryRight, rotaryLeft, colon, warnLed, calStart, calStop;
uint8_t   sysState, nextState, warnState, warnLevel, warnLedCycle, flashSequence, sensorState,
          sensorCycles, dispState, counter, tempOffset;
uint8_t   pinBitmaskButton, pinBitmaskEncA, pinBitmaskEncB, pinBitmaskLed, portId;
uint16_t  vcc, co2, oldCo2, hum, pos, alt;
int16_t   error, temp;
uint32_t  waitTime, halfSecTime, sensTime, endTime, flashTime, dispTime, animateTime, vccTime;

volatile uint8_t a0, b0;

enum { STATE_INIT = 1, STATE_SHOWCO2, STATE_SHOWTEMP, STATE_SHOWHUM, STATE_SHOWVCC, STATE_SETALT, STATE_SETTEMP, STATE_SETPRESSURE, STATE_CALINIT, STATE_CALWAIT,
       STATE_CALFINISHED, STATE_SELCO2, STATE_SELTEMP, STATE_SELHUM, STATE_SELVCC, STATE_SELCAL, STATE_WAIT, STATE_WAIT3SEC, STATE_SENSORERROR
     };
enum { DISPLAY_CO2 = 1, DISPLAY_TEMP, DISPLAY_HUM, DISPLAY_VCC, DISPLAY_TIME, DISPLAY_TEXT, DISPLAY_SETALT, DISPLAY_SETTEMP, DISPLAY_RAND, DISPLAY_BLANK };
enum { SENSOR_INIT = 1, SENSOR_WAIT, SENSOR_MEASURE, SENSOR_READ, SENSOR_CALSTART, SENSOR_CALWAIT, SENSOR_CALSTOP, SENSOR_CALFRC, SENSOR_NOTREADY };
enum { DISP_INIT = 1, DISP_IDLE, DISP_COUNT };
enum { WARN_INIT = 1, WARN_IDLE, WARN_FLASH };

PORT_t* const port[] = { &PORTA, &PORTB };

#ifdef DEBUGSERIAL
void PrintUint64(uint64_t& value) {
  Serial.print("0x");
  Serial.print((uint32_t)(value >> 32), HEX);
  Serial.println((uint32_t)(value & 0xFFFFFFFF), HEX);
}
#endif

//------------------------------------------------------------------------------------------

void setup() {

#ifdef DEBUGSERIAL
  Serial.begin(115200, SERIAL_TX_ONLY);
  delay(100);
  Serial.println("Init...");
#endif

  Wire.begin();

  alpha.init(DISPLAY_ADDRESS, DISPLAY_DIGITS, DISPLAY_BRIGHTNESS);
  alpha.print("INIT");
  delay(1000);

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  rotary = new RotaryEncoder(RotaryEncoder::LatchMode::FOUR3);

  // enable input pins with pullup (active low)
  pinBitmaskButton = digitalPinToBitMask(BUTTON_PIN);
  pinBitmaskEncA = digitalPinToBitMask(ENCA_PIN);
  pinBitmaskEncB = digitalPinToBitMask(ENCB_PIN);
  pinBitmaskLed = digitalPinToBitMask(STATUS_LED);

  // assume that all inputs use the same port
  portId = digitalPinToPort(BUTTON_PIN);

  port[portId]->DIRCLR = pinBitmaskEncA | pinBitmaskEncB | pinBitmaskButton;
  port[portId]->DIRSET = pinBitmaskLed;

  register8_t *pinCtrlReg;
  pinCtrlReg = (&(port[portId]->PIN0CTRL) + digitalPinToBitPosition(BUTTON_PIN));
  *pinCtrlReg = PORT_PULLUPEN_bm;
  pinCtrlReg = (&(port[portId]->PIN0CTRL) + digitalPinToBitPosition(ENCA_PIN));
  *pinCtrlReg = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  pinCtrlReg = (&(port[portId]->PIN0CTRL) + digitalPinToBitPosition(ENCB_PIN));
  *pinCtrlReg = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

  setLed(false);

  sensor.begin(Wire, SCD41_I2C_ADDR_62);
  delay(50);

  // Ensure sensor is in clean state
  error = sensor.wakeUp();
#ifdef DEBUGSERIAL
  if (error != NO_ERROR)
    Serial.println("Error trying to execute wakeUp()");
#endif

  error = sensor.stopPeriodicMeasurement();
#ifdef DEBUGSERIAL
  if (error != NO_ERROR)
    Serial.println("Sensor: Error trying to execute stopPeriodicMeasurement()");
#endif

  //error = sensor.performFactoryReset();
  //if (error != NO_ERROR)
  //  Serial.println("Sensor: Error trying to execute performFactoryReset()");

  // Read out information about the sensor
  uint64_t serialNumber = 0;
  error = sensor.getSerialNumber(serialNumber);
  if (error != NO_ERROR) {
#ifdef DEBUGSERIAL
    Serial.println("Sensor: Error trying to execute getSerialNumber()");
#endif
    sens = false;
  }
  else {
#ifdef DEBUGSERIAL
    Serial.print("Sensor: Serial number: ");
    PrintUint64(serialNumber);
#endif
    sens = true;
    alt = SENSOR_ALTITUDE;
    setAlt(alt);
  }

  error = sensor.setTemperatureOffset(0.0);
#ifdef DEBUGSERIAL
  if (error != NO_ERROR)
    Serial.println("Error trying to execute setTemperatureOffset()");
#endif

  // show software version
  alpha.print(VERSION);
  delay(2000);

  // inititialize ADC for voltage reading
  ADCinit();
  vcc = measureVoltage();

  sysState = STATE_INIT;
  sensorState = SENSOR_INIT;
  warnState = WARN_INIT;
  dispState = DISP_INIT;

  buttonPressed = false;
  update = false;
  updateFailed = false;
  bVcc = false;
  calStart = false;
  calStop = false;
  rotaryRight = rotaryLeft = false;
  warnLevel = 0;
  a0 = b0 = 0;
  pos = 0;
  co2 = oldCo2 = 0;
  temp = hum = 0;

  sensTime = flashTime = dispTime = animateTime = vccTime = millis();
}

//------------------------------------------------------------------------------------

void loop() {

  bool button;
  static bool stableState = false;
  static uint8_t confidenceCounter = 0;
  static uint8_t stableThreshold = 10;

  // read rotary encoder
  RotaryEncoder::Direction dir = rotary->getDirection();
  if (dir == RotaryEncoder::Direction::CLOCKWISE) rotaryRight = true;
  if (dir == RotaryEncoder::Direction::COUNTERCLOCKWISE) rotaryLeft = true;

  // read low-active button state
  button = (port[portId]->IN & pinBitmaskButton) ? false : true;

  // debounce
  if (button == stableState) confidenceCounter = 0;
  else {
    confidenceCounter++;
    if (confidenceCounter >= stableThreshold) {
      stableState = button;
      if (stableState == true) buttonPressed = true;
    }
  }

  // handle all states
  sysState = handleState(sysState);
  sensorState = handleSensor(sensorState);
  dispState = handleDisplay(dispState);
  warnState = handleWarning(warnState);

  // reset change flags
  rotaryRight = rotaryLeft = buttonPressed = false;

  delay(10);
}

//----------------------------------------------------------------------------------------------

// main state machine - handles all events
uint8_t handleState(uint8_t state) {
  uint32_t currentTime = millis();

  switch (state) {
    case STATE_INIT:
      if (update) return STATE_SHOWCO2;
      if (updateFailed || sens == false) return STATE_SENSORERROR;
      break;

    case STATE_SHOWCO2:
      if (update) update = false;
      if (buttonPressed) {
        // change to altitude set mode
        showDisplay(DISPLAY_SETALT, alt, NULL, 0, 0, false);
        endTime = currentTime;
        return STATE_SETALT;
      }
      if (rotaryRight) return STATE_SELTEMP;
      break;

    case STATE_SHOWTEMP:
      if (update) {
        showDisplay(DISPLAY_TEMP, temp, NULL, 0, 0, false);
        update = false;
      }
      if (buttonPressed) {
        // change to temperature correction mode
        tempOffset = getTempOff();
        showDisplay(DISPLAY_SETTEMP, tempOffset, NULL, 0, 0, false);
        endTime = currentTime;
        return STATE_SETTEMP;
      }
      if (rotaryRight) return STATE_SELHUM;
      if (rotaryLeft) return STATE_SELCO2;
      break;

    case STATE_SHOWHUM:
      if (update) {
        showDisplay(DISPLAY_HUM, hum, NULL, 0, 0, false);
        update = false;
      }
      if (rotaryRight) return STATE_SELVCC;
      if (rotaryLeft) return STATE_SELTEMP;
      break;

    case STATE_SHOWVCC:
      if (bVcc) {
        showDisplay(DISPLAY_VCC, vcc, NULL, 0, 0, false);
        bVcc = false;
      }
      if (rotaryRight) return STATE_SELCAL;
      if (rotaryLeft) return STATE_SELHUM;
      break;

    case STATE_SETALT:
      if (buttonPressed) {
        // save new altitude
        setAlt(alt);
        update = true;
        return STATE_SHOWCO2;
      }
      if (rotaryRight) {
        endTime = currentTime;
        alt++;
        showDisplay(DISPLAY_SETALT, alt, NULL, 0, 0, false);
      }
      if (rotaryLeft) {
        endTime = currentTime;
        if (alt) alt--;
        showDisplay(DISPLAY_SETALT, alt, NULL, 0, 0, false);
      }
      if (currentTime - endTime > END_DELAY) {
        // exit after idle time
        update = true;
        return STATE_SHOWCO2;
      }
      break;

    case STATE_SETTEMP:
      if (buttonPressed) {
        // save new temperature offset
        setTempOff(tempOffset);
        update = true;
        return STATE_SHOWTEMP;
      }
      if (rotaryRight) {
        endTime = currentTime;
        if (tempOffset < TEMPOFFSET_MAX) tempOffset++;
        showDisplay(DISPLAY_SETTEMP, tempOffset, NULL, 0, 0, false);
      }
      if (rotaryLeft) {
        endTime = currentTime;
        if (tempOffset) tempOffset--;
        showDisplay(DISPLAY_SETTEMP, tempOffset, NULL, 0, 0, false);
      }
      if (currentTime - endTime > END_DELAY) {
        // exit after idle time
        update = true;
        return STATE_SHOWTEMP;
      }
      break;

    case STATE_SETPRESSURE:
      break;

    case STATE_CALINIT:
      dt = DateTime(2026, 1, 1, 0, CAL_MINUTES, CAL_SECONDS);
      colon = true;
      halfSecTime = millis();
      calStart = true;
      calStop = false;
      setLed (true);
      return STATE_CALWAIT;

    case STATE_CALWAIT:
      if (buttonPressed) {
        // abort calibration wait time
        showDisplay(DISPLAY_CO2, co2, NULL, 0, 0, false);
        calStop = true;
        setLed(false);
        return STATE_SHOWCO2;
      }
      // show time with flashing colon
      if (currentTime - halfSecTime > HALFSEC_DELAY) {
        halfSecTime = currentTime;
        // exit condition
        if (dt.minute() == 0 && dt.second() == 0) return STATE_CALFINISHED;

        showDisplay(DISPLAY_TIME, 0, NULL, dt.minute(), dt.second(), colon);
        if (colon == false) dt = dt - (TimeSpan)1;
        colon = !colon;
      }
      break;

    case STATE_CALFINISHED:
      calStop = true;
      showDisplay(DISPLAY_TEXT, 0, "DONE", 0, 0, false);
      setLed(false);
      return STATE_WAIT;

    case STATE_SELCO2:
      showDisplay(DISPLAY_TEXT, 0, "CO2 ", 0, 0, false);
      nextState = STATE_SHOWCO2;
      waitTime = currentTime;
      return STATE_WAIT3SEC;

    case STATE_SELTEMP:
      showDisplay(DISPLAY_TEXT, 0, "TEMP", 0, 0, false);
      nextState = STATE_SHOWTEMP;
      waitTime = currentTime;
      return STATE_WAIT3SEC;

    case STATE_SELHUM:
      showDisplay(DISPLAY_TEXT, 0, "HUM ", 0, 0, false);
      nextState = STATE_SHOWHUM;
      waitTime = currentTime;
      return STATE_WAIT3SEC;

    case STATE_SELVCC:
      showDisplay(DISPLAY_TEXT, 0, "VCC ", 0, 0, false);
      bVcc = true;
      nextState = STATE_SHOWVCC;
      waitTime = currentTime;
      return STATE_WAIT3SEC;

    case STATE_SELCAL:
      showDisplay(DISPLAY_TEXT, 0, "CAL ", 0, 0, false);
      nextState = STATE_CALINIT;
      waitTime = currentTime;
      return STATE_WAIT3SEC;

    case STATE_WAIT:
      // calibration finished
      if (buttonPressed) {
        co2 = SENSOR_TARGETCO2;
        showDisplay (DISPLAY_CO2, co2, NULL, 0, 0, false);
        return STATE_SHOWCO2;
      }
      break;

    case STATE_WAIT3SEC:
      if (currentTime - waitTime > WAIT_DELAY) {
        update = true;
        return nextState;
      }
      if (rotaryRight) {
        if (nextState == STATE_SHOWCO2) return STATE_SELTEMP;
        if (nextState == STATE_SHOWTEMP) return STATE_SELHUM;
        if (nextState == STATE_SHOWHUM) return STATE_SELVCC;
        if (nextState == STATE_SHOWVCC) return STATE_SELCAL;
      }
      if (rotaryLeft) {
        if (nextState == STATE_SHOWTEMP) return STATE_SELCO2;
        if (nextState == STATE_SHOWHUM) return STATE_SELTEMP;
        if (nextState == STATE_SHOWVCC) return STATE_SELHUM;
        if (nextState == STATE_CALINIT) return STATE_SELVCC;
      }
      break;

    case STATE_SENSORERROR:
      showDisplay(DISPLAY_TEXT, 0, "SENS", 0, 0, false);
      delay(1000);
      return STATE_INIT;
  }
  return state;
}

//----------------------------------------------------------------------------------------------

uint8_t handleSensor(uint8_t state) {
  bool dataReady;
  uint32_t currentTime = millis();

  if (!sens) return state;
  if (currentTime - sensTime > SENSOR_DELAY) {
    sensTime = currentTime;

    switch (state) {
      case SENSOR_INIT:
#ifdef DEBUGSERIAL
        Serial.println("Sensor: Init...");
#endif
        sensor.startLowPowerPeriodicMeasurement();
        sensorCycles = SENSOR_CYCLES;
        return SENSOR_WAIT;

      case SENSOR_WAIT:
        if (calStart) {
          calStart = false;
          return SENSOR_CALSTART;
        }
        sensor.getDataReadyStatus(dataReady);
        if (dataReady) return SENSOR_READ;

        if (--sensorCycles == 0) return SENSOR_NOTREADY;
        break;

      case SENSOR_MEASURE:
#ifdef DEBUGSERIAL
        Serial.println("Sensor: MeasureSingleShot...");
#endif
        sensor.measureSingleShot();
        return SENSOR_READ;

      case SENSOR_READ:
#ifdef DEBUGSERIAL
        Serial.println("Sensor: Reading...");
#endif
        if (readSensor()) {
          update = true;
          updateFailed = false;
          warnLevel &= ~IND_NODATA;

          // set warning levels for indicator LED
          if (co2 <= CO2LEVEL_800) warnLevel &= ~(IND_MEDIUM | IND_HIGH);
          if (co2 > CO2LEVEL_800) warnLevel |= IND_MEDIUM;
          if (co2 > CO2LEVEL_1500) warnLevel |= IND_HIGH;
        }
        else {
#ifdef DEBUGSERIAL
          Serial.println("Sensor: Reading failed!");
#endif
          updateFailed = true;
          warnLevel |= IND_NODATA;
        }
        sensorCycles = SENSOR_CYCLES;
        return SENSOR_WAIT;

      case SENSOR_CALSTART:
        // switch to normal periodic measurement
#ifdef DEBUGSERIAL
        Serial.println("Sensor: Start calibration cycle");
#endif
        return SENSOR_CALWAIT;

      case SENSOR_CALWAIT:
        if (calStop) return SENSOR_CALSTOP;
        break;

      case SENSOR_CALSTOP:
#ifdef DEBUGSERIAL
        Serial.println("Sensor: Stopping calibration cycle");
#endif
        return SENSOR_CALFRC;

      case SENSOR_CALFRC:
        uint16_t frcCorrection;
#ifdef DEBUGSERIAL
        Serial.println("Sensor: Calibrate...");
#endif
        sensor.stopPeriodicMeasurement();
        delay(500);
        sensor.performForcedRecalibration(SENSOR_TARGETCO2, frcCorrection);
        delay(400);
        return SENSOR_INIT;

      case SENSOR_NOTREADY:
        updateFailed = true;
        co2 = 0;
        return SENSOR_INIT;
    }
  }
  return state;
}

//----------------------------------------------------------------------------------------------

uint8_t handleDisplay(uint8_t state) {
  uint32_t currentTime = millis();
  static uint8_t oldState = STATE_INIT;

  // display animation and counter effect
  if (currentTime - dispTime > ANIMATE_DELAY) {
    dispTime = currentTime;

    switch (state) {
      case DISP_INIT:
        if (sysState == STATE_INIT) showDisplay (DISPLAY_RAND, 0, NULL, 0, 0, false);
        else {
          showDisplay (DISPLAY_BLANK, 0, NULL, 0, 0, false);
          return DISP_IDLE;
        }
        break;

      case DISP_IDLE:
        if (oldState != sysState) {
          oldState = sysState;
          if (sysState == STATE_SHOWCO2) showDisplay (DISPLAY_CO2, co2, NULL, 0, 0, false);
          break;
        }
        // initialize counter if value changed
        if (sysState == STATE_SHOWCO2) {
          if (oldCo2 != co2) {
            counter = 0;
            return DISP_COUNT;
          }
        }
        break;

      case DISP_COUNT:
        if (sysState == STATE_SHOWCO2) {
          if (counter == 0) {
            int diff = oldCo2 - co2;
            diff = diff < 0 ? -diff : diff;
            if (diff < 50) {
              if (oldCo2 < co2) oldCo2++;
              if (oldCo2 > co2) oldCo2--;
            }
            else oldCo2 = co2;

            showDisplay (DISPLAY_CO2, oldCo2, NULL, 0, 0, false);

            if (oldCo2 == co2) return DISP_IDLE;
          }
          if (counter++) counter = 0;
        }
        else return DISP_IDLE;
        break;
    }
  }
  return state;
}

//----------------------------------------------------------------------------------------------

uint8_t handleWarning(uint8_t state) {
  uint32_t currentTime = millis();

  // peridically read Vcc voltage
  if (currentTime - vccTime > VCC_DELAY) {
    vccTime = currentTime;
    vcc = measureVoltage();
    bVcc = true;

    if (vcc < VCC_LOWLIMIT) warnLevel |= IND_VCCLOW;
    else warnLevel &= ~IND_VCCLOW;
  }
  // warning LED sequence
  if (currentTime - flashTime > FLASH_DELAY) {
    flashTime = currentTime;

    switch (state) {
      case WARN_INIT:
        warnLedCycle = 0;
        return WARN_IDLE;

      case WARN_IDLE:
        if (warnLedCycle) warnLedCycle--;
        if (warnLedCycle) break;

        if (sysState == STATE_SHOWCO2 || sysState == STATE_SHOWTEMP || sysState == STATE_SHOWHUM || sysState == STATE_SHOWVCC) {
          warnLed = false;
          setLed(warnLed);

          if (warnLevel) {
            if (warnLevel & IND_MEDIUM) flashSequence = 1;
            if (warnLevel & IND_HIGH) flashSequence = 2;
            if (warnLevel & IND_NODATA) flashSequence = 3;
            if (warnLevel & IND_VCCLOW) flashSequence = 4;
            warnLedCycle = 20;
            warnLed = true;
            return WARN_FLASH;
          }
        }
        break;

      case WARN_FLASH:
        if (!warnLed) {
          flashSequence--;
          if (flashSequence == 0) {
            setLed(false);
            return WARN_IDLE;
          }
        }
        setLed(warnLed);
        warnLed = !warnLed;
        break;
    }
  }
  return state;
}

//----------------------------------------------------------------------------------------------

void showDisplay(uint8_t type, int16_t val, const char *text, uint8_t min, uint8_t sec, bool dot) {
  char buf[DISPLAY_DIGITS + 1];

  switch (type) {
    case DISPLAY_CO2:
      if (val == 0) sprintf(buf, "C---");
      if (val && val < 1000) sprintf(buf, "C%u", val);
      else sprintf(buf, "%u", val);
      break;

    case DISPLAY_TEMP:
      // show temperature in a valid range only
      uint16_t temp;
      if (val >= -99 && val <= 999) {
        temp = (val < 0 ? -val : val);

        if (temp >= 100) buf[0] = '0' + (temp / 100) % 10;
        else buf[0] = ' ';
        if (val < 0) buf[0] = '-';

        buf[1] = ('0' + (temp / 10) % 10) | DOT;
        buf[2] = '0' + temp % 10;
        buf[3] = 'C';
      }
      break;

    case DISPLAY_HUM:
      if (val >= 0 && val < 100) sprintf(buf, " %02u%%", val);
      break;

    case DISPLAY_VCC:
      // check for valid votage range
      if (val >= 1800 && val <= 6000) {
        buf[0] = ' ';
        buf[1] = ('0' + (val / 1000)) | DOT;
        buf[2] = '0' + ((val / 100) % 10);
        buf[3] = 'V';
      }
      break;

    case DISPLAY_TEXT:
      sprintf(buf, "%s", text);
      break;

    case DISPLAY_TIME:
      // show time
      buf[0] = ' ';
      buf[1] = ('0' + min % 10) | (dot ? DOT : 0);
      buf[2] = '0' + sec / 10;
      buf[3] = '0' + sec % 10;
      break;

    case DISPLAY_SETALT:
      sprintf(buf, "A%03u", val);
      break;

    case DISPLAY_SETTEMP:
      // show temperature offset
      buf[0] = 'T';
      buf[1] = (val >= 0 ? '+' : '-');
      buf[2] = ('0' + (val / 10) % 10) | DOT;
      buf[3] = '0' + (val % 10);
      break;

    case DISPLAY_RAND:
      uint8_t i;
      uint16_t raw[DISPLAY_DIGITS];
      for (i = 0; i < DISPLAY_DIGITS; i++) {
        raw[i] = 1 << random(0, 14);
        raw[i] |= 1 << random(0, 14);
      }
      alpha.printraw(raw);
      return;

    case DISPLAY_BLANK:
      alpha.clear();
      return;
  }
  buf[DISPLAY_DIGITS] = 0;
  alpha.print(buf);
}

void setLed(bool led) {
  if (led) port[portId]->OUTSET = pinBitmaskLed; else port[portId]->OUTCLR = pinBitmaskLed;
}

void setAlt(uint16_t altitude) {
  sensor.setSensorAltitude(altitude);
}

uint8_t getTempOff(void) {
  float tempOff;

  sensor.stopPeriodicMeasurement();
  delay(50);
  sensor.getTemperatureOffset(tempOff);
  delay(50);
  sensor.startLowPowerPeriodicMeasurement();

#ifdef DEBUGSERIAL
  char buf[40];
  sprintf(buf, "getTempOffset: %u", (uint16_t)((tempOff * 10.0) + 0.5)));
  Serial.println(buf);
#endif
  return (uint8_t)((tempOff * 10.0) + 0.5);
}

void setTempOff(uint8_t tempOff) {
  sensor.stopPeriodicMeasurement();
  delay(50);
  sensor.setTemperatureOffset((float)((float)tempOff / 10.0));
  //sensor.persistSettings();
  delay(50);
  sensor.startLowPowerPeriodicMeasurement();
}

bool readSensor(void) {
  bool dataReady = false;
  uint16_t co2Concentration = 0;
  float temperature = 0.0;
  float relativeHumidity = 0.0;

  error = sensor.getDataReadyStatus(dataReady);
  if (error != NO_ERROR) {
#ifdef DEBUGSERIAL
    Serial.println("Sensor: Error trying to execute getDataReadyStatus(): ");
#endif
    return false;
  }
  if (!dataReady) {
#ifdef DEBUGSERIAL
    Serial.println("Sensor: Data not Ready!");
#endif
    return false;
  }

  error = sensor.readMeasurement(co2Concentration, temperature, relativeHumidity);
  if (error != NO_ERROR) {
#ifdef DEBUGSERIAL
    Serial.println("Sensor: Error trying to execute readMeasurement(): ");
#endif
    return false;
  }

#ifdef DEBUGSERIAL
  // Print results in physical units.
  Serial.print("Sensor: CO2 concentration [ppm]: ");
  Serial.print(co2Concentration);
  Serial.println();
  Serial.print("Sensor: Temperature [°C]: ");
  Serial.print(temperature);
  Serial.println();
  Serial.print("Sensor: Relative Humidity [RH]: ");
  Serial.print(relativeHumidity);
  Serial.println();
#endif
  co2 = (int16_t)co2Concentration;
  temp = (int16_t)((temperature * 10.0) + 0.5);
  hum = (int16_t)relativeHumidity;

  warnLevel = 0;
  if (co2 > CO2LEVEL_800) warnLevel = IND_MEDIUM;
  if (co2 > CO2LEVEL_1500) warnLevel = IND_HIGH;

  return true;
}

void ADCinit(void) {
  VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc | ADC_SAMPCAP_bm;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
}

uint16_t measureVoltage(void) {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  uint16_t accumulatedVal = ADC0.RES;
  if (accumulatedVal == 0) return 0;
  return (uint16_t)(72019200UL / accumulatedVal);
}

// ------------------------------- ISR Rotary --------------------------------

ISR(PORTA_PORT_vect) {
  uint8_t in = PORTA.IN;
  uint8_t a = (in & pinBitmaskEncA) ? 1 : 0;
  uint8_t b = (in & pinBitmaskEncB) ? 1 : 0;

  PORTA.INTFLAGS = 0xFF;

  // handle rotary encoder changes
  rotary->tick(a, b);
}
