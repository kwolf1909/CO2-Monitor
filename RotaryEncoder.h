// -----
// RotaryEncoder.h - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
//
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
//
// This work is licensed under a BSD 3-Clause style license,
// https://www.mathertel.de/License.aspx.
//
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 16.06.2019 pin initialization using INPUT_PULLUP
// 10.11.2020 Added the ability to obtain the encoder RPM
// 29.01.2021 Options for using rotary encoders with 2 state changes per latch.
// 06.06.2024 Implementation of tick() with passing the input values for more performant implementations.
// 21.02.2025 Documentation and Constructor without hardware initialization added.
// -----

class RotaryEncoder {
public:
  enum class Direction {
    NOROTATION = 0,
    CLOCKWISE = 1,
    COUNTERCLOCKWISE = -1
  };

  enum class LatchMode {
    FOUR3 = 1,  // 4 steps, Latch at position 3 only (compatible to older versions)
    FOUR0 = 2,  // 4 steps, Latch at position 0 (reverse wirings)
    TWO03 = 3   // 2 steps, Latch at position 0 and 3
  };

  // Constructor that initializes the RotaryEncoder without hardware setup.
  RotaryEncoder(LatchMode mode = LatchMode::FOUR0);

  // retrieve the current position
  long getPosition();

  // simple retrieve of the direction the knob was rotated last time. 0 = No rotation, 1 = Clockwise, -1 = Counter Clockwise
  Direction getDirection();

  // adjust the current position
  void setPosition(long newPosition);

  // Use this tick variant when a faster method than digitalRead is available and provide the values directly.
  // The 2 pins provided in the class creation are ignored.
  void tick(int sig1, int sig2);

  // Returns the time in milliseconds between the current observed
  unsigned long getMillisBetweenRotations() const;

private:
  LatchMode _mode;  // Latch mode from initialization

  volatile int8_t _oldState;

  volatile long _position;         // Internal position (4 times _positionExt)
  volatile long _positionExt;      // External position
  volatile long _positionExtPrev;  // External position (used only for direction checking)

  unsigned long _positionExtTime;      // The time the last position change was detected.
  unsigned long _positionExtTimePrev;  // The time the previous position change was detected.
};

#define LATCH0 0  // input state at position 0
#define LATCH3 3  // input state at position 3

// The array holds the values �1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down

// ----- Initialization and Default Values -----

/**
 * @brief Default constructor that initializes the RotaryEncoder with non-hardware specific setup.
 *
 * This constructor creates a RotaryEncoder instance with only software initialization.
 * No pins are configured or reserved. This is useful for scenarios where:
 * - Pins will be managed externally
 * - Hardware-specific tick() will be called with explicit pin values
 * - Deferred or dynamic pin configuration is needed
 *
 * @param mode The latch mode defining the encoder sensitivity.
 *   See RotaryEncoder.h for details on the available modes.
 *
 * Initialization sets:
 * - Encoder mode
 * - Position counter to 0
 * - Internal state variables to initial values (no motion detected)
 * - Timestamp tracking for rotation speed calculation
 *
 * @note To use this constructor effectively, call tick(sig1, sig2) with explicit pin values
 *       in your main loop, or use the two-parameter constructor if you need automatic pin handling.
 *
 * @see RotaryEncoder(int pin1, int pin2, LatchMode mode) for hardware-managed pin setup
 * @see tick(int sig1, int sig2) for software-managed pin input
 */
RotaryEncoder::RotaryEncoder(LatchMode mode) {
  _mode = mode;

  // start with position 0;
  _position = 0;
  _oldState = 0;
  _positionExtPrev = _positionExt = 0;
  _positionExtTimePrev = _positionExtTime = millis();
}  // RotaryEncoder()

long RotaryEncoder::getPosition() {
  return _positionExt;
}  // getPosition()

RotaryEncoder::Direction RotaryEncoder::getDirection() {
  RotaryEncoder::Direction ret = Direction::NOROTATION;

  if (_positionExtPrev > _positionExt) {
    ret = Direction::COUNTERCLOCKWISE;
    _positionExtPrev = _positionExt;
  } else if (_positionExtPrev < _positionExt) {
    ret = Direction::CLOCKWISE;
    _positionExtPrev = _positionExt;
  } else {
    ret = Direction::NOROTATION;
    _positionExtPrev = _positionExt;
  }

  return ret;
}

void RotaryEncoder::setPosition(long newPosition) {
  switch (_mode) {
    case LatchMode::FOUR3:
    case LatchMode::FOUR0:
      // only adjust the external part of the position.
      _position = ((newPosition << 2) | (_position & 0x03L));
      _positionExt = newPosition;
      _positionExtPrev = newPosition;
      break;

    case LatchMode::TWO03:
      // only adjust the external part of the position.
      _position = ((newPosition << 1) | (_position & 0x01L));
      _positionExt = newPosition;
      _positionExtPrev = newPosition;
      break;
  }  // switch

}  // setPosition()

// When a faster method than digitalRead is available you can _tick with the 2 values directly.
void RotaryEncoder::tick(int sig1, int sig2) {
  unsigned long now = millis();
  int8_t thisState = sig1 | (sig2 << 1);

  if (_oldState != thisState) {
    _position += KNOBDIR[thisState | (_oldState << 2)];
    _oldState = thisState;

    switch (_mode) {
      case LatchMode::FOUR3:
        if (thisState == LATCH3) {
          // The hardware has 4 steps with a latch on the input state 3
          _positionExt = _position >> 2;
          _positionExtTimePrev = _positionExtTime;
          _positionExtTime = now;
        }
        break;

      case LatchMode::FOUR0:
        if (thisState == LATCH0) {
          // The hardware has 4 steps with a latch on the input state 0
          _positionExt = _position >> 2;
          _positionExtTimePrev = _positionExtTime;
          _positionExtTime = now;
        }
        break;

      case LatchMode::TWO03:
        if ((thisState == LATCH0) || (thisState == LATCH3)) {
          // The hardware has 2 steps with a latch on the input state 0 and 3
          _positionExt = _position >> 1;
          _positionExtTimePrev = _positionExtTime;
          _positionExtTime = now;
        }
        break;
    }  // switch
  }  // if
}  // tick()

unsigned long RotaryEncoder::getMillisBetweenRotations() const {
  return (_positionExtTime - _positionExtTimePrev);
}

// End
