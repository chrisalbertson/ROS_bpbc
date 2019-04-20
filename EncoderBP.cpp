
#include <Arduino.h>
#include "EncoderBP.h"

// Using "static" limits the scope to just this *.cpp file
static const int8_t dirTable[] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0  };

 EncoderBP::EncoderBP(int pin1, int pin2) {
  
  _pin1 = pin1;
  _pin2 = pin2;
  _position = 0;
}

// Need this because of silly STM32 Arduino limitataion there
// pinMode() can not be called from a global constuctor
void EncoderBP::setup() {
  
  // Setup the input pins
  pinMode(_pin1, INPUT_PULLUP);
  pinMode(_pin2, INPUT_PULLUP);

  // It actualy takes a few microseconds for the pullups to settle
  // (It is OK to do a delay here as this constructor is never called
  // from inside a loop.)
  delay(1);

  int val1 = digitalRead(_pin1);
  int val2 = digitalRead(_pin2);
  
  _oldState = val1 | (val2 << 1);

  _position = 0;
}


void EncoderBP::tick()
{
  int val1 = digitalRead(_pin1);
  int val2 = digitalRead(_pin2);
  
  byte curState = val1 | (val2 << 1);

  if (_oldState != curState) {
    _position += dirTable[curState | (_oldState<<2)];
    
    _oldState = curState;
  }
}
