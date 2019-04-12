
#include "Arduino.h"
#include "EncoderBP.h"


const int8_t dirTable[] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0  };

 EncoderBP::EncoderBP(int pin1, int pin2) {
  
  _pin1 = pin1;
  _pin2 = pin2;
  
  // Setup the input pins
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);

  // It actualy takes a few microseconds for the pullups to settle
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
