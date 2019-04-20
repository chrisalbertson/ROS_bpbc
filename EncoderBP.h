
// EncoderBP.h
// Library for using rotary encoders with STM32 "Blue Pill" in Arduino environment.

#ifndef ENCODERBP_H
#define ENCODERBP_H

#include "Arduino.h"


class EncoderBP
{
  public:

    EncoderBP(int pin1, int pin2);

    void setup();

    // get the current position
    long  getPos() {
      return _position;
    }


    void setPos(long newPosition) {
      _position = newPosition;
    }

    void tick();

  private:
    int _pin1, _pin2;
    volatile byte _oldState;
    volatile long _position;
};

#endif /* ENCODERBP_H */
