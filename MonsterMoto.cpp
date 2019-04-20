
#include <Arduino.h>
#include "MonsterMoto.h"



// This shoud go in constructor but there is a silly limitation with
// Arduino and calling pinMode in a constuctor.  See the link below
// http://wiki.stm32duino.com/index.php?title=API#Important_information_about_global_constructor_methods
void MonsterMoto::setup()
{
  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
    pinMode(enpin[i],  OUTPUT);
  }
  // Initialize braked and enabled
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    digitalWrite(enpin[i],  HIGH);
  }
}


void MonsterMoto::motorOff(byte motor)
{
  // Initialize braked
  for (byte i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between 0 and 255, higher the number, the faster
 */
void MonsterMoto::motorGo(byte motor, byte direct, int pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1){
        digitalWrite(inApin[motor], HIGH);
      }
      else {
        digitalWrite(inApin[motor], LOW);
      }
      
      // Set inB[motor]
      if ((direct==0)||(direct==2)) {
        digitalWrite(inBpin[motor], HIGH);
      }
      else {
        digitalWrite(inBpin[motor], LOW);
      }
      
      analogWrite(pwmpin[motor], pwm);
    }
    digitalWrite(enpin[motor],  HIGH);
  }
}
