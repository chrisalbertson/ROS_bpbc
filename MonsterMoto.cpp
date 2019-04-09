
#include "Arduino.h"
#include "MonsterMoto.h"




MonsterMoto::MonsterMoto()
{

  
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
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
  }
}

