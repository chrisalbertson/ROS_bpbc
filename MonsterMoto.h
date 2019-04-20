#ifndef MonsterMoto_h
#define MonsterMoto_h

#include <Arduino.h>
#include "pin_assign.h"


#define MAXPWM 255  // The AVR can only do 8-bits

#define BRAKEVCC 0
#define CW       1
#define CCW      2
#define BRAKEGND 3

#define CS_THRESHOLD 100

class MonsterMoto
{
  public:
    MonsterMoto(){}
    void setup();
    void motorOff(byte motor);
    void motorGo(byte motor, byte direct, int pwm);
   
  private:
    /*  VNH2SP30 pin definitions
        xxx[0] controls '1' outputs
        xxx[1] controls '2' outputs */
// The following are pin asignments for Arduino.  
// TODO:  Move this to pin_assign.h and use #if to compile for AVR or STM32 platform.
//    const byte inApin[2] = {7, 4}; // INA: Clockwise input
//    const byte inBpin[2] = {8, 9}; // INB: Counter-clockwise input
//    const byte pwmpin[2] = {5, 6}; // PWM input
//    const byte cspin[2]  = {2, 3}; // CS: Current sense ANALOG input
//    const byte enpin[2]  = {0, 1}; // EN: Status of switches output (Analog pin)


    const byte inApin[2] = {MM_CW1,  MM_CW2};  // INA: Clockwise input
    const byte inBpin[2] = {MM_CCW1, MM_CCW2}; // INB: Counter-clockwise input
    const byte pwmpin[2] = {MM_PWM1, MM_PWM2}; // PWM input
    const byte cspin[2]  = {MM_CS1,  MM_CS2};  // CS: Current sense ANALOG input
    const byte enpin[2]  = {MM_EN1,  MM_EN2};  // EN: Status of switches output (Analog pin)
    
//    int statpin = 13;  
};


#endif
