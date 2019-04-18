#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

// This are ther pins used the STM Nucleo STM32F4xx boards and the Monster Motor Sheild.

// For Motor 1           UNO  Description
#define MM_EN1  PA0   // A0   Enable
#define MM_CS1  PA4   // A2   Current Sense            <<<< MUST LIMIT VOLTS TO 3V3
#define MM_CW1  PA8   // D7   CW H-Bridge Control
#define MM_CCW1 PA9   // D8   CCW H-Bridge Control
#define MM_PWM1 PB4   // D5   PWM Speed Control

// For Motor 2           UNO  Description
#define MM_EN2  PA1   // A1   Enable
#define MM_CS2  PB0   // A3   Current Sense            <<<< MUST LIMIT VOLTS TO 3V3
#define MM_CW2  PB5   // D4   CW H-Bridge Control
#define MM_CCW2 PC7   // D9   CCW H-Bridge Control
#define MM_PWM2 PB10  // D6   PWM Speed Control

// Encoders              CN7  Description
#define ENC1A   PC10  // 1    Motor 1 Phase A
#define ENC1B   PC12  // 3    Motor 1 Phase B
#define ENC2A   PA13  // 13   Motor 2 Phase A
#define ENC2B   PA14  // 15   Motor 2 Phase B
//
//
// Pin locations on CN7.  Note there is a "dot" printed nere pin #1
// Thepins loaction loke like the diagram below.  (The end of the Nucleo
// with the USB conetion is up.)
//    PC10    1 .
//    PC11    3 .
//            . .
//            . .
//            . .
//            . .
//    PA13   13 .
//    PA14   14 .
//    +5      . 18
//    GND     . 20
//            . .
//  
#endif //PIN_ASSIGN_H
  
            
