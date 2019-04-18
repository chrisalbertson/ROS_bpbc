



/*
    ROS-bpbc.ino
    ROS Blue Pill Base Controler

    Author Chris Albertson  albertson.chris@gmail.com
    Copyright April 2019

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros_lib.h>
#include "NUC_ros.h"

// This file is needed to fool the Arduino build system into lookning
// in .../Aduino/Libraries/ for other #include files.   The file ros_lib.h
// is empty except for a comment.
// Note the this is not needed if we use the system <ros.h> but is only
// needed if a local "ros.h" isused.
//#include <ros_lib.h>

// Include local version of ros.h, not the system version
//#include "ros.h" 



#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "EncoderBP.h"
#include "odometry.h"
#include "pin_assign.h"


// need for Blu-Pill" board
//#define LED_BUILTIN PC13
#define LED_ON  HIGH
#define LED_OFF LOW 
static bool ledState;

ros::NodeHandle  nh;

#define PID_PERIOD 100


EncoderBP encoderLeft( ENC1A, ENC1B);
EncoderBP encoderRight(ENC2A, ENC2B);


// The followont two funtions look pointless but are required by the Arduino
// system because ISPs appearenly can not be non-static members of a class.
void ispLeft() {
  encoderLeft.tick();
}
void ispRight() {
  encoderRight.tick();
}

// TODO:  meterPerTick needs to be computed from parameters in setup()
// meter per encoder tick is wheel circumfrence / encoder ticks per wheel revoution
const float meterPerTick = (0.13 * 3.1415) / (75.0 * 64.0);
const float base_width = 0.3;
const float PeriodSeconds = float(PID_PERIOD) / 1000.0;

long encoderLeftLastValue  = 0L;
long encoderRightLastValue = 0L;

Odometer odo(meterPerTick, base_width, PeriodSeconds);



#include "MonsterMoto.h"

MonsterMoto moto;
#define LEFT_MOTOR   0
#define RIGHT_MOTOR  1

#include <PID_v1.h>

// These global variables are used by the PID library.
// TODO Kp, Ki and Kd should be parameters
double leftSetpoint = 0.0;
double leftInput,  leftOutput;
double rightSetpoint = 0.0;
double rightInput, rightOutput;
double Kp = 2, Ki = 5, Kd = 1;

// Create one PID object for each motor.  The Input and output units
// will be "meters"
PID leftWheelPID( &leftInput,  &leftOutput,  &leftSetpoint,  Kp, Ki, Kd, DIRECT);
PID rightWheelPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

// An LCD is optional but make it easy to see what is happening and is
// the only way to show status if there is not ROS computer connected
#define HAVE_LCD 0
#if HAVE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif // HAVE_LCD

// Functions declaratons (silly C-language requirement)
void cmd_velCallback( const geometry_msgs::Twist& toggle_msg);
void broadcastTf();

// Subscribe to Twist messages on cmd_vel.
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCallback);


// millisecond to delay in main loop
#define LOOPDELAY 1

// This is an Arduino convention.  Place everything that needs to run just
// once in the setup() funtion.  The environment will call setup()
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Make sure the motors are stopped.
  // Call the driver directly and also set the PID setpoints.
  moto.motorOff(LEFT_MOTOR);
  moto.motorOff(RIGHT_MOTOR);
  leftSetpoint  = 0.0;
  rightSetpoint = 0.0;


  // Set up interrupt on encoder pins.   NOte the function ispLeft and ispRight
  // are required by the Arduino sysem because ISPs can not be non-static class
  // members.
  attachInterrupt(ENC1A, ispLeft,  CHANGE);
  attachInterrupt(ENC1B, ispLeft,  CHANGE);
  attachInterrupt(ENC2A, ispRight, CHANGE);
  attachInterrupt(ENC2B, ispRight, CHANGE);


  // Start PID controlers. All we need next is data
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

  // Connect to ROS computer and wait for connection
  nh.initNode();

  // Advertize odometry and transform
  odo.setupPubs(nh);

  // Subscribe to cmd_vel
  nh.subscribe(sub_cmd_vel);
  
  nh.loginfo("starting...");
}


// The PID period is independent from the loop() period.  This defines
// the time between updates to the motor speed.
//#define PID_PERIOD 100
unsigned long NextPIDMillis = 0;


// This loop() funtion is an arduino convnetion.  It is call by the environment
// inside a tight lop and runs forever or until the CPU is reset or powered off
void loop() {


  // The PID runs every PID_PERIOD milliseconds,  Check if it is time
  if (millis() >= NextPIDMillis) {

    // It is time to run, store the NEXT tine to run.
    NextPIDMillis = millis() + PID_PERIOD;

    // blink the LED to show we are alive
    toggleLED();

    // Read the encoders.  If some kind of error has caused a jump by
    // and unreasonable amout we do NOT want to use the encoder value.
    // It is better to skip a cycle thenuse invalid data
    long encLeft  = encoderLeft.getPos();
    long encRight = encoderRight.getPos();

    if (!encoderJump(encLeft,  encoderLeftLastValue,
                     encRight, encoderRightLastValue)) {
    
      // No encoderjump detected.  Do the "normal" cycle

      // Figure out how far we have gone then convert
      // this distance to velocity and send it to the PID controler.
      //
      // distLeft and distRight are a distance in meters that were traveled
      // from the  last time we read the encoders.  The values can be postive
      // or negative depending on the direction the wheel moved.
      //
      // leftInput and rightInput are the average velocity in meters per second
      // that were traveled durring the last sample period.
      float distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValue);
      float distRight = meterPerTick * float(encRight - encoderRightLastValue);
      leftInput  = distLeft  / PeriodSeconds;
      rightInput = distRight / PeriodSeconds;

      // Capture the time when encoders were read.
      ros::Time current_time = nh.now();

      // re-compute the motor drive power based on previous measured speed.
      // The PID control tries to keep the motor runing at a constant
      // setpoint speed.  Input and Output are global variables
      leftWheelPID.Compute();
      rightWheelPID.Compute();
      setMotorSpeeds();

      // Publish odometry
      odo.update_publish(current_time, distLeft, distRight);
    }

    // Save values for "next time".
    encoderLeftLastValue  = encLeft;
    encoderRightLastValue = encRight;
  }

  // handle any data movements across the serial interface
  nh.spinOnce();
}


// This funtion is called every time we receive a Twist message.
// We do not send the commanded speed to the wheels.  Rather we set
// thePID loops set point to the commanded sprrd.
void cmd_velCallback( const geometry_msgs::Twist& twist_msg) {

  // TODO:  This needs to be a parameter
  float width_robot = 0.3;

  // We only use two numbers from the Twist message.
  //    linear.x  is the forward speed in meters per second.
  //              (The "x" axis points forward.)
  //    angular.y is the rotation about the z or vertical
  //              axis in radians per second.
  //
  float vel_x   = twist_msg.linear.x;
  float vel_th  = twist_msg.angular.z;

  // Compute the wheel speeds in meters per second.
  double left_vel  =  vel_x - vel_th * width_robot / 2.0;
  double right_vel =  vel_x + vel_th * width_robot / 2.0;

  // Show the Twist message on the LCD.
  //displayStatus(&vel_x, &vel_th);

  // The PID works in unit of meters per second, so no
  // conversion is needed.  The motor speeds will be adjusted
  // when the PID controller run next.  The wheels will
  // run at this speed untill the next Twist message makes
  // the next adjustment to the PID's setpoint.
  leftSetpoint  = left_vel;
  rightSetpoint = right_vel;
}

void setMotorSpeeds() {
  // Set the controler based on calulation from PID
  byte direction;
  int  speed;

  speed  = int(0.5 + fabs(leftOutput));
  if (leftOutput >  0.001) {
    direction = CW;
  }
  else if (leftOutput < -0.001) {
    direction = CCW;
  }
  else {
    direction = BRAKEGND;
    speed = 0;
  }
  moto.motorGo(LEFT_MOTOR, direction, speed);

  speed  = int(0.5 + fabs(rightOutput));
  if (rightOutput >  0.001) {
    direction = CW;
  }
  else if (rightOutput < -0.001) {
    direction = CCW;
  }
  else {
    direction = BRAKEGND;
    speed = 0;
  }
  moto.motorGo(RIGHT_MOTOR, direction, speed);
}

#if LCD
void displayStatus(float *vel_x, float *vel_th) {
  // set the cursor (column, line)
  lcd.setCursor(0, 0);
  lcd.print("X");
  lcd.print(*vel_x);

  lcd.setCursor(0, 1);
  lcd.print("T");
  lcd.print(*vel_th);
}
#endif //LCD

bool encoderJump(long enc_left,  long last_enc_left,
                 long enc_right, long last_enc_right) {

  if ((abs(enc_left  - last_enc_left)  > 20000) ||
      (abs(enc_right - last_enc_right) > 20000)) {

    // TODO: Log this
    // "Ignoring encoder jump"
    return true;
  }
  return false;
}


void toggleLED() {
  if (ledState) {
    digitalWrite(LED_BUILTIN, LED_ON);   
  }
  else {
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
  ledState = !ledState;
}

/*  Used for debugging
 *   
void flashLED(int count) {
  digitalWrite(LED_BUILTIN, LED_OFF);
  delay(1000);
  for (int i=0; i<count; i++) {
    digitalWrite(LED_BUILTIN, LED_ON);
    delay(50);
    digitalWrite(LED_BUILTIN, LED_OFF);
    ledState = true;
    delay(200);
  }
}
*/
