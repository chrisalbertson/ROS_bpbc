
// HOW TO COMPILE With Arduino IDE
// 1. Must use STM32 "Core"
// 2. Board: is "Nucleo64"
// 3. Board Part Number: "Nucleo F4xxRE"
// 4. U(S)ART  "Enabled (generic serial)"
// 5. USB Support: "None"


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

// This file is needed to fool the Arduino build system into lookning
// in .../Aduino/Libraries/ for other #include files.   The file ros_lib.h
// is empty except for a comment.
// Note the this is not needed if we use the system <ros.h> but is only
// needed if a local "ros.h" isused.
//#include <ros_lib.h>

// Include local version of ros.h, not the system version
//#include "ros.h"
#include <ros_lib.h>
#include "NUC_ros.h"


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

#define ODO_PERIOD 200  // Millis between /tf and /odom publication
#define PID_PERIOD  50  // Millis between each PID calculation


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
//const float meterPerTick = (0.13 * 3.1415) / (75.0 * 64.0); // Thumper

const float meterPerTick = 0.0003828125;  // Woodie
const float base_width   = 0.195;         // Woodie

long encoderLeftLastValue  = 0L;
long encoderRightLastValue = 0L;

Odometer odo(meterPerTick, base_width);



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

//double Kp = 60, Ki = 100, Kd = 1;  //  Usable but not respncive enough
double Kp = 40.0, Ki = 0.0, Kd = 0.0;

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
// DEBUG FOLLOWS
void MotorTest();

// Subscribe to Twist messages on cmd_vel.
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCallback);

// This is an Arduino convention.  Place everything that needs to run just
// once in the setup() funtion.  The environment will call setup()
void setup() {

  // Set the PWM frequency so the sound is not so anoyying
  // TODO remove the comment fromthe below after version 1.6.0 of the STM32 Core is released.
  //      This funtion is not available in 1.5.0
  // analogWriteFrequency(6000);

  // This should be in constuctor but there is a limtation.  See this link
  // http://wiki.stm32duino.com/indx.php?title=API#Important_information_about_global_constructor_methods
  moto.setup();

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
  encoderLeft.setup();
  encoderRight.setup();
  attachInterrupt(digitalPinToInterrupt(ENC1A), ispLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), ispLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), ispRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), ispRight, CHANGE);



  // Start PID controlers. All we need next is data
  leftWheelPID.SetSampleTime(PID_PERIOD);
  rightWheelPID.SetSampleTime(PID_PERIOD);
  leftWheelPID.SetOutputLimits(-255, 255);
  rightWheelPID.SetOutputLimits(-255, 255);
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


// These are all used in the loop() below
static long encoderLeftLastValuePid  = 0;
static unsigned long millis_last_LEFT = 0;

static long encoderRightLastValuePid  = 0;
static unsigned long millis_last_RIGHT = 0;

static unsigned long NextPubMillis   = 0;
static long encoderLeftLastValueOdo  = 0;
static long encoderRightLastValueOdo = 0;
static long timeLastOdo              = 0;

// This loop() funtion is an arduino convention.  It is called by the environment
// inside a tight loop and runs forever or until the CPU is reset or powered off.
//
void loop() {
  float seconds_from_last;
  long  millis_from_last;
  float distLeft;
  float distRight;

  
  // Three things run here all on their own schedule
  //  1.  The Left Wheel PID controler
  //  2.  The Right Wheel PID controler
  //  3.  The Odometry and TF publisher
  // Each wheel has it's own PID control and might do it's computation at
  // different times.
    
  // Get encoder values
  long encLeft  = encoderLeft.getPos();
  long encRight = encoderRight.getPos();
  long curMillis = millis();  // capture time when encoders are sampled


  //=========> LEFT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValuePid);
  
  //figure out how fast the LEFT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_LEFT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  leftInput  = distLeft  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.
  if (leftWheelPID.Compute()) {
    setMotorSpeed(LEFT_MOTOR, leftOutput);
    
    encoderLeftLastValuePid = encLeft;
    millis_last_LEFT = curMillis;
  }
  
  //==========> RIGHT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distRight = meterPerTick * float(encRight - encoderRightLastValuePid);
  
  //figure out how fast the RIGHT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_RIGHT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  rightInput  = distRight  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.
  if (rightWheelPID.Compute()) {
    setMotorSpeed(RIGHT_MOTOR, rightOutput);
    
    encoderRightLastValuePid = encRight;
    millis_last_RIGHT = curMillis;
  }

  //==========> OdometryPublsher
  //
  // Check if it is time to publish /odom and /tf
  if (curMillis >= NextPubMillis) {
    NextPubMillis = curMillis + ODO_PERIOD;

    // Figure out how far we have gone in meters from last PID computation
    distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValueOdo);
    distRight = meterPerTick * float(encRight - encoderRightLastValueOdo);

  
    // Blink the LED to show we are alive
    toggleLED();
  
    // Publish odometry
    float odoInterval = float(curMillis - timeLastOdo) / 1000.0;
    odo.update_publish(nh.now(), odoInterval, distLeft, distRight);

    encoderLeftLastValueOdo  = encLeft;
    encoderRightLastValueOdo = encRight;
    timeLastOdo = curMillis;
    }
 
  
  // handle any data movements across the serial interface
  nh.spinOnce();
  delay(1);
}


// This funtion is called every time we receive a Twist message.
// We do not send the commanded speed to the wheels.  Rather we set
// thePID loops set point to the commanded sprrd.
void cmd_velCallback( const geometry_msgs::Twist& twist_msg) {

  

  // We only use two numbers from the Twist message.
  //    linear.x  is the forward speed in meters per second.
  //              (The "x" axis points forward.)
  //    angular.y is the rotation about the z or vertical
  //              axis in radians per second.
  //
  float vel_x   = twist_msg.linear.x;
  float vel_th  = twist_msg.angular.z;

  // This is a "hack".  It turns ou the motors have a minimum
  // speed because of internal friction.   If the commanded speed is
  // below a threshold we replace the commanded speed with zero.
  // TODO:  Find a better threshold, make it a parameter
  if (fabs(vel_x)  < 0.001) vel_x  = 0.0;
  if (fabs(vel_th) < 0.001) vel_th = 0.0;

  // Compute the wheel speeds in meters per second.
  float left_vel  =  vel_x - (vel_th * base_width / 2.0);
  float right_vel =  vel_x + (vel_th * base_width / 2.0);

  //char buff[40];
  //snprintf(buff, 100, "CMD_VEL %f, %f", left_vel, right_vel);
  //nh.loginfo(buff);

  // Show the Twist message on the LCD.
  //displayStatus(&vel_x, &vel_th);

  // The PID works in units of meters per second, so no
  // conversion is needed.
  leftSetpoint  = left_vel;
  rightSetpoint = right_vel;
}

void setMotorSpeed(byte motor, float pidOutput) {
  // Set the controler based on calulation from PID
  const float deadZone = 0.5;
  byte direction;

  int speed  = int(0.5 + fabs(pidOutput));
  
  if (pidOutput >  deadZone) {
    direction = CW;
  }
  else if (pidOutput < -deadZone) {
    direction = CCW;
  }
  else {
    direction = BRAKEGND;
    speed = 0;
  }
  moto.motorGo(motor, direction, speed);
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

void toggleLED() {
  if (ledState) {
    digitalWrite(LED_BUILTIN, LED_ON);
  }
  else {
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
  ledState = !ledState;
}
