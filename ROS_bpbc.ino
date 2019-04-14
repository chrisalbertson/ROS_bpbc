



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

#include <ArduinoHardware.h>
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "EncoderBP.h"
#include "odometry.h"

ros::NodeHandle  nh;

#define PID_PERIOD 100

// TODO: Change these pin numbers to the pins connected to your encoder.
//       All pins should have interrupt capability

#define ENCPIN1A 1
#define ENCPIN1B 2
#define ENCPIN2A 3
#define ENCPIN2B 4

EncoderBP encoderLeft( ENCPIN1A, ENCPIN1B);
EncoderBP encoderRight(ENCPIN2A, ENCPIN2B);


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

long encoderLeftLastValue  = 0L;
long encoderRightLastValue = 0L;

Odometer odo(nh, meterPerTick, base_width, float(PID_PERIOD)/1000.0);



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

// Uses an LCD is optional but make it easy to see what is happening and is
// the only way to show status if there is not ROS computer connected
#define HAVE_LCD 1
#if HAVE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x38);  // Set the LCD I2C address
#endif // HAVE_LCD

// Functions declaratons (silly C-language requirement)
void cmd_velCallback( const geometry_msgs::Twist& toggle_msg);
void broadcastTf();

// Subscribe to Twist messages on cmd_vel.
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_velCallback);

// Setup   transfomer broadcaster.  This is very much automated
// We just need to feed it data which we get from wheel encoders
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//char base_link[] = "/base_link";
//char odom[]      = "/odom";


// millisecond to delay in main loop
#define LOOPDELAY 1




// This is an Arduino convention.  Place everything that needs to run just
// once in the setup() funtion.  The environment will call setup()
void setup() {

  // The very first thing we need to do is make sure the motors are
  // stopped.  Call the driver directly and also set the PID setpoints.
  moto.motorOff(LEFT_MOTOR);
  moto.motorOff(RIGHT_MOTOR);
  leftSetpoint  = 0.0;
  rightSetpoint = 0.0;


#if HAVE_LCD
  // before talking to ROS or anything else, set up uyjer LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  lcd.print("Connecting...");
  //         0000000000111111
  //         1234567890123456
#endif // HAVE_LCD 


  // Set up interrupt on encoder pins.   NOte the function ispLeft and ispRight
  // are required by the Arduino sysem because ISPs can not be non-static class
  // members.
  attachInterrupt(ENCPIN1A, ispLeft,  CHANGE);
  attachInterrupt(ENCPIN1B, ispLeft,  CHANGE);
  attachInterrupt(ENCPIN2A, ispRight, CHANGE);
  attachInterrupt(ENCPIN2B, ispRight, CHANGE);


  // Start PID controlers. All we need next is data
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

  // Connect to ROS computer and wait for connection
  nh.initNode();
  while (!nh.connected()) {
    nh.spinOnce();
  }

#if HAVE_LCD
  lcd.setCursor(0, 1);
  lcd.print("Connected      ");
  //         0000000000111111
  //         1234567890123456
#endif // HAVE_LCD 

  // First log message.
  nh.loginfo("ROS_bpbc stating...");

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

    // Read the encoders.  If some kind of error has caused a jump by
    // and unreasonable amout we do NOT want to use the encoder value.
    // It is better to skip a cycle thenuse invalid data
    long encLeft  = encoderLeft.getPos();
    long encRight = encoderRight.getPos();

    if (encoderJump(encLeft,  encoderLeftLastValue,
                    encRight, encoderRightLastValue)) {

      nh.logwarn("Encoder jump detected.");
    }
    else {

      // No encoderjump detected.  Do the "normal" cycle

      // Figure out how far we have gone then convert
      // this distance to velocity and send it to the PID controler.
      float distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValue);
      float distRight = meterPerTick * float(encRight - encoderRightLastValue);
      leftInput  = distLeft  / float(1000 * PID_PERIOD);
      rightInput = distRight / float(1000 * PID_PERIOD);

      // Capture the time when encoders were read.
      ros::Time current_time = nh.now();

      // re-compute the the motor drive based on previous measured speed.
      // The PID control tries to keep the motor runing at a constant
      // setpoint speed.  Input and Output are global variables
      leftWheelPID.Compute();
      rightWheelPID.Compute();
      setMotorSpeeds();

      // Publish odometry
      odo.update_publish(current_time, distLeft, distRight);
    }
  }

  // handle any data movements across the erial interface
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
  displayStatus(&vel_x, &vel_th);

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


void displayStatus(float *vel_x, float *vel_th) {
#if LCD
  // set the cursor (column, line)
  lcd.setCursor(0, 0);
  lcd.print("X");
  lcd.print(*vel_x);

  lcd.setCursor(0, 1);
  lcd.print("T");
  lcd.print(*vel_th);
#endif //LCD
}

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
