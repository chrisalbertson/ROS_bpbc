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
#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

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
double Kp=2, Ki=5, Kd=1;

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

// Create a ROS node and subscribe to Twist messages on cmd_vel.
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_velCallback);

// Setup   transfomer broadcaster.  This is very much automated
// We just need to feed it data which we get from wheel encoders
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "/base_link";
char odom[]      = "/odom";


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
  lcd.print("Hello");
  //         0000000000111111 
  //         1234567890123456
#endif // HAVE_LCD 

  // Start PID controlers. All we need next is data
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

  // Connect to ROS computer and start the FT brodcastr
  nh.initNode();
  broadcaster.init(nh);

}


// The PID period is independent from the loop() period.  This defines
// the time between updata to the motr speed.
#define PID_PERIOD 100
unsigned long NextPIDMillis = 0;


// This loop() funtion is an arduino convnetion.  It is call by the environment
// inside a tight lop and runs forever or until the CPU is reset or powered off
void loop() {
  

  // The PID runs every PID_PERIOD milliseconds,  Check if it is time
  if (millis() >= NextPIDMillis) {


    // It is time to run, store the NEXT tine to run.
    NextPIDMillis = millis() + PID_PERIOD;

    // re-compute the the motor drive based on previous measured speed.
    // The PID control tries to keep the motor runing at a constant
    // setpoint speed.  Input and Output are global variables
    leftWheelPID.Compute();
    rightWheelPID.Compute();

    // Set the controler based on calulation from PID    
    byte direction;
    if (     leftOutput >  0.001) {
      direction = CW;
    }
    else if (leftOutput < -0.001) { 
      direction = CCW;
    }
    else {
      direction = BRAKEGND;
    }
    moto.motorGo(LEFT_MOTOR, direction, int(0.5 + fabs(leftOutput)));

    
    if (     rightOutput >  0.001) {
      direction = CW;
    }
    else if (rightOutput < -0.001) { 
      direction = CCW;
    }
    else {
      direction = BRAKEGND;
    }
    moto.motorGo(RIGHT_MOTOR, direction, int(0.5 + fabs(rightOutput)));
  }
    

  // Send out the transform braodcast if needed then handle any
  // data movement then jump back to top ofloop
  broadcastTf();
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

void broadcastTf(){
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0; 
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
}

void displayStatus(float *vel_x, float *vel_th){
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



  
