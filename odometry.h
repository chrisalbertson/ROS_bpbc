
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <ArduinoHardware.h>
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>



class Odometer {
  public:
  Odometer(ros::NodeHandle &nh, float metersPerTick, float base_width, float deltaTime);
  
  void update_publish(ros::Time current_time, float distLeft, float distRight);
      

private:

void  update_odom(float distLeft, float distRight, float& vel_x, float& vel_theta);
void  publish_odom(ros::Time current_time, float vx, float vth);
void  broadcastTf(ros::Time current_time);
float normalize_angle(float angle);



float _metersPerTick;
float _base_width;  // Meters

float _d_time;      // Seconds

float _cur_x;       // Meters
float _cur_y;       // Meters
float _cur_theta;   // Radians



};
#endif /* ODOMETRY_H */
