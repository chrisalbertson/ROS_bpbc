
#ifndef ODOMETRY_H
#define ODOMETRY_H


#include <ros_lib.h>
#include "NUC_ros.h"

// Include local version of ros.h, not the system version
//#include "ros.h" 

#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


class Odometer {
  public:
  Odometer(const float metersPerTick, const float base_width);

  void setupPubs(ros::NodeHandle &nh);
  
  void update_publish(ros::Time current_time, const float odoInterval, const float distLeft, const float distRight);
      

private:

void  update_odom(const float odoInterval, 
                  const float distLeft, const float distRight, float& vel_x, float& vel_theta);
                  
void  update_kinematics(const float distLeft, const float distRight, float& vel_x, float& vel_theta);
void  publish_odom(ros::Time current_time, const float vx, const float vth);
void  broadcastTf(ros::Time current_time);
float normalize_angle(float angle);


float _metersPerTick;
float _base_width;  // Meters

float _cur_x;       // Meters
float _cur_y;       // Meters
float _cur_theta;   // Radians
};
#endif /* ODOMETRY_H */
