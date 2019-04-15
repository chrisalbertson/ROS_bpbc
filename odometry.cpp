/*
    odametry.cpp
    Send odom messages and broadcast TF

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

#include "odometry.h"

char base_link[] = "/base_link";
char odom[]      = "/odom";

nav_msgs::Odometry odomMsg;
ros::Publisher     odom_pub(odom, &odomMsg);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster tfBroadcaster;


Odometer::Odometer(ros::NodeHandle &nh, float metersPerTick, float base_width, float deltaTime){
  _metersPerTick = metersPerTick;
  _base_width    = base_width;
  _d_time        = deltaTime;
  _cur_x         = 0.0;
  _cur_y         = 0.0;
  _cur_theta     = 0.0;

  tfBroadcaster.init(nh);
}
       

void Odometer::update_publish(ros::Time current_time, const float distLeft, const float distRight) {

  float vel_x;
  float vel_theta;

  update_odom(distLeft, distRight, vel_x, vel_theta);
  publish_odom(current_time, vel_x, vel_theta);
  broadcastTf(current_time);
}

void Odometer::update_odom(float distLeft, float distRight, float& vel_x, float& vel_theta) {

  float dist;
  float d_theta;

  dist = (distRight + distLeft) / 2.0;

  // Check for the special case of driving in a straight line
  // then compute current loation relativeto previous location
  if (abs((distRight - distLeft) < (dist / 1000.0))) {

    // drove in sraight line
    d_theta = 0.0;
    _cur_x += dist * cos(_cur_theta);
    _cur_y += dist * sin(_cur_theta);
  }
  else {

    // The path was along an arc
    d_theta = (distRight - distLeft) / _base_width;
    float r = dist / d_theta;

    _cur_x += r * (sin(d_theta + _cur_theta) - sin(_cur_theta));
    _cur_y -= r * (cos(d_theta + _cur_theta) - cos(_cur_theta));
    _cur_theta = normalize_angle(_cur_theta + d_theta);
  }

  vel_x     = dist    / _d_time;
  vel_theta = d_theta / _d_time;

  return;
}


void Odometer::publish_odom(ros::Time current_time, float vx, float vth) {
  
//  quat = tf.transformations.quaternion_from_euler(0, 0, _cur_theta);
//  quat = tf::createQuaternionFromYaw(cur_theta);
//  current_time = nh.now();

 
  odomMsg.header.stamp          = current_time;
  odomMsg.header.frame_id       = odom;
  odomMsg.child_frame_id        = base_link;
  odomMsg.pose.pose.position.x  = _cur_x;
  odomMsg.pose.pose.position.y  = _cur_y;
  odomMsg.pose.pose.position.z  = 0.0;
  odomMsg.pose.pose.orientation = tf::createQuaternionFromYaw(_cur_theta);

  odomMsg.twist.twist.linear.x  = vx;
  odomMsg.twist.twist.linear.y  = 0;
  odomMsg.twist.twist.angular.z = vth;

  odom_pub.publish(&odomMsg);
}


void Odometer::broadcastTf(ros::Time current_time) {

  t.header.stamp            = current_time;
  t.header.frame_id         = odom;
  t.child_frame_id          = base_link;
  t.transform.translation.x = _cur_x;
  t.transform.translation.y = _cur_y;
  t.transform.translation.z = 0.0;
  t.transform.rotation      = tf::createQuaternionFromYaw(-_cur_theta);
  
  
  tfBroadcaster.sendTransform(t);
}


float Odometer::normalize_angle(float angle) {
  while (angle > PI) {
    angle -= 2.0 * PI;
  }
  while (angle < -PI) {
    angle += 2.0 * PI;
  }
  return angle;
}
