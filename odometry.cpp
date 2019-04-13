
#include "odometry.h"

void Odometer::update_publish(float distLeft, float distRight) {

  float vel_x;
  float vel_theta;

  update(distLeft, distRight, &vel_x, &vel_theta);
  publish_odom(vel_x, vel_theta);
}

void Odometer::update(float distLeft, float distRight, float& vel_x, float& vel_theta) {

  float dist;
  float d_theta;

  dist = (distRight + distLeft) / 2.0;

  // Check for the special case of driving in a straight line
  // then compute current loation relativeto previous location
  if abs((distRight - distLeft) < (dist / 1000.0)) {

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


void Odometer::publish_odom(float vx, float vth) {
  
  quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta);
  current_time = rospy.Time.now();

  br = tf.TransformBroadcaster();
  br.sendTransform((cur_x, cur_y, 0),
                   tf.transformations.quaternion_from_euler(0, 0, -cur_theta),
                   current_time,
                   "base_link",
                   "odom");

  odom = Odometry();
  odom.header.stamp = current_time;
  odom.header.frame_id = 'odom';

  odom.pose.pose.position.x = _cur_x;
  odom.pose.pose.position.y = _cur_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = Quaternion(*quat);

  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.01;
  odom.pose.covariance[14] = 99999;
  odom.pose.covariance[21] = 99999;
  odom.pose.covariance[28] = 99999;
  odom.pose.covariance[35] = 0.01;

  odom.child_frame_id = 'base_link';
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = vth;
  odom.twist.covariance = odom.pose.covariance;

  odom_pub.publish(odom);
}

void Odometer::normalize_angle(float angle) {
  while (angle > pi) {
    angle -= 2.0 * pi
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}
