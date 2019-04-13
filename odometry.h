
#ifndef ODOMETRY_H
#define ODOMETRY_H


class Odometer {
  public:
  Odometer(float metersPerTick, float base_width, float deltaTime) {
        _ticks_per_meter = ticks_per_meter;
        _base_width = base_width;
        _deltaTime deltaTime;
        _odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10);
        _cur_x     = 0.0;
        _cur_y     = 0.0;
        _cur_theta = 0.0;
  }

   void update_publish(float distLeft, float distRight);
      

private:

void update(float distLeft, float distRight, float& vel_x, float& vel_theta);
void publish_odom(float vx, float vth);
void normalize_angle(float angle);

_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

float _metersPerTick;
float _base_width;

float _cur_x;
float _cur_y;
float _cur_theta;



};
#endif /* ODOMETRY_H */
