#ifndef _ROS_H_
#define _ROS_H_

#include <ros/node_handle.h>
#include "ArduinoHardware.h"

namespace ros
{  
  // NodeHandle to have 10 Publishers, 10 Subscriber,
  // 1024 bytes for input buffer and 1024 bytes for output buffer.
  typedef NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> NodeHandle;
}

#endif
