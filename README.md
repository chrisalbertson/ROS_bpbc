# ROS_bpbc - Blue Pill Base Controler

_WARNING_  I'm calling the version number "minus 1".  This means it is not finished.   

This project is a self-contained base controler for ROS and any diferential drive
mobile base.   The software runs on the common and very low cost "blue Pill" board
and drives a very widely cloned "monstrer motor shield"  The software subscribes to
cmd_vel messages and pulbishes odometry and TF.  It connects to a larger computer
such as a notebook PC or Raspberry Pi with a USB cable.  
This makes building a ROS based diferential drive robot simply plug and play 

By using the ROS Serial package, the ROS controler node runs on the 
STM32F103 ARM CPU chip on the Blue Pill board. 
