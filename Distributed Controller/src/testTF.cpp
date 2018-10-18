#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <Vector3.h> for some god damn reason it works without including this
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/timer_options.h>
#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
	// main is used to initialize node and to read parameters
	// by using node handle
  ros::init(argc, argv, "coop");
  ros::NodeHandle n("~");


  

  return 0;
}