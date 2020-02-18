#include "visual_servoing/visual_servoing.hpp"
#include "ros/ros.h"
//------------------------------------
//Main function:
//------------------------------------
int main(int argc, char **argv)
{
  ROS_INFO("Starting IBVS");
  ros::init(argc, argv, "visual_servoing_node");
  NonHoloVisualServoing n_h_vsObject;
  // VisualServoing vsObject;
 ros::spin();
}
