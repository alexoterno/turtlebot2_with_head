#include "visual_servoing/visual_servoing.hpp"
#include "ros/ros.h"
//------------------------------------
//Main function:
//------------------------------------
int main(int argc, char **argv)
{
  int counter;
  std::string vs_joints;
  // printf("Program Name Is: %s",argv[0]);
  if(argc==1){
    printf("\nNo Extra Command Line Argument Passed Other Than Program Name");
  }
  if(argc>=2)
  {
    // printf("\nNumber Of Arguments Passed: %d",argc);
    // printf("\n----Following Are The Command Line Arguments Passed----");
    for(counter=0;counter<argc;counter++){
      // std::cout << argv[counter]<< std::endl;
      if(strcmp(argv[counter],"pan")==0){
        vs_joints = "pan";
        break;
      }
      if(strcmp(argv[counter],"pan_tilt")==0){
        vs_joints = "pan_tilt";
        break;
      }
    }
  }
  // std::cout << "visual_servoing with : " << vs_joints << std::endl;
  ROS_INFO("Starting IBVS");
  ros::init(argc, argv, "visual_servoing_node");
  NonHoloVisualServoing n_h_vsObject(vs_joints);
  // HoloVisualServoing vsObject;
  ros::spin();
}
