#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "string.h"

class visualServoing{
  private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber image_robotis_sub_;
    ros::Subscriber image_kinect_sub_;
    ros::Subscriber image_kinect_depth_sub_;

    std::string str_kinect;
    std::string str_depth_kinect;
    std::string str_robotis;


  public:
    // callback for sensor image steam
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type);
    // constructor
    visualServoing();

};

//------------------------------------
// constructor:
//------------------------------------
visualServoing::visualServoing(){
  ROS_INFO("Initializing values");
  str_kinect = "Kinect camera";
  str_depth_kinect = "Depth Kinect camera";
  str_robotis = "Robotis OP3 Head  camera";
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);

  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/robotis_op3/camera/image_raw", 1000, boost::bind (&visualServoing::cameraCallback, this,  _1, &str_robotis));
  image_kinect_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1000, boost::bind (&visualServoing::cameraCallback, this,  _1, &str_kinect));
  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1000, boost::bind (&visualServoing::cameraCallback, this,  _1, &str_depth_kinect));

}

//------------------------------------
// callback to get current features from sensor image stream
//------------------------------------
void visualServoing::cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type){
  ROS_INFO_STREAM("I recieved an Image from " << *camera_type);
  ROS_INFO_STREAM(msg->header.frame_id);
  geometry_msgs::Twist out_cmd_vel;
  // out_cmd_vel.linear.x = 0;
  // out_cmd_vel.linear.y = 0;
  // out_cmd_vel.linear.z = 0;
  // out_cmd_vel.angular.x = 0;
  // out_cmd_vel.angular.y = 0;
  // out_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(out_cmd_vel);
}

//------------------------------------
//Main function:
//------------------------------------
int main(int argc, char **argv)
{
  ROS_INFO("Starting IBVS");
  ros::init(argc, argv, "visual_servoing_node");
  visualServoing vsObject;
 ros::spin();
}
