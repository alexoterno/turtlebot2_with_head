#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "string.h"

#include "visp/vpImageIo.h"
#include "visp_bridge/image.h"
#include "visp_bridge/camera.h"
#include "visp_bridge/3dpose.h"
#include "visp/vpDisplayX.h"
#include "visp/vpCameraParameters.h"
#include "visp/vpHomogeneousMatrix.h"
#include "visp/vpFeaturePoint.h"
#include "visp/vpFeatureDepth.h"
#include "visp/vpVelocityTwistMatrix.h"
#include "visp/vpServo.h"
#include "visp/vpPioneer.h"
#include "visp/vpFeatureBuilder.h"
#include "visp/vpDot.h"
#include "visp/vpDot2.h"


class VisualServoing{
  private:
    ros::NodeHandle nh_; // node handle
    ros::Publisher cmd_vel_pub_; // publisher of velocity commands
    ros::Subscriber image_robotis_sub_; // subscriber to head robotis camera images
    ros::Subscriber image_kinect_sub_; // subscriber to kinect camera images
    ros::Subscriber image_kinect_depth_sub_; // subscriber to depth kinect camera images
    ros::Subscriber PoseTarget_tracker_sub_;  // pose_stamped
    ros::Subscriber CamInfoParam_sub_; // Camera parameters
    ros::Subscriber Status_sub_;  // tracker status

    std::string str_kinect;
    std::string str_depth_kinect;
    std::string str_robotis;

    vpImage<unsigned char> im; // image container for current frame
    sensor_msgs::Image current_rgb_msg;
    vpDisplayX disp; // display device
    bool camera_info; //Is equal to one if the informations about the camera are recieved
    bool valid_pose, valid_pose_prev;
    vpCameraParameters cam; // store camera parameters
    double t_start_loop;
    double tinit;
    double Z, Zd;
    double depth;
    // double lambda;
    double thresh;
    double high_ratio;
    double low_ratio;
    vpHomogeneousMatrix cMo;
    geometry_msgs::Twist out_cmd_vel;
    vpPoint origin;
    vpFeaturePoint s_x, s_xd;
    vpFeatureDepth s_Z, s_Zd;
    vpVelocityTwistMatrix cVe;
    vpMatrix eJe;
    vpColVector v; // control velocity vector
    vpColVector vi;
    double mu;
    vpAdaptiveGain lambda_adapt;
    vpServo task;
    vpPioneer robot; //vpRobotPioneer
    double max_linear_vel;
    double max_angular_vel;

  public:
    void init_vs();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type); // callback for sensor image steam
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg); // callback for current pose
    void CameraInfoCallback(const sensor_msgs::CameraInfo& msg); // callback to get camarea parameters
    void statusCallback(const std_msgs::Int8ConstPtr& msg); // callback to get tracker status
    VisualServoing(); // constructor

};

//------------------------------------
// constructor:
//------------------------------------
VisualServoing::VisualServoing(){
  ROS_INFO("Initializing values");
  str_kinect = "Kinect camera";
  str_depth_kinect = "Depth Kinect camera";
  str_robotis = "Robotis OP3 Head  camera";
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);

  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/robotis_op3/camera/image_raw", 10, boost::bind (&VisualServoing::cameraCallback, this,  _1, &str_robotis));
  // image_kinect_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 10, boost::bind (&VisualServoing::cameraCallback, this,  _1, &str_kinect));
  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 10, boost::bind (&VisualServoing::cameraCallback, this,  _1, &str_depth_kinect));
  PoseTarget_tracker_sub_   = nh_.subscribe("/visp_auto_tracker/object_position", 10, &VisualServoing::poseCallback, this);
  CamInfoParam_sub_ = nh_.subscribe("/camera_info", 10, &VisualServoing::CameraInfoCallback, this);
  Status_sub_ = nh_.subscribe("/visp_auto_tracker/status", 10, &VisualServoing::statusCallback, this);
  // disp.init(im, 0, 0, "a"); // display initialization
  // ROS_INFO("Display device initialized");
}

//------------------------------------
// Initializing visual variables function
//------------------------------------
void VisualServoing::init_vs(){
  camera_info = false;
  valid_pose = false;
  valid_pose_prev = false;
  depth = 0.1;
  // lambda = 1.;
  Z = Zd = depth;
  v.resize(2);
  vi.resize(2);
  v = 0; vi = 0;
  mu = 4;
  t_start_loop = 0.0;
  tinit = 0.0;
  thresh = 0.02;
  high_ratio = 1 + thresh;
  low_ratio = 1 - thresh;
  max_linear_vel = 0.5;
  max_angular_vel = vpMath::rad(50);
  lambda_adapt.initStandard(3, 0.2, 40);
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
  task.setLambda(lambda_adapt);
  cVe = robot.get_cVe();
  eJe = robot.get_eJe();
  task.set_cVe(cVe);
  task.set_eJe(eJe);
  vpImagePoint ip(0,0);

  // Create the current x visual feature
  vpFeatureBuilder::create(s_x, cam, ip);

  // Create the desired x* visual feature
  s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()) ; //in the image plan
  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 in the camera plan
  s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 in the camera plan

  // Add the feature
  task.addFeature(s_Z, s_Zd);
  std::cout << "cVe" << std::endl;
  std::cout << cVe << std::endl;
  std::cout << "eJe" << std::endl;
  std::cout << eJe << std::endl;
  ROS_INFO("Initializing visual variables ended");
}

//------------------------------------
// callback to get current features from sensor image stream
//------------------------------------
// void VisualServoing::cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type){
//   current_rgb_msg = *msg; // convert sensor_msgs::ImageConstPtr to sensor_msgs::Image
//   ROS_INFO_STREAM("I recieved an Image from " << *camera_type);
//   ROS_INFO_STREAM("Width : " << current_rgb_msg.width << "\t Height : " << current_rgb_msg.height);
//   // im = visp_bridge::toVispImage(current_rgb_msg); // get the image from sensor msg
//   // vpDisplay::display(im);
// }

//------------------------------------
// callback to get the camera parameters
//------------------------------------
void VisualServoing::CameraInfoCallback(const sensor_msgs::CameraInfo& msg){
  ROS_INFO("Received Camera INFO");
  cam = visp_bridge::toVispCameraParameters(msg);
  cam.printParameters();
 // Stop the subscriber (we don't need it anymore)
 init_vs();
 this->CamInfoParam_sub_.shutdown();
 camera_info = true;
 ROS_INFO("CameraInfoCallback subscriber shutdown");
 }

//------------------------------------
// callback to get current targe pose from visp_auto_tracker
//------------------------------------
void VisualServoing::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  std::cout << camera_info << std::endl;
  if (!camera_info ){
    ROS_INFO("Waiting for the camera parameters");
    return;
  }
  try{
    t_start_loop = vpTime::measureTimeMs();
    ROS_INFO("New pose received");
    cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);
    // std::cout << "cMo" << std::endl;
    // std::cout << cMo << std::endl;
    origin.setWorldCoordinates(0,0,0); // the coordinates of the point in the object frame
    //Compute the feature parameters in the camera frame (vpTracker::cP) and than compute the projection of these parameters in the image plane (vpTracker::p)
    origin.project(cMo);
    Z = origin.get_Z(); // Get the point Z coordinate in the camera frame
    std::cout << "Z : " << Z << std::endl;
    if ((Z <= depth * high_ratio) && (Z >= depth * low_ratio)){//IF1
        while(1){ //Infinite Loop -- Here we have finished our task

            //Send message
            ROS_INFO("Im Done"); //Just to check
            //Publishing to fit to the multi master process and send signal to next task
            // pubTalker_.publish(msgDone);
            exit(1);
        }
    }//END IF1
    if (Z <= 0)
      ROS_DEBUG("Z <= 0");
    if (!valid_pose || Z <= 0) {
      ROS_DEBUG("not a valid pose");
      out_cmd_vel.linear.x = 0;
      out_cmd_vel.linear.y = 0;
      out_cmd_vel.linear.z = 0;
      out_cmd_vel.angular.x = 0;
      out_cmd_vel.angular.y = 0;
      out_cmd_vel.angular.z = 0.1; // turn to find the marker
      cmd_vel_pub_.publish(out_cmd_vel);
      valid_pose = false;
      valid_pose_prev = valid_pose;
      return;
    }
  // Update the current x feature
  s_x.set_xyZ(origin.p[0], origin.p[1], Z); //Feature coordinates expressed in the image plane p. They correspond to 2D normalized coordinates expressed in meters
  // $ x $ and $ y $ represent the coordinates of the point in the image plan and are the parameters of the visual feature $ s $. $ Z $ is the 3D coordinate in the camera frame representing the depth
  // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd));
  cVe = robot.get_cVe();
  task.set_cVe(cVe);
  // Get the robot jacobian
  eJe = robot.get_eJe();
  // Update the jacobian that will be used to compute the control law
  task.set_eJe(eJe);
  // Compute the control law. Velocities are computed in the mobile robot reference frame
  v = task.computeControlLaw() ;
  // std::cout << "velocity" << std::endl;
  // std::cout << v << std::endl;
  if (0) { //valid_pose_prev == false) {
      // Start a new visual servo
      ROS_INFO("Reinit visual servo");

      tinit = t_start_loop;
      vi = v;
    }
  v = v - vi*exp(-mu*(t_start_loop - tinit)/1000.);

  if (std::abs(v[0]) > max_linear_vel || std::abs(v[1]) > max_angular_vel) {
      ROS_INFO("Vel exceed max allowed");
      for (unsigned int i=0; i< v.size(); i++){
        ROS_INFO("v[%d]=%f", i, v[i]);
      v = 0;
      }
    }
  std::cout << "velocity" << std::endl;
  std::cout << v << std::endl;
  out_cmd_vel.linear.x = v[0];
  out_cmd_vel.linear.y = 0;
  out_cmd_vel.linear.z = 0;
  out_cmd_vel.angular.x = 0;
  out_cmd_vel.angular.y = 0;
  out_cmd_vel.angular.z = v[1];

  cmd_vel_pub_.publish(out_cmd_vel);
  valid_pose_prev = valid_pose;
  valid_pose = false;
  }

  catch(...) {
    ROS_INFO("Catch an exception: set vel to 0");
    out_cmd_vel.linear.x = 0;
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(out_cmd_vel);
  }
}

//------------------------------------
// callback to get current tracker status from visp_auto_tracker
//------------------------------------
void VisualServoing::statusCallback(const std_msgs::Int8ConstPtr& msg){
  // ROS_INFO("/visp_auto_tracker/status : %d", msg->data); // see http://wiki.ros.org/visp_auto_tracker#Tracker_states
  if (msg->data == 3)
    valid_pose = true;
  else
    valid_pose = false;
}

//------------------------------------
//Main function:
//------------------------------------
int main(int argc, char **argv)
{
  ROS_INFO("Starting IBVS");
  ros::init(argc, argv, "visual_servoing_node");
  VisualServoing vsObject;
 ros::spin();
}
