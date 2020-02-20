#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"
#include "string.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include <eigen3/Eigen/Dense>

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

class HoloVisualServoing{
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
    HoloVisualServoing(); // constructor
    ~HoloVisualServoing(); // deconstructor

};

/////////////////////////////////////
// Non holonomic constraints case //
///////////////////////////////////
class NonHoloVisualServoing{
  private:
    ros::NodeHandle nh_; // node handle
    ros::Publisher mobile_base_pub_; // publisher of mobile base velocities
    ros::Publisher head_pan_pub_ ; // publisher of head pan velocity
    ros::Publisher head_tilt_pub_ ; // publisher of head tilt velocity
    ros::Subscriber image_robotis_sub_; // subscriber to head robotis camera images
    ros::Subscriber image_kinect_sub_; // subscriber to kinect camera images
    ros::Subscriber image_kinect_depth_sub_; // subscriber to depth kinect camera images
    ros::Subscriber PoseTarget_tracker_sub_;  // pose_stamped
    ros::Subscriber CamInfoParam_sub_; // Camera parameters
    ros::Subscriber Status_sub_;  // tracker status
    ros::Subscriber Mobile_base_pose_sub_;  // Mobile Base Pose
    ros::Subscriber Head_pan_pose_sub_;  // Head pan pose
    ros::Subscriber Head_tilt_pose_sub_;  // Head tilt pose

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
    // double J_Robot[3][3]={0};
    // double J_Robot_invert[3][3]={0};
    Eigen::MatrixXd J_Robot_invert;
    Eigen::MatrixXd camera_velocities;
    Eigen::MatrixXd robot_velocities;
    vpHomogeneousMatrix cMo;
    geometry_msgs::Twist out_cmd_vel;
    std_msgs::Float64 out_pan_vel;
    std_msgs::Float64 out_tilt_vel;
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
    double mobile_base_pose_x;
    double mobile_base_pose_y;
    double mobile_base_pose_s;
    double head_pan_angle;
    double head_tilt_angle;
    double roll, pitch, yaw;
    double x_pan_robot;

  public:
    void init_vs();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type); // callback for sensor image steam
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg); // callback for current pose
    void CameraInfoCallback(const sensor_msgs::CameraInfo& msg); // callback to get camarea parameters
    void statusCallback(const std_msgs::Int8ConstPtr& msg); // callback to get tracker status
    void mobileBasePoseCallback(const nav_msgs::OdometryConstPtr& msg); // callback to get the mobile base pose
    void headPanPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    void headTiltPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    NonHoloVisualServoing(); // constructor
    ~NonHoloVisualServoing(); // deconstructor

};
