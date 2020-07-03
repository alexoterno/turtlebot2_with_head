#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"
#include "string.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include "eigen3/Eigen/Dense"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <message_filters/sync_policies/exact_time.h>
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
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "stdio.h"
#include "unistd.h"
#include "termios.h"

#include <map>


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
    ros::Subscriber Head_joint_sub_;  // Head joint angle/velocity

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
    Eigen::MatrixXd J_Robot;
    Eigen::MatrixXd camera_velocities;
    Eigen::MatrixXd robot_velocities;
    Eigen::MatrixXd out_command;
    Eigen::MatrixXd lambda;
    vpHomogeneousMatrix cMo;
    geometry_msgs::Twist out_cmd_vel;
    std_msgs::Float64 out_pan_vel;
    std_msgs::Float64 out_tilt_vel;
    double pan_;
    double tilt_;
    double yaw_;
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
    std::string vs_joints;
    const double L11 = 0.019;
    const double L12 = -0.050; // y_camera is downward
    const double L13 = 0.014;
    const double L21 = 0.029;
    const double L22 = 0.010;
    const double L23 = 0.019;
    const double L31 = 0.086;
    const double L32 = 0.000;
    const double L33 = 0.818;
    double det, min_vel_linear, min_vel_angular;
    double delta_t, new_t, old_t;
    bool valid_time;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::JointState, geometry_msgs::PoseStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

  public:
    void init_vs();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type); // callback for sensor image steam
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg); // callback for current pose
    void CameraInfoCallback(const sensor_msgs::CameraInfo& msg); // callback to get camarea parameters
    void statusCallback(const std_msgs::Int8ConstPtr& msg); // callback to get tracker status
    void mobileBasePoseCallback(const nav_msgs::OdometryConstPtr& msg); // callback to get the mobile base pose
    void headPanPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    void headTiltPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    void callback(const nav_msgs::OdometryConstPtr& msg1, const sensor_msgs::JointStateConstPtr& msg2, const geometry_msgs::PoseStampedConstPtr& msg3);
    void headJointCallback(const sensor_msgs::JointStateConstPtr& msg);
    NonHoloVisualServoing(const std::string s); // constructor
    ~NonHoloVisualServoing(); // deconstructor

};

/////////////////////////////////////
// Non holonomic kinematics class //
///////////////////////////////////
class KinematicsNonHoloVS{
  private:
    ros::NodeHandle nh_; // node handle
    ros::Publisher mobile_base_pub_; // publisher of mobile base velocities
    ros::Publisher head_pan_pub_ ; // publisher of head pan velocity
    ros::Publisher head_tilt_pub_ ; // publisher of head tilt velocity
    // ros::Subscriber image_robotis_sub_; // subscriber to head robotis camera images
    // ros::Subscriber image_kinect_sub_; // subscriber to kinect camera images
    // ros::Subscriber image_kinect_depth_sub_; // subscriber to depth kinect camera images
    // ros::Subscriber PoseTarget_tracker_sub_;  // pose_stamped
    // ros::Subscriber CamInfoParam_sub_; // Camera parameters
    // ros::Subscriber Status_sub_;  // tracker status
    ros::Subscriber Mobile_base_pose_sub_;  // Mobile Base Pose
    // ros::Subscriber Head_pan_pose_sub_;  // Head pan pose
    // ros::Subscriber Head_tilt_pose_sub_;  // Head tilt pose
    ros::Subscriber Head_joint_sub_;  // Head joint angle/velocity

    std::string str_kinect;
    std::string str_depth_kinect;
    std::string str_robotis;

    // vpImage<unsigned char> im; // image container for current frame
    // sensor_msgs::Image current_rgb_msg;
    // vpDisplayX disp; // display device
    // bool camera_info; //Is equal to one if the informations about the camera are recieved
    // bool valid_pose, valid_pose_prev;
    // vpCameraParameters cam; // store camera parameters
    // double t_start_loop;
    // double tinit;
    // double Z, Zd;
    // double depth;
    // double lambda;
    double thresh;
    double high_ratio;
    double low_ratio;
    // double J_Robot[3][3]={0};
    // double J_Robot_invert[3][3]={0};
    Eigen::MatrixXd J_Robot_invert;
    Eigen::MatrixXd J_Robot;
    Eigen::MatrixXd camera_velocities;
    Eigen::MatrixXd cam_vel_required;
    Eigen::MatrixXd robot_velocities;
    Eigen::MatrixXd out_command;
    Eigen::MatrixXd lambda;
    // vpHomogeneousMatrix cMo;
    geometry_msgs::Twist out_cmd_vel;
    std_msgs::Float64 out_pan_vel;
    std_msgs::Float64 out_tilt_vel;
    double pan_;
    double tilt_;
    double yaw_;
    // vpPoint origin;
    // vpFeaturePoint s_x, s_xd;
    // vpFeatureDepth s_Z, s_Zd;
    // vpVelocityTwistMatrix cVe;
    // vpMatrix eJe;
    // vpColVector v; // control velocity vector
    // vpColVector vi;
    // double mu;
    // vpAdaptiveGain lambda_adapt;
    // vpServo task;
    // vpPioneer robot; //vpRobotPioneer
    double max_linear_vel;
    double max_angular_vel;
    double mobile_base_pose_x;
    double mobile_base_pose_y;
    double mobile_base_pose_s;
    double head_pan_angle;
    double head_tilt_angle;
    double roll, pitch, yaw;
    double x_pan_robot;
    std::string vs_joints;
    const double L11 = 0.019;
    const double L12 = -0.050; // y_camera is downward
    const double L13 = 0.014;
    const double L21 = 0.029;
    const double L22 = 0.010;
    const double L23 = 0.019;
    const double L31 = 0.086;
    const double L32 = 0.000;
    const double L33 = 0.818;
    double det, min_vel_linear, min_vel_angular;
    double delta_t, new_t, old_t;
    // bool valid_time;
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::JointState, geometry_msgs::PoseStamped> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // boost::shared_ptr<Sync> sync;
    // Map for speed keys
    std::map<char, std::vector<float>> moveBindings;
    std::map<char, std::vector<float>> speedBindings;
    const char* msg;
    float speed; // Linear velocity (m/s)
    float turn; // Angular velocity (rad/s)
    char key;



  public:
    void init_vs();
    // void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type); // callback for sensor image steam
    // void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg); // callback for current pose
    // void CameraInfoCallback(const sensor_msgs::CameraInfo& msg); // callback to get camarea parameters
    // void statusCallback(const std_msgs::Int8ConstPtr& msg); // callback to get tracker status
    void mobileBasePoseCallback(const nav_msgs::OdometryConstPtr& msg); // callback to get the mobile base pose
    // void headPanPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    // void headTiltPoseCallback(const control_msgs::JointControllerStateConstPtr& msg); // callback to get the head pan pose
    // void callback(const nav_msgs::OdometryConstPtr& msg1, const sensor_msgs::JointStateConstPtr& msg2, const geometry_msgs::PoseStampedConstPtr& msg3);
    void headJointCallback(const sensor_msgs::JointStateConstPtr& msg);
    int getch(void);
    void moveRobot(Eigen::MatrixXd cam_vel);
    KinematicsNonHoloVS(const std::string s); // constructor
    ~KinematicsNonHoloVS(); // deconstructor

};
