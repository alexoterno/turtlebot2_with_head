#include "visual_servoing/visual_servoing.hpp"



//------------------------------------
// constructor:
//------------------------------------
NonHoloVisualServoing::NonHoloVisualServoing(const std::string s){
  ROS_INFO("Non Holonomic Visual Servoing Algorithm");
  ROS_INFO("Initializing values");
  str_kinect = "Kinect camera";
  str_depth_kinect = "Depth Kinect camera";
  str_robotis = "Robotis OP3 Head  camera";
  mobile_base_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
  head_pan_pub_ = nh_.advertise<std_msgs::Float64>("head_pan_position", 10);
  head_tilt_pub_ = nh_.advertise<std_msgs::Float64>("head_tilt_position", 10);

  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/robotis_op3/camera/image_raw", 10, boost::bind (&NonHoloVisualServoing::cameraCallback, this,  _1, &str_robotis));
  // image_kinect_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 10, boost::bind (&NonHoloVisualServoing::cameraCallback, this,  _1, &str_kinect));
  // image_robotis_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 10, boost::bind (&NonHoloVisualServoing::cameraCallback, this,  _1, &str_depth_kinect));

  PoseTarget_tracker_sub_   = nh_.subscribe("/visp_auto_tracker/object_position", 1, &NonHoloVisualServoing::poseCallback, this);
  CamInfoParam_sub_ = nh_.subscribe("/camera_info", 1, &NonHoloVisualServoing::CameraInfoCallback, this);
  Status_sub_ = nh_.subscribe("/visp_auto_tracker/status", 1, &NonHoloVisualServoing::statusCallback, this);
  Mobile_base_pose_sub_ = nh_.subscribe("/odom", 1, &NonHoloVisualServoing::mobileBasePoseCallback, this);
  // Head_pan_pose_sub_ = nh_.subscribe("/robotis_op3/head_pan_position/state", 1, &NonHoloVisualServoing::headPanPoseCallback, this);
  // Head_tilt_pose_sub_ = nh_.subscribe("/robotis_op3/head_tilt_position/state", 1, &NonHoloVisualServoing::headTiltPoseCallback, this);
  Head_joint_sub_ = nh_.subscribe("joint_states", 1, &NonHoloVisualServoing::headJointCallback, this);


  // message_filters::Subscriber<geometry_msgs::PoseStamped> poseTarget_tracker_sub_(nh_, "/visp_auto_tracker/object_position", 1);
  // message_filters::Subscriber<nav_msgs::Odometry> mobile_base_pose_sub_(nh_, "/odom", 1);
  // message_filters::Subscriber<control_msgs::JointControllerState> head_pan_pose_sub_(nh_, "/robotis_op3/head_pan_position/state", 1);
  // message_filters::Subscriber<control_msgs::JointControllerState> head_tilt_pose_sub_(nh_, "/robotis_op3/head_tilt_position/state", 1);
  // message_filters::Subscriber<sensor_msgs::JointState> head_joint_sub_(nh_, "joint_states", 1);
  // sync.reset(new Sync(MySyncPolicy(10), mobile_base_pose_sub_, head_joint_sub_, poseTarget_tracker_sub_));
  // sync->registerCallback(boost::bind(&NonHoloVisualServoing::callback, this, _1, _2, _3));
  // disp.init(im, 0, 0, "a"); // display initialization
  // ROS_INFO("Display device initialized");
  vs_joints = s;
  ros::spin();
}

//------------------------------------
// Deconstructor:
//------------------------------------
NonHoloVisualServoing::~NonHoloVisualServoing(){
}

//------------------------------------
// Initializing visual variables function
//------------------------------------
void NonHoloVisualServoing::init_vs(){
  camera_info = false;
  valid_pose = false;
  valid_pose_prev = false;
  depth = 0.075;
  // lambda = 1.;
  Z = Zd = depth;
  // v.resize(2);
  // vi.resize(2);
  v = 0; vi = 0;
  mu = 4;
  t_start_loop = 0.0;
  tinit = 0.0;
  thresh = 0.05;
  high_ratio = 1 + thresh;
  low_ratio = 1 - thresh;
  max_linear_vel = 0.5;
  min_vel_angular = 0.007;
  min_vel_linear = 0.025;
  max_angular_vel = vpMath::rad(30);
  lambda_adapt.initStandard(3, 0.2, 40);
  // Eye in hand visual servoing with the following control law where camera velocities are computed
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Set the interaction matrix type (current, desired, mean or user defined) and how its inverse is computed
  task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
  task.setLambda(lambda_adapt);
  // cVe = robot.get_cVe();
  // eJe = robot.get_eJe();
  // task.set_cVe(cVe);
  // task.set_eJe(eJe);
  vpImagePoint ip(0,0);

  // Create the current x visual feature
  vpFeatureBuilder::create(s_x, cam, ip);

  // Build a 2D point visual feature from the point coordinates in the image plan x and y.
  //The parameter Z which describes the depth, is set in the same time.
  s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  task.addFeature(s_x, s_xd); //, vpFeaturePoint::selectX()) ; //in the image plan
  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 in the camera plan
  s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 image plan

  // Add the feature
  task.addFeature(s_Z, s_Zd);
  // std::cout << "cVe" << std::endl;
  // std::cout << cVe << std::endl;
  // std::cout << "eJe" << std::endl;
  // std::cout << eJe << std::endl;

  if(vs_joints == "pan"){
    J_Robot = Eigen::MatrixXd::Zero(4, 4);
    J_Robot_invert = Eigen::MatrixXd::Zero(4, 4);
    robot_velocities = Eigen::MatrixXd::Zero(4,1);
    camera_velocities = Eigen::MatrixXd::Zero(4,1);
    lambda = Eigen::MatrixXd::Identity(4,4);
    // lambda(1,1) = 0.9;
  }
  else if(vs_joints == "pan_tilt"){
    J_Robot_invert = Eigen::MatrixXd::Zero(5, 7); // moore pseudo invert
    J_Robot = Eigen::MatrixXd::Zero(7, 5);
    robot_velocities = Eigen::MatrixXd::Zero(5,1);
    camera_velocities = Eigen::MatrixXd::Zero(7,1);
    lambda = Eigen::MatrixXd::Identity(5,5);
    // lambda(2,2) = 0.2;
  }

  out_command = Eigen::MatrixXd::Zero(4,1);
  pan_ = 0;
  tilt_ = 0;
  std::cout << "visual_servoing with : " << vs_joints << std::endl;
  valid_time = false;
  ROS_INFO("Initializing visual variables ended");
}

//------------------------------------
// callback to get current features from sensor image stream
//------------------------------------
// void NonHoloVisualServoing::cameraCallback(const sensor_msgs::ImageConstPtr& msg, const std::string *camera_type){
//   current_rgb_msg = *msg; // convert sensor_msgs::ImageConstPtr to sensor_msgs::Image
//   ROS_INFO_STREAM("I recieved an Image from " << *camera_type);
//   ROS_INFO_STREAM("Width : " << current_rgb_msg.width << "\t Height : " << current_rgb_msg.height);
//   // im = visp_bridge::toVispImage(current_rgb_msg); // get the image from sensor msg
//   // vpDisplay::display(im);
// }

//------------------------------------
// callback to get the mobile base pose
//------------------------------------
void NonHoloVisualServoing::mobileBasePoseCallback(const nav_msgs::OdometryConstPtr& msg){
  mobile_base_pose_x = msg->pose.pose.position.x;
  mobile_base_pose_y = msg->pose.pose.position.y;
  mobile_base_pose_s = pow((pow(mobile_base_pose_x, 2)+pow(mobile_base_pose_y, 2)),0.5);
  // ROS_INFO("x: [%f], y: [%f], s : [%f]", mobile_base_pose_x, mobile_base_pose_y, mobile_base_pose_s);
  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y , msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO("Yaw : [%f]", yaw);
}

//------------------------------------
// callback to get the head joint position
//------------------------------------
void NonHoloVisualServoing::headJointCallback(const sensor_msgs::JointStateConstPtr& msg){
  head_pan_angle = msg ->position[0];
  head_tilt_angle = msg ->position[1];
  // ROS_INFO("pan ange: [%f]", head_pan_angle);
  // ROS_INFO("tilt angle: [%f]", head_tilt_angle);
}

//------------------------------------
// callback to get the head pan pose
//------------------------------------
void NonHoloVisualServoing::headPanPoseCallback(const control_msgs::JointControllerStateConstPtr& msg){
  head_pan_angle = msg ->process_value;
  ROS_INFO("pan ange: [%f]", head_pan_angle);
}

//------------------------------------
// callback to get the head tilt base pose
//------------------------------------
void NonHoloVisualServoing::headTiltPoseCallback(const control_msgs::JointControllerStateConstPtr& msg){
  head_tilt_angle = msg ->process_value;
  ROS_INFO("tilt angle: [%f]", head_tilt_angle);
}

//------------------------------------
// callback to get the camera parameters
//------------------------------------
void NonHoloVisualServoing::CameraInfoCallback(const sensor_msgs::CameraInfo& msg){
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
void NonHoloVisualServoing::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  if (!camera_info ){
    ROS_INFO("Waiting for the camera parameters");
    return;
  }
  try{
    t_start_loop = vpTime::measureTimeMs();
    ROS_INFO("New pose received");
    new_t = ros::Time::now().toNSec();
    if(!valid_time){
      old_t = new_t;
      valid_time = true;
    }
    delta_t = (new_t - old_t)/1e9;
    old_t = new_t;
    std::cout << "Hz : " << 1/delta_t << std::endl;
    std::cout << "delta_t : " << delta_t << std::endl;
    cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);
    // std::cout << "cMo" << std::endl;
    // std::cout << cMo << std::endl;
    origin.setWorldCoordinates(0,0,0); // the coordinates of the point in the object frame
    //Compute the feature parameters in the camera frame (vpTracker::cP) and than compute the projection of these parameters in the image plane (vpTracker::p)
    origin.project(cMo);
    Z = origin.get_Z(); // Get the point Z coordinate in the camera frame
    std::cout << "Z : " << Z << std::endl;
    pan_ = head_pan_angle;
    tilt_ = head_tilt_angle;
    yaw_ = yaw;
    std::cout << "pan_ : " << pan_ << std::endl;
    std::cout << "tilt_ : " << tilt_ << std::endl;
    std::cout << "yaw_ : " << yaw_ << std::endl;
    // std::cout << "s_x : " << s_x.get_s() << std::endl;
    // std::cout << "s_xd : " << s_xd.get_s()  << std::endl;
    // std::cout << "s_Z x: " << s_Z.get_x()  << std::endl;
    // std::cout << "s_Zd x: " << s_Zd.get_x()  << std::endl;
    // std::cout << "s_Z y: " << s_Z.get_y()  << std::endl;
    // std::cout << "s_Zd y: " << s_Zd.get_y()  << std::endl;
    // std::cout << "s_Z : " << s_Z.get_s()  << std::endl;
    // std::cout << "s_Zd : " << s_Zd.get_s()  << std::endl;
    // if ((Z <= depth * high_ratio) && (Z >= depth * low_ratio)){//IF1
    //     while(1){ //Infinite Loop -- Here we have finished our task
    //
    //         //Send message
    //         ROS_INFO("I\'m Done"); //Just to check
    //         //Publishing to fit to the multi master process and send signal to next task
    //         // pubTalker_.publish(msgDone);
    //         exit(1);
    //     }
    // }//END IF1
    if (Z <= 0)
      ROS_DEBUG("Z <= 0");
    if (!valid_pose || Z <= 0) {
      ROS_DEBUG("not a valid pose");
      out_cmd_vel.linear.x = 0;
      out_cmd_vel.linear.y = 0;
      out_cmd_vel.linear.z = 0;
      out_cmd_vel.angular.x = 0;
      out_cmd_vel.angular.y = 0;
      out_cmd_vel.angular.z = 0.1; // turn to find the marker 0.1
      out_pan_vel.data = pan_;
      out_tilt_vel.data = tilt_;
      mobile_base_pub_.publish(out_cmd_vel);
      // head_pan_pub_.publish(out_pan_vel);
      // head_tilt_pub_.publish(out_tilt_vel);
      valid_pose = false;
      valid_pose_prev = valid_pose;
      return;
    }
  // Update the current x feature
  s_x.set_xyZ(origin.p[0], origin.p[1], Z); //Feature coordinates expressed in the image plane p. They correspond to 2D normalized coordinates expressed in meters
  // $ x $ and $ y $ represent the coordinates of the point in the image plan and are the parameters of the visual feature $ s $. $ Z $ is the 3D coordinate in the camera frame representing the depth
  // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd));
  // cVe = robot.get_cVe();
  // task.set_cVe(cVe);
  // Get the robot jacobian
  // eJe = robot.get_eJe();
  // Update the jacobian that will be used to compute the control law
  // task.set_eJe(eJe);
  // Compute the control law. Velocities are computed in the mobile robot reference frame
  v = task.computeControlLaw();
  // if (0) { //valid_pose_prev == false) {
  //     // Start a new visual servo
  //     ROS_INFO("Reinit visual servo");
  //
  //     tinit = t_start_loop;
  //     vi = v;
  //   }
  // v = v - vi*exp(-mu*(t_start_loop - tinit)/1000.);
  if (
    (std::abs(v[0]) <= min_vel_linear * high_ratio) &&
    (std::abs(v[2]) <= min_vel_angular * high_ratio) &&
    (std::abs(v[3]) <= min_vel_angular * high_ratio) &&
    (std::abs(v[4]) <= min_vel_angular * high_ratio)
    // (std::abs(v[4]) <= min_vel_angular) && (std::abs(v[5]) <= min_vel_angular)
  ){
    std::cout << "velocity" << std::endl;
    std::cout << v << std::endl;
    //Send message
    ROS_INFO("I\'m Done"); //Just to check
    //Publishing to fit to the multi master process and send signal to next task
    // pubTalker_.publish(msgDone);
    exit(1);
  }
  if(vs_joints == "pan"){
    // std::cout << "pan only" << std::endl;
    J_Robot (0, 0) = -1;
    J_Robot (0, 1) = -1;
    J_Robot (1, 0) = -L22-L13;
    J_Robot (1, 1) = -cos(pan_)*L31+sin(pan_)*L32-L22-L13;
    J_Robot (1, 2) = -sin(pan_+yaw_);
    J_Robot (1, 3) = -cos(pan_+yaw_);
    J_Robot (2, 0) = -L23+L11;
    J_Robot (2, 1) = -sin(pan_)*L31-sin(pan_)*L32-L22-L13;
    J_Robot (2, 2) = cos(pan_+yaw_);
    J_Robot (2, 3) = -sin(pan_+yaw_);
    J_Robot (3, 2) = sin(yaw_);
    J_Robot (3, 3) = -cos(yaw_);
    J_Robot_invert = J_Robot.inverse();

    camera_velocities (0) = v[4];
    camera_velocities (1) = v[0];
    camera_velocities (2) = v[2];

    robot_velocities = lambda*J_Robot_invert * camera_velocities;

    out_command(1) = delta_t*robot_velocities(0);
    out_command(2) = robot_velocities(1);
    out_command(3) = pow((pow(robot_velocities(2), 2)+pow(robot_velocities(3), 2)),0.5); //robot_velocities(2)*cos(yaw_)+robot_velocities(3)*sin(yaw_);
  }
  else if(vs_joints == "pan_tilt"){
    // std::cout << "pan_tilt" << std::endl;
    J_Robot (0, 0) = -1;

    J_Robot (1, 1) = -cos(tilt_);
    J_Robot (1, 2) = -cos(tilt_);

    J_Robot (2, 1) = sin(tilt_);
    J_Robot (2, 2) = sin(tilt_);

    J_Robot (3, 1) = -cos(tilt_)*L13-sin(tilt_)*L12-L22;
    J_Robot (3, 2) = -cos(tilt_)*L13-sin(tilt_)*L12-L22-cos(pan_)*L31+sin(tilt_)*L32;
    J_Robot (3, 3) = -sin(pan_+yaw_);
    J_Robot (3, 4) = -cos(pan_+yaw_);

    J_Robot (4, 0) = L13;
    J_Robot (4, 1) = sin(tilt_)*(L11-L23);
    J_Robot (4, 2) = sin(tilt_)*(-sin(pan_)*L31-cos(tilt_)*L32+L11-L23);
    J_Robot (4, 3) = sin(tilt_)*cos(pan_+yaw_);
    J_Robot (4, 4) = -sin(tilt_)*sin(pan_+yaw_);

    J_Robot (5, 0) = -L12;
    J_Robot (5, 1) = cos(tilt_)*(L11-L23);
    J_Robot (5, 2) = cos(tilt_)*(-sin(pan_)*L31-cos(tilt_)*L32+L11-L23);
    J_Robot (5, 3) = cos(tilt_)*cos(pan_+yaw_);
    J_Robot (5, 4) = -cos(tilt_)*sin(pan_+yaw_);

    J_Robot (6, 3) = sin(yaw_);
    J_Robot (6, 4) = -cos(yaw_);

    J_Robot_invert = (J_Robot.transpose()*J_Robot).inverse()*J_Robot.transpose(); // Moore pseudo inverse

    camera_velocities (0) = v[3];
    camera_velocities (1) = v[4];
    camera_velocities (2) = v[5];
    camera_velocities (3) = v[0];
    camera_velocities (4) = v[1];
    camera_velocities (5) = v[2];

    robot_velocities = lambda*J_Robot_invert * camera_velocities;

    out_command(0) = delta_t*robot_velocities(0);
    out_command(1) = delta_t*robot_velocities(1);
    out_command(2) = robot_velocities(2);
    out_command(3) = pow((pow(robot_velocities(3), 2)+pow(robot_velocities(4), 2)),0.5);
  }
  std::cout << "J_Robot" << std::endl;
  std::cout << J_Robot << std::endl;
  std::cout << "lambda" << std::endl;
  std::cout << lambda << std::endl;
  std::cout << "J_Robot_invert" << std::endl;
  std::cout << J_Robot_invert << std::endl;
  std::cout << "robot_velocities" << std::endl;
  std::cout << robot_velocities << std::endl;
  std::cout << "camera_velocities" << std::endl;
  std::cout << camera_velocities << std::endl;
  std::cout << "out_command" << std::endl;
  std::cout << out_command << std::endl;
  //////////////////////
  if (std::abs(out_command(3)) > max_linear_vel || std::abs(out_command(0)) > max_angular_vel || std::abs(out_command(1)) > max_angular_vel || std::abs(delta_t*out_command(2)) > max_angular_vel) {
    if(std::abs(out_command(3)) > max_linear_vel){
      ROS_INFO("linear speed exceed max allowed");
      out_command(3) = max_linear_vel;
    }
    else if(std::abs(out_command(0)) > max_angular_vel){
      ROS_INFO("Tilt angular spee exceed max allowed");
      out_command(0) = max_angular_vel;
    }
    else if(std::abs(out_command(1)) > max_angular_vel){
      ROS_INFO("Pan angular speed exceed max allowed");
      out_command(1) = max_angular_vel;
    }
    else if(std::abs(delta_t*out_command(1)) > max_angular_vel){
      ROS_INFO("Robot angular speed exceed max allowed");
      out_command(1) = max_angular_vel;
    }
  }
  std::cout << "velocity" << std::endl;
  std::cout << v << std::endl;
  out_cmd_vel.linear.x = out_command(3);
  out_cmd_vel.linear.y = 0;
  out_cmd_vel.linear.z = 0;
  out_cmd_vel.angular.x = 0;
  out_cmd_vel.angular.y = 0;
  out_cmd_vel.angular.z = out_command(2);
  pan_ += out_command(1);
  out_pan_vel.data = pan_;
  tilt_ += out_command(0);
  out_tilt_vel.data = tilt_;

  mobile_base_pub_.publish(out_cmd_vel);
  head_pan_pub_.publish(out_pan_vel);
  if(vs_joints == "pan_tilt"){
    head_tilt_pub_.publish(out_tilt_vel);
  }
  valid_pose_prev = valid_pose;
  valid_pose = false;
  std::cout << "out_cmd_vel : " << out_cmd_vel << std::endl;
  std::cout << "out_pan_vel : " << out_pan_vel << std::endl;
  std::cout << "out_tilt_vel : " << out_tilt_vel << std::endl;
  }

  catch(...) {
    ROS_INFO("Catch an exception: set vel to 0");
    out_cmd_vel.linear.x = 0;
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = 0;
    out_pan_vel.data = pan_;
    out_tilt_vel.data = tilt_;
    mobile_base_pub_.publish(out_cmd_vel);
    // head_pan_pub_.publish(out_pan_vel);
    // head_tilt_pub_.publish(out_tilt_vel);
  }
}

//------------------------------------
// callback to get current tracker status from visp_auto_tracker
//------------------------------------
void NonHoloVisualServoing::statusCallback(const std_msgs::Int8ConstPtr& msg){
  // ROS_INFO("/visp_auto_tracker/status : %d", msg->data); // see http://wiki.ros.org/visp_auto_tracker#Tracker_states
  if (msg->data == 3)
    valid_pose = true;
  else
    valid_pose = false;
}

//------------------------------------
// callback
//------------------------------------

void NonHoloVisualServoing::callback(const nav_msgs::OdometryConstPtr& msg1, const sensor_msgs::JointStateConstPtr& msg2, const geometry_msgs::PoseStampedConstPtr& msg3){
  mobileBasePoseCallback(msg1);
  headJointCallback(msg2);
  poseCallback(msg3);
}
