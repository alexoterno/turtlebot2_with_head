#include "visual_servoing/visual_servoing.hpp"



//------------------------------------
// constructor:
//------------------------------------
NonHoloVisualServoing::NonHoloVisualServoing(){
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
  PoseTarget_tracker_sub_   = nh_.subscribe("/visp_auto_tracker/object_position", 10, &NonHoloVisualServoing::poseCallback, this);
  CamInfoParam_sub_ = nh_.subscribe("/camera_info", 10, &NonHoloVisualServoing::CameraInfoCallback, this);
  Status_sub_ = nh_.subscribe("/visp_auto_tracker/status", 10, &NonHoloVisualServoing::statusCallback, this);
  Mobile_base_pose_sub_ = nh_.subscribe("/odom", 10, &NonHoloVisualServoing::mobileBasePoseCallback, this);
  Head_pan_pose_sub_ = nh_.subscribe("/robotis_op3/head_pan_position/state", 10, &NonHoloVisualServoing::headPanPoseCallback, this);
  Head_tilt_pose_sub_ = nh_.subscribe("/robotis_op3/head_tilt_position/state", 10, &NonHoloVisualServoing::headTiltPoseCallback, this);
  // disp.init(im, 0, 0, "a"); // display initialization
  // ROS_INFO("Display device initialized");
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
  depth = 0.15;
  // lambda = 1.;
  Z = Zd = depth;
  // v.resize(2);
  // vi.resize(2);
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
  // Eye in hand visual servoing with the following control law where camera velocities are computed
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Set the interaction matrix type (current, desired, mean or user defined) and how its inverse is computed
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
  task.setLambda(lambda_adapt);
  // cVe = robot.get_cVe();
  // eJe = robot.get_eJe();
  // task.set_cVe(cVe);
  // task.set_eJe(eJe);
  vpImagePoint ip(0,0);

  // Create the current x visual feature
  vpFeatureBuilder::create(s_x, cam, ip);

  // Create the desired x* visual feature
  s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()) ; //in the image plan
  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 in the camera plan
  s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0 image plan

  // Add the feature
  task.addFeature(s_Z, s_Zd);
  // std::cout << "cVe" << std::endl;
  // std::cout << cVe << std::endl;
  // std::cout << "eJe" << std::endl;
  // std::cout << eJe << std::endl;
  J_Robot_invert = Eigen::MatrixXd::Zero(3, 3);
  J_Robot_invert(0, 0)= 1;
  x_pan_robot = 0.1;
  robot_velocities = Eigen::MatrixXd::Zero(3,1);
  camera_velocities = Eigen::MatrixXd::Zero(3,1);
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
  ROS_INFO("Yaw : [%f]", yaw);
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
      out_pan_vel.data = 0.;
      out_tilt_vel.data = 0.;
      mobile_base_pub_.publish(out_cmd_vel);
      head_pan_pub_.publish(out_pan_vel);
      head_tilt_pub_.publish(out_tilt_vel);
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
  //
  /*DO CONTROL*/
  // std::cout << "J_Robot_invert: " << std::endl;
  J_Robot_invert (0, 1) = cos(head_pan_angle)/(mobile_base_pose_s + x_pan_robot);
  J_Robot_invert (0, 2) = -sin(head_pan_angle)/(mobile_base_pose_s + x_pan_robot);
  J_Robot_invert (1, 1) = sin(head_pan_angle);
  J_Robot_invert (1, 2) = cos(head_pan_angle);
  J_Robot_invert (2, 1) = -cos(head_pan_angle)/(mobile_base_pose_s + x_pan_robot);
  J_Robot_invert (2, 2) = sin(head_pan_angle)/(mobile_base_pose_s + x_pan_robot);
  // std::cout << J_Robot_invert << std::endl;
  camera_velocities (0) = v[4];
  camera_velocities (1) = v[0];
  camera_velocities (2) = v[2];
  // std::cout << "camera_velocities" << std::endl;
  // std::cout << camera_velocities << std::endl;
  robot_velocities = J_Robot_invert * camera_velocities;
  std::cout << "robot_velocities" << std::endl;
  std::cout << robot_velocities << std::endl;
  //////////////////////
  if (std::abs(v[0]) > max_linear_vel || std::abs(v[1]) > max_angular_vel) {
      ROS_INFO("Vel exceed max allowed");
      for (unsigned int i=0; i< v.size(); i++){
        ROS_INFO("v[%d]=%f", i, v[i]);
      v = 0;
      }
    }
  // std::cout << "velocity" << std::endl;
  // std::cout << v << std::endl;
  out_cmd_vel.linear.x = robot_velocities(1);
  out_cmd_vel.linear.y = 0;
  out_cmd_vel.linear.z = 0;
  out_cmd_vel.angular.x = 0;
  out_cmd_vel.angular.y = 0;
  out_cmd_vel.angular.z = robot_velocities(2);
  out_pan_vel.data = robot_velocities(0);
  out_tilt_vel.data = 0.;

  mobile_base_pub_.publish(out_cmd_vel);
  head_pan_pub_.publish(out_pan_vel);
  head_tilt_pub_.publish(out_tilt_vel);
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
    out_pan_vel.data = 0.;
    out_tilt_vel.data = 0.;
    mobile_base_pub_.publish(out_cmd_vel);
    head_pan_pub_.publish(out_pan_vel);
    head_tilt_pub_.publish(out_tilt_vel);
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
