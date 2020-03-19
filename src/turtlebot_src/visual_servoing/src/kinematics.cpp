#include "visual_servoing/visual_servoing.hpp"



//------------------------------------
// constructor:
//------------------------------------
KinematicsNonHoloVS::KinematicsNonHoloVS(const std::string s){
  ROS_INFO("Non Holonomic Visual Servoing Algorithm");
  ROS_INFO("Initializing values");

  mobile_base_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
  head_pan_pub_ = nh_.advertise<std_msgs::Float64>("head_pan_position", 10);
  head_tilt_pub_ = nh_.advertise<std_msgs::Float64>("head_tilt_position", 10);

  Mobile_base_pose_sub_ = nh_.subscribe("/odom", 1, &KinematicsNonHoloVS::mobileBasePoseCallback, this);
  Head_joint_sub_ = nh_.subscribe("joint_states", 1, &KinematicsNonHoloVS::headJointCallback, this);
  vs_joints = s;
  init_vs();
  printf("%s", msg);
  printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);
  while(true){
    // Get the pressed key
    key = KinematicsNonHoloVS::getch();
    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      cam_vel_required(0) = moveBindings[key][0];
      cam_vel_required(1) = moveBindings[key][1];
      cam_vel_required(2) = moveBindings[key][2];
      cam_vel_required(3) = moveBindings[key][3];
      cam_vel_required(4) = moveBindings[key][4];
      cam_vel_required(5) = moveBindings[key][5];
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
      // std::cout << "Camera velocities : " << cam_vel_required << std::endl;
    }
    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }
    // Otherwise, set the robot to stop
    else
    {
      cam_vel_required(0) = 0;
      cam_vel_required(1) = 0;
      cam_vel_required(2) = 0;
      cam_vel_required(3) = 0;
      cam_vel_required(4) = 0;
      cam_vel_required(5) = 0;
      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }
      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
    }
    cam_vel_required(0) *= turn;
    cam_vel_required(1) *= turn;
    cam_vel_required(2) *= turn;
    cam_vel_required(3) *= speed;
    cam_vel_required(4) *= speed;
    cam_vel_required(5) *= speed;
    std::cout << std::endl << "Camera velocities required : " << std::endl;
    std::cout << cam_vel_required << std::endl;
    moveRobot(cam_vel_required);
    ros::spinOnce();
  }
  // ros::spin();
}

//------------------------------------
// Deconstructor:
//------------------------------------
KinematicsNonHoloVS::~KinematicsNonHoloVS(){
}

//------------------------------------
// Initializing visual variables function
//------------------------------------
void KinematicsNonHoloVS::init_vs(){
  thresh = 0.05;
  high_ratio = 1 + thresh;
  low_ratio = 1 - thresh;
  max_linear_vel = 0.10;
  min_vel_angular = 0.007;
  min_vel_linear = 0.025;
  max_angular_vel = vpMath::rad(15);

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
  cam_vel_required = Eigen::MatrixXd::Zero(6,1);
  pan_ = 0;
  tilt_ = 0;
  speedBindings = {
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
  };
  // Map for movement keys
  moveBindings = {
    {'7', {0, 1, 0, 0, 0, 0}},
    {'8', {0, -1, 0, 0, 0, 0}},
    {'4', {1, 0, 0, 0, 0, 0}},
    {'5', {-1, 0, 0, 0, 0, 0}},
    {'1', {0, 0, 0, 0, 0, 1}},
    {'2', {0, 0, 0, 0, 0, -1}},
    {'6', {1, 1, 0, 0, 0, 0}},
    {'3', {-1, -1, 0, 0, 0, 0}},
    {'v', {0, 0, 0, 1, 0, 0}},
    {'b', {0, 0, 0, -1, 0, 0}},
    {'n', {0, 0, 0, 0, 0, 1}},
    {'m', {0, 0, 0, 0, 0, -1}}
  };
  msg = R"(
    7/8 : increase/decrease wy
    4/5 : increase/decrease wx
    1/2 : increase/decrease vz
    6/3 : increase/decrease wy and wx
    v/b : increase/decrease vx
    n/m : increase/decrease vy
    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%
    CTRL-C to quit
  )";
  speed = 0.1; // Linear velocity (m/s)
  turn = 0.25; // Angular velocity (rad/s)
  key = ' ';
  std::cout << "visual_servoing with : " << vs_joints << std::endl;
  ROS_INFO("Initializing visual variables ended");
}

//------------------------------------
// callback to get the mobile base pose
//------------------------------------
void KinematicsNonHoloVS::mobileBasePoseCallback(const nav_msgs::OdometryConstPtr& msg){
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
void KinematicsNonHoloVS::headJointCallback(const sensor_msgs::JointStateConstPtr& msg){
  head_pan_angle = msg ->position[0];
  head_tilt_angle = msg ->position[1];
  // ROS_INFO("pan ange: [%f]", head_pan_angle);
  // ROS_INFO("tilt angle: [%f]", head_tilt_angle);
}

//------------------------------------
// callback to get the head pan pose
//------------------------------------
// void KinematicsNonHoloVS::headPanPoseCallback(const control_msgs::JointControllerStateConstPtr& msg){
//   head_pan_angle = msg ->process_value;
//   ROS_INFO("pan ange: [%f]", head_pan_angle);
// }

//------------------------------------
// callback to get the head tilt base pose
//------------------------------------
// void KinematicsNonHoloVS::headTiltPoseCallback(const control_msgs::JointControllerStateConstPtr& msg){
//   head_tilt_angle = msg ->process_value;
//   ROS_INFO("tilt angle: [%f]", head_tilt_angle);
// }

//---------------------------------
// For non-blocking keyboard inputs
//----------------------------------------
int KinematicsNonHoloVS::getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}


//------------------------------------
// callback to get current targe pose from visp_auto_tracker
//------------------------------------
void KinematicsNonHoloVS::moveRobot(Eigen::MatrixXd cam_vel){
    pan_ = head_pan_angle;
    tilt_ = head_tilt_angle;
    yaw_ = yaw;
    std::cout << "pan_ : " << pan_ << std::endl;
    std::cout << "tilt_ : " << tilt_ << std::endl;
    std::cout << "yaw_ : " << yaw_ << std::endl;
    std::cout << "yaw_  (deg): " << yaw_*180/3.14159 << std::endl;
    std::cout << "cos(yaw_) : " << cos(yaw_) << std::endl;
    std::cout << "sin(yaw_) : " << sin(yaw_) << std::endl;

    // if (
      // (std::abs(v[0]) <= min_vel_linear * high_ratio) &&
      // (std::abs(v[2]) <= min_vel_angular * high_ratio) &&
      // (std::abs(v[3]) <= min_vel_angular * high_ratio) &&
      // (std::abs(v[4]) <= min_vel_angular * high_ratio)
      // (std::abs(v[4]) <= min_vel_angular) && (std::abs(v[5]) <= min_vel_angular)
    // ){
      // std::cout << "velocity" << std::endl;
      // std::cout << v << std::endl;
      //Send message
      // ROS_INFO("I\'m Done"); //Just to check
      //Publishing to fit to the multi master process and send signal to next task
      // pubTalker_.publish(msgDone);
      // exit(1);
    // }
    if(vs_joints == "pan"){
      // std::cout << "pan only" << std::endl;
      J_Robot (0, 0) = -1;
      J_Robot (0, 1) = -1;
      J_Robot (1, 0) = -L22-L13;
      J_Robot (1, 1) = -cos(pan_)*L31+sin(pan_)*L32-L22-L13;
      J_Robot (1, 2) = -sin(pan_+yaw_);
      J_Robot (1, 3) = -cos(pan_+yaw_);
      J_Robot (2, 0) = -L23+L11;
      J_Robot (2, 1) = -sin(pan_)*L31-cos(pan_)*L32-L23+L11;
      J_Robot (2, 2) = cos(pan_+yaw_);
      J_Robot (2, 3) = -sin(pan_+yaw_);
      J_Robot (3, 2) = sin(yaw_);
      J_Robot (3, 3) = -cos(yaw_);
      J_Robot_invert = J_Robot.inverse();

      camera_velocities (0) = cam_vel_required(1);
      camera_velocities (1) = cam_vel_required(3);
      camera_velocities (2) = cam_vel_required(5);

      robot_velocities = lambda*J_Robot_invert * camera_velocities;

      out_command(1) = robot_velocities(0);
      out_command(2) = robot_velocities(1);
      out_command(3) = robot_velocities(2)*cos(yaw_)+robot_velocities(3)*sin(yaw_); //pow((pow(robot_velocities(2), 2)+pow(robot_velocities(3), 2)),0.5); //
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

      camera_velocities(0) = cam_vel_required(0);
      camera_velocities(1) = cam_vel_required(1);
      camera_velocities(2) = cam_vel_required(2);
      camera_velocities(3) = cam_vel_required(3);
      camera_velocities(4) = cam_vel_required(4);
      camera_velocities(5) = cam_vel_required(5);



      robot_velocities = lambda*J_Robot_invert * camera_velocities;

      out_command(0) = robot_velocities(0);
      out_command(1) = robot_velocities(1);
      out_command(2) = robot_velocities(2);
      out_command(3) = robot_velocities(3)*cos(yaw_)+robot_velocities(4)*sin(yaw_); //pow((pow(robot_velocities(3), 2)+pow(robot_velocities(4), 2)),0.5);
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

    if(std::abs(out_command(3)) > max_linear_vel){
      ROS_INFO("linear speed exceed max allowed");
      out_command(3) = out_command(3)/(std::abs(out_command(3)))*max_linear_vel; // sign * max_linear_vel
    }
    if(std::abs(out_command(0)) > max_angular_vel){
      ROS_INFO("Tilt angular spee exceed max allowed");
      out_command(0) = out_command(0)/(std::abs(out_command(0)))*max_angular_vel;
    }
    if(std::abs(out_command(1)) > max_angular_vel){
      ROS_INFO("Pan angular speed exceed max allowed");
      out_command(1) = out_command(1)/(std::abs(out_command(1)))*max_angular_vel;
    }
    if(std::abs(out_command(2)) > max_angular_vel){
      ROS_INFO("Robot angular speed exceed max allowed");
      out_command(2) = out_command(2)/(std::abs(out_command(2)))*max_angular_vel;
    }

    // std::cout << "velocity" << std::endl;
    // std::cout << v << std::endl;
    out_cmd_vel.linear.x = out_command(3);
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = out_command(2);
    // pan_ += out_command(1);
    out_pan_vel.data = out_command(1); //pan_;
    // tilt_ += out_command(0);
    out_tilt_vel.data = out_command(0); //tilt_;

    mobile_base_pub_.publish(out_cmd_vel);
    head_pan_pub_.publish(out_pan_vel);
    if(vs_joints == "pan_tilt"){
      head_tilt_pub_.publish(out_tilt_vel);
    }
    std::cout << "out_cmd_vel : " << out_cmd_vel << std::endl;
    std::cout << "out_pan_vel : " << out_pan_vel << std::endl;
    std::cout << "out_tilt_vel : " << out_tilt_vel << std::endl;

}
