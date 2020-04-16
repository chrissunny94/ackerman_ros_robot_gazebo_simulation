/*
 * rbcar_robot_control
 * Copyright (c) 2015, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik
 * \brief Controller for the RBCAR robot in ackermann-steering (single)
    Control steering (2 direction axes) and traction axes (4W) of the RBCAR single Ackerman drive kinematics
    transforms the commands received from the joystick (or other high level controller) in motor position / velocity commands
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/get_mode.h>
#include <robotnik_msgs/set_odometry.h>
#include <robotnik_msgs/ptz.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

//#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"


#define PI 3.1415926535
#define RBCAR_MIN_COMMAND_REC_FREQ   5.0
#define RBCAR_MAX_COMMAND_REC_FREQ   150.0

#define RBCAR_D_WHEELS_M            	2.5    // distance from front to back axis, car-like kinematics
#define RBCAR_WHEEL_DIAMETER	    	0.470   // wheel avg diameter - may need calibration according to tyre pressure
#define RBCAR_JOINT_STATES_TIME_WINDOW	1.0		// accepted time deviation to process joint_sttate
        
using namespace std;

class RbcarControllerClass {

public:

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;

  // Diagnostics
  diagnostic_updater::Updater diagnostic_;				// General status diagnostic updater
  diagnostic_updater::FrequencyStatus freq_diag_;		// Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
  ros::Time last_command_time_;					// Last moment when the component received a command
  diagnostic_updater::FunctionDiagnosticTask command_freq_;

  // Robot model 
  std::string robot_model_;
  
  // Velocity and position references to low level controllers
  ros::Publisher ref_vel_flw_;
  ros::Publisher ref_vel_frw_;
  ros::Publisher ref_vel_blw_;
  ros::Publisher ref_vel_brw_;
  ros::Publisher ref_pos_flw_;
  ros::Publisher ref_pos_frw_;

  // Joint states published by the joint_state_controller of the Controller Manager
  ros::Subscriber joint_state_sub_;

  // High level robot command
  ros::Subscriber cmd_sub_;
	
  //ros::Subscriber gyro_sub_;

  // Services
  ros::ServiceServer srv_SetOdometry_;
  
  // Ackermann Topics - control action - traction - velocity
  std::string frw_vel_topic_;
  std::string flw_vel_topic_;
  std::string brw_vel_topic_;
  std::string blw_vel_topic_;
  
  // Ackerman Topics - control action - steering - position
  std::string frw_pos_topic_;
  std::string flw_pos_topic_;

  // Joint names - traction - velocity 
  std::string joint_front_right_wheel;
  std::string joint_front_left_wheel;
  std::string joint_back_left_wheel;
  std::string joint_back_right_wheel;
    
  // Joint names - steering - position
  std::string joint_front_right_steer;
  std::string joint_front_left_steer;

  // Indexes to joint_states
  int frw_vel_, flw_vel_, blw_vel_, brw_vel_;
  int frw_pos_, flw_pos_;

  // Robot Speeds
  double linearSpeedXMps_;
  double linearSpeedYMps_;
  double angularSpeedRads_;

  // Robot Positions
  double robot_pose_px_;
  double robot_pose_py_;
  double robot_pose_pa_;
  double robot_pose_vx_;
  double robot_pose_vy_;
  
  // Robot Joint States
  sensor_msgs::JointState joint_state_;
  
  // Command reference
  ackermann_msgs::AckermannDriveStamped base_vel_msg_;
  
  // External speed references
  double v_ref_;
  double alfa_ref_;
  double pos_ref_pan_;
  double pos_ref_tilt_;
  
  // Flag to indicate if joint_state has been read
  bool read_state_; 
  
  // Robot configuration parameters 
  double rbcar_wheel_diameter_; 
  double rbcar_d_wheels_;

  // IMU values
  double ang_vel_x_;
  double ang_vel_y_;
  double ang_vel_z_;

  double lin_acc_x_;
  double lin_acc_y_;
  double lin_acc_z_;

  double orientation_x_;
  double orientation_y_;
  double orientation_z_;
  double orientation_w_;

  // Parameter that defines if odom tf is published or not
  bool publish_odom_tf_;

  ros::Subscriber imu_sub_; 
  
  // Publisher for odom topic
  ros::Publisher odom_pub_; 

  // Broadcaster for odom tf  
  tf::TransformBroadcaster odom_broadcaster;


/*!	\fn RbcarControllerClass::RbcarControllerClass()
 * 	\brief Public constructor
*/
RbcarControllerClass(ros::NodeHandle h) : diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  desired_freq_(100),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&RbcarControllerClass::check_command_subscriber, this, _1))
  {

  ROS_INFO("rbcar_robot_control_node - Init ");
  
  ros::NodeHandle rbcar_robot_control_node_handle(node_handle_, "rbcar_robot_control");

  // Get robot model from the parameters
  if (!private_node_handle_.getParam("model", robot_model_)) {
	  ROS_ERROR("Robot model not defined.");
	  exit(-1);
	  }
  else ROS_INFO("Robot Model : %s", robot_model_.c_str());

  // Ackermann configuration - traction - topics


  private_node_handle_.param<std::string>("frw_vel_topic", frw_vel_topic_, "rbcar/right_front_axle_controller/command");
  private_node_handle_.param<std::string>("flw_vel_topic", flw_vel_topic_, "rbcar/left_front_axle_controller/command");
  private_node_handle_.param<std::string>("blw_vel_topic", blw_vel_topic_, "rbcar/left_rear_axle_controller/command");
  private_node_handle_.param<std::string>("brw_vel_topic", brw_vel_topic_, "rbcar/right_rear_axle_controller/command");

  // Ackermann configuration - traction - joint names 
  private_node_handle_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "right_front_axle");
  private_node_handle_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "left_front_axle");
  private_node_handle_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "left_rear_axle");
  private_node_handle_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "right_rear_axle");

  // Ackermann configuration - direction - topics
  private_node_handle_.param<std::string>("frw_pos_topic", frw_pos_topic_, "rbcar/right_steering_joint_controller/command");
  private_node_handle_.param<std::string>("flw_pos_topic", flw_pos_topic_, "rbcar/left_steering_joint_controller/command");

  private_node_handle_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "right_steering_joint"); 
  private_node_handle_.param<std::string>("joint_front_left_steer", joint_front_left_steer, "left_steering_joint");

  // Robot parameters
  if (!private_node_handle_.getParam("rbcar_d_wheels", rbcar_d_wheels_))
        rbcar_d_wheels_ = RBCAR_D_WHEELS_M;
  if (!private_node_handle_.getParam("rbcar_wheel_diameter", rbcar_wheel_diameter_))
    rbcar_wheel_diameter_ = RBCAR_WHEEL_DIAMETER;
  ROS_INFO("rbcar_d_wheels_ = %5.2f", rbcar_d_wheels_);
  ROS_INFO("rbcar_wheel_diameter_ = %5.2f", rbcar_wheel_diameter_);

  private_node_handle_.param("publish_odom_tf", publish_odom_tf_, true);
  if (publish_odom_tf_) ROS_INFO("PUBLISHING odom->base_footprint tf");
  else ROS_INFO("NOT PUBLISHING odom->base_footprint tf");
  
  // Robot Speeds
  linearSpeedXMps_   = 0.0;
  linearSpeedYMps_   = 0.0;
  angularSpeedRads_  = 0.0;

  // Robot Positions
  robot_pose_px_ = 0.0;
  robot_pose_py_ = 0.0;
  robot_pose_pa_ = 0.0;
  robot_pose_vx_ = 0.0;
  robot_pose_vy_ = 0.0;
  
  // Robot state space control references
  v_ref_ = 0.0;
  alfa_ref_ = 0.0;
  pos_ref_pan_ = 0.0;

  // Imu variables
  ang_vel_x_ = 0.0; ang_vel_y_ = 0.0; ang_vel_z_ = 0.0;
  lin_acc_x_ = 0.0; lin_acc_y_ = 0.0; lin_acc_z_ = 0.0;
  orientation_x_ = 0.0; orientation_y_ = 0.0; orientation_z_ = 0.0; orientation_w_ = 1.0;

  // Advertise controller services
  srv_SetOdometry_ = rbcar_robot_control_node_handle.advertiseService("set_odometry",  &RbcarControllerClass::srvCallback_SetOdometry, this);

  // Subscribe to joint states topic
  joint_state_sub_ = node_handle_.subscribe<sensor_msgs::JointState>("joint_states", 1, &RbcarControllerClass::jointStateCallback, this);

  // Subscribe to imu data
  imu_sub_ = node_handle_.subscribe("imu/data", 1, &RbcarControllerClass::imuCallback, this);

  // Adevertise reference topics for the controllers 
  ref_vel_frw_ = node_handle_.advertise<std_msgs::Float64>( frw_vel_topic_, 50);
  ref_vel_flw_ = node_handle_.advertise<std_msgs::Float64>( flw_vel_topic_, 50);
  ref_vel_blw_ = node_handle_.advertise<std_msgs::Float64>( blw_vel_topic_, 50);
  ref_vel_brw_ = node_handle_.advertise<std_msgs::Float64>( brw_vel_topic_, 50);  
  ref_pos_frw_ = node_handle_.advertise<std_msgs::Float64>( frw_pos_topic_, 50);
  ref_pos_flw_ = node_handle_.advertise<std_msgs::Float64>( flw_pos_topic_, 50);
  	  
  // Subscribe to command topic
  cmd_sub_ = rbcar_robot_control_node_handle.subscribe<ackermann_msgs::AckermannDriveStamped>("command", 1, &RbcarControllerClass::commandCallback, this);
    
  // Publish odometry 
  odom_pub_ = private_node_handle_.advertise<nav_msgs::Odometry>("odom", 1000);

  // Component frequency diagnostics
  diagnostic_.setHardwareID("rbcar_robot_control - simulation");
  diagnostic_.add( freq_diag_ );
  diagnostic_.add( command_freq_ );
    
  // Topics freq control 
  // For /rbcar_robot_control/command
  double min_freq = RBCAR_MIN_COMMAND_REC_FREQ; // If you update these values, the
  double max_freq = RBCAR_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
  
  subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("~command", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
  subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control
  
  // Flag to indicate joint_state has been read
  read_state_ = false;
}

/// Controller startup in realtime
int starting()
{
  // Initialize joint indexes according to joint names 
  if (read_state_) {
		vector<string> joint_names = joint_state_.name;
		frw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_wheel)) - joint_names.begin();
		flw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_wheel)) - joint_names.begin();
		blw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_wheel)) - joint_names.begin();
		brw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_wheel)) - joint_names.begin();
		frw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_steer)) - joint_names.begin();
		flw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_steer)) - joint_names.begin();
    return 0;
  }else{
		ROS_WARN("RbcarControllerClass::starting: joint_states are not being received");
		return -1;
	}
}

/// Controller update loop 
void UpdateControl()
{
  // Compute state control actions
  // State feedback error 4 position loops / 4 velocity loops 
  // Single steering 
  double d1 =0.0;
  double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
  double alfa_ref_left = 0.0;
  double alfa_ref_right = 0.0;
  if (alfa_ref_!=0.0) {  // div/0
     d1 =  d / tan (alfa_ref_);
     alfa_ref_left = atan2( d, d1 - 0.105);
     alfa_ref_right = atan2( d, d1 + 0.105);
     if (alfa_ref_<0.0) {
		alfa_ref_left = alfa_ref_left - PI;
		alfa_ref_right = alfa_ref_right - PI;
		}     
     }
  else {
     alfa_ref_left = 0.0;
     alfa_ref_right = 0.0;
     }
     
   // Angular position ref publish
   std_msgs::Float64 frw_ref_pos_msg;
   std_msgs::Float64 flw_ref_pos_msg;
   std_msgs::Float64 brw_ref_pos_msg;
   std_msgs::Float64 blw_ref_pos_msg;

   flw_ref_pos_msg.data = alfa_ref_left;
   frw_ref_pos_msg.data = alfa_ref_right;
     
   // Linear speed ref publish (could be improved by setting correct speed to each wheel according to turning state
   // w = v_mps / (PI * D);   w_rad = w * 2.0 * PI
   double ref_speed_joint = 2.0 * v_ref_ / RBCAR_WHEEL_DIAMETER;
   
   std_msgs::Float64 frw_ref_vel_msg;
   std_msgs::Float64 flw_ref_vel_msg;
   std_msgs::Float64 brw_ref_vel_msg;
   std_msgs::Float64 blw_ref_vel_msg;
   frw_ref_vel_msg.data = -ref_speed_joint;
   flw_ref_vel_msg.data = -ref_speed_joint;
   brw_ref_vel_msg.data = -ref_speed_joint;
   blw_ref_vel_msg.data = -ref_speed_joint;
   
   // Publish msgs traction and direction
   ref_vel_frw_.publish( frw_ref_vel_msg );
   ref_vel_flw_.publish( flw_ref_vel_msg );
   ref_vel_blw_.publish( blw_ref_vel_msg );
   ref_vel_brw_.publish( brw_ref_vel_msg );
   ref_pos_frw_.publish( frw_ref_pos_msg );
   ref_pos_flw_.publish( flw_ref_pos_msg );
}

// Update robot odometry depending on kinematic configuration
void UpdateOdometry()
{
	// Get angles
    double a1, a2;
    
    if( (ros::Time::now() - joint_state_.header.stamp).toSec() > RBCAR_JOINT_STATES_TIME_WINDOW){
		ROS_WARN_THROTTLE(2, "RbcarControllerClass::UpdateOdometry: joint_states are not being received");
		return;
	}
		
	//		joint_state_.position[frw_pos_], joint_state_.position[flw_pos_])
    a1 = radnorm2( joint_state_.position[frw_pos_] );
    a2 = radnorm2( joint_state_.position[flw_pos_] );

    // Linear speed of each wheel [mps]
	double v3, v4; 
	// filtering noise from the Velocity controller when the speed is 0.0 (by using an open loop with desired speed)
	if( v_ref_ == 0.0){
		v3 = 0.0;
		v4 = 0.0;
	}else{
		v3 = joint_state_.velocity[blw_vel_] * (rbcar_wheel_diameter_ / 2.0);
		v4 = joint_state_.velocity[brw_vel_] * (rbcar_wheel_diameter_ / 2.0);
	}
    // Turning angle front 
    double fBetaRads = (a1 + a2) / 2.0;
	
	// Linear speed
    double fSamplePeriod = 1.0 / desired_freq_;  // Default sample period    
    double v_mps = -(v3 + v4) / 2.0;

    // Compute orientation just integrating imu gyro (not so reliable with the simulated imu)
    // robot_pose_pa_ += ang_vel_z_ * fSamplePeriod;

	// Compute orientation converting imu orientation estimation
	tf::Quaternion q(orientation_x_, orientation_y_, orientation_z_, orientation_w_);
	// ROS_INFO("ox=%5.2f oy=%5.2f oz=%5.2f ow=%5.2f", orientation_x_, orientation_y_, orientation_z_, orientation_w_);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robot_pose_pa_ = yaw;

    // Normalize
    while (robot_pose_pa_ >= PI)
         robot_pose_pa_ -= 2.0 * PI;
    while (robot_pose_pa_ <= (-PI))
         robot_pose_pa_ += 2.0 * PI;
    
    double vx = v_mps * cos(fBetaRads) * cos(robot_pose_pa_);
    double vy = v_mps * cos(fBetaRads) * sin(robot_pose_pa_);

    // Positions
    robot_pose_px_ += vx * fSamplePeriod;
    robot_pose_py_ += vy * fSamplePeriod;
	  
    // Compute Velocity (linearSpeedXMps_ computed in control
   	robot_pose_vx_ = vx;
    robot_pose_vy_ = vy;
          
    // ROS_INFO("Odom estimated x=%5.2f  y=%5.2f a=%5.2f", robot_pose_px_, robot_pose_py_, robot_pose_pa_);
}

// Publish robot odometry tf and topic depending 
void PublishOdometry()
{
	ros::Time current_time = ros::Time::now();
	
    //first, we'll publish the transform over tf
    // TODO change to tf_prefix 
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = robot_pose_px_;
    odom_trans.transform.translation.y = robot_pose_py_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = orientation_x_;
	odom_trans.transform.rotation.y = orientation_y_;
	odom_trans.transform.rotation.z = orientation_z_;
	odom_trans.transform.rotation.w = orientation_w_;
	
    // send the transform over /tf
	// activate / deactivate with param
	// this tf in needed when not using robot_pose_ekf
    if (publish_odom_tf_) odom_broadcaster.sendTransform(odom_trans);  
        
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
	// Position
    odom.pose.pose.position.x = robot_pose_px_;
    odom.pose.pose.position.y = robot_pose_py_;
    odom.pose.pose.position.z = 0.0;
	// Orientation
    odom.pose.pose.orientation.x = orientation_x_;
	odom.pose.pose.orientation.y = orientation_y_;
	odom.pose.pose.orientation.z = orientation_z_;
	odom.pose.pose.orientation.w = orientation_w_;
	// Pose covariance
    for(int i = 0; i < 6; i++)
      		odom.pose.covariance[i*6+i] = 0.1;  // test 0.001

    //set the velocity
    odom.child_frame_id = "base_footprint";
	// Linear velocities
    odom.twist.twist.linear.x = robot_pose_vx_;
    odom.twist.twist.linear.y = robot_pose_vy_;
	odom.twist.twist.linear.z = 0.0;
	// Angular velocities
	odom.twist.twist.angular.x = ang_vel_x_;
	odom.twist.twist.angular.y = ang_vel_y_;
    odom.twist.twist.angular.z = ang_vel_z_;
	// Twist covariance
	for(int i = 0; i < 6; i++)
     		odom.twist.covariance[6*i+i] = 0.1;  // test 0.001

    //publish the message
    odom_pub_.publish(odom);
}

/// Controller stopping
void stopping()
{}


/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time_).toSec();

	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
		// If no command is received, set Speed References to 0
		// Turning angle can stay in the previous position.
		v_ref_ = 0.0;
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}


// Set the base velocity command
// void setCommand(const geometry_msgs::Twist &cmd_vel)
void setCommand(const ackermann_msgs::AckermannDriveStamped &msg)
{   
    // Mapping - linear = v_ref_, angular = alfa_ref_ 
	double speed_limit = 10.0;  // m/s
	double angle_limit = PI/4.0;   // there should be also urdf limits
    v_ref_ = saturation(msg.drive.speed, -speed_limit, speed_limit);  
    alfa_ref_ = saturation(msg.drive.steering_angle, -angle_limit, angle_limit);
}

// Service SetOdometry 
bool srvCallback_SetOdometry(robotnik_msgs::set_odometry::Request &request, robotnik_msgs::set_odometry::Response &response )
{
	// ROS_INFO("rbcar_odometry::set_odometry: request -> x = %f, y = %f, a = %f", req.x, req.y, req.orientation);
	robot_pose_px_ = request.x;
	robot_pose_py_ = request.y;
	robot_pose_pa_ = request.orientation;

	response.ret = true;
	return true;
}

// Topic command
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{	
  joint_state_ = *msg;
  read_state_ = true;
}

// Topic command
// void commandCallback(const geometry_msgs::TwistConstPtr& msg)
// void commandCallback(const ackermann_msgs::AckermannDriveStamped& msg)
void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
  // Safety check
  last_command_time_ = ros::Time::now();
  subs_command_freq->tick();			// For diagnostics

  base_vel_msg_ = *msg;
  this->setCommand(base_vel_msg_);
}

// Imu callback
void imuCallback( const sensor_msgs::Imu& imu_msg){

	orientation_x_ = imu_msg.orientation.x;
	orientation_y_ = imu_msg.orientation.y;
	orientation_z_ = imu_msg.orientation.z;
	orientation_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
	lin_acc_z_ = imu_msg.linear_acceleration.z;
}

double saturation(double u, double min, double max)
{
 if (u>max) u=max;
 if (u<min) u=min;
 return u; 
}

double radnorm( double value ) 
{
  while (value > PI) value -= PI;
  while (value < -PI) value += PI;
  return value;
}

double radnorm2( double value ) 
{
  while (value > 2.0*PI) value -= 2.0*PI;
  while (value < -2.0*PI) value += 2.0*PI;
  return value;
}

bool spin()
{
    ROS_INFO("rbcar_robot_control::spin()");
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (starting() == 0)
      {
	    while(ros::ok() && node_handle_.ok()) {
          UpdateControl();
          UpdateOdometry();
          PublishOdometry();
          diagnostic_.update();
          ros::spinOnce();
	      r.sleep();
          }
	      ROS_INFO("END OF ros::ok() !!!");
      } else {
       // No need for diagnostic here since a broadcast occurs in start
       // when there is an error.
       usleep(1000000);
       ros::spinOnce();
      }
   }

   return true;
}

}; // Class RbcarControllerClass

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rbcar_robot_control");

	ros::NodeHandle n;		
  	RbcarControllerClass scc(n);

	// ros::ServiceServer service = n.advertiseService("set_odometry", &rbcar_node::set_odometry, &scc);
    scc.spin();

	return (0);
}

