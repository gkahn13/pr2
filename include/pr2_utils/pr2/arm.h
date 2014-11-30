#ifndef __PR2_ARM_H__
#define __PR2_ARM_H__

#include "pr2_utils/pr2_sim/arm.h"
#include "pr2_utils/utils/utils.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> JointCommandClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandClient;

#include <Eigen/Eigen>
using namespace Eigen;

namespace pr2 {

#define DEFAULT_SPEED .04

class Arm {
	std::vector<std::string> joint_name_suffixes = {"_shoulder_pan_joint",
	                                                 "_shoulder_lift_joint",
	                                                 "_upper_arm_roll_joint",
	                                                 "_elbow_flex_joint",
	                                                 "_forearm_roll_joint",
	                                                 "_wrist_flex_joint",
	                                                 "_wrist_roll_joint"};

	std::string gripper_joint_name_suffix = "_gripper_joint";

public:
	enum ArmType { left, right };
	enum Posture { untucked, tucked, up, side, mantis };

	Arm(ArmType a);

	void execute_pose_trajectory(const std::vector<Matrix4d>& pose_traj, double speed=DEFAULT_SPEED, bool block=true);
	void execute_joint_trajectory(const StdVectorJ& joint_traj, double speed=DEFAULT_SPEED, bool block=true);

	void go_to_posture(Arm::Posture posture, double speed=DEFAULT_SPEED, bool block=true);
	void go_to_pose(const Matrix4d& pose, double speed=DEFAULT_SPEED, bool block=true);
	void go_to_joints(const VectorJ& joints, double speed=DEFAULT_SPEED, bool block=true);

	void close_gripper(double max_effort=0, bool block=true, double timeout=10.0);
	void open_gripper(double max_effort=0, bool block=true, double timeout=10.0);
	void set_gripper(double pct, double max_effort=0, bool block=true, double timeout=10.0);

	void teleop();

	Matrix4d get_pose();
	VectorJ get_joints();

	Matrix4d fk(const VectorJ& joints);
	bool ik(const Matrix4d& pose, VectorJ& joints);

private:
	ArmType arm_type;
	pr2_sim::Simulator *sim;
	pr2_sim::Arm *arm_sim;

	std::string arm_name;
	std::vector<std::string> joint_names;
	std::string gripper_joint_name;

	VectorJ current_joints;
	double current_grasp;
	double max_joint_velocity;

	ros::NodeHandle *nh_ptr;
	ros::Subscriber joint_state_sub;
	sensor_msgs::JointState last_joint_state;
	bool received_joint_state;

	JointCommandClient *joint_command_client;
	double min_grasp, max_grasp, default_max_effort;
	GripperCommandClient *gripper_command_client;

	ros::Publisher display_trajectory_pub;

	void display_trajectory(const StdVectorJ& joint_traj);
	void _joint_state_callback(const sensor_msgs::JointStateConstPtr& joint_state);
};

}

#endif
