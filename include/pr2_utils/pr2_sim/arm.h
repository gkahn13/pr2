#ifndef __PR2_SIM_ARM_H__
#define __PR2_SIM_ARM_H__

#include "simulator.h"
#include "utils.h"
#include "pr2_utils/utils/utils.h"

#include <Eigen/Eigen>
using namespace Eigen;

#include <openrave-core.h>
namespace rave = OpenRAVE;

#define ARM_DIM 7
typedef Matrix<double, ARM_DIM, 1> VectorJ;
typedef std::vector<VectorJ, aligned_allocator<VectorJ>> StdVectorJ;

namespace pr2_sim {

class Arm {
	friend class Camera;

	std::vector<std::string> joint_name_suffixes = {"_shoulder_pan_joint",
	                                                 "_shoulder_lift_joint",
	                                                 "_upper_arm_roll_joint",
	                                                 "_elbow_flex_joint",
	                                                 "_forearm_roll_joint",
	                                                 "_wrist_flex_joint",
	                                                 "_wrist_roll_joint"};

public:
	enum ArmType { left, right };
	enum Posture { untucked, tucked, up, side, mantis };

	Arm(ArmType a, Simulator *s);
	Arm(ArmType a) : Arm(a, NULL) { }

	void set_posture(Arm::Posture posture);
	void set_pose(const Matrix4d& pose);
	void set_joints(const VectorJ& joint_values);

	void teleop();

	Matrix4d get_pose();
	VectorJ get_joints();
	void get_joint_limits(VectorJ& l, VectorJ& u);

	Matrix4d fk(const VectorJ& joints);
	bool ik(const Matrix4d& pose, VectorJ& joints);
	bool ik_lookat(const Vector3d& position, const Vector3d& point, VectorJ& joints);

private:
	ArmType arm_type;
	Simulator *sim;

	std::string manip_name, tool_frame;
	rave::RobotBase::ManipulatorPtr manip;

	std::vector<int> joint_indices;
	int num_joints;
	VectorJ lower, upper;

	rave::Transform fk_origin;
	std::vector<rave::Vector> arm_joint_axes, arm_link_trans;

//	VectorJ _closer_joint_angles(const VectorJ& new_joints, const VectorJ& curr_joints);
};

}

#endif
