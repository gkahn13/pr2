#include "pr2_utils/pr2_sim/arm.h"

namespace pr2_sim {

/**
 *
 * Arm constructors
 *
 */

Arm::Arm(ArmType a, Simulator *s) : arm_type(a), sim(s) {
	if (s == NULL) {
		sim = new Simulator();
	}

	tool_frame = (arm_type == ArmType::left) ? "l_gripper_tool_frame" : "r_gripper_tool_frame";
	manip_name = (arm_type == ArmType::left) ? "leftarm" : "rightarm";
	manip = sim->robot->GetManipulator(manip_name);

	joint_indices = manip->GetArmIndices();
	num_joints = joint_indices.size();

	std::vector<double> lower_vec(num_joints), upper_vec(num_joints);
	sim->robot->GetDOFLimits(lower_vec, upper_vec, joint_indices);

	lower = VectorJ(lower_vec.data());
	upper = VectorJ(upper_vec.data());

	lower(5) = -M_PI/2; // enforce _wrist_flex_joint so carmine doesn't collide
	upper(5) = M_PI/2;

	sim->update(); // must do so the origin transform is correct
	Matrix4d eye = Matrix4d::Identity();
	fk_origin = eigen_to_rave(sim->transform_from_to(eye, "torso_lift_link", "base_link"));

	arm_joint_axes = { rave::Vector(0,0,1),
			rave::Vector(0,1,0),
			rave::Vector(1,0,0),
			rave::Vector(0,1,0),
			rave::Vector(1,0,0),
			rave::Vector(0,1,0),
			rave::Vector(1,0,0) };

	arm_link_trans = { rave::Vector(0, (arm_type == ArmType::left) ? 0.188 : -0.188, 0),
			rave::Vector(0.1, 0, 0),
			rave::Vector(0, 0, 0),
			rave::Vector(0.4, 0, 0),
			rave::Vector(0, 0, 0),
			rave::Vector(.321, 0, 0),
			rave::Vector(.18, 0, 0) };
}

/**
 * Arm command methods
 */

void Arm::set_posture(Arm::Posture posture) {
	std::vector<double> j;

	// joint values defined for left arm
	switch(posture) {
	case Posture::untucked:
		j = {0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0};
		break;
	case Posture::tucked:
		j = {0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09};
		break;
	case Posture::up:
		j = {0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27};
		break;
	case Posture::side:
		j = {1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074};
		break;
	case Posture::mantis:
		j = {2.03018192, -0.05474993, 1.011, -1.47618716,  0.55995636, -1.42855926,  0.823};
		break;
	default:
		j = {0, 0, 0, 0, 0, 0, 0};
		break;
	}

	if (arm_type == ArmType::right) {
		j = {-j[0], j[1], -j[2], j[3], -j[4], j[5], -j[6]};
	}

	VectorJ j_vec(j.data());
	set_joints(j_vec);
}

void Arm::set_pose(const Matrix4d& pose) {
	VectorJ joints;
	if (ik(pose, joints)) {
		set_joints(joints);
	} else {
		std::cout << "Cannot set arm pose to:\n" << pose << "\nIK failed\n";
	}
}

void Arm::set_joints(const VectorJ& joint_values) {
	std::vector<double> joint_values_vec(joint_values.rows());
	for(int i=0; i < num_joints; ++i) {
		joint_values_vec[i] = std::min(joint_values(i), upper(i));
		joint_values_vec[i] = std::max(joint_values(i), lower(i));
	}

	sim->robot->SetDOFValues(joint_values_vec, rave::KinBody::CheckLimitsAction::CLA_Nothing, joint_indices);
}

void Arm::teleop() {
	double pos_step = .01;
	std::map<int,rave::Vector> delta_position =
	{
			{'a' , rave::Vector(0, pos_step, 0)},
			{'d' , rave::Vector(0, -pos_step, 0)},
			{'w' , rave::Vector(pos_step, 0, 0)},
			{'x' , rave::Vector(-pos_step, 0, 0)},
			{'+' , rave::Vector(0, 0, pos_step)},
			{'-' , rave::Vector(0, 0, -pos_step)},
	};

	double angle_step = 2.0*(M_PI/180);
	std::map<int,rave::Vector> delta_angle =
	{
			{'p', rave::Vector(angle_step, 0, 0)},
			{'o', rave::Vector(-angle_step, 0, 0)},
			{'k', rave::Vector(0, angle_step, 0)},
			{'l', rave::Vector(0, -angle_step, 0)},
			{'n', rave::Vector(0, 0, angle_step)},
			{'m', rave::Vector(0, 0, -angle_step)},
	};

	std::cout << manip_name << " teleop\n";

	char c;
	while ((c = pr2_utils::getch()) != 'q') {

		rave::Transform pose = eigen_to_rave(get_pose());
		if (delta_position.count(c) > 0) {
			pose.trans += delta_position[c];
		} else if (delta_angle.count(c) > 0) {
			pose.rot = rave::geometry::quatFromAxisAngle(rave::geometry::axisAngleFromQuat(pose.rot) + delta_angle[c]);
		}
		set_pose(rave_to_eigen(pose));
	}

	std::cout << manip_name << " end teleop\n";
}

/**
 * Arm state info methods
 */

Matrix4d Arm::get_pose() {
	Matrix4d pose = Matrix4d::Identity();
	return sim->transform_from_to(pose, tool_frame, "base_link");
}

VectorJ Arm::get_joints() {
	std::vector<double> joint_values;
	sim->robot->GetDOFValues(joint_values, joint_indices);
	return VectorJ(joint_values.data());
}

void Arm::get_joint_limits(VectorJ& l, VectorJ& u) {
	l = lower;
	u = upper;
}

/**
 * Arm fk/ik
 */

Matrix4d Arm::fk(const VectorJ& joints) {
	rave::Transform pose_mat = fk_origin;

	rave::Transform R;
	for(int i=0; i < ARM_DIM; ++i) {
		R.rot = rave::geometry::quatFromAxisAngle(arm_joint_axes[i], joints(i));
		R.trans = arm_link_trans[i];
		pose_mat = pose_mat*R;
	}

	return rave_to_eigen(pose_mat);

//	VectorJ curr_joints = get_joints();
//
//	set_joints(joints);
//	Matrix4d pose = get_pose();
//
//	set_joints(curr_joints);
//	return pose;
}


/**
 * \param pose desired pose of tool_frame in frame "base_link"
 */
bool Arm::ik(const Matrix4d& pose, VectorJ& joints) {
	Matrix4d pose_world = sim->transform_from_to(pose, "base_link", "world");

	std::vector<double> joint_vec;
	if (sim->ik_for_link(pose_world, manip, tool_frame, joint_vec)) {
		joints = VectorJ(joint_vec.data()); // TODO: add closer_angle
		sim->update();
//		joints = closer_joint_angles(joints, get_joints());
		return true;
	} else {
		return false;
	}
}

/**
 * \brief Finds a rotation matrix that looks at point from position, then calls usual IK
 *        (will not always find a solution, even when it should)
 * \param position desired gripper position in frame "base_link"
 * \param point 3d point to look at in frame "base_link"
 */
bool Arm::ik_lookat(const Vector3d& position, const Vector3d& point, VectorJ& joints) {
	Vector3d position_world = sim->transform_from_to(position, "base_link", "world");
	Matrix4d eye = Matrix4d::Identity();
	Vector3d direction_world = sim->transform_from_to(eye, "base_link", "world").block<3,3>(0,0)*(point - position);
	direction_world.normalize();

	Vector3d red, green, blue;
	red = direction_world;
	red.normalize();
	green = red.cross(Vector3d(0,0,1));
	green.normalize();
	blue = red.cross(green);
	blue.normalize();

	Matrix4d pose_world = Matrix4d::Identity();
	pose_world.block<3,3>(0,0) << red, green, blue;
	pose_world.block<3,1>(0,3) = position_world;

	Matrix4d pose_base = sim->transform_from_to(pose_world, "world", "base_link");
	return ik(pose_base, joints);
}

/**
 *
 * Arm private methods
 *
 */



}
