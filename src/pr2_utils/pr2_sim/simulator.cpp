#include "pr2_utils/pr2_sim/simulator.h"

#include <ros/package.h>

namespace pr2_sim {

/**
 *
 * Simulator constructors
 *
 */

void SetViewer(rave::EnvironmentBasePtr penv, const std::string& viewername)
{
    rave::ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->Add(viewer);

    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

Simulator::Simulator(std::string env_file, bool view, bool use_ros) : use_ros(use_ros) {
	if (env_file == "") {
		std::string pkg_path("pr2_utils");
		env_file = ros::package::getPath(pkg_path) + "/robots/my-pr2-beta-sim.robot.xml";
	}

	rave::RaveInitialize(true, rave::Level_Info);
	env = rave::RaveCreateEnvironment();
	env->Load(env_file);

	std::vector<rave::RobotBasePtr> robots;
	env->GetRobots(robots, 0);
	assert(robots.size() > 0);
	robot = robots[0];

	if (view) {
		viewer_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(SetViewer, env, "qtcoin")));
	}

	if (use_ros) {
		nh_ptr = new ros::NodeHandle();
		joint_state_sub = nh_ptr->subscribe("/joint_states", 1, &Simulator::_joint_state_callback, this);

		received_joint_state = false;
		ROS_INFO("Waiting for first joint state in simulator...");
		while (!received_joint_state && !ros::isShuttingDown()) {
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		}
	}

	sleep(1);
}

/**
 *
 * Simulator public methods
 *
 */

/**
 * \brief Updates robot joints to match ROS /joint_states
 */
void Simulator::update() {
	if (!use_ros) {
		ROS_INFO("Cannot update, Simulator instantiated not using ROS");
		return;
	}
	if (!received_joint_state) {
		ROS_INFO("Cannot update simulator, no joint state message received");
		return;
	}

	sensor_msgs::JointState msg = last_joint_state;

	std::vector<int> indices;
	std::vector<double> joint_values;
	for(int i=0; i < msg.name.size(); ++i) {
		std::string &name = msg.name[i];
		double &joint_value = msg.position[i];

		for(const rave::KinBody::JointPtr& joint : robot->GetJoints()) {
			if (joint->GetName() == name) {
				indices.push_back(joint->GetDOFIndex());
				joint_values.push_back(joint_value);
				break;
			}
		}
	}

	std::vector<double> lower_limits, upper_limits;
	robot->GetDOFLimits(lower_limits, upper_limits, indices);
	for(int i=0; i < indices.size(); ++i) {
		joint_values[i] = std::max(joint_values[i], lower_limits[i]);
		joint_values[i] = std::min(joint_values[i], upper_limits[i]);
	}

	robot->SetDOFValues(joint_values, 0, indices);
}

/**
 * \param pose 4x4 matrix
 * \param ref_link_name frame of pose
 * \param targ_link_name desired new frame
 */
Matrix4d Simulator::transform_from_to(const Matrix4d& pose, const std::string ref_link_name, const std::string targ_link_name) {
	Matrix4d ref_from_world;
	if (ref_link_name != "world") {
		ref_from_world = rave_to_eigen(robot->GetLink(ref_link_name)->GetTransform());
	} else {
		ref_from_world.setIdentity();
	}

	Matrix4d targ_from_world;
	if (targ_link_name != "world") {
		targ_from_world = rave_to_eigen(robot->GetLink(targ_link_name)->GetTransform());
	} else {
		targ_from_world.setIdentity();
	}

	Matrix4d targ_from_ref = targ_from_world.inverse()*ref_from_world;
	Matrix4d mat_in_targ = targ_from_ref*pose;
	return mat_in_targ;
}

Vector3d Simulator::transform_from_to(const Vector3d& position, const std::string ref_link_name, const std::string targ_link_name) {
	Matrix4d pose = Matrix4d::Identity();
	pose.block<3,1>(0,3) = position;
	return transform_from_to(pose, ref_link_name, targ_link_name).block<3,1>(0,3);
}

/**
 * \brief Perform IK for an arbitrary link attached to the manipulator
 *        e.g. you might want IK for PR2 "r_gripper_tool_frame" instead of the openrave EE frame
 * \param pose 4x4 matrix "world frame from link frame"
 * \param manip
 * \param link_name frame of pose
 * \param joint_values return value
 * \return True if IK successful
 */
bool Simulator::ik_for_link(const Matrix4d& pose, const rave::RobotBase::ManipulatorPtr manip, const std::string link_name,
			std::vector<double>& joint_values) {
	rave::KinBody::LinkPtr link = robot->GetLink(link_name);

	if (!robot->DoesAffect(manip->GetArmIndices().back(), link->GetIndex())) {
		std::cout << "Link " << link_name << " is not attached to the end effector of the manipulator " << manip->GetName() << "\n";
		return false;
	}

	rave::Transform link_pose_mat = link->GetTransform();
	rave::Transform ee_pose_mat = manip->GetEndEffectorTransform();
	rave::Transform link_to_ee = link_pose_mat.inverse()*ee_pose_mat;

	rave::Transform ee_from_world = eigen_to_rave(pose)*link_to_ee;

	bool success = manip->FindIKSolution(rave::IkParameterization(ee_from_world),
			joint_values, rave::IkFilterOptions::IKFO_CheckEnvCollisions);

	return success;
}

/**
 *
 * Plotting methods
 *
 */

void Simulator::clear_plots(int num) {
	if ((num < 0) || (num > handles.size())) {
		handles.clear();
	} else {
		handles = std::vector<rave::GraphHandlePtr>(handles.begin(), handles.end()-num);
	}
}

void Simulator::plot_point(rave::Vector pos, rave::Vector color, float size) {
	float *ppoints = new float[3];
	ppoints[0] = pos.x;
	ppoints[1] = pos.y;
	ppoints[2] = pos.z;

	color.w = 1;

	handles.push_back(env->plot3(ppoints, 1, sizeof(float), size, color, 1));
}


void Simulator::plot_point(Vector3d pos, Vector3d color, float size) {
	rave::Vector pos_vec = eigen_to_rave(pos);
	rave::Vector color_vec = eigen_to_rave(color);
	plot_point(pos_vec, color_vec, size);
}

void Simulator::plot_segment(Vector3d p0, Vector3d p1, Vector3d color) {
	float points[6];
	points[0] = p0(0); points[1] = p0(1); points[2] = p0(2);
	points[3] = p1(0); points[4] = p1(1); points[5] = p1(2);
	float c[6];
	for(int i=0; i < 3; ++i) { c[i] = color(i); }
	for(int i=0; i < 3; ++i) { c[i+3] = color(i); }
	handles.push_back(env->drawlinestrip(points, 2, sizeof(float)*3, 3.0f, c));
}

void Simulator::plot_segment(rave::Vector p0, rave::Vector p1, Vector3d color) {
	plot_segment(rave_to_eigen(p0), rave_to_eigen(p1), color);
}

void Simulator::plot_triangle(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d color, double alpha) {
	float points[9];
	points[0] = p0(0); points[1] = p0(1); points[2] = p0(2);
	points[3] = p1(0); points[4] = p1(1); points[5] = p1(2);
	points[6] = p2(0); points[7] = p2(1); points[8] = p2(2);
	handles.push_back(env->drawtrimesh(points, sizeof(float)*3, NULL, 1, rave::Vector(color(0), color(1), color(2), alpha)));
}

void Simulator::plot_transform(rave::Transform T, float length) {
	Matrix4d M = rave_to_eigen(T);

	Vector3d o = M.block<3,1>(0,3);
	Matrix<double,6,3> I;
	I << Matrix3d::Identity(), Matrix3d::Identity();

	float ppoints[6];
	for(int i=0; i < 3; ++i) { ppoints[i] = o(i); }
	float colors[6];
	for(int i=0; i < 3; ++i) {
		Vector3d endpt = o + length*M.block<3,1>(0,i);
		for(int i=0; i < 3; ++i) { ppoints[i+3] = endpt(i); }

		Matrix<double,6,1> c = I.block<6,1>(0,i);
		for(int i=0; i < 6; ++i) { colors[i] = c(i); }
		handles.push_back(env->drawlinestrip(ppoints, 2, sizeof(float)*3, 10.0f, colors));
	}
}

void Simulator::plot_transform(Matrix4d T, float length) {
	plot_transform(eigen_to_rave(T), length);
}

void Simulator::plot_gaussian(Vector3d mean, Matrix3d cov, Vector3d color) {
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,1.0);

	Matrix3d cov_chol = cov.llt().matrixL();
	for(int i=0; i < 1000; ++i) {
		Vector3d z = {distribution(generator), distribution(generator), distribution(generator)};
		z.normalize();
		Vector3d x = cov_chol*z + mean;
		plot_point(x, color, .002);
	}

	SelfAdjointEigenSolver<Matrix3d> eig(cov);
	Vector3d eigenvalues = eig.eigenvalues();
	Matrix3d eigenvectors = eig.eigenvectors();

	for(int i=0; i < 3; ++i) {
		Vector3d color = Vector3d::Zero();
		color(i) = 1;
		plot_segment(mean - (eigenvalues(i)/1.)*eigenvectors.col(i), mean + (eigenvalues(i)/1.)*eigenvectors.col(i), color);
	}
}


/**
 *
 * Simulator private methods
 *
 */

void Simulator::_joint_state_callback(const sensor_msgs::JointStateConstPtr& joint_state) {
	last_joint_state = *joint_state;
	received_joint_state = true;
}

}
