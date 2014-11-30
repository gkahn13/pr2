#ifndef __PR2_SIMULATOR_H__
#define __PR2_SIMULATOR_H__

#include "utils.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Eigen>
using namespace Eigen;

#include <openrave-core.h>
namespace rave = OpenRAVE;

#include <random>
#include <boost/thread/thread.hpp>

typedef Matrix<double, 7, 1> VectorJ; // Number of joints in PR2 arm

namespace pr2_sim {

class Simulator {
	friend class Arm;
public:
	Simulator(std::string env_file, bool view, bool use_ros);
	Simulator(bool view, bool use_ros) : Simulator("", view, use_ros) { }
	Simulator() : Simulator(false, false) { }

	void update();

	// frame methods
	Matrix4d transform_from_to(const Matrix4d& pose, const std::string ref_link_name, const std::string targ_link_name);
	Vector3d transform_from_to(const Vector3d& position, const std::string ref_link_name, const std::string targ_link_name);
	bool ik_for_link(const Matrix4d& pose, const rave::RobotBase::ManipulatorPtr manip, const std::string link_name,
			std::vector<double>& joint_values);

	// plotting methods
	void clear_plots(int num_to_clear=-1);

	void plot_point(rave::Vector pos, rave::Vector color, float size=.01);
	void plot_point(Vector3d pos, Vector3d color, float size=.01);

	void plot_segment(Vector3d p0, Vector3d p1, Vector3d color);
	void plot_segment(rave::Vector p0, rave::Vector p1, Vector3d color);

	void plot_triangle(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d color, double alpha=1.);

	void plot_transform(rave::Transform T, float length=0.1);
	void plot_transform(Matrix4d T, float length=0.1);

	void plot_gaussian(Vector3d mean, Matrix3d cov, Vector3d color);

private:
	rave::EnvironmentBasePtr env;
	rave::RobotBasePtr robot;
	boost::shared_ptr<boost::thread> viewer_thread;
	std::vector<rave::GraphHandlePtr> handles;

	bool use_ros;
	ros::NodeHandle *nh_ptr;
	ros::Subscriber joint_state_sub;
	sensor_msgs::JointState last_joint_state;
	bool received_joint_state;

	void _joint_state_callback(const sensor_msgs::JointStateConstPtr& joint_state);
};

}

#endif
