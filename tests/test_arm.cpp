#include "pr2_utils/pr2/arm.h"

#include "pr2_utils/pr2_sim/simulator.h"
#include "pr2_utils/pr2_sim/arm.h"
#include "pr2_utils/pr2_sim/camera.h"

void test_movement() {
	pr2::Arm arm(pr2::Arm::ArmType::right);

	std::cout << "Going to mantis\n";
	arm.go_to_posture(pr2::Arm::Posture::mantis);
	arm.teleop();

	std::cout << "Current joints: " << arm.get_joints().transpose() << "\n";
	std::cout << "Current pose:\n" << arm.get_pose() << "\n";

	std::cout << "Press enter to exit\n";
	std::cin.ignore();

}

void test_match_transforms() {
	// simulator
	pr2_sim::Simulator sim(true, true);
	pr2_sim::Arm arm_sim(pr2_sim::Arm::ArmType::right, &sim);
	pr2_sim::Camera cam_sim(&arm_sim, &sim);

	// real
	pr2::Arm arm(pr2::Arm::ArmType::right);

	// check if match
	sim.update();
	Matrix4d real_arm_pose = arm.get_pose();
	Matrix4d rave_arm_pose = arm_sim.get_pose();

	std::cout << "real_arm_pose:\n" << real_arm_pose << "\n";
	std::cout << "rave_arm_pose:\n" << rave_arm_pose << "\n";

	Matrix4d rave_cam_pose = cam_sim.get_pose();

	std::cout << "rave_cam_pose:\n" << rave_cam_pose << "\n";
}

void print_joints() {
	pr2::Arm arm(pr2::Arm::ArmType::right);

	while(!ros::isShuttingDown()) {
		std::cout << arm.get_joints().transpose() << "\n";
		ros::Duration(.5).sleep();
	}
}

void test_gripper() {
	pr2::Arm arm(pr2::Arm::ArmType::right);
	ros::Duration(2).sleep();

	std::cout << "Closing gripper\n";
	arm.close_gripper();

	std::cout << "Opening gripper\n";
	arm.open_gripper();
}

int main(int argc, char** argv) {
	// Init the ROS node
	ros::init(argc, argv, "test_arm");
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);


	test_movement();
//	test_match_transforms();
//	print_joints();
//	test_gripper();
}
