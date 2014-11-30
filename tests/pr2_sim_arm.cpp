#include "pr2_utils/pr2_sim/arm.h"

int main() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm rarm(pr2_sim::Arm::right, &sim);
	pr2_sim::Arm larm(pr2_sim::Arm::left, &sim);

//	arm.set_posture(pr2_sim::Arm::Posture::mantis);

	while(!ros::isShuttingDown()) {
		rarm.teleop();
		larm.teleop();

	}

//	std::cout << "Press enter to exit\n";
//	std::cin.ignore();
}
