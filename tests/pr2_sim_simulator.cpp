#include "pr2_utils/pr2_sim/simulator.h"

#include <iostream>

int main() {
	pr2_sim::Simulator sim(true, false);

	sim.plot_point(rave::Vector(1,1,0), rave::Vector(1,0,0), .1);

	sim.plot_segment(Vector3d(0,0,0), {1,1,0}, {0,1,0});

	sim.plot_triangle(Vector3d(0,0,0), {1,0,0}, {1,1,0}, {0,0,1}, 0.25);

	Vector3d mean(1,1,1);
	Matrix3d cov;
	cov << 1, 0,  0,
		   0, .1, 0,
		   0, 0,  .1;
	sim.plot_gaussian(mean, cov, {0,0,1});

	Matrix4d T = Matrix4d::Identity();
	T.block<3,1>(0,3) = Vector3d(1,0,0);
	sim.plot_transform(T);

	std::cout << "Press enter to exit\n";
	std::cin.ignore();
}
