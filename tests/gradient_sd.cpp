#include "pr2_utils/pr2_sim/camera.h"
#include "pr2_utils/pr2_sim/arm.h"
#include "pr2_utils/pr2_sim/simulator.h"
#include "pr2_utils/utils/timer.h"

double signed_distance(const VectorJ& j, const Vector3d& point, const std::vector<geometry3d::Triangle>& triangles3d,
		pr2_sim::Arm& arm, pr2_sim::Camera& cam, pr2_sim::Simulator& sim, bool plot) {
	std::vector<geometry3d::TruncatedPyramid> truncated_frustum = cam.truncated_view_frustum(cam.get_pose(j), triangles3d, false);
	if (plot) {
		for(const geometry3d::TruncatedPyramid& pyramid : truncated_frustum) {
			pyramid.plot(sim, "base_link", {0,1,0}, true, true, 0.1);
		}
	}
	return cam.signed_distance(point, truncated_frustum);
}

VectorJ signed_distance_grad(const VectorJ& j, const Vector3d& point, const std::vector<geometry3d::Triangle>& triangles3d,
		pr2_sim::Arm& arm, pr2_sim::Camera& cam, pr2_sim::Simulator& sim) {
	const double step = 1e-5;
	VectorJ grad;

	VectorJ j_p = j, j_m = j;
	for(int i=0; i < j.rows(); ++i) {
		j_p(i) += step;
		j_m(i) -= step;

		double sd_p = signed_distance(j_p, point, triangles3d, arm, cam, sim, false);
		double sd_m = signed_distance(j_m, point, triangles3d, arm, cam, sim, false);
		grad(i) = (sd_p - sd_m) / (2*step);

		j_p(i) = j(i);
		j_m(i) = j(i);
	}

	return grad;
}

VectorJ fast_signed_distance_grad(const VectorJ& j, const Vector3d& point, const std::vector<geometry3d::Triangle>& triangles3d,
		pr2_sim::Arm& arm, pr2_sim::Camera& cam, pr2_sim::Simulator& sim) {

	std::vector<geometry3d::TruncatedPyramid> truncated_frustum = cam.truncated_view_frustum(cam.get_pose(j), triangles3d, false);
	pr2_sim::RelativePyramid rel_pyramid(cam.get_pose(j), &cam, truncated_frustum, triangles3d, point);

	const double step = 1e-5;
	VectorJ grad;

	VectorJ j_p = j, j_m = j;
	for(int i=0; i < j.rows(); ++i) {
		j_p(i) += step;
		j_m(i) -= step;

		double sd_p = rel_pyramid.signed_distance(cam.get_pose(j_p), point);
		double sd_m = rel_pyramid.signed_distance(cam.get_pose(j_m), point);
		grad(i) = (sd_p - sd_m) / (2*step);

		j_p(i) = j(i);
		j_m(i) = j(i);
	}

	return grad;
}


void test_gradient_sd() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm arm(pr2_sim::Arm::right, &sim);
	arm.set_posture(pr2_sim::Arm::Posture::mantis);

	pr2_sim::Camera cam(&arm, &sim);
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.7,0,.8}, {.7,0,1.1}, {.7,-.3,.7})};
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.5,0,.5}, {.8,0,.6}, {.5,-.3,.9})};
	Vector3d table_center(.2,.7,.5);
	std::vector<geometry3d::Triangle> triangles3d = {
//			geometry3d::Triangle(table_center, table_center+Vector3d(.5,-1.4,0), table_center+Vector3d(.5,0,0)),
//			geometry3d::Triangle(table_center, table_center+Vector3d(0,-1.4,0), table_center+Vector3d(.5,-1.4,0)),
			geometry3d::Triangle(table_center+Vector3d(.25,-.7,0), table_center+Vector3d(.25,-.7,.2), table_center+Vector3d(.25,-1.2,0)),
			geometry3d::Triangle(table_center+Vector3d(.25,-1.2,0), table_center+Vector3d(.25,-.7,.2), table_center+Vector3d(.25,-1.2,.2))};

//	Vector3d point(0.640, -0.170, 0.560);
	Vector3d point(0.500, -0.170, 0.560);
	VectorJ j = arm.get_joints();

	pr2_utils::Timer timer;
	while(true) {
		sim.clear_plots();
		sim.plot_point(sim.transform_from_to(point, "base_link", "world"), {0,0,1}, .02);
		for(const geometry3d::Triangle& tri3d : triangles3d) {
			tri3d.plot(sim, "base_link", {1,0,0}, true, 0.25);
		}

		double sd = signed_distance(j, point, triangles3d, arm, cam, sim, false);

		pr2_utils::Timer_tic(&timer);
		VectorJ sd_grad = signed_distance_grad(j, point, triangles3d, arm, cam, sim);
		double sd_grad_time = pr2_utils::Timer_toc(&timer);

		pr2_utils::Timer_tic(&timer);
		VectorJ sd_grad_fast = fast_signed_distance_grad(j, point, triangles3d, arm, cam, sim);
		double sd_grad_fast_time = pr2_utils::Timer_toc(&timer);

		std::cout << "sd: " << sd << "\n";
		std::cout << "sd_grad: " << sd_grad.transpose() << "\n";
		std::cout << "sd_grad_fast: " << sd_grad_fast.transpose() << "\n";
		std::cout << "sd_grad diff: " << (sd_grad - sd_grad_fast).norm() << "\n";
		std::cout << "sd_grad time: " << 1e3*sd_grad_time << " ms\n";
		std::cout << "sd_grad_fast time: " << 1e3*sd_grad_fast_time << " ms\n";


		std::cout << "Press enter to continue\n";
		std::cin.ignore();

		j -= (M_PI/64.)*(sd_grad_fast/sd_grad_fast.norm());
		arm.set_joints(j);
	}

}


int main() {
	test_gradient_sd();
}

