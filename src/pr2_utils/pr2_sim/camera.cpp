#include "pr2_utils/pr2_sim/camera.h"

#include <unordered_set>

namespace pr2_sim {

/**
 *
 * RelativePyramid constructors/methods
 *
 */

RelativePyramid::RelativePyramid(const Matrix4d& cam_pose, Camera *cam,
		const std::vector<geometry3d::TruncatedPyramid>& truncated_frustum,
		const std::vector<geometry3d::Triangle>& obstacles, const Vector3d& sd_point) : cam(cam) {
	Matrix4d cam_pose_inv = cam_pose.inverse();

	geometry3d::TruncatedPyramid sd_pyramid3d;
	double sd = INFINITY;
	for(const geometry3d::TruncatedPyramid& pyramid3d : truncated_frustum) {
		double sd_tmp = pyramid3d.signed_distance(sd_point);
		if (sd_tmp < sd) {
			sd = sd_tmp;
			sd_pyramid3d = pyramid3d;
		}
	}

	for(const geometry3d::Segment& sd_seg3d : sd_pyramid3d.get_side_segments()) {
		bool found_clipping_tri3d = false;
		for(const geometry3d::Triangle& tri3d : obstacles) {
			for(const geometry3d::Segment& tri_seg3d : tri3d.get_segments()) {
				Vector3d tri_closest, sd_closest;
				tri_seg3d.lines_closest_points(sd_seg3d, tri_closest, sd_closest);
				double dist = (tri_closest - sd_closest).norm();

				if (dist < 1e-3) {
					points.push_back(tri_closest);
					point_frames.push_back("base_link");
					found_clipping_tri3d = true;
				}
				if (found_clipping_tri3d) { break; }
			}
			if (found_clipping_tri3d) { break; }
		}

		if (!found_clipping_tri3d) {
			points.push_back(cam_pose_inv.block<3,3>(0,0)*sd_seg3d.p1 + cam_pose_inv.block<3,1>(0,3));
			point_frames.push_back("camera_rgb_optical_frame");
		}
	}

}

geometry3d::TruncatedPyramid RelativePyramid::construct_pyramid(const Matrix4d& cam_pose) {
	std::vector<Vector3d> abs_points3d;
	for(int i=0; i < 3; ++i) {
		Vector3d pt = points[i];
		const std::string& frame = point_frames[i];

		assert(frame.find("base_link") != std::string::npos || frame.find("camera") != std::string::npos);

		if (frame.find("camera") != std::string::npos) {
			pt = cam_pose.block<3,3>(0,0)*pt + cam_pose.block<3,1>(0,3);
		}

		Vector2d pixel = cam->pixel_from_point(cam_pose, pt);
		geometry3d::Segment abs_seg3d = cam->segment_through_pixel(cam_pose, pixel);

		abs_points3d.push_back(abs_seg3d.p0);
		abs_points3d.push_back(abs_seg3d.p1);
	}

	return geometry3d::TruncatedPyramid(abs_points3d);
}

double RelativePyramid::signed_distance(const Matrix4d& cam_pose, const Vector3d& point) {
	return construct_pyramid(cam_pose).signed_distance(point);
}

/**
 *
 * Camera constructors
 *
 */

Camera::Camera(Arm* arm, Simulator* sim, double height, double width, double focal_length,
		double fx, double fy, double cx, double cy, double min_range, double max_range) :
		arm(arm), sim(sim), height(height), width(width), focal_length(focal_length),
		fx(fx), fy(fy), cx(cx), cy(cy), min_range(min_range), max_range(max_range) {

	tool_to_camera << 0.01946268, -0.00792993,  0.99977914, -0.11101548,
	        0.99311233, -0.11540324, -0.02024824, -0.01603762,
	        0.11553832,  0.99328707,  0.00562925, -0.07359911,
	        0.        ,  0.        ,  0.        ,  1.;

	P << fx, 0,  cx,
		 0,  fy, cy,
		 0,  0,  1;
}

/**
 * Camera state info methods
 */

Matrix4d Camera::get_pose() {
	return arm->get_pose()*tool_to_camera;
}

Matrix4d Camera::get_pose(const VectorJ& joints) {
	return arm->fk(joints)*tool_to_camera;
}

/**
 * Camera matrix methods
 */

/**
 * \brief Projects point onto image plane (no rounding or boundary checking)
 * \param point in frame "base_link"
 */
Vector2d Camera::pixel_from_point(const Matrix4d& cam_pose, const Vector3d& point) {
	Matrix4d point_pose = Matrix4d::Identity(), point_transform_cam;

	point_pose.block<3,1>(0,3) = point;
	point_transform_cam = cam_pose.lu().solve(point_pose);

	Vector3d y = P*point_transform_cam.block<3,1>(0,3);

	Vector2d pixel(y(1)/y(2), y(0)/y(2));
	return pixel;
}

/**
 * \brief Returns segment from camera origin through pixel
 */
geometry3d::Segment Camera::segment_through_pixel(const Matrix4d& cam_pose, const Vector2d& pixel, double start_dist) {
	Vector2d pixel_centered = pixel - Vector2d(height/2., width/2.);

	Vector3d pixel3d_centered_m_min = start_dist*Vector3d(pixel_centered(1)/fx, pixel_centered(0)/fy, 1);
	Vector3d pixel3d_centered_m_max = max_range*Vector3d(pixel_centered(1)/fx, pixel_centered(0)/fy, 1);

	Matrix3d rot = cam_pose.block<3,3>(0,0);
	Vector3d trans = cam_pose.block<3,1>(0,3);

	Vector3d p0 = (rot*pixel3d_centered_m_min + trans);
	Vector3d p1 = (rot*pixel3d_centered_m_max + trans);

	return geometry3d::Segment(p0, p1);
}

geometry3d::Segment Camera::segment_through_pixel(const Matrix4d& cam_pose, const Vector2d& pixel) {
	return segment_through_pixel(cam_pose, pixel, min_range);
}

/**
 * Calculate frustum methods
 */


/**
 * \brief Truncates view frustum against triangles3d
 * \param triangles3d
 * \param include_truncated_pyramids for signed-distance gradient, might not want to include truncated pyramids
 *                                   since they don't change because the environment is static (so gradient will be zero)
 */
std::vector<geometry3d::TruncatedPyramid> Camera::truncated_view_frustum(const Matrix4d& cam_pose,
		const std::vector<geometry3d::Triangle>& triangles3d, bool include_truncated_pyramids) {
	// clip triangles against view frustum
	geometry3d::Segment seg3d_a = segment_through_pixel(cam_pose, {0, width});
	geometry3d::Segment seg3d_b = segment_through_pixel(cam_pose, {0, 0});
	geometry3d::Segment seg3d_c = segment_through_pixel(cam_pose, {height, 0});
	geometry3d::Segment seg3d_d = segment_through_pixel(cam_pose, {height, width});

	geometry3d::ViewFrustum frustum(seg3d_a.p0, seg3d_a.p1,
			seg3d_b.p0, seg3d_b.p1,
			seg3d_c.p0, seg3d_c.p1,
			seg3d_d.p0, seg3d_d.p1);

	std::vector<geometry3d::Triangle> clipped_triangles3d;
	for(const geometry3d::Triangle& tri3d : triangles3d) {
		std::vector<geometry3d::Triangle> tri3d_clipped = frustum.clip_triangle(tri3d);
		clipped_triangles3d.insert(clipped_triangles3d.end(), tri3d_clipped.begin(), tri3d_clipped.end());
	}

	// project clipped triangles to 2d
	std::vector<geometry2d::Triangle> triangles2d = project_triangles(cam_pose, clipped_triangles3d);

	// initialize segments2d with segments from clipped_triangles2d
	std::unordered_set<geometry2d::Segment, geometry2d::SegmentHash, geometry2d::SegmentEqualTo> segments2d;
	for(const geometry2d::Triangle& tri2d : triangles2d) {
		for(const geometry2d::Segment& seg2d : tri2d.get_segments()) {
			segments2d.insert(seg2d);
		}
	}
	segments2d.insert({geometry2d::Segment({0,0},{height,0}),
			geometry2d::Segment({height,0},{height,width}),
			geometry2d::Segment({height,width},{0,width}),
			geometry2d::Segment({0,width},{0,0})}); // needed to add (even though didn't have to in python)

	std::unordered_set<Vector2d, geometry2d::PointHash, geometry2d::PointEqualTo> points2d;
	// initialize points2d with segments2d image plane corners
	points2d.insert({Vector2d(0,0), Vector2d(height,0), Vector2d(0,width), Vector2d(height,width)});
	// add segments2d points
	for(const geometry2d::Segment& seg2d : segments2d) {
		points2d.insert(seg2d.p0);
		points2d.insert(seg2d.p1);
	}
	// add segment intersections
	std::vector<geometry2d::Segment> segments2d_list(segments2d.begin(), segments2d.end());
	int segments2d_size = segments2d.size(); // had to do this outside of loop, weird...
	for(int i=0; i < segments2d_size-1; ++i) {
		const geometry2d::Segment& seg2d = segments2d_list[i];
		for(int j=i+1; j < segments2d_size; ++j) {
			const geometry2d::Segment& other_seg2d = segments2d_list[j];
			Vector2d intersection;
			if (seg2d.intersection(other_seg2d, intersection)) {
				points2d.insert(intersection);
			}
		}
	}

	// partition the image plane into non-intersecting triangles
	std::unordered_set<geometry2d::Triangle, geometry2d::TriangleHash, geometry2d::TriangleEqualTo> partition_triangles2d;
	for(const Vector2d& pt2d : points2d) {
		// find other points that don't cross anything in segments2d
		std::unordered_set<Vector2d, geometry2d::PointHash, geometry2d::PointEqualTo> points2d_in_los;
		for(const Vector2d& other_pt2d : points2d) {
			if (pt2d != other_pt2d) {
				geometry2d::Segment seg2d(pt2d, other_pt2d);
				int check_count = 0;
				for(const geometry2d::Segment& check_seg2d : segments2d) {
					Vector2d intersection;
					bool found_intersection = seg2d.intersection(check_seg2d, intersection);
					if (found_intersection && !seg2d.is_endpoint(intersection)) {
						break;
					}
					check_count++;
				}

				if (check_count == segments2d.size()) {
					// didn't intersect with any existing segment, so add it
					points2d_in_los.insert(other_pt2d);
				}
			}
		}

		// points2d_in_los --> segments2d_in_los
		std::vector<geometry2d::Segment> segments2d_in_los;
		for(const Vector2d& other_pt2d : points2d_in_los) {
			segments2d_in_los.push_back(geometry2d::Segment(pt2d, other_pt2d));
		}
		// sort segments by angle
		geometry2d::Segment seg2d_compare(Vector2d(0,0), Vector2d(-1,0));
		std::sort(segments2d_in_los.begin(), segments2d_in_los.end(), geometry2d::SegmentAngleCompare(seg2d_compare));

		// get new partition triangles
		std::unordered_set<geometry2d::Triangle, geometry2d::TriangleHash, geometry2d::TriangleEqualTo> new_partition_triangles2d;
		for(int i=0; i < segments2d_in_los.size()-1; ++i) {
			geometry2d::Segment tri_seg2d = geometry2d::Segment(segments2d_in_los[i].p1, segments2d_in_los[i+1].p1);
			// check top segment of triangle against segments2d
			int check_count = 0;
			for(const geometry2d::Segment& check_seg2d : segments2d) {
				Vector2d intersection;
				bool found_intersection = tri_seg2d.intersection(check_seg2d, intersection);
				if (found_intersection && !tri_seg2d.is_endpoint(intersection)) {
					break;
				}
				check_count++;
			}

			if (check_count == segments2d.size()) {
				geometry2d::Triangle tri2d(pt2d, tri_seg2d.p0, tri_seg2d.p1);
				if (tri2d.area() > geometry2d::epsilon) {
					new_partition_triangles2d.insert(geometry2d::Triangle(pt2d, tri_seg2d.p0, tri_seg2d.p1));
				}
			}
		}

		// update partition and new segments
		for(const geometry2d::Triangle& new_tri2d : new_partition_triangles2d) {
			partition_triangles2d.insert(new_tri2d);
			for(const geometry2d::Segment& new_seg2d : new_tri2d.get_segments()) {
				segments2d.insert(new_seg2d);
			}
		}

		double total_area = 0;
		for(const geometry2d::Triangle& tri2d : partition_triangles2d) {
			total_area += tri2d.area();
		}
		if (total_area >= width*height) {
			break;
		}

	}

	// now that we have tesselated the projection into triangles
	// find out which 3d triangle each projection belongs to
	// by shooting rays through the 2d triangles
	Vector3d camera_position = get_pose().block<3,1>(0,3);
	std::vector<geometry3d::TruncatedPyramid> pyramids3d;
	for(const geometry2d::Triangle& tri2d : partition_triangles2d) {
		// get 3d segments through triangle2d center and vertices
		geometry3d::Segment center_seg3d = segment_through_pixel(cam_pose, tri2d.get_center(), 0); // segment from camera origin
		std::vector<geometry3d::Segment> vertices_seg3d;
		for(const Vector2d& vertex : tri2d.get_vertices()) {
			vertices_seg3d.push_back(segment_through_pixel(cam_pose, vertex));
		}

		// find closest triangle that the center hits
		double min_dist = INFINITY;
		geometry3d::Triangle min_tri3d;
		for(const geometry3d::Triangle& tri3d : triangles3d) {
			Vector3d intersection;
			if (tri3d.intersection(center_seg3d, intersection)) {
				double dist = (intersection - camera_position).norm();
				if (dist < min_dist) {
					min_dist = dist;
					min_tri3d = tri3d;
				}
			}
		}

		// create pyramid from 3d intersections
		if (min_dist == INFINITY) {
			// no intersection, so pyramid of segments with length max_range
			pyramids3d.push_back(geometry3d::TruncatedPyramid(vertices_seg3d[0].p0, vertices_seg3d[0].p1,
					vertices_seg3d[1].p0, vertices_seg3d[1].p1,
					vertices_seg3d[2].p0, vertices_seg3d[2].p1));
		} else if (min_dist >= min_range) {
			if (include_truncated_pyramids) {
				std::vector<Vector3d> tri3d_intersections;
				for(const geometry3d::Segment& vertex_seg3d : vertices_seg3d) {
					tri3d_intersections.push_back(min_tri3d.closest_point_on_segment(vertex_seg3d));
				}

				pyramids3d.push_back(geometry3d::TruncatedPyramid(vertices_seg3d[0].p0,	tri3d_intersections[0],
						vertices_seg3d[1].p0, tri3d_intersections[1],
						vertices_seg3d[2].p0, tri3d_intersections[2]));
			}
		}
	}


	return pyramids3d;
}

/**
 * \brief Projects triangles onto image plane and returns 2d triangles
 */
std::vector<geometry2d::Triangle> Camera::project_triangles(const Matrix4d& cam_pose, const std::vector<geometry3d::Triangle>& triangles3d) {
	std::vector<geometry2d::Triangle> triangles2d;

	Vector2d a_proj, b_proj, c_proj;
	for(const geometry3d::Triangle& triangle3d : triangles3d) {
		a_proj = pixel_from_point(cam_pose, triangle3d.a);
		b_proj = pixel_from_point(cam_pose, triangle3d.b);
		c_proj = pixel_from_point(cam_pose, triangle3d.c);

		triangles2d.push_back(geometry2d::Triangle(a_proj, b_proj, c_proj));
	}

	return triangles2d;
}

bool Camera::is_in_fov(const Vector3d& point, const std::vector<geometry3d::TruncatedPyramid>& truncated_frustum) {
	for(const geometry3d::TruncatedPyramid& pyramid3d : truncated_frustum) {
		if (pyramid3d.is_inside(point)) {
			return true;
		}
	}
	return false;
}

double Camera::signed_distance(const Vector3d& point, const std::vector<geometry3d::TruncatedPyramid>& truncated_frustum) {
	double sd = INFINITY;
	for(const geometry3d::TruncatedPyramid& pyramid3d : truncated_frustum) {
		sd = std::min(sd, pyramid3d.signed_distance(point));
	}
	return sd;
}

/**
 * \brief Returns radial distance error according to http://www.mdpi.com/1424-8220/12/2/1437/htm
 * \return error roughly in meters (roughly because depends on focal_length)
 */
double Camera::radial_distance_error(const Matrix4d& cam_pose, const Vector3d& point) {
	Vector2d pixel = pixel_from_point(cam_pose, point);
	Vector2d pixel_centered = pixel - Vector2d(height,width)/2.0;
	Vector2d pixel_centered_mm = pixel_centered*focal_length;
	double error = pixel_centered_mm.transpose()*Vector2d(.007, .01).asDiagonal()*pixel_centered_mm;
	return error;
}

/**
 * \brief Returns standard deviations (in camera frame) of measuring point according to http://www.mdpi.com/1424-8220/12/2/1437/htm
 * \param point 3d point in frame base_link
 */
Vector3d Camera::measurement_standard_deviation(const Matrix4d& cam_pose, const Vector3d& point) {
	Matrix4d cam_pose_inv = cam_pose.inverse();
	Matrix3d cam_rot_inv = cam_pose_inv.block<3,3>(0,0);
	Vector3d cam_trans_inv = cam_pose_inv.block<3,1>(0,3);
	Vector3d point_cam = cam_rot_inv*point + cam_trans_inv;
	double Z_squared = point_cam(2)*point_cam(2);

	Vector2d pixel_centered = pixel_from_point(cam_pose, point) - Vector2d(height,width)/2.0;

	Vector3d sigmas;
	sigmas(0) = (fabs(pixel_centered(1))/(fy*fy))*Z_squared;
	sigmas(1) = (fabs(pixel_centered(0))/(fx*fx))*Z_squared;
	sigmas(2) = (1/fx)*Z_squared;
	return fx*sigmas; // fx so it is in terms of meters
}

void Camera::plot(Vector3d color, std::string frame, bool fill, bool with_sides, double alpha) {
	Matrix4d cam_pose = get_pose();
	geometry3d::Segment seg3d_a = segment_through_pixel(cam_pose, {0, width});
	geometry3d::Segment seg3d_b = segment_through_pixel(cam_pose, {0, 0});
	geometry3d::Segment seg3d_c = segment_through_pixel(cam_pose, {height, 0});
	geometry3d::Segment seg3d_d = segment_through_pixel(cam_pose, {height, width});

	geometry3d::ViewFrustum frustum(seg3d_a.p0, seg3d_a.p1,
			seg3d_b.p0, seg3d_b.p1,
			seg3d_c.p0, seg3d_c.p1,
			seg3d_d.p0, seg3d_d.p1);

	frustum.plot(*sim, frame, color, with_sides, fill, alpha);
}

}
