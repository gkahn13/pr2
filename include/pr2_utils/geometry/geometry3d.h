#ifndef __GEOMETRY3D_H__
#define __GEOMETRY3D_H__

#include "geometry2d.h"
#include "pr2_utils/pr2_sim/simulator.h"

#include <vector>
#include <unordered_set>

#include <Eigen/Eigen>
using namespace Eigen;

namespace geometry3d {

const double epsilon = 1e-5;


/**
 * Classes to allow for use in unordered_set/sorting
 */


class PointHash {
public:
	long operator()(const Vector3d& x) const {
		return 0;
	}
};

class PointEqualTo {
public:
	bool operator()(const Vector3d& a, const Vector3d& b) const {
		return ((a-b).norm() < epsilon);
	}
};

class PointLessThan {
public:
	enum Coordinate {x=0, y=1};
	PointLessThan(Coordinate c) : c(c) { }

	bool operator()(const Vector3d& a, const Vector3d& b) const {
		return (a(c) < b(c));
	}

private:
	Coordinate c;
};


class Segment {
public:
	Vector3d p0, p1;

	Segment() { p0.setZero(); p1.setZero(); }
	Segment(const Vector3d& p0_pt, const Vector3d& p1_pt) : p0(p0_pt), p1(p1_pt) { }

	/**
	 * \brief Finds closest point on segment to x
	 *        min_{0<=t<=1} ||t*(p1-p0) + p0 - x||_{2}^{2}
	 */
	inline Vector3d closest_point_to(const Vector3d& x) const {
		Vector3d v = p1 - p0;
		Vector3d b = p0 - x;

		double t = -v.dot(b) / v.squaredNorm();
		if ((0 <= t) && (t <= 1)) {
			return t*(p1 - p0) + p0;
		} else {
			if ((x - p0).norm() < (x - p1).norm()) {
				return p0;
			} else {
				return p1;
			}
		}
	}

	/**
	 * TODO: doesn't work
	 * \brief Finds intersection point with another segment
	 * \param intersection stores intersection (if found)
	 * \return True if intersection found
	 */
	inline bool intersection(const Segment& other, Vector3d& intersection) const {
		// w = p1 - p0
		// v = other.p1 - other.p0
		// s*w + p0 = t*v + other.p0

		Vector3d w = p1 - p0;
		Vector3d v = other.p1 - other.p0;

		Matrix<double,2,3> A;
		A << w(0), v(0),
				w(1), v(1),
				w(2), v(2);
		Vector3d b = other.p0 - p0;

		if (fabs(A.determinant()) < epsilon) {
			return false;
		}

		Vector2d soln = A.lu().solve(b);
		double s = soln(0), t = -soln(1);

		intersection = s*w + p0;

		if ((-epsilon <= s) && (s <= 1+epsilon) && (-epsilon <= t) && (t <= 1+epsilon)) {
			return true;
		}
		return false;

	}

	/**
	 * \brief Finds the closest points on the two segments
	 *        but treats the segments as infinite lines
	 */
	inline void lines_closest_points(const Segment& other, Vector3d& closest, Vector3d& other_closest) const {
		Vector3d w = p1 - p0;
		Vector3d v = other.p1 - other.p0;

		Matrix<double,3,2> A;
		A << w(0), v(0),
				w(1), v(1),
				w(2), v(2);
		Vector3d b = other.p0 - p0;

		Vector2d soln = A.colPivHouseholderQr().solve(b);

//		Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//		Matrix<double,3,2> A_pinv = svd.matrixV()*svd.matrixU().transpose();
//		Vector2d soln = svd.solve(b);

//		Vector2d soln = A.lu().solve(b); // TODO: will this work?
		double s = soln(0), t = -soln(1);

		closest = s*w + p0;
		other_closest = t*v + other.p0;
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color) const {
		Vector3d p0_world = sim.transform_from_to(p0, frame, "world");
		Vector3d p1_world = sim.transform_from_to(p1, frame, "world");
		sim.plot_segment(p0_world, p1_world, color);
	}
};

class Hyperplane {
public:
	Hyperplane(const Vector3d& o, const Vector3d n) : origin(o), normal(n) { }

	/**
	 * \brief Find distance from hyperplane to point
	 */
	inline double distance_to(const Vector3d& point) const {
		return normal.dot(point - origin);
	}

	/**
	 * \brief Finds intersection point with another segment
	 * \param intersection stores intersection (if found)
	 * \return true if intersection found
	 */
	inline bool intersection(const Segment& segment, Vector3d& intersection) const {
		// x = t*(p1 - p0) + p0
		// n'*(x - origin) = 0
		//combine to get
		// n'*(t*(p1-p0) + p0 - origin) = 0
		// solve for t

		Vector3d v = segment.p1 - segment.p0;
		Vector3d w = segment.p0 - origin;
		double t = -normal.dot(w) / normal.dot(v);

		if ((0-epsilon <= t) && (t <= 1+epsilon)) {
			intersection = t*(segment.p1 - segment.p0) + segment.p0;
			return true;
		}

		return false;
	}

private:
	Vector3d origin, normal;
};


class Halfspace {
public:
	Vector3d origin, normal;

	Halfspace() : origin(Vector3d(0,0,-1e10)), normal(Vector3d(0,0,1)) { }
	Halfspace(const Vector3d& o, const Vector3d n) : origin(o), normal(n) { }

	/**
	 * \brief True if x is in the halfspace
	 */
	inline bool contains(const Vector3d& x) const {
		return (normal.dot(x - origin) >= 0);
	}

	/**
	 * \brief Clips segment against halfspace
	 * \return false if segment not in halfspace, else true
	 */
	inline bool clip_segment(const Segment& segment, Segment& clipped_segment) const {
		bool contains_p0 = contains(segment.p0);
		bool contains_p1 = contains(segment.p1);

		if ((!contains_p0) && (!contains_p1)) {
			return false;
		}

		if (contains_p0 && contains_p1) {
			clipped_segment = Segment(segment.p0, segment.p1);
		} else {
			Vector3d intersection;
			bool found_intersection = get_hyperplane().intersection(segment, intersection);
			assert(found_intersection);
			if (contains_p0) {
				clipped_segment = Segment(intersection, segment.p0);
			} else {
				clipped_segment = Segment(intersection, segment.p1);
			}
		}

		return true;
	}

	inline Hyperplane get_hyperplane() const {
		return Hyperplane(origin, normal);
	}

	inline Halfspace get_complement() const {
		return Halfspace(origin, -normal);
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color) const {
		Vector3d o = sim.transform_from_to(origin, frame, "world");
		Vector3d n = sim.transform_from_to(normal, frame, "world");
		sim.plot_segment(o, o + 0.5*n, color);
	}
};


class Triangle {
public:
	Vector3d a, b, c;

	Triangle() { a.setZero(); b.setZero(); c.setZero(); }
	Triangle(const Vector3d& a_pt, const Vector3d& b_pt, const Vector3d c_pt) : a(a_pt), b(b_pt), c(c_pt) { }

	/**
	 * \brief Finds the closest point on triangle to x
	 * 		  by rotating and projecting
	 * 		  then return the closest point unrotated
	 */
	inline Vector3d closest_point_to(const Vector3d& x) const {
		Vector3d align_target = {0,0,1};
		Matrix3d rotation = rotation_to_align_with(align_target);

		Vector3d a_rot, b_rot, c_rot, x_rot;
		a_rot = rotation*a;
		b_rot = rotation*b;
		c_rot = rotation*c;
		x_rot = rotation*x;

		double tri_rot_z = a_rot(2);

		Vector2d p_rot_2d;
		p_rot_2d << x_rot(0), x_rot(1);

		geometry2d::Triangle tri_2d = geometry2d::Triangle(a_rot.segment<2>(0), b_rot.segment<2>(0), c_rot.segment<2>(0));
		if (tri_2d.is_inside(p_rot_2d)) {
			// then distance is just the z of the rotated triangle, but with x,y from the point
			// then rotate back to original frame
			Vector3d p_rot_proj;
			p_rot_proj << p_rot_2d, tri_rot_z;
			return rotation.inverse()*p_rot_proj;
		} else {
			// find closest point on 2d triangle
			// connect back in 3d, then rotate back to original frame
			Vector2d closest_pt_2d;
			bool found_closest_pt = tri_2d.closest_point_to(p_rot_2d, closest_pt_2d);
			assert(found_closest_pt);
			Vector3d closest_pt_3d;
			closest_pt_3d << closest_pt_2d, tri_rot_z;
			return rotation.inverse()*closest_pt_3d;
		}
	}

	/**
	 * \brief Returns shortest distance to point x
	 */
	inline double distance_to(const Vector3d& x) const {
		return (closest_point_to(x) - x).norm();
	}

	/**
	 * \brief Finds intersection of segment with triangle by
	 *        finding intersection of segment with hyperplane and
	 *        if the intersection is in the triangle, return true
	 */
	inline bool intersection(const Segment& segment, Vector3d& intersection) const {
		bool found_intersection = get_hyperplane().intersection(segment, intersection);
		if (found_intersection && (intersection - closest_point_to(intersection)).norm() < epsilon) {
			return true;
		}

		return false;
	}

	inline Vector3d closest_point_on_segment(const Segment& segment) const {
		Vector3d point;
		Hyperplane hyperplane = get_hyperplane();
		if (!hyperplane.intersection(segment, point)) {
			if (distance_to(segment.p0) < distance_to(segment.p1)) {
				point = segment.p0;
			} else {
				point = segment.p1;
			}
		}

		return point;
	}

	inline double area() const {
		Matrix3d rotation = rotation_to_align_with({0,0,1});
		return geometry2d::triangle_area((rotation*a).segment<2>(0),
				(rotation*b).segment<2>(0),
				(rotation*c).segment<2>(0));
	}

	inline std::vector<Vector3d> get_vertices() const {
		return {a, b, c};
	}

	inline std::vector<Segment> get_segments() const {
		return {Segment(a, b), Segment(b, c), Segment(c, a)};
	}

	inline Hyperplane get_hyperplane() const {
		Vector3d origin = (a+b+c)/3.0;
		Vector3d normal = (a-b).cross(a-c);
		return Hyperplane(origin, normal);
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool fill=false, double alpha=0.25) const {
		Vector3d a_world = sim.transform_from_to(a, frame, "world");
		Vector3d b_world = sim.transform_from_to(b, frame, "world");
		Vector3d c_world = sim.transform_from_to(c, frame, "world");

		sim.plot_segment(a_world, b_world, color);
		sim.plot_segment(b_world, c_world, color);
		sim.plot_segment(c_world, a_world, color);

		if (fill) {
			sim.plot_triangle(a_world, b_world, c_world, color, alpha);
		}
	}

private:
	/**
	 * \brief Finds rotation to align the normal of this triangle to the target
	 * \param target align normal to this
	 * \return rotation rotation used to align
	 */
	Matrix3d rotation_to_align_with(const Vector3d& target) const {
		Vector3d source = (b-a).cross(c-a);
		source.normalize();

		Matrix3d rotation = Matrix3d::Identity();

		double dot = source.dot(target);
		if (!isnan(dot)) {
			double angle = acos(dot);
			if (!isnan(angle)) {
				Vector3d cross = source.cross(target);
				double cross_norm = cross.norm();
				if ((!isnan(cross_norm)) && (cross_norm > epsilon)) {
					cross /= cross_norm;
					rotation = Eigen::AngleAxis<double>(angle, cross).toRotationMatrix();
				}
			}
		}

		return rotation;
	}
};

class TruncatedPyramid {
	/**
	 * A truncated pyramid with origin triangle a0, b0, c0
     * and end triangle a1, b1, c1
     *
     *       a0          a1
     *      /  \        /  \
     *     /    \      /    \
     *    b0----c0    b1----c1
	 */
public:
	TruncatedPyramid() { };
	TruncatedPyramid(const Vector3d& a0, const Vector3d& a1,
			const Vector3d& b0, const Vector3d& b1,
			const Vector3d& c0, const Vector3d& c1) : a0(a0), a1(a1), b0(b0), b1(b1), c0(c0), c1(c1) { }
	TruncatedPyramid(const std::vector<Vector3d>& ps) :
		TruncatedPyramid(ps[0], ps[1], ps[2], ps[3], ps[4], ps[5]) { }

	/**
	 * \brief Checks if point p is inside by comparing against intersection of halfspaces
	 */
	inline bool is_inside(const Vector3d& p) const {
		for(const Halfspace& halfspace : get_halfspaces()) {
			if (!halfspace.contains(p)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * \brief Computes signed-distance by computing the sign via is_inside
	 *        and the distance using the min distance to the faces
	 */
	inline double signed_distance(const Vector3d& point) const {
		double sign = (is_inside(point)) ? -1 : 1;
		double dist = INFINITY;
		for(const Triangle& tri : get_faces()) {
			dist = std::min(dist, tri.distance_to(point));
		}

		return sign*dist;
	}

	inline std::vector<Halfspace> get_halfspaces() const {
		std::vector<Halfspace> halfspaces = {Halfspace((a0+b0+c0)/3.0,  (b0-c0).cross(a0-c0)), // # closer triangle
				                   Halfspace((a0+a1+c0+c1)/4.0, (a1-a0).cross(c0-a0)), // right side
				                   Halfspace((a0+a1+b0+b1)/4.0, (b1-b0).cross(a0-b0)), // left side
				                   Halfspace((b0+b1+c0+c1)/4.0, (c1-c0).cross(b0-c0)), // bottom side
				                   Halfspace((a1+b1+c1)/3.0, (a1-c1).cross(b1-c1))}; // further triangle

		Vector3d center = (a0+b0+c0+a1+b1+c1)/6.0;
		for(Halfspace& halfspace : halfspaces) {
			if (!halfspace.contains(center)) {
				halfspace.normal *= -1;
			}
		}

		return halfspaces;
	}

	inline std::vector<Triangle> get_faces() const {
		return {Triangle(a0, b0, c0),
            Triangle(a0, a1, c1),
            Triangle(a0, c0, c1),
            Triangle(a0, a1, b1),
            Triangle(a0, b0, b1),
            Triangle(b0, b1, c1),
            Triangle(b0, c0, c1),
            Triangle(a1, b1, c1)};
	}

	inline std::vector<Segment> get_side_segments() const {
		return {Segment(a0, a1), Segment(b0, b1), Segment(c0, c1)};
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool with_sides=false, bool fill=false, double alpha=0.25) const {
		Vector3d a0_world = sim.transform_from_to(a0, frame, "world");
		Vector3d b0_world = sim.transform_from_to(b0, frame, "world");
		Vector3d c0_world = sim.transform_from_to(c0, frame, "world");
		Vector3d a1_world = sim.transform_from_to(a1, frame, "world");
		Vector3d b1_world = sim.transform_from_to(b1, frame, "world");
		Vector3d c1_world = sim.transform_from_to(c1, frame, "world");

		if (with_sides) {
			sim.plot_segment(a0_world, a1_world, color);
			sim.plot_segment(b0_world, b1_world, color);
			sim.plot_segment(c0_world, c1_world, color);
		}

		sim.plot_segment(a0_world, b0_world, color);
		sim.plot_segment(b0_world, c0_world, color);
		sim.plot_segment(c0_world, a0_world, color);
		sim.plot_segment(a1_world, b1_world, color);
		sim.plot_segment(b1_world, c1_world, color);
		sim.plot_segment(c1_world, a1_world, color);

		if (fill) {
			if (with_sides) {
				sim.plot_triangle(a0_world, a1_world, c1_world, color, alpha);
				sim.plot_triangle(a0_world, c0_world, c1_world, color, alpha);
				sim.plot_triangle(a0_world, a1_world, b1_world, color, alpha);
				sim.plot_triangle(a0_world, b0_world, b1_world, color, alpha);
				sim.plot_triangle(b0_world, b1_world, c1_world, color, alpha);
				sim.plot_triangle(b0_world, c0_world, c1_world, color, alpha);
			}

			sim.plot_triangle(a0_world, b0_world, c0_world, color, alpha);
			sim.plot_triangle(a1_world, b1_world, c1_world, color, alpha);
		}
	}

private:
	Vector3d a0, a1, b0, b1, c0, c1;
};

class ViewFrustum {
	/**
	 * A truncated pyramid with origin rectangle a0, b0, c0, d0
     * and end rectangle a1, b1, c1, d1 arranged as
     *   b0 --- a0       b1 --- a1
     *   |      |        |      |
     *   |      |        |      |
     *   c0 --- d0       c1 --- d1
	 */
public:
	ViewFrustum(const Vector3d& a0, const Vector3d& a1,
			const Vector3d& b0, const Vector3d& b1,
			const Vector3d& c0, const Vector3d& c1,
			const Vector3d& d0, const Vector3d& d1) : a0(a0), a1(a1), b0(b0), b1(b1), c0(c0), c1(c1), d0(d0), d1(d1) { }

	/**
	 * \brief Checks if point p is inside by comparing against intersection of halfspaces
	 */
	inline bool is_inside(const Vector3d& p) const {
		for(const Halfspace& halfspace : get_halfspaces()) {
			if (!halfspace.contains(p)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * \brief Clips triangle against the faces (http://www.cs.uu.nl/docs/vakken/gr/2011/Slides/08-pipeline2.pdf)
	 */
	inline std::vector<Triangle> clip_triangle(const Triangle& triangle) const {
		std::vector<Triangle> triangles = {triangle};
		std::vector<Halfspace> halfspaces = get_halfspaces();
		for(int i=0; i < halfspaces.size(); ++i) {
			std::vector<Halfspace> splitting_halfspaces = {halfspaces[i]};
			if (i == 0) {
				splitting_halfspaces.push_back(halfspaces[i].get_complement());
			}

			// clip all triangles against the halfspace
			std::vector<Triangle> new_triangles;
			for (const Halfspace& halfspace : splitting_halfspaces) {
				for(const Triangle& tri : triangles) {
					std::vector<Segment> tri_segments = tri.get_segments();

					// clip triangle segments against halfspace
					std::vector<Segment> clipped_segments;
					for(const Segment& tri_seg : tri_segments) {
						Segment clipped_segment;
						if (halfspace.clip_segment(tri_seg, clipped_segment)) {
							clipped_segments.push_back(clipped_segment);
						}
					}

					if (clipped_segments.size() == 2) {
						// only one triangle left
						new_triangles.push_back(Triangle(clipped_segments[0].p0, clipped_segments[1].p0, clipped_segments[0].p1));
					} else if (clipped_segments.size() == 3) {
						// two triangles left
						// get segments that cross halfspace (i.e. ignore the one fully in the halfspace)
						std::vector<Segment> crossing_segments;
						for(int i=0; i < 3; ++i) {
							const Segment& tri_seg = tri_segments[i];
							if (!(halfspace.contains(tri_seg.p0) && halfspace.contains(tri_seg.p1))) {
								crossing_segments.push_back(clipped_segments[i]);
							}
						}

						assert(crossing_segments.size() == 0 || crossing_segments.size() == 2);

						if (crossing_segments.size() == 2) {
							new_triangles.push_back(Triangle(crossing_segments[0].p0, crossing_segments[0].p1, crossing_segments[1].p0));
							new_triangles.push_back(Triangle(crossing_segments[0].p1, crossing_segments[1].p0, crossing_segments[1].p1));
						} else {
							new_triangles.push_back(tri);
						}
					}
				}
			}

			triangles = new_triangles;
		}

		return triangles;
	}

	inline std::vector<Halfspace> get_halfspaces() const {
		std::vector<Halfspace> halfspaces = {Halfspace((a0+b0+c0+d0)/4.0, (b0-c0).cross(d0-c0)), // small rectangle
				Halfspace((a0+a1+d0+d1)/4.0, (a1-a0).cross(d0-a0)), // right side
				Halfspace((a0+a1+b0+b1)/4.0, (b0-a0).cross(a1-a0)), // top side
				Halfspace((b0+b1+c0+c1)/4.0, (c1-c0).cross(b0-c0)), // left side
				Halfspace((c0+c1+d0+d1)/4.0, (d1-d0).cross(c0-d0)), // bottom side
				Halfspace((a1+b1+c1+d1)/4.0, (a1-d1).cross(c1-d1))}; // big rectangle

		Vector3d center = (a0+b0+c0+d0+a1+b1+c1+d1)/8.0;
		for(Halfspace& halfspace : halfspaces) {
			if (!halfspace.contains(center)) {
				halfspace.normal *= -1;
			}
		}

		return halfspaces;
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool with_sides=false, bool fill=false, double alpha=0.25) const {
		Vector3d a0_world = sim.transform_from_to(a0, frame, "world");
		Vector3d b0_world = sim.transform_from_to(b0, frame, "world");
		Vector3d c0_world = sim.transform_from_to(c0, frame, "world");
		Vector3d d0_world = sim.transform_from_to(d0, frame, "world");
		Vector3d a1_world = sim.transform_from_to(a1, frame, "world");
		Vector3d b1_world = sim.transform_from_to(b1, frame, "world");
		Vector3d c1_world = sim.transform_from_to(c1, frame, "world");
		Vector3d d1_world = sim.transform_from_to(d1, frame, "world");

		if (with_sides) {
			sim.plot_segment(a0_world, a1_world, color);
			sim.plot_segment(b0_world, b1_world, color);
			sim.plot_segment(c0_world, c1_world, color);
			sim.plot_segment(d0_world, d1_world, color);
		}

		sim.plot_segment(a0_world, b0_world, color);
		sim.plot_segment(b0_world, c0_world, color);
		sim.plot_segment(c0_world, d0_world, color);
		sim.plot_segment(d0_world, a0_world, color);
		sim.plot_segment(a1_world, b1_world, color);
		sim.plot_segment(b1_world, c1_world, color);
		sim.plot_segment(c1_world, d1_world, color);
		sim.plot_segment(d1_world, a1_world, color);

		if (fill) {
			if (with_sides) {
				sim.plot_triangle(a0_world, a1_world, d1_world, color, alpha);
				sim.plot_triangle(a0_world, d0_world, d1_world, color, alpha);

				sim.plot_triangle(a0_world, a1_world, b1_world, color, alpha);
				sim.plot_triangle(a0_world, b0_world, b1_world, color, alpha);

				sim.plot_triangle(b0_world, b1_world, c1_world, color, alpha);
				sim.plot_triangle(b0_world, c0_world, c1_world, color, alpha);

				sim.plot_triangle(c0_world, c1_world, d1_world, color, alpha);
				sim.plot_triangle(c0_world, d0_world, d1_world, color, alpha);
			}

			sim.plot_triangle(a0_world, b0_world, c0_world, color, alpha);
			sim.plot_triangle(a0_world, c0_world, d0_world, color, alpha);

			sim.plot_triangle(a1_world, b1_world, c1_world, color, alpha);
			sim.plot_triangle(a1_world, c1_world, d1_world, color, alpha);
		}
	}

private:
	Vector3d a0, b0, c0, d0, a1, b1, c1, d1;
};


}

#endif
