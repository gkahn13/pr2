#ifndef __GEOMETRY2D_H__
#define __GEOMETRY2D_H__

#include <vector>

#include <Eigen/Eigen>
using namespace Eigen;

namespace geometry2d {

const double epsilon = 1e-5;

class Halfspace {
public:
	Halfspace(const Vector2d& o, const Vector2d& n) : origin(o), normal(n) { }

	/**
	 * \brief True if x is in the halfspace
	 */
	inline bool contains(const Vector2d& x) const {
		return (normal.dot(x - origin) >= epsilon);
	}

	// TODO: add plotting

private:
	Vector2d origin, normal;
};


class Segment {
public:
	Vector2d p0, p1;

	Segment(const Vector2d& p0_pt, const Vector2d& p1_pt) : p0(p0_pt), p1(p1_pt) { }

	/**
	 * \brief Finds closest point on segment to x
	 *        min_{0<=t<=1} ||t*(p1-p0) + p0 - x||_{2}^{2}
	 */
	inline Vector2d closest_point_to(const Vector2d& x) const {
		Vector2d v = p1 - p0;
		Vector2d b = p0 - x;

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
	 * \brief Finds intersection point with another segment
	 * \param intersection stores intersection (if found)
	 * \return True if intersection found
	 */
	inline bool intersection(const Segment& other, Vector2d& intersection) const {
		// for speed, check if axis-aligned separation
		if ((std::max(p0(0), p1(0)) < std::min(other.p0(0), other.p1(0))) ||
				(std::max(other.p0(0), other.p1(0)) < std::min(p0(0), p1(0))) ||
				(std::max(p0(1), p1(1)) < std::min(other.p0(1), other.p1(1))) ||
				(std::max(other.p0(1), other.p1(1)) < std::min(p0(1), p1(1)))) {
			return false;
		}

		// w = p1 - p0
		// v = other.p1 - other.p0
		// s*w + p0 = t*v + other.p0

		Vector2d w = p1 - p0;
		Vector2d v = other.p1 - other.p0;

		Matrix2d A;
		A << w(0), v(0),
				w(1), v(1);
		Vector2d b = other.p0 - p0;

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
	 * \brief Returns vector angle between this and other
	 */
	inline double angle(const Segment& other) const {
		Vector2d a = p1 - p0;
		Vector2d b = other.p1 - other.p0;

		return (atan2(a(0)*b(1) - a(1)*b(0), a(0)*b(0) + a(1)*b(1)));
	}

	inline bool is_endpoint(const Vector2d& point) const {
		return (((p0 - point).norm() < epsilon) || ((p1 - point).norm() < epsilon));
	}

	inline bool is_parallel(const Segment& other) const {
		Vector2d slope = p1 - p0;
		slope.normalize();

		Vector2d other_slope = other.p1 - other.p0;
		other_slope.normalize();

		return (((slope - other_slope).norm() < epsilon) || ((slope + other_slope).norm() < epsilon));
	}

	// TODO: add plotting

	// TODO: add operator for comparison
};

inline double triangle_area(const Vector2d& a, const Vector2d& b, const Vector2d& c) {
	return fabs((c(0)*(a(1) - b(1)) + a(0)*(b(1) - c(1)) + b(0)*(c(1) - a(1))) / 2.0);
}

class Triangle {
public:
	Vector2d a, b, c;

	Triangle(const Vector2d& a_pt, const Vector2d& b_pt, const Vector2d& c_pt) : a(a_pt), b(b_pt), c(c_pt) { }

	/**
	 * \brief Finds the closest point on triangle to x
	 * 		  by checking distances to the triangle edges
	 * \param x
	 * \param closest stores closest point to x (if x is not inside the triangle)
	 * \return true if x is not inside the triangle
	 */
	inline bool closest_point_to(const Vector2d& x, Vector2d& closest) const {
		if (is_inside(x)) {
			return false;
		}

		double closest_dist = INFINITY;

		for(const Segment& s : get_segments()) {
			Vector2d s_min_pt = s.closest_point_to(x);
			double dist = (x - s_min_pt).norm();
			if (dist < closest_dist) {
				closest_dist = dist;
				closest = s_min_pt;
			}
		}

		return true;
	}

	/**
	 * \brief Determines if x is inside the triangle
	 */
	inline bool is_inside(const Vector2d& x) const {
		double total_area = area();
		double area0 = triangle_area(a, b, x);
		double area1 = triangle_area(b, c, x);
		double area2 = triangle_area(c, a, x);

		return (fabs(total_area - (area0 + area1 + area2)) < epsilon);
	}

	inline double area() const {
		return triangle_area(a, b, c);
	}

	inline Vector2d get_center() const {
		return ((a+b+c)/3.0);
	}

	inline std::vector<Vector2d> get_vertices() const {
		return {a, b, c};
	}

	inline std::vector<Segment> get_segments() const {
		return {Segment(a, b), Segment(b, c), Segment(c, a)};
	}

	/**
	 * \brief Halfspaces of edges point outwards
	 */
	inline std::vector<Halfspace> get_halfspaces() const {
		std::vector<Segment> segments = get_segments();
		std::vector<Vector2d> other_points = {c, a, b};

		std::vector<Halfspace> halfspaces;
		for(int i=0; i < 3; ++i) {
			Vector2d origin = (segments[i].p0 + segments[i].p1) / 2.0;

			Vector2d colinear = segments[i].p1 - segments[i].p0;
			Vector2d normal = {-colinear(1), colinear(0)};
			if (normal.dot(other_points[i] - origin) < 0) {
				halfspaces.push_back(Halfspace(origin, normal));
			} else {
				halfspaces.push_back(Halfspace(origin, -normal));
			}
		}

		return halfspaces;
	}

	// TODO: add plotting

	// TODO: add operator for comparison
};


/**
 * Classes to allow for use in unordered_set/sorting
 */

class PointHash {
public:
	long operator()(const Vector2d& x) const {
		return 0;
	}
};

class PointEqualTo {
public:
	bool operator()(const Vector2d& a, const Vector2d& b) const {
		return ((a-b).norm() < epsilon);
	}
};

class PointLessThan {
public:
	enum Coordinate {x=0, y=1};
	PointLessThan(Coordinate c) : c(c) { }

	bool operator()(const Vector2d& a, const Vector2d& b) const {
		return (a(c) < b(c));
	}

private:
	Coordinate c;
};


class SegmentHash {
public:
	long operator()(const Segment& x) const {
		return 0;
	}
};

class SegmentEqualTo {
public:
	bool operator()(const Segment& a, const Segment& b) const {
		return (((a.p0 - b.p0).norm() < epsilon && ((a.p1 - b.p1).norm() < epsilon)) ||
				(((a.p0 - b.p1).norm() < epsilon) && ((a.p1 - b.p0).norm() < epsilon)));
	}
};

class SegmentAngleCompare {
public:
	SegmentAngleCompare(const Segment& compare_segment) : compare_segment(compare_segment) { }

	bool operator()(const Segment& a, const Segment& b) const {
		return (a.angle(compare_segment) < b.angle(compare_segment));
	}

private:
	Segment compare_segment;
};

class TriangleHash {
public:
	long operator()(const Triangle& x) const {
		return 0;
	}
};

class TriangleEqualTo {
public:
	bool operator()(const Triangle& a, const Triangle& b) const {
		std::vector<Vector2d> a_vertices = a.get_vertices();
		std::vector<Vector2d> b_vertices = b.get_vertices();

		std::sort(a_vertices.begin(), a_vertices.end(), PointLessThan(PointLessThan::Coordinate::x));
		std::sort(a_vertices.begin(), a_vertices.end(), PointLessThan(PointLessThan::Coordinate::y));

		std::sort(b_vertices.begin(), b_vertices.end(), PointLessThan(PointLessThan::Coordinate::x));
		std::sort(b_vertices.begin(), b_vertices.end(), PointLessThan(PointLessThan::Coordinate::y));

		for(int i=0; i < 3; ++i) {
			if ((a_vertices[i] - b_vertices[i]).norm() > epsilon) {
				return false;
			}
		}
		return true;
	}
};



}

#endif
