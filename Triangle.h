#ifndef __TRIANGLE_LEGS__
#define __TRIANGLE_LEGS__

#include "Point.h"

class Triangle {
	Point p1;
	Point p2;
	Point p3;
public:
	Triangle() = default;
	
	Triangle(Point np1, Point np2, Point np3) {
		p1 = np1;
		p2 = np2;
		p3 = np3;
	}

	static float getArea(Triangle t) {
		Point side1 = t.p2 - t.p1;
		Point side2 = t.p3 - t.p1;

		float sl1 = side1.norm();
		float sl2 = side2.norm();

		// by definition
		float cos_angle = (side1 * side2) / (sl1 * sl2);
		float sin_angle = sqrt(1 - cos_angle * cos_angle);

		return 0.5 * sl1 * sl2 * sin_angle;
	}

	static float getInscribedCircleArea(Triangle t) {
		float sl1 = getSideLength(t.p1, t.p2);
		float sl2 = getSideLength(t.p2, t.p3);
		float sl3 = getSideLength(t.p3, t.p1);

		float semi_perimeter = 0.5 * (sl1 + sl2 + sl3);
		float area = getArea(t);
		float r = area / semi_perimeter;

		return M_PI * r * r;
	}

	static float getSideLength(Point np1, Point np2) {
		return (np1 - np2).norm();
	}
};

class TriangleSet {
	Triangle triangles[3];
	typedef float (*triangle_calculation)(Triangle);
public:
	TriangleSet(Triangle t1, Triangle t2, Triangle t3) {
		triangles[0] = t1;
		triangles[1] = t2;
		triangles[2] = t3;
	}

	TriangleSet(Triangle ntriangles[]) {
		triangles[0] = ntriangles[0];
		triangles[1] = ntriangles[1];
		triangles[2] = ntriangles[2];
	}

	static float getMinimumArea(Triangle *triangles, int num_triangles) {
		return getMinimum(triangles, num_triangles, &Triangle::getArea);
	}

	static float getMinimumInscribedCircleArea(Triangle *triangles, int num_triangles) {
		return getMinimum(triangles, num_triangles, &Triangle::getInscribedCircleArea);
	}

	static float getMinimum(Triangle *triangles, int num_triangles, triangle_calculation calculation) {
		float min_area = calculation(triangles[0]);
		for (int i = 1; i < num_triangles; i++) {
			float temp_area = calculation(triangles[i]);
			if (temp_area < min_area) {
				min_area = temp_area;
			}
		}

		return min_area;
	}
};

#endif