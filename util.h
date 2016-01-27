#pragma once

#include <sstream>
#include <cmath>
using namespace std;

template <typename T>
struct coord {
	T x;
	T y;

	string toString() {
		ostringstream s;
		s << x << ", " << y << endl;
		return s.str();
	}

	T dist(const coord<T>& other) {
		return sqrt((x - other.x)*(x - other.x) + (y - other.y)*(y - other.y));
	}
};

template <typename T>
struct line {
	T x1;
	T y1;
	T x2;
	T y2;

	// From https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	bool intersects(const line<T>& other, coord<T>* point) {
		T x12 = x1 - x2;
		T x34 = other.x1 - other.x2;
		T y12 = y1 - y2;
		T y34 = other.y1 - other.y2;

		T c = x12 * y34 - y12 * x34;

		if (abs(c) < 0.01) {
			return false;
		}
		else {
			T a = x1 * y2 - y1 * x2;
			T b = other.x1 * other.y2 - other.y1 * other.x2;

			T x = (a * x34 - b * x12) / c;
			T y = (a * y34 - b * y12) / c;

			if (min(x1,x2)-0.01 <= x && x <= max(x1,x2)+0.01 && min(other.x1,other.x2)-0.01 <= x && x <= max(other.x1,other.x2)+0.01 &&
				min(y1,y2)-0.01 <= y && y <= max(y1,y2)+0.01 && min(other.y1,other.y2)-0.01 <= y && y <= max(other.y1,other.y2)+0.01) {

				point->x = x;
				point->y = y;
				return true;
			}
			else {
				return false;
			}
		}
	}
};

struct distance_measurement {
	double distance;
	double angle;
};