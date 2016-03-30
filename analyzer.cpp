#include "analyzer.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>

using namespace std;

// Normall distributed noise added to LIDAR test 
std::normal_distribution<> LIDAR_VARIANCE(0.0f, 1.0f);
std::random_device rd;
std::mt19937 random_gen{rd()};

void Analyzer::scanSweep(
	coord<double> source, double angle, 
	vector<line<double>> shape,
	vector<float> &angles, vector<float> &distances) 
{
	angles.clear();
	distances.clear();
	for (int step = 0; step < 50; ++step) {
		float theta = step * (M_PI*2.0f / 50.0f);
		line<double> scan = {
			source.x,
			source.y,
			source.x + 2000*cos(angle + theta),
			source.y + 2000*sin(angle + theta)
		};
		float bestDist = 1000000;
		for (auto line : shape) {
			coord<double> point;
			if (line.intersects(scan, &point)) {
				float dx = point.x - source.x;
				float dy = point.y - source.y;
				float dist = sqrt(dx*dx + dy*dy);
				if (dist < bestDist)
					bestDist = dist;
			}
		}
		angles.push_back(theta);
		distances.push_back(bestDist + LIDAR_VARIANCE(random_gen));
	}
}

coord<double> Analyzer::scan(coord<double> source, double angle, vector<line<double>> shape) {

	// First, create the second point in the line segment (as far away from source as need be)
	coord<double> directionVector = {
		cos(angle * M_PI / 180),
		sin(angle * M_PI / 180)
	};

	// Transformation to make reasoning about this easier. Multiply each x by t.x and each y
	// by t.y, and now everything is in the positive x, positive y quadrant
	coord<double> t = {
		directionVector.x >= 0.0f ? 1.0f : -1.0f,
		directionVector.y >= 0.0f ? 1.0f : -1.0f
	};
	const line<double>& rightMost = *max_element(shape.begin(), shape.end(),
		[&t](const line<double>& a, const line<double>& b) {
		return max(a.x1*t.x, a.x2*t.x) < max(b.x1*t.x, b.x2*t.x);
	});
	const line<double>& bottomMost = *max_element(shape.begin(), shape.end(),
		[&t](const line<double>& a, const line<double>& b) {
		return max(a.y1*t.y, a.y2*t.y) < max(b.y1*t.y, b.y2*t.y);
	});
	double maxX = max(rightMost.x1*t.x, rightMost.x2*t.x) + 1;
	double maxY = max(bottomMost.y1*t.y, bottomMost.y2*t.y) + 1;
	cout << "maxX: " << maxX << ", maxY: " << maxY << endl;
	cout << "source: " << source.toString();
	cout << "t: " << t.toString();
	double xAtMaxY;
	double yAtMaxX;
	double m = abs(tan(angle * M_PI / 180));

	// Slope of 0 -> don't bother calculating xAtMaxY to avoid errors
	if (m == 0) {
		xAtMaxY = source.x*t.x;
	}
	else {
		xAtMaxY = (maxY - source.y*t.y) / m + source.x*t.x;
	}
	yAtMaxX = (maxX - source.x*t.x) * m + source.y*t.y;
	cout << "tan(angle): " << m << endl;
	cout << "maxX: " << maxX << ", " << yAtMaxX << endl;
	cout << "maxY: " << xAtMaxY << ", " << maxY << endl;

	coord<double> dest;
	// Slope of zero -> maxY isn't going to matter
	if (xAtMaxY > maxX || m == 0) {
		dest = {
			maxX,
			yAtMaxX
		};
	}
	else {
		dest = {
			xAtMaxY,
			maxY
		};
	}
	cout << "dest: " << dest.x << ", " << dest.y << endl;

	// Here we undo the t transform, so we're operating with correct directions from here
	// on out
	line<double> scan = {
		source.x,
		source.y,
		dest.x*t.x,
		dest.y*t.y
	};

	vector<coord<double>> intersects;
	for (auto line : shape) {
		coord<double> point;
		if (line.intersects(scan, &point)) {
			intersects.push_back(point);
		}
	}

	double minDist = -1;
	coord<double> closest;
	for (auto coord : intersects) {
		if (minDist < 0 || minDist > coord.dist(source)) {
			minDist = coord.dist(source);
			closest = coord;
		}
	}
	if (intersects.size() == 0) {
		closest = source;
	}

	return closest;
}

vector<line<double>> Analyzer::getMap(vector<distance_measurement> measurements) {
	// TODO: Implement for real / bring in Mark's stuff
	// This is all just to get some numbers to draw
	vector<coord<double>> shape = {
		{ -3.5, -2.2 },
		{ -3, -3.9 },
		{ 1.8, -4.1 },
		{ 2.2, -2.1 },
		{ 4.5, -2.1 }
	};

	if (measurements.size() > 10) {
		shape.push_back({ 4.5, 1.5 });
	}
	if (measurements.size() > 15) {
		shape.push_back({ -5.2, 2.3 });
	}
	if (measurements.size() > 20) {
		shape.push_back({ -5.1, -2.1 });
	}

	vector<line<double>> shapeLines;

	for (unsigned int i = 0; i < shape.size(); ++i) {
		shapeLines.push_back({
			shape[i].x,
			shape[i].y,
			shape[i < shape.size() - 1 ? i + 1 : 0].x,
			shape[i < shape.size() - 1 ? i + 1 : 0].y
		});
	}

	return shapeLines;
}