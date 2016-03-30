#include <vector>
#include "util.h"

namespace Analyzer {
	void scanSweep(coord<double> source, double angle, vector<line<double>> shape, vector<float> &angles, vector<float> &distances);
	coord<double> scan(coord<double> source, double angle, vector<line<double>> shape);
	vector<line<double>> getMap(vector<distance_measurement> measurements);
};