#include <vector>
#include "util.h"

namespace Analyzer {
	coord<double> scan(coord<double> source, double angle, vector<line<double>> shape);
	vector<line<double>> getMap(vector<distance_measurement> measurements);
};