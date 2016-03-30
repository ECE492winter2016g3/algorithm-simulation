
#include <iostream>
#include <sstream>

#include "analyzer.h"
#include "mapping.h"

int main(int argc, char *argv[]) {
	// Lines
	// vector<coord<double>> shape = {
	// 	{ -3, -2 },
	// 	{ -3, -4 },
	// 	{ 2, -4 },
	// 	{ 2, -2 },
	// 	{ 4, -2 },
	// 	{ 4, 2 },
	// 	{ -5, 2 },
	// 	{ -5, -2 }
	// };
	vector<coord<double>> shape = {
		{2, 2},
		{-2, 2},
		{-2, 4},
		{-4, 4},
		{-4, -4},
		{-3, -4},
		{-3, -3},
		{-2, -3},
		{-2, -4},
		{4, -4},
		{4, -2},
		{2, -2}
	};
	for (auto &&coord: shape) {
		coord.x *= 20;
		coord.y *= 20;
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

	// Vehicle
	coord<double> vehiclePosition = { 0, 0 };
	double vehicleAngle = 0;

	// Storage for tmp data
	vector<float> angles{};
	vector<float> distances{};

	// Mapping Algo init
	map_state mapState;
	mapping_init(&mapState);
	Analyzer::scanSweep(vehiclePosition, vehicleAngle, shapeLines, angles, distances);

	mapState.theta = vehicleAngle;
	mapping_initial_scan(&mapState, angles.size(), &angles[0], &distances[0]);

	mapping_dump(&mapState, "test.png");
	int c = 0;
	for (int i = 0; i < 5; ++i) {
		// Drive some
		std::cout << "Turn " << ((M_PI/8)/(M_PI*2)*360) << "...\n";
		vehicleAngle += 0.5;
		// vehiclePosition = {
		// 	vehiclePosition.x + cos(vehicleAngle)*10, 
		// 	vehiclePosition.y + sin(vehicleAngle)*10
		// };

		// Update mapping
		Analyzer::scanSweep(vehiclePosition, vehicleAngle, shapeLines, angles, distances);
		//mapping_update_lin(&mapState, angles.size(), &angles[0], &distances[0]);
		mapping_update_rot(&mapState, angles.size(), &angles[0], &distances[0], 0.5);

		std::stringstream ss;
		ss << "test" << (c++) << ".png";
		mapping_dump(&mapState, ss.str().c_str());
	}
	for (int i = 0; i < 3; ++i) {
		// Drive some
		std::cout << "Drive " << ((M_PI/8)/(M_PI*2)*360) << "...\n";
		//vehicleAngle += 0.6;
		vehiclePosition = {
			vehiclePosition.x + cos(vehicleAngle)*10, 
			vehiclePosition.y + sin(vehicleAngle)*10
		};

		// Update mapping
		Analyzer::scanSweep(vehiclePosition, vehicleAngle, shapeLines, angles, distances);
		mapping_update_lin(&mapState, angles.size(), &angles[0], &distances[0]);
		//mapping_update_rot(&mapState, angles.size(), &angles[0], &distances[0], 0.5);

		std::stringstream ss;
		ss << "test" << (c++) << ".png";
		mapping_dump(&mapState, ss.str().c_str());
	}
	for (int i = 0; i < 2; ++i) {
		// Drive some
		std::cout << "Turn " << ((M_PI/8)/(M_PI*2)*360) << "...\n";
		vehicleAngle += 0.7;
		// vehiclePosition = {
		// 	vehiclePosition.x + cos(vehicleAngle)*10, 
		// 	vehiclePosition.y + sin(vehicleAngle)*10
		// };

		// Update mapping
		Analyzer::scanSweep(vehiclePosition, vehicleAngle, shapeLines, angles, distances);
		//mapping_update_lin(&mapState, angles.size(), &angles[0], &distances[0]);
		mapping_update_rot(&mapState, angles.size(), &angles[0], &distances[0], 0.5);

		std::stringstream ss;
		ss << "test" << (c++) << ".png";
		mapping_dump(&mapState, ss.str().c_str());
	}
	for (int i = 0; i < 3; ++i) {
		// Drive some
		std::cout << "Drive " << ((M_PI/8)/(M_PI*2)*360) << "...\n";
		//vehicleAngle += 0.6;
		vehiclePosition = {
			vehiclePosition.x + cos(vehicleAngle)*10, 
			vehiclePosition.y + sin(vehicleAngle)*10
		};

		// Update mapping
		Analyzer::scanSweep(vehiclePosition, vehicleAngle, shapeLines, angles, distances);
		mapping_update_lin(&mapState, angles.size(), &angles[0], &distances[0]);
		//mapping_update_rot(&mapState, angles.size(), &angles[0], &distances[0], 0.5);

		std::stringstream ss;
		ss << "test" << (c++) << ".png";
		mapping_dump(&mapState, ss.str().c_str());
	}
	std::cout << "Final theta: " << (mapState.theta/(2*M_PI)*360) << "\n";
	std::cout << "Final pos: " << mapState.x << "," << mapState.y << "\n";
	return 0;
}