
#include <SDL.h>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "plotter.h"
#include "util.h"
#include "analyzer.h"

using namespace std;

int main(int argc, char* argv[]) {

	// Underlying polygon representing the room
	vector<coord<double>> shape = {
		{ -3, -2 },
		{ -3, -4 },
		{ 2, -4 },
		{ 2, -2 },
		{ 4, -2 },
		{ 4, 2 },
		{ -5, 2 },
		{ -5, -2 }
	};
	vector<line<double>> shapeLines;
	for (unsigned int i = 0; i < shape.size(); ++i) {
		shapeLines.push_back({
			shape[i].x,
			shape[i].y,
			shape[i < shape.size() - 1 ? i + 1 : 0].x,
			shape[i < shape.size() - 1 ? i + 1 : 0].y
		});
	}

	// Initial conditions
	coord<double> vehiclePosition = { 0, 0 };
	double vehicleAngle = 0;
	double sensorAngle = 0;
	bool sensorActive = true;

	// Movement Configuration
	double moveSpeed = 0.03;
	double rotationSpeed = 7.5;
	double sensorRotationIncrement = 10.1;

	// Data recorded during simulation
	vector<coord<double>> points;
	vector<line<double>> estimate;
	vector<line<double>> allLines;
	coord<double> point;
	vector<distance_measurement> records;

	// Set up the plotter
	Plotter p;
	p.init();

	bool quit = false;
	SDL_Event e;
	while (!quit) {
		while (SDL_PollEvent(&e) != 0) {
			if (e.type == SDL_QUIT) {
				quit = true;
			}
			else if (e.type == SDL_KEYDOWN) {
				if (e.key.keysym.sym == SDLK_SPACE) {
					sensorActive = !sensorActive;
				}
			}
		}
		const Uint8 *state = SDL_GetKeyboardState(NULL);
		if (state[SDL_SCANCODE_W]) {
			vehiclePosition.x += cos(vehicleAngle * M_PI / 180) * moveSpeed;
			vehiclePosition.y += sin(vehicleAngle * M_PI / 180) * moveSpeed;
		}
		if (state[SDL_SCANCODE_A]) {
			vehicleAngle -= rotationSpeed;
		}
		if (state[SDL_SCANCODE_D]) {
			vehicleAngle += rotationSpeed;
		}

		p.clear();

		if (sensorActive) {
			point = Analyzer::scan(vehiclePosition, vehicleAngle + sensorAngle, shapeLines);
			sensorAngle += sensorRotationIncrement;
			points.push_back(point);
			records.push_back({ point.dist(vehiclePosition), sensorAngle });
			estimate = Analyzer::getMap(records);
			allLines.clear();
			allLines.reserve(shapeLines.size() + estimate.size());
			allLines.insert(allLines.end(), shapeLines.begin(), shapeLines.end());
			allLines.insert(allLines.end(), estimate.begin(), estimate.end());
		}

		p.calibrate(points, allLines);
		p.plot(0, points, shapeLines);

		p.plot(1, { vehiclePosition }, {{
			vehiclePosition.x, vehiclePosition.y,
			vehiclePosition.x + cos(vehicleAngle * M_PI / 180) * 0.2,
			vehiclePosition.y + sin(vehicleAngle * M_PI / 180) * 0.2,
		}});

		if (sensorActive) {
			p.plot(2, {}, { { vehiclePosition.x, vehiclePosition.y, point.x, point.y } });
		}

		p.plot(3, {}, estimate);
		SDL_Delay(100);

		p.present();
	}

	for (auto record : records) {
		cout << "distance: " << record.distance << ", angle: " << record.angle << endl;
	}

	return 0;
}