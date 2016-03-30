#pragma once

#include <sdl.h>
#include "util.h"
#include <vector>

class Plotter {
private:
	const int SCREEN_WIDTH = 640;
	const int SCREEN_HEIGHT = 480;

	coord<double> topLeft;
	coord<double> botRight;

	SDL_Window* window;
	SDL_Surface* surface;
	SDL_Renderer* renderer;

public:
	Plotter();
	~Plotter();

	void init();

	void clear();
	void calibrate(vector<coord<double>> points, vector<line<double>> lines);
	void plot(int color, vector<coord<double>> points, vector<line<double>> lines);
	void present();
};