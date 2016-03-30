#include "plotter.h"
#include <stdio.h>
#include <vector>

Plotter::Plotter() {

}

Plotter::~Plotter() {
	SDL_DestroyWindow(window);
	SDL_Quit();
}

void Plotter::init() {
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
	}
	else {

		window = SDL_CreateWindow("Mapping Robot Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
		if (window == NULL) {
			printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		}
		else {
			// Create renderer
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
			if (renderer == NULL) {
				printf("Renderer could not be created! SDL_Error: %s/n", SDL_GetError());
			}
			else {
				SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
				SDL_RenderFillRect(renderer, NULL);
			}
			surface = SDL_GetWindowSurface(window);
		}
	}
}

void Plotter::clear() {
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);
}

void Plotter::calibrate(vector<coord<double>> points, vector<line<double>> lines) {
	topLeft = { 0, 0 };
	botRight = { 0, 0 };
	for (int i = 0; i < points.size(); ++i) {
		if (points[i].x > botRight.x) {
			botRight.x = points[i].x;
		}
		if (points[i].x < topLeft.x) {
			topLeft.x = points[i].x;
		}
		if (points[i].y > botRight.y) {
			botRight.y = points[i].y;
		}
		if (points[i].y < topLeft.y) {
			topLeft.y = points[i].y;
		}
	}
	for (int i = 0; i < lines.size(); ++i) {
		if (lines[i].x1 > botRight.x) {
			botRight.x = lines[i].x1;
		}
		if (lines[i].x1 < topLeft.x) {
			topLeft.x = lines[i].x1;
		}
		if (lines[i].y1 > botRight.y) {
			botRight.y = lines[i].y1;
		}
		if (lines[i].y1 < topLeft.y) {
			topLeft.y = lines[i].y1;
		}
		if (lines[i].x2 > botRight.x) {
			botRight.x = lines[i].x2;
		}
		if (lines[i].x2 < topLeft.x) {
			topLeft.x = lines[i].x2;
		}
		if (lines[i].y2 > botRight.y) {
			botRight.y = lines[i].y2;
		}
		if (lines[i].y2 < topLeft.y) {
			topLeft.y = lines[i].y2;
		}
	}

	SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);

	int x = (int) (0 - topLeft.x) / (botRight.x - topLeft.x) * (SCREEN_WIDTH - 20) + 10;
	int y = (int) (0 - topLeft.y) / (botRight.y - topLeft.y) * (SCREEN_HEIGHT - 20) + 10;

	SDL_RenderDrawLine(renderer, x, 0, x, SCREEN_HEIGHT);
	SDL_RenderDrawLine(renderer, 0, y, SCREEN_WIDTH, y);
}

void Plotter::plot(int color, vector<coord<double>> points, vector<line<double>> lines) {
	int r, g, b;
	if (color == 0) {
		r = 255;
		g = 255;
		b = 255;
	}
	else if (color == 1) {
		r = 255;
		g = 0;
		b = 0;
	}
	else if (color == 2) {
		r = 0;
		g = 255;
		b = 0;
	}
	else if (color == 3) {
		r = 0;
		g = 0;
		b = 255;
	}

	int x;
	int y;
	int x2;
	int y2;

	SDL_SetRenderDrawColor(renderer, r, g, b, 255);
	//printf("tl: %i,%i, br: %i,%i\n", topLeft.x, topLeft.y, botRight.x, botRight.y);
	for (int i = 0; i < points.size(); ++i) {
		x = ((double) (points[i].x - topLeft.x) / (botRight.x - topLeft.x)) * (SCREEN_WIDTH - 20) + 10;
		y = ((double) (points[i].y - topLeft.y) / (botRight.y - topLeft.y)) * (SCREEN_HEIGHT - 20) + 10;
		//printf("before: %i,%i, after: %i,%i\n", points[i].x, points[i].y, x, y);

		SDL_RenderDrawPoint(renderer, x, y);
		SDL_RenderDrawPoint(renderer, x-1, y-1);
		SDL_RenderDrawPoint(renderer, x+1, y+1);
		SDL_RenderDrawPoint(renderer, x+1, y-1);
		SDL_RenderDrawPoint(renderer, x-1, y+1);
	}
	for (int i = 0; i < lines.size(); ++i) {
		x = ((double)(lines[i].x1 - topLeft.x) / (botRight.x - topLeft.x)) * (SCREEN_WIDTH - 20) + 10;
		y = ((double)(lines[i].y1 - topLeft.y) / (botRight.y - topLeft.y)) * (SCREEN_HEIGHT - 20) + 10;
		x2 = ((double)(lines[i].x2 - topLeft.x) / (botRight.x - topLeft.x)) * (SCREEN_WIDTH - 20) + 10;
		y2 = ((double)(lines[i].y2 - topLeft.y) / (botRight.y - topLeft.y)) * (SCREEN_HEIGHT - 20) + 10;

		SDL_RenderDrawLine(renderer, x, y, x2, y2);
	}
}

void Plotter::present() {
	SDL_RenderPresent(renderer);
}