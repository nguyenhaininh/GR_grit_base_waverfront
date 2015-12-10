/*
 * wavefront.cpp
 *
 *  Created on: Dec 10, 2015
 *      Author: haininh
 */
#include "wavefront.hpp"

using namespace std;

namespace wandrian {
namespace plans {
namespace wavefront {

Wavefront::Wavefront() :
		robot_size(0) {
}

Wavefront::~Wavefront() {
}

// example worlds
// O -> (go right)
// |
// | (go down)
//World the First 16 * 8
//
//{ { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, X, X, X },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, X, X, X, X, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0 } }
//
//World the Second
//
//{ { 0, 0, 0, 0, 0, 0, 0, X, X, X, X, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0 },
//  { 0, 0, 0, X, X, X, X, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 },
//  { 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0 } }

void Wavefront::initialize(CellPtr _start, CellPtr _goal, double robot_size) {
	this->robot_size = robot_size;

	WORLD_MAP = {
		{	0, X, X, 0, 0, 0, 0, 0},
		{	0, X, X, 0, 0, 0, 0, 0},
		{	0, 0, 0, 0, 0, 0, 0, 0},
		{	0, 0, 0, 0, 0, 0, 0, 0},
		{	0, 0, 0, 0, 0, 0, 0, 0},
		{	0, 0, 0, 0, X, 0, 0, 0},
		{	0, 0, 0, X, X, X, 0, 0},
		{	0, 0, 0, X, X, 0, 0, 0}
	};
	start = _start;
	goal = _goal;

	WORLD_MAP[goal.x][goal.y] = GOAL;
}

void Wavefront::cover() {
	Cell current = start;
	Cell next_cell;
	while (!path.empty()) {
		next_cell = path.top();
		path.pop();
//		PointPtr new_position = PointPtr(
//				new Point(next_cell.x * robot_size, next_cell.y * robot_size));
//		go_to(new_position, STRICTLY);
		go_by_step(current, next_cell);
		current = next_cell;
	}
}

bool Wavefront::go_to(PointPtr position, bool flexibly) {
	std::cout << "    pos: " << position->x << "," << position->y << "\n";
	if (behavior_go_to != NULL)
		return behavior_go_to(position, flexibly);
	return true;
}

bool Wavefront::go_by_step(Cell current, Cell next) {
	if (current.x == next.x || current.y == next.y) { // Von Neuman neighbors
		PointPtr new_position = PointPtr(
				new Point(next.x * robot_size, next.y * robot_size));
		return go_to(new_position, STRICTLY);
	}

	Cell step;
	PointPtr new_position;
	step.x = current.x;
	step.y = next.y;
	new_position = PointPtr(
			new Point(step.x * robot_size, step.y * robot_size));
	go_to(new_position, STRICTLY);
	new_position = PointPtr(
			new Point(next.x * robot_size, next.y * robot_size));
	return go_to(new_position, STRICTLY);
}

/***** 1st Stage - Building world map *****/
void Wavefront::wave_fill() {
	queue<Cell> wave;
	int current_score;
	Cell next_cell, visiting_cell;

	next_cell = goal;
	wave.push(next_cell);
	while (!wave.empty()) {
		visiting_cell = wave.front();
		wave.pop();
		current_score = get_cell_value(visiting_cell) + 1;

		// 8-directions version
		// 0 0 0
		// 0 1 0
		// 0 0 0

		// 1st row
		next_cell.x = visiting_cell.x - 1;
		next_cell.y = visiting_cell.y - 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		next_cell.x = visiting_cell.x;
		next_cell.y = visiting_cell.y - 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		next_cell.x = visiting_cell.x + 1;
		next_cell.y = visiting_cell.y - 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		// 2nd row
		next_cell.x = visiting_cell.x - 1;
		next_cell.y = visiting_cell.y;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		next_cell.x = visiting_cell.x + 1;
		next_cell.y = visiting_cell.y;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		// 3rd row
		next_cell.x = visiting_cell.x - 1;
		next_cell.y = visiting_cell.y + 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		next_cell.x = visiting_cell.x;
		next_cell.y = visiting_cell.y + 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}

		next_cell.x = visiting_cell.x + 1;
		next_cell.y = visiting_cell.y + 1;
		if (check_coordinate(next_cell)
				== true&& get_cell_value(next_cell) == UNVISITED) {
			WORLD_MAP[next_cell.x][next_cell.y] = current_score;
			wave.push(next_cell);
		}
	}

	// debug: print filled world map
	for (int i = 0; i < MAP_Y; i++) {
		for (int j = 0; j < MAP_X; j++) {
			printf("%d", WORLD_MAP[j][i]);
		}
		printf("\n");
	}
}

bool Wavefront::check_coordinate(Cell input) {
	return !((input.x < 0 || input.y < 0 || input.x > (MAP_X - 1)
			|| input.y > (MAP_Y - 1)));
}

int Wavefront::get_cell_value(Cell input) {
	return WORLD_MAP[input.x][input.y];
}

/***** 2nd Stage - Coverage path generate *****/
// tie-break rule: west first, clockwise: chon ben trai truoc, cung chieu kim dong ho
void Wavefront::path_planning() {
	// TODO: generate coverage path
	queue<Cell> trajectory;
	Cell current = start;
	Cell next;
	do {
		next = get_next_hop(current);
		if(next.x == -1 && next.y == -1) {
			// go back to previous
			next = trajectory.front();
			trajectory.pop();
		}
		else {
			WORLD_MAP[next.x][next.y] = VISITED;
			path.push(next);
			trajectory.push(next);
		}

		current = next;
	} while (current != goal);
}

Cell Wavefront::get_next_hop(Cell current) {
	Cell next, tmp;
	next.x = -1;
	next.y = -1;
	int max_score = 2;

	tmp.x = current.x - 1;
	tmp.y = current.y;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x - 1;
	tmp.y = current.y - 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x;
	tmp.y = current.y - 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x + 1;
	tmp.y = current.y - 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x + 1;
	tmp.y = current.y;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x + 1;
	tmp.y = current.y + 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x;
	tmp.y = current.y + 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	tmp.x = current.x - 1;
	tmp.y = current.y + 1;
	if (check_coordinate(tmp) == true && get_cell_value(tmp) > max_score) {
		max_score = get_cell_value(tmp);
		next = tmp;
	}

	return next;
}

}
}
}

