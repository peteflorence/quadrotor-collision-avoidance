#ifndef VALUE_GRID_H
#define VALUE_GRID_H

#include <iostream>
#include "trajectory_selector_utils.h"


class ValueGrid {
public:
	
	void SetResolution(float meters_per_cell) {
		resolution = meters_per_cell;
	};
	
	void SetSize(size_t num_cols, size_t num_rows) {
		width = num_cols;
		height = num_rows;
	};

	void SetOrigin(double x_in_world, double y_in_world) {
		cell_0_x_in_world = x_in_world;
		cell_0_y_in_world = y_in_world;
	};

	void SetValues(std::vector<int8_t> const& data) {
		values = data;
	}

	int GetValueOfPosition(Vector3 const& position_in_world_frame);

private:
	Eigen::Matrix<Scalar, 2, 1> transformIntoValueGridFrame(Vector3 const& point);
	void IndexInValueGrid(Eigen::Matrix<Scalar, 2, 1> const& position_in_value_grid_frame, size_t& x_index, size_t& y_index);
	int ValueFromIndex(size_t x_index, size_t y_index);

	float resolution;
	size_t width;
	size_t height;

	double cell_0_x_in_world;
	double cell_0_y_in_world;

	std::vector<int8_t> values;

};

#endif