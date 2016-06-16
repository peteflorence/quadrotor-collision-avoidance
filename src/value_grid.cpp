#include "value_grid.h"

int ValueGrid::GetValueOfPosition(Vector3 const& position_in_world_frame) {
	Eigen::Matrix<Scalar, 2, 1> point_in_value_grid_frame = transformIntoValueGridFrame(position_in_world_frame);
	size_t col_index, row_index;
	IndexInValueGrid(point_in_value_grid_frame, col_index, row_index);
	// NEED TO CHECK BOUNDS
	std::cout << ValueFromIndex(0,0) << " is value at 0,0" << std::endl;
	std::cout << "Width, Height " << width << " " << height << std::endl;

	return ValueFromIndex(col_index, row_index);
}

Eigen::Matrix<Scalar, 2, 1> ValueGrid::transformIntoValueGridFrame(Vector3 const& point) {
	return Eigen::Matrix<Scalar, 2, 1>(point(0) - cell_0_x_in_world, point(1) - cell_0_y_in_world);
}

void ValueGrid::IndexInValueGrid(Eigen::Matrix<Scalar, 2, 1> const& position_in_value_grid_frame, size_t& col_index, size_t& row_index) {
	col_index = static_cast<size_t> (position_in_value_grid_frame(0) / resolution);
	row_index = static_cast<size_t> (position_in_value_grid_frame(1) / resolution);
}

int ValueGrid::ValueFromIndex(size_t col_index, size_t row_index) {
	return values[row_index*width + col_index];
}

