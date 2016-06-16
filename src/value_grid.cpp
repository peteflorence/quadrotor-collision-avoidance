#include "value_grid.h"

int ValueGrid::GetValueOfPosition(Vector3 const& position_in_world_frame) {
	Eigen::Matrix<Scalar, 2, 1> point_in_value_grid_frame = transformIntoValueGridFrame(position_in_world_frame);
	size_t x_index, y_index;
	IndexInValueGrid(point_in_value_grid_frame, x_index, y_index);
	return ValueFromIndex(x_index, y_index);
}

Eigen::Matrix<Scalar, 2, 1> ValueGrid::transformIntoValueGridFrame(Vector3 const& point) {
	return Eigen::Matrix<Scalar, 2, 1>(point(0) - cell_0_x_in_world, point(1) - cell_0_y_in_world);
}

void ValueGrid::IndexInValueGrid(Eigen::Matrix<Scalar, 2, 1> const& position_in_value_grid_frame, size_t& x_index, size_t& y_index) {
	x_index = static_cast<size_t> (position_in_value_grid_frame(0) / resolution);
	y_index = static_cast<size_t> (position_in_value_grid_frame(1) / resolution);
}

int ValueGrid::ValueFromIndex(size_t x_index, size_t y_index) {
	return values[y_index*width + x_index];
}

