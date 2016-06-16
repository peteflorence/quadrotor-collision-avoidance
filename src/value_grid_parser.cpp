#include "value_grid_parser.h"

void ValueGridParser::UpdateValueGrid(nav_msgs::OccupancyGrid const& value_grid) {
	this->value_grid = value_grid;
	std::cout << "I just updated my value grid" << std::endl;
	std::cout << VectorFromPoseUnstamped(this->value_grid.info.origin) << " is (0,0) cell in map" << std::endl;
};