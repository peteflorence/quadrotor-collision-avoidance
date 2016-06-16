#include "value_grid.h"

void ValueGrid::UpdateValueGrid(nav_msgs::OccupancyGrid * value_grid) {
	//auto t1 = std::chrono::high_resolution_clock::now();
	value_grid_ptr = value_grid;
	//std::cout << "I just updated my value grid" << std::endl;
	//std::cout << VectorFromPoseUnstamped(this->value_grid.info.origin) << " is (0,0) cell in map" << std::endl;
	// auto t2 = std::chrono::high_resolution_clock::now();
	// 	std::cout << "Just updating value grid pointer took "
 //      		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
 //      		<< " microseconds\n";
};