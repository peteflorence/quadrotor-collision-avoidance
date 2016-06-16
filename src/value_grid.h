#include <iostream>
#include <chrono>
#include "trajectory_selector_utils.h"
#include <nav_msgs/OccupancyGrid.h>


class ValueGrid {
public:
  void UpdateValueGrid(nav_msgs::OccupancyGrid * value_grid);

private:
	nav_msgs::OccupancyGrid* value_grid_ptr;

};