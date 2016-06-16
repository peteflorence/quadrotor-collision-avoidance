#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include "trajectory_selector_utils.h"


class ValueGridParser {
public:
  void UpdateValueGrid(nav_msgs::OccupancyGrid const& value_grid);

private:
	nav_msgs::OccupancyGrid value_grid;

};