#include "gtest/gtest.h"

#include "attitude_generator.h"
#include "depth_image_collision_evaluator.h"
#include "kd_tree.h"
#include "motion.h"
#include "motion_library.h"
#include "motion_selector.h"
#include "motion_selector_node.h"
#include "motion_visualizer.h"
#include "nanoflann.hpp"
#include "value_grid.h"
#include "value_grid_evaluator.h"


const double TOLERANCE = 1e-4;


int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "behavior_selector_tests");
  return RUN_ALL_TESTS();
}