#include "trajectory_selector.h"
#include <iostream>

void TrajectorySelector::Test() {

  std::cout << "Printing from inside TrajectorySelector " << std::endl;  
  trajectory_library.TestLibrary();
  trajectory_evaluator.TestEvaluator();

}

void TrajectorySelector::EvalTrajectories() {
  trajectory_library.Initialize2DLibrary();
  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  for (auto iterator = trajectory_iterator_begin; iterator != trajectory_iterator_end; iterator++) {
    std::cout << iterator->getPosition(10.0) << std::endl;
  }


}