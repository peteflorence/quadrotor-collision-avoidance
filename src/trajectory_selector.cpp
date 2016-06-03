#include "trajectory_selector.h"
#include <iostream>

void TrajectorySelector::Test() {

  std::cout << "Printing from inside TrajectorySelector " << std::endl;  
  trajectory_library.TestLibrary();
  trajectory_evaluator.TestEvaluator();

}

void TrajectorySelector::InitializeLibrary() {
  trajectory_library.Initialize2DLibrary();
};

void TrajectorySelector::setInitialVelocity(Vector3 const& initialVelocity) {
  trajectory_library.setInitialVelocity(initialVelocity);
};



Vector3 TrajectorySelector::computeAccelerationDesiredFromBestTrajectory() {
  this->EvalAllTrajectories();
  return Vector3(0,0,0);

};


void TrajectorySelector::EvalAllTrajectories() {

  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  // evaluate the probability of collision for all trajectories at final time


  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    std::cout << trajectory->getPosition(10.0) << std::endl;
  }

  // evaluate the probability of 


};


Eigen::Matrix<Scalar, Eigen::Dynamic, 3> TrajectorySelector::sampleTrajectoryForDrawing(size_t trajectory_index, double start_time, double final_time, size_t num_samples) {
  Trajectory trajectory_to_sample = trajectory_library.getTrajectoryFromIndex(trajectory_index);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time(num_samples, 3);
  double sampling_time = 0;
  double sampling_interval = (final_time - start_time) / num_samples;
  
  for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
    sampling_time = start_time + sampling_interval*sample_index;
    sample_points_xyz_over_time.row(sample_index) = trajectory_to_sample.getPosition(sampling_time);
  }

  return sample_points_xyz_over_time;
}