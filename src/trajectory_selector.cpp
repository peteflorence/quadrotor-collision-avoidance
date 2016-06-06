#include "trajectory_selector.h"
#include <iostream>

void TrajectorySelector::Test() {
  std::cout << "Printing from inside TrajectorySelector " << std::endl;  
  trajectory_library.TestLibrary();
  trajectory_evaluator.TestEvaluator();
}

void TrajectorySelector::InitializeLibrary(double const& final_time) {
  trajectory_library.Initialize2DLibrary(final_time);
  //num_trajectories = getNumTrajectories();
  this->final_time = final_time;

  size_t num_samples = 10;
  double sampling_time = 0;
  double sampling_interval = (final_time - start_time) / num_samples;
  for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      sampling_time_vector(sample_index) = sampling_time;
  }

};

void TrajectorySelector::setInitialVelocity(Vector3 const& initialVelocity) {
  trajectory_library.setInitialVelocityAllTrajectories(initialVelocity);
};

size_t TrajectorySelector::getNumTrajectories() {
  return trajectory_library.getNumTrajectories();
};

Vector3 TrajectorySelector::getSigmaAtTime(double const & t) {
  return trajectory_library.getSigmaAtTime(t);
};

Vector3 TrajectorySelector::computeAccelerationDesiredFromBestTrajectory(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples, Vector3 const& carrot_body_frame) {
  EvaluateCollisionProbabilities(point_cloud_xyz_samples);
  EvaluateGoalProgress(carrot_body_frame);
  return Vector3(0,0,0);
};


void TrajectorySelector::EvaluateGoalProgress(Vector3 const& carrot_body_frame) {

  Eigen::Matrix<Scalar, 25, 1> GoalProgressEvaluations;

  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  size_t i = 0;
  Vector3 final_trajectory_position;
  double distance;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    final_trajectory_position = trajectory->getPosition(final_time);
    distance = (final_trajectory_position - carrot_body_frame).norm();
    GoalProgressEvaluations(i) = distance; 
    i++;
  }
};


void TrajectorySelector::EvaluateCollisionProbabilities(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples) {

  Eigen::Matrix<Scalar, 25, 1> CollisionProbabilities;
  
  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();


  size_t i = 0;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    computeProbabilityOfCollisionOneTrajectory(*trajectory, point_cloud_xyz_samples); 
    i++;
  }
};

void TrajectorySelector::computeProbabilityOfCollisionOneTrajectory(Trajectory trajectory, Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples) {
  float probability_no_collision = 1;
  float probability_of_collision_one_step_one_obstacle;
  float probability_no_collision_one_step_one_obstacle;
  Vector3 trajectory_position;
  Vector3 point;

  for (size_t time_step_index = 0; time_step_index < 10; time_step_index++) {
    for (size_t point_index = 0; point_index < 100; point_index++) {
      trajectory_position = trajectory.getPosition(sampling_time_vector(time_step_index));
      point = point_cloud_xyz_samples.row(point_index);

      probability_of_collision_one_step_one_obstacle = computeProbabilityOfCollisionOneStepOneObstacle(trajectory_position, point);
      probability_no_collision_one_step_one_obstacle = 1.0 - probability_no_collision_one_step_one_obstacle;
      probability_no_collision = probability_no_collision * probability_no_collision_one_step_one_obstacle;
    }
  }
  return;
};

float TrajectorySelector::computeProbabilityOfCollisionOneStepOneObstacle(Vector3 const& trajectory_position, Vector3 const& point) {
  return 0;
};



Eigen::Matrix<Scalar, Eigen::Dynamic, 3> TrajectorySelector::sampleTrajectoryForDrawing(size_t trajectory_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples) {
  Trajectory trajectory_to_sample = trajectory_library.getTrajectoryFromIndex(trajectory_index);
  double sampling_time;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time(num_samples,3);

  for (size_t time_index = 0; time_index < num_samples; time_index++) {
    //std::cout << "the position I sample is " << trajectory_to_sample.getPosition(sampling_time) << std::endl;
    sampling_time = sampling_time_vector(time_index);
    sample_points_xyz_over_time.row(time_index) = trajectory_to_sample.getPosition(sampling_time);
  }

  return sample_points_xyz_over_time;
}