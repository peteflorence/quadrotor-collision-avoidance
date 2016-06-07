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
  // THIS MUST CHANGE FOR 3D FLIGHT
  trajectory_library.setInitialVelocityAllTrajectories(Vector3(initialVelocity(0), initialVelocity(1), 0.0));
};


size_t TrajectorySelector::getNumTrajectories() {
  return trajectory_library.getNumTrajectories();
};

Vector3 TrajectorySelector::getSigmaAtTime(double const & t) {
  return trajectory_library.getSigmaAtTime(t);
};

Vector3 TrajectorySelector::getInverseSigmaAtTime(double const & t) {
  return trajectory_library.getInverseSigmaAtTime(t);
};

void TrajectorySelector::computeBestTrajectory(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples, Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration) {
  //EvaluateCollisionProbabilities(point_cloud_xyz_samples);
  EvaluateGoalProgress(carrot_body_frame);

  desired_acceleration << 0,0,0;
  best_traj_index = 0;
  float best_traj_objective_value = 0;
  for (size_t traj_index = 0; traj_index < 25; traj_index++) {
    if (GoalProgressEvaluations(traj_index) > best_traj_objective_value) {
      best_traj_index = traj_index;
      best_traj_objective_value = GoalProgressEvaluations(traj_index);
    }
  }

  desired_acceleration = trajectory_library.getTrajectoryFromIndex(best_traj_index).getAcceleration();

};


void TrajectorySelector::EvaluateGoalProgress(Vector3 const& carrot_body_frame) {

  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  double initial_distance = carrot_body_frame.norm();

  size_t i = 0;
  Vector3 final_trajectory_position;
  double distance;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    final_trajectory_position = trajectory->getPosition(final_time);
    distance = (final_trajectory_position - carrot_body_frame).norm();
    GoalProgressEvaluations(i) = initial_distance - distance; 
    i++;
  }
};


void TrajectorySelector::EvaluateCollisionProbabilities(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples) {
  
  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  size_t i = 0;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    CollisionProbabilities(i) = computeProbabilityOfCollisionOneTrajectory(*trajectory, point_cloud_xyz_samples); 
    i++;
  }
};

double TrajectorySelector::computeProbabilityOfCollisionOneTrajectory(Trajectory trajectory, Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples) {
  double probability_no_collision = 1;
  double probability_of_collision_one_step_one_obstacle;
  double probability_no_collision_one_step_one_obstacle;
  Vector3 trajectory_position;
  Vector3 point;

  Vector3 sigma_sensor;
  sigma_sensor << 0.3, 0.3, 0.3;
  Vector3 sigma_at_time;
  Vector3 total_sigma;
  Vector3 inverse_total_sigma;

  for (size_t time_step_index = 0; time_step_index < 10; time_step_index++) {
    sigma_at_time = getSigmaAtTime(sampling_time_vector(time_step_index)); 
    total_sigma = sigma_at_time + sigma_at_time; 
    inverse_total_sigma << 1.0/sigma_at_time(0), 1.0/sigma_at_time(1), 1.0/sigma_at_time(2);
    for (size_t point_index = 0; point_index < 100; point_index++) {
      trajectory_position = trajectory.getPosition(sampling_time_vector(time_step_index));
      point = point_cloud_xyz_samples.row(point_index);

      probability_of_collision_one_step_one_obstacle = computeProbabilityOfCollisionOneStepOneObstacle(trajectory_position, point, inverse_total_sigma);
      probability_no_collision_one_step_one_obstacle = 1.0 - probability_of_collision_one_step_one_obstacle;
      probability_no_collision = probability_no_collision * probability_no_collision_one_step_one_obstacle;
    }
  }
  return 1 - probability_no_collision;
};

double TrajectorySelector::computeProbabilityOfCollisionOneStepOneObstacle(Vector3 const& robot_position, Vector3 const& depth_position, Vector3 const& inverse_total_sigma) {
  double volume = 4.18;
  
  double denominator = std::sqrt( 2*2*2*M_PI*M_PI*M_PI*(1.0/inverse_total_sigma(0))*(1.0/inverse_total_sigma(1))*(1.0/inverse_total_sigma(2)) );
  double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

  return volume / denominator * std::exp(exponent);
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