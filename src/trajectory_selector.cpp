#include "trajectory_selector.h"
#include <iostream>

TrajectoryLibrary* TrajectorySelector::GetTrajectoryLibraryPtr() {
  return &trajectory_library;
};

ValueGridEvaluator* TrajectorySelector::GetValueGridEvaluatorPtr() {
  return &value_grid_evaluator;
};

void TrajectorySelector::InitializeLibrary(double const& final_time) {
  trajectory_library.Initialize2DLibrary(final_time);
  this->final_time = final_time;

  size_t num_samples = 10;
  double sampling_time = 0;
  double sampling_interval = (final_time - start_time) / num_samples;
  for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      sampling_time_vector(sample_index) = sampling_time;
  }
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

void TrajectorySelector::computeBestDijkstraTrajectory(Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf, size_t &best_traj_index, Vector3 &desired_acceleration) {
  EvaluateDijkstraCost(carrot_world_frame, tf);
  EvaluateTerminalVelocityCost();
  //DijkstraEvaluations = Normalize(DijkstraEvaluations);
  //TerminalVelocityEvaluations = Normalize(TerminalVelocityEvaluations);

  desired_acceleration << 0,0,0;
  best_traj_index = 0;
  float current_objective_value;
  float best_traj_objective_value = EvaluateObjective(0);
  for (size_t traj_index = 1; traj_index < 25; traj_index++) {
    current_objective_value = EvaluateObjective(traj_index);
    if (current_objective_value > best_traj_objective_value) {
      best_traj_index = traj_index;
      best_traj_objective_value = current_objective_value;
    }
  }

  //std::cout << "## best_traj_index was " << best_traj_index << std::endl;
  //std::cout << "## best_traj_objective_value " << best_traj_objective_value << std::endl; 

  desired_acceleration = trajectory_library.getTrajectoryFromIndex(best_traj_index).getAcceleration();

  return;
}

Eigen::Matrix<Scalar, 25, 1> TrajectorySelector::Normalize(Eigen::Matrix<Scalar, 25, 1> cost) {
  double largest = cost(0);
  size_t largest_index = 0;
  double current;
  for (size_t i = 1; i < cost.size(); i++) {
    current = cost(i);
    if (current > largest) {
      largest_index = i;
      largest = current;
    }
  }

  for (size_t i = 0; i < cost.size(); i++) {
    cost(i) = cost(i)/largest;
  }
  return cost;
}

float TrajectorySelector::EvaluateObjective(size_t index) {
  return DijkstraEvaluations(index) + 2*TerminalVelocityEvaluations(index);
}


void TrajectorySelector::EvaluateDijkstraCost(Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf) {

  ValueGrid* value_grid_ptr = value_grid_evaluator.GetValueGridPtr();

  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  size_t i = 0;
  Vector3 ortho_body_frame_position;
  geometry_msgs::PoseStamped pose_ortho_body_frame_position;
  geometry_msgs::PoseStamped pose_world_frame_position = PoseFromVector3(Vector3(0,0,0), "world");
  int current_value;
  // Iterate over trajectories
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    
    DijkstraEvaluations(i) = 0;
    double sampling_time;
    // Iterate over sampling times
    for (size_t time_index = 0; time_index < sampling_time_vector.size(); time_index++) {
      
      sampling_time = sampling_time_vector(time_index);
      ortho_body_frame_position = trajectory->getPosition(sampling_time);
      
      geometry_msgs::PoseStamped pose_ortho_body_frame_position = PoseFromVector3(ortho_body_frame_position, "ortho_body");
      tf2::doTransform(pose_ortho_body_frame_position, pose_world_frame_position, tf);
      Vector3 world_frame_position = VectorFromPose(pose_world_frame_position);

      // Then evaluate Dijkstra cost
      current_value = value_grid_ptr->GetValueOfPosition(world_frame_position);
      if ((current_value == 0) && ((world_frame_position - carrot_world_frame).norm() > 1.0)) {
        current_value = 1000;
      }

      DijkstraEvaluations(i) += current_value;

    }
    
    i++;
  }

  std::cout << "At the end of all this, my Dijkstra evaluations are: " << DijkstraEvaluations << std::endl;

};


void TrajectorySelector::computeBestTrajectory(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples, Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration) {
  //EvaluateCollisionProbabilities(point_cloud_xyz_samples); // INSTANTANEOUS LOW LATENCY
  EvaluateGoalProgress(carrot_body_frame); 
  EvaluateTerminalVelocityCost();

  desired_acceleration << 0,0,0;
  best_traj_index = 0;
  float current_objective_value;
  float best_traj_objective_value = GoalProgressEvaluations(0) + 2.0*TerminalVelocityEvaluations(0);
  for (size_t traj_index = 1; traj_index < 25; traj_index++) {
    current_objective_value = GoalProgressEvaluations(traj_index) + 2.0*TerminalVelocityEvaluations(traj_index);
    if (current_objective_value > best_traj_objective_value) {
      best_traj_index = traj_index;
      best_traj_objective_value = GoalProgressEvaluations(traj_index);
    }
  }

  //std::cout << "## best_traj_index was " << best_traj_index << std::endl;
  //std::cout << "## best_traj_objective_value " << best_traj_objective_value << std::endl; 

  desired_acceleration = trajectory_library.getTrajectoryFromIndex(best_traj_index).getAcceleration();

};


void TrajectorySelector::EvaluateGoalProgress(Vector3 const& carrot_body_frame) {

  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  double initial_distance = carrot_body_frame.norm();

  //std::cout << "initial_distance is " << initial_distance << std::endl;

  size_t i = 0;
  Vector3 final_trajectory_position;
  double distance;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    final_trajectory_position = trajectory->getTerminalStopPosition(final_time);
    distance = (final_trajectory_position - carrot_body_frame).norm();
    GoalProgressEvaluations(i) = initial_distance - distance; 
    i++;
  }
};


void TrajectorySelector::EvaluateTerminalVelocityCost() {

  // for each traj in trajectory_library.trajectories
  std::vector<Trajectory>::const_iterator trajectory_iterator_begin = trajectory_library.GetTrajectoryIteratorBegin();
  std::vector<Trajectory>::const_iterator trajectory_iterator_end = trajectory_library.GetTrajectoryIteratorEnd();

  size_t i = 0;
  double final_trajectory_speed;
  double distance;
  for (auto trajectory = trajectory_iterator_begin; trajectory != trajectory_iterator_end; trajectory++) {
    final_trajectory_speed = trajectory->getVelocity(final_time).norm();
    TerminalVelocityEvaluations(i) = 0;
    
    // cost on going too fast
    double soft_top_speed = 5.0;
    if (final_trajectory_speed > (soft_top_speed-1.0)) {
      TerminalVelocityEvaluations(i) -= ((soft_top_speed-1.0) - final_trajectory_speed)*((soft_top_speed-1.0) - final_trajectory_speed);
    }

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
    if (time_index < num_samples - 1) {
      sample_points_xyz_over_time.row(time_index) = trajectory_to_sample.getPosition(sampling_time);
    }
    else {
      sample_points_xyz_over_time.row(time_index) = trajectory_to_sample.getTerminalStopPosition(sampling_time);
    }
  }
  return sample_points_xyz_over_time;
}