#include "motion_selector.h"

MotionLibrary* MotionSelector::GetMotionLibraryPtr() {
  return &motion_library;
};

ValueGridEvaluator* MotionSelector::GetValueGridEvaluatorPtr() {
  return &value_grid_evaluator;
};

DepthImageCollisionEvaluator* MotionSelector::GetDepthImageCollisionEvaluatorPtr() {
  return &depth_image_collision_evaluator;
};


void MotionSelector::InitializeLibrary(bool use_3d_library, double const& final_time, double soft_top_speed, double a_max_horizontal, double min_speed_at_max_acceleration_total, double max_acceleration_total) {
  this->use_3d_library = use_3d_library;

  motion_library.InitializeLibrary(use_3d_library, a_max_horizontal, min_speed_at_max_acceleration_total, max_acceleration_total);
  InitializeObjectiveVectors();
  this->soft_top_speed = soft_top_speed;
  last_desired_acceleration << 0, 0, 0;
  this->final_time = final_time;

  size_t num_samples = 10;
  double sampling_time = 0;
  double sampling_interval = (final_time - start_time) / num_samples;
  for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      sampling_time_vector(sample_index) = sampling_time;
  }

  sampling_time = 0;
  sampling_interval = (final_time - start_time) / num_samples_collision;
  for (size_t sample_index = 0; sample_index < num_samples_collision; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      collision_sampling_time_vector(sample_index) = sampling_time;
  }
};

void MotionSelector::InitializeObjectiveVectors() {
  for (size_t i = 0; i < getNumMotions(); i++) {
    dijkstra_evaluations.push_back(0.0);
    goal_progress_evaluations.push_back(0.0);
    terminal_velocity_evaluations.push_back(0.0);
    altitude_evaluations.push_back(0.0);

    collision_probabilities.push_back(0.0);
    no_collision_probabilities.push_back(0.0);

    objectives_dijkstra.push_back(0.0);
    objectives_euclid.push_back(0.0);
  }
}

void MotionSelector::UpdateTimeHorizon(double const& final_time) {
  this->final_time = final_time;

  size_t num_samples = 10;
  double sampling_time = 0;
  double sampling_interval = (final_time - start_time) / num_samples;
  for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      sampling_time_vector(sample_index) = sampling_time;
  }

  sampling_time = 0;
  sampling_interval = (final_time - start_time) / num_samples_collision;
  for (size_t sample_index = 0; sample_index < num_samples_collision; sample_index++) {
      sampling_time = start_time + sampling_interval*(sample_index+1);
      collision_sampling_time_vector(sample_index) = sampling_time;
  }
};


size_t MotionSelector::getNumMotions() {
  return motion_library.getNumMotions();
};

// Euclidean Evaluator
void MotionSelector::computeBestEuclideanMotion(Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration) {
  EvaluateCollisionProbabilities();
  EvaluateGoalProgress(carrot_body_frame); 
  EvaluateTerminalVelocityCost();
  if (use_3d_library) {EvaluateAltitudeCost();};
  EvaluateObjectivesEuclid();

  desired_acceleration << 0,0,0;
  best_traj_index = 0;
  float current_objective_value;
  float best_traj_objective_value = objectives_euclid.at(0);
  for (size_t traj_index = 1; traj_index < getNumMotions(); traj_index++) {
    current_objective_value = objectives_euclid.at(traj_index);
    if (current_objective_value > best_traj_objective_value) {
      best_traj_index = traj_index;
      best_traj_objective_value = current_objective_value;
    }
  }

  double angle_to_goal = 0;
  if (carrot_body_frame(2) != 0) {
    angle_to_goal = atan2(carrot_body_frame(1), carrot_body_frame(0));
    if (angle_to_goal < 0.0) {
      angle_to_goal = -1 * angle_to_goal;
    }
    angle_to_goal = 180.0/M_PI * angle_to_goal;
  }

  if ( (collision_probabilities.at(0) < 0.05) && (angle_to_goal < 30) && (carrot_body_frame.norm() > motion_library.getMotionFromIndex(0).getTerminalStopPosition(0.5).norm() ))  {
    best_traj_index = 0;
  }

  desired_acceleration = motion_library.getMotionFromIndex(best_traj_index).getAcceleration();
  last_desired_acceleration = desired_acceleration;
};

void MotionSelector::EvaluateObjectivesEuclid() {
  for (int i = 0; i < getNumMotions(); i++) {
    objectives_euclid.at(i) = EvaluateWeightedObjectiveEuclid(i)*no_collision_probabilities.at(i) + collision_reward*collision_probabilities.at(i);
  }
}

double MotionSelector::EvaluateWeightedObjectiveEuclid(size_t const& motion_index) {
  return goal_progress_evaluations.at(motion_index) + terminal_velocity_evaluations.at(motion_index) + altitude_evaluations.at(motion_index);
}


// Dijkstra Evaluator
void MotionSelector::computeBestDijkstraMotion(Vector3 const& carrot_body_frame, Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf, size_t &best_traj_index, Vector3 &desired_acceleration) {
  EvaluateCollisionProbabilities();
  EvaluateDijkstraCost(carrot_world_frame, tf);
  EvaluateGoalProgress(carrot_body_frame); 
  EvaluateTerminalVelocityCost();
  EvaluateObjectivesDijkstra();

  desired_acceleration << 0,0,0;
  best_traj_index = 0;
  double current_objective_value;
  double best_traj_objective_value = objectives_dijkstra.at(0);
  for (size_t traj_index = 1; traj_index < getNumMotions(); traj_index++) {
    current_objective_value = objectives_dijkstra.at(traj_index);
    if (current_objective_value > best_traj_objective_value) {
      best_traj_index = traj_index;
      best_traj_objective_value = current_objective_value;
    }
  }
  desired_acceleration = motion_library.getMotionFromIndex(best_traj_index).getAcceleration();
  return;
}

void MotionSelector::EvaluateObjectivesDijkstra() {
  for (int i = 0; i < getNumMotions(); i++) {
    objectives_dijkstra.at(i) = EvaluateWeightedObjectiveDijkstra(i)*no_collision_probabilities.at(i) + collision_reward*collision_probabilities.at(i);
  }
}


double MotionSelector::EvaluateWeightedObjectiveDijkstra(size_t index) {
  return dijkstra_evaluations.at(index) + 1.0*terminal_velocity_evaluations.at(index);
}

void MotionSelector::EvaluateDijkstraCost(Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf) {

  ValueGrid* value_grid_ptr = value_grid_evaluator.GetValueGridPtr();

  std::vector<Motion>::const_iterator motion_iterator_begin = motion_library.GetMotionIteratorBegin();
  std::vector<Motion>::const_iterator motion_iterator_end = motion_library.GetMotionIteratorEnd();

  size_t i = 0;
  Vector3 ortho_body_frame_position;
  geometry_msgs::PoseStamped pose_ortho_body_frame_position;
  geometry_msgs::PoseStamped pose_world_frame_position = PoseFromVector3(Vector3(0,0,0), "world");
  int current_value;
  // Iterate over motions
  for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
    
   dijkstra_evaluations.at(i) = 0;
    double sampling_time;
    //Iterate over sampling times
    for (size_t time_index = 0; time_index < sampling_time_vector.size(); time_index++) {
      
      sampling_time = sampling_time_vector(time_index);
      ortho_body_frame_position = motion->getPosition(sampling_time);
      
      geometry_msgs::PoseStamped pose_ortho_body_frame_position = PoseFromVector3(ortho_body_frame_position, "ortho_body");
      tf2::doTransform(pose_ortho_body_frame_position, pose_world_frame_position, tf);
      Vector3 world_frame_position = VectorFromPose(pose_world_frame_position);

      // // Then evaluate Dijkstra cost
      current_value = value_grid_ptr->GetValueOfPosition(world_frame_position);

      if ((current_value == 0) && ((world_frame_position - carrot_world_frame).norm() > 1.0)) {
        current_value = 1000;
      }

      dijkstra_evaluations.at(i) -= current_value;

    }
    i++;
  }
};

void MotionSelector::EvaluateGoalProgress(Vector3 const& carrot_body_frame) {
  std::vector<Motion>::const_iterator motion_iterator_begin = motion_library.GetMotionIteratorBegin();
  std::vector<Motion>::const_iterator motion_iterator_end = motion_library.GetMotionIteratorEnd();
  double initial_distance = carrot_body_frame.norm();
  double time_to_eval = 0.5;
  size_t i = 0;
  Vector3 final_motion_position;
  double distance;
  for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
    final_motion_position = motion->getTerminalStopPosition(time_to_eval);
    distance = (final_motion_position - carrot_body_frame).norm();
    goal_progress_evaluations.at(i) = initial_distance - distance; 
    i++;
  }
};

void MotionSelector::EvaluateTerminalVelocityCost() {
  std::vector<Motion>::const_iterator motion_iterator_begin = motion_library.GetMotionIteratorBegin();
  std::vector<Motion>::const_iterator motion_iterator_end = motion_library.GetMotionIteratorEnd();
  size_t i = 0;
  double final_motion_speed;
  double distance;
  for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
    final_motion_speed = motion->getVelocity(0.5).norm();
    terminal_velocity_evaluations.at(i) = 0;
    if (final_motion_speed > soft_top_speed) {
      terminal_velocity_evaluations.at(i) -= 2.0*(soft_top_speed - final_motion_speed)*(soft_top_speed - final_motion_speed);
    }
    i++;
  }
};

void MotionSelector::EvaluateAltitudeCost() {
  std::vector<Motion>::const_iterator motion_iterator_begin = motion_library.GetMotionIteratorBegin();
  std::vector<Motion>::const_iterator motion_iterator_end = motion_library.GetMotionIteratorEnd();
  size_t i = 0;
  double minimum_altitude = 0.7;
  double maximum_altitude = 5.0;
  double final_altitude;
  for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
    final_altitude = motion->getPosition(0.1)(2);
    altitude_evaluations.at(i) = 0;
    if (final_altitude < minimum_altitude) {
      altitude_evaluations.at(i) -= 10.0*(final_altitude - minimum_altitude)*(final_altitude - minimum_altitude);
    }
    else if (final_altitude > maximum_altitude) {
      altitude_evaluations.at(i) -= 10.0*(final_altitude - minimum_altitude)*(final_altitude - minimum_altitude);
    }
    i++;
  }
}

void MotionSelector::EvaluateCollisionProbabilities() {
  std::vector<Motion>::const_iterator motion_iterator_begin = motion_library.GetMotionIteratorBegin();
  std::vector<Motion>::const_iterator motion_iterator_end = motion_library.GetMotionIteratorEnd();
  size_t i = 0;
  for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
    collision_probabilities.at(i) = computeProbabilityOfCollisionOneMotion(*motion);
    no_collision_probabilities.at(i) = 1.0 - collision_probabilities.at(i); 
    i++;
  }
};

double MotionSelector::computeProbabilityOfCollisionOneMotion(Motion motion) {
  double probability_no_collision = 1;
  double probability_no_collision_one_step = 1.0;
  double probability_of_collision_one_step_one_depth = 1.0;
  Vector3 robot_position;
  Vector3 robot_position_rdf;
  Vector3 sigma_robot_position;

  for (size_t time_step_index = 0; time_step_index < num_samples_collision; time_step_index++) {
    sigma_robot_position = 0.1*motion_library.getLASERSigmaAtTime(collision_sampling_time_vector(time_step_index)); 
    robot_position = motion.getPositionLASER(collision_sampling_time_vector(time_step_index));
    probability_no_collision_one_step = 1 - depth_image_collision_evaluator.computeProbabilityOfCollisionNPositionsKDTree_Laser(robot_position, sigma_robot_position);

    sigma_robot_position = 0.1*motion_library.getSigmaAtTime(collision_sampling_time_vector(time_step_index)); 
    robot_position = motion.getPosition(collision_sampling_time_vector(time_step_index));
    robot_position_rdf = motion.getPositionRDF(collision_sampling_time_vector(time_step_index));
    
    probability_of_collision_one_step_one_depth = depth_image_collision_evaluator.computeProbabilityOfCollisionNPositionsKDTree_DepthImage(robot_position, sigma_robot_position);
    probability_of_collision_one_step_one_depth = depth_image_collision_evaluator.AddOutsideFOVPenalty(robot_position_rdf, probability_of_collision_one_step_one_depth);

    probability_no_collision_one_step = probability_no_collision_one_step * (1 - probability_of_collision_one_step_one_depth);
    probability_no_collision = probability_no_collision * probability_no_collision_one_step;    
  }
  return 1.0 - probability_no_collision;
};

double MotionSelector::computeProbabilityOfCollisionOneMotion_MonteCarlo(Motion motion, std::vector<Vector3> sampled_initial_velocities, size_t n) { 
  Vector3 robot_position;
  size_t collision_count = 0;
  for (size_t i = 0; i < n; i++) {
    for (size_t time_step_index = 0; time_step_index < num_samples_collision; time_step_index++) {
      robot_position = motion.getPositionRDF_MonteCarlo(collision_sampling_time_vector(time_step_index), sampled_initial_velocities[i]);
      
      if (depth_image_collision_evaluator.computeDeterministicCollisionOnePositionKDTree(robot_position)) {
        collision_count++;
        break;
      }
    }
  }
  return (collision_count*1.0)/(n*1.0);
};

Eigen::Matrix<Scalar, Eigen::Dynamic, 3> MotionSelector::sampleMotionForDrawing(size_t motion_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples) {
  Motion motion_to_sample = motion_library.getMotionFromIndex(motion_index);
  double sampling_time;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time(num_samples,3);

  for (size_t time_index = 0; time_index < num_samples; time_index++) {
    sampling_time = sampling_time_vector(time_index);
    if (time_index < num_samples - 1) {
      sample_points_xyz_over_time.row(time_index) = motion_to_sample.getPosition(sampling_time);
    }
    else {
      sample_points_xyz_over_time.row(time_index) = motion_to_sample.getPosition(sampling_time);
    }
  }
  return sample_points_xyz_over_time;
}
