#ifndef MOTION_SELECTOR_H
#define MOTION_SELECTOR_H

#include "motion_library.h"
#include "depth_image_collision_evaluator.h"
#include "value_grid_evaluator.h"

#include <Eigen/Dense>
#include <math.h>

// This ROS stuff should go.  Only temporary.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseStamped.h"

class MotionSelector {
public:

  MotionLibrary* GetMotionLibraryPtr();
  ValueGridEvaluator* GetValueGridEvaluatorPtr();
  DepthImageCollisionEvaluator* GetDepthImageCollisionEvaluatorPtr();

  void InitializeLibrary(double const& final_time, double soft_top_speed, double a_max_horizontal, double, double);
  void UpdateTimeHorizon(double const& final_time);
  size_t getNummotions();
  
  void computeTakeoffMotion(Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration);
  void computeBestEuclideanMotion(Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration);
  void computeBestDijkstraMotion(Vector3 const& carrot_body_frame, Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf, size_t &best_traj_index, Vector3 &desired_acceleration);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sampleMotionForDrawing(size_t motion_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples);

  Eigen::Matrix<Scalar, 25, 1> getCollisionProbabilities() {
    return collision_probabilities;
  }

  void SetSoftTopSpeed(double top_speed) {this->soft_top_speed = top_speed;}

private:
  
  MotionLibrary motion_library;
  ValueGridEvaluator value_grid_evaluator;
  DepthImageCollisionEvaluator depth_image_collision_evaluator;

  // For Euclidean
  void EvaluateObjectivesEuclid();
  double EvaluateWeightedObjectiveEuclid(size_t const& motion_index);
 
  // For Dijkstra
  void EvaluateObjectivesDijkstra();
  double EvaluateWeightedObjectiveDijkstra(size_t index);
  
  // Evaluate individual objectives
  void EvaluateDijkstraCost(Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf);
  void EvaluateGoalProgress(Vector3 const& carrot_body_frame);
  void EvaluateTerminalVelocityCost();
  void EvaluateCollisionProbabilities();
  double computeProbabilityOfCollisionOneMotion(Motion motion);
  double computeProbabilityOfCollisionOneMotion_MonteCarlo(Motion motion, std::vector<Vector3> sampled_initial_velocities, size_t n);


  Eigen::Matrix<Scalar, 25, 1> FilterSmallProbabilities(Eigen::Matrix<Scalar, 25, 1> to_filter);
  Eigen::Matrix<Scalar, 25, 1> Normalize0to1(Eigen::Matrix<Scalar, 25, 1> cost);
  Eigen::Matrix<Scalar, 25, 1> MakeAllGreaterThan1(Eigen::Matrix<Scalar, 25, 1> cost);
  
  double final_time;
  double start_time = 0.0;

  Eigen::Matrix<Scalar, 10, 1> sampling_time_vector;

  Eigen::Matrix<Scalar, 20, 1> collision_sampling_time_vector;
  size_t num_samples_collision = collision_sampling_time_vector.size();

  Eigen::Matrix<Scalar, 25, 1> dijkstra_evaluations;
  Eigen::Matrix<Scalar, 25, 1> goal_progress_evaluations;
  Eigen::Matrix<Scalar, 25, 1> terminal_velocity_evaluations;
  Eigen::Matrix<Scalar, 25, 1> collision_probabilities;
  Eigen::Matrix<Scalar, 25, 1> no_collision_probabilities;

  Eigen::Matrix<Scalar, 25, 1> objectives_dijkstra;
  Eigen::Matrix<Scalar, 25, 1> objectives_euclid;

  double soft_top_speed;

  double collision_reward = -10000;
  //double collision_reward = -100000000; // PLAGUE setting

  Vector3 last_desired_acceleration;

};

#endif
