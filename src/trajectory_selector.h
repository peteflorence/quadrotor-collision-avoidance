#ifndef TRAJECTORY_SELECTOR_H
#define TRAJECTORY_SELECTOR_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "trajectory_library.h"
#include "trajectory_evaluator.h"
#include "value_grid_evaluator.h"
#include "laser_scan_collision_evaluator.h"

// This ROS stuff should go.  Only temporary.
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseStamped.h"


class TrajectorySelector {
public:

  TrajectoryLibrary* GetTrajectoryLibraryPtr();
  ValueGridEvaluator* GetValueGridEvaluatorPtr();
  LaserScanCollisionEvaluator* GetLaserScanCollisionEvaluatorPtr();

  
  void InitializeLibrary(double const& final_time);
  size_t getNumTrajectories();
  
  Vector3 getSigmaAtTime(double const& t);
  Vector3 getInverseSigmaAtTime(double const & t);
  void computeBestTrajectory(Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration);
  void computeBestDijkstraTrajectory(Vector3 const& carrot_body_frame, Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf, size_t &best_traj_index, Vector3 &desired_acceleration);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sampleTrajectoryForDrawing(size_t trajectory_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples);

private:
  
  TrajectoryLibrary trajectory_library;
  TrajectoryEvaluator trajectory_evaluator;
  ValueGridEvaluator value_grid_evaluator;
  LaserScanCollisionEvaluator laser_scan_collision_evaluator;

  float EvaluateObjective(size_t index);
  Eigen::Matrix<Scalar, 25, 1> Normalize(Eigen::Matrix<Scalar, 25, 1> cost);
  double EvaluateWeightedObjectivesWithCollision(size_t const& trajectory_index);

  void EvaluateDijkstraCost(Vector3 const& carrot_world_frame, geometry_msgs::TransformStamped const& tf);
  void EvaluateGoalProgress(Vector3 const& carrot_body_frame);
  void EvaluateTerminalVelocityCost();
  void EvaluateCollisionProbabilities();
  double computeProbabilityOfCollisionOneTrajectory(Trajectory trajectory);
  double computeProbabilityOfCollisionOneStepOneObstacle(Vector3 const& trajectory_position, Vector3 const& point, Vector3 const& inverse_sigma_at_time);

  double final_time;
  double start_time = 0.0;

  Eigen::Matrix<Scalar, 10, 1> sampling_time_vector;

  Eigen::Matrix<Scalar, 20, 1> collision_sampling_time_vector;
  size_t num_samples_collision = collision_sampling_time_vector.size();

  Eigen::Matrix<Scalar, 25, 1> DijkstraEvaluations;
  Eigen::Matrix<Scalar, 25, 1> GoalProgressEvaluations;
  Eigen::Matrix<Scalar, 25, 1> TerminalVelocityEvaluations;
  Eigen::Matrix<Scalar, 25, 1> CollisionProbabilities;
  Eigen::Matrix<Scalar, 25, 1> NoCollisionProbabilities;

};

#endif