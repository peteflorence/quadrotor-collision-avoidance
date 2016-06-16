#ifndef TRAJECTORY_SELECTOR_H
#define TRAJECTORY_SELECTOR_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "trajectory_library.h"
#include "trajectory_evaluator.h"
#include "value_grid.h"

#include <nav_msgs/OccupancyGrid.h>


class TrajectorySelector {
public:
  

  void Test();

  TrajectoryLibrary* getTrajectoryLibraryPtr();
  
  void InitializeLibrary(double const& final_time);
  size_t getNumTrajectories();
  
  Vector3 getSigmaAtTime(double const& t);
  Vector3 getInverseSigmaAtTime(double const & t);
  void computeBestTrajectory(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples, Vector3 const& carrot_body_frame, size_t &best_traj_index, Vector3 &desired_acceleration);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sampleTrajectoryForDrawing(size_t trajectory_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples);
  void PassInUpdatedValueGrid(nav_msgs::OccupancyGrid * value_grid);

private:
  
  TrajectoryLibrary trajectory_library;
  TrajectoryEvaluator trajectory_evaluator;
  ValueGrid value_grid;

  void EvaluateGoalProgress(Vector3 const& carrot_body_frame);
  void EvaluateTerminalVelocityCost(Vector3 const& carrot_body_frame);
  void EvaluateCollisionProbabilities(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples);
  double computeProbabilityOfCollisionOneTrajectory(Trajectory trajectory, Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples);
  double computeProbabilityOfCollisionOneStepOneObstacle(Vector3 const& trajectory_position, Vector3 const& point, Vector3 const& inverse_sigma_at_time);

  double final_time;
  double start_time = 0.0;

  Eigen::Matrix<Scalar, 10, 1> sampling_time_vector;

  Eigen::Matrix<Scalar, 25, 1> GoalProgressEvaluations;
  Eigen::Matrix<Scalar, 25, 1> TerminalVelocityEvaluations;
  Eigen::Matrix<Scalar, 25, 1> CollisionProbabilities;

};

#endif