#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "trajectory_library.h"
#include "trajectory_evaluator.h"


class TrajectorySelector {
public:
  void Test();
  
  void InitializeLibrary();
  void setInitialVelocity(Vector3 const& initialVelocity);
  size_t getNumTrajectories();
  
  Vector3 getSigmaAtTime(double const& t);
  Vector3 computeAccelerationDesiredFromBestTrajectory(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sampleTrajectoryForDrawing(size_t trajectory_index, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector, size_t num_samples);

private:
  
  TrajectoryLibrary trajectory_library;
  TrajectoryEvaluator trajectory_evaluator;

  void EvalAllTrajectories(Eigen::Matrix<Scalar, 100, 3> const& point_cloud_xyz_samples);

};