#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "trajectory_library.h"
#include "trajectory_evaluator.h"


class TrajectorySelector {
public:
  void Test();
  void EvalTrajectories();

private:
  
  TrajectoryLibrary trajectory_library;
  TrajectoryEvaluator trajectory_evaluator;

};