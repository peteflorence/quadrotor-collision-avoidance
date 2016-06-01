#include "trajectory.h"

void Trajectory::TestTrajectory() {

  std::cout << "Printing from inside Trajectory " << std::endl;  

}

void Trajectory::setInitialVelocity(Eigen::Vector3d const& initial_velocity) {
  this->initial_velocity = initial_velocity;
};

Eigen::Vector3d Trajectory::getPosition(double const& t) {
  return 0.5*acceleration*t*t + initial_velocity*t;
};

Eigen::Vector3d Trajectory::getVelocity(double const& t) {
  return acceleration*t + initial_velocity;
};

Eigen::Matrix3d Trajectory::getCovariance(double const& t) {
  Eigen::Vector3d covariances = initial_velocity*t*t;
  return covariances.asDiagonal();
};