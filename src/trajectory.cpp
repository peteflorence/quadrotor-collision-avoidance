#include "trajectory.h"

void Trajectory::TestTrajectory() {

  std::cout << "Printing from inside Trajectory " << std::endl;  

}

void Trajectory::setInitialVelocity(Vector3 const& initial_velocity) {
  this->initial_velocity = initial_velocity;
};

Vector3 Trajectory::getPosition(Scalar const& t) {
  return 0.5*acceleration*t*t + initial_velocity*t;
};

Vector3 Trajectory::getVelocity(Scalar const& t) {
  return acceleration*t + initial_velocity;
};

Matrix3 Trajectory::getCovariance(Scalar const& t) {
  //Vector3 covariances = initial_velocity*t*t;
  //return covariances.asDiagonal();
  return Matrix3::Identity();
};