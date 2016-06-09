#include "trajectory.h"

void Trajectory::TestTrajectory() {
  std::cout << "Printing from inside Trajectory " << std::endl;  
}

void Trajectory::setAccelerationMax(double const& acceleration_max) {
  this->a_max_horizontal = acceleration_max;
};

void Trajectory::setInitialVelocity(Vector3 const& initial_velocity_to_set) {
  initial_velocity = initial_velocity_to_set;
};

void Trajectory::setInitialAcceleration(Vector3 const& initial_acceleration_to_set) {
  initial_acceleration = initial_acceleration_to_set;
};

void Trajectory::setAcceleration(Vector3 const& acceleration) {
  this->acceleration = acceleration;
};

Vector3 Trajectory::getAcceleration() const{
  return this->acceleration;
}

Vector3 Trajectory::getPosition(Scalar const& t) const {
  return 0.5*acceleration*t*t + initial_velocity*t;
};

Vector3 Trajectory::getTerminalStopPosition(Scalar const& t) const {
  Vector3 position_end_of_trajectory = getPosition(t);
  Vector3 velocity_end_of_trajectory = getVelocity(t);

  double speed = velocity_end_of_trajectory.norm();
  double stop_t = speed / a_max_horizontal;

  return 0.5*a_max_horizontal*(velocity_end_of_trajectory)/speed*stop_t*stop_t + velocity_end_of_trajectory*stop_t + position_end_of_trajectory;

}

Vector3 Trajectory::getInitialVelocity() const {
  return initial_velocity;
};



Vector3 Trajectory::getVelocity(Scalar const& t) const {
  return acceleration*t + initial_velocity;
};

Matrix3 Trajectory::getCovariance(Scalar const& t) const {
  //Vector3 covariances = initial_velocity*t*t;
  //return covariances.asDiagonal();
  return Matrix3::Identity();
};

// Vector1 Trajectory::MatrixSpeedTest(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance) const {
// 	return (robot_position - depth_position).transpose() * covariance.inverse() * (robot_position - depth_position);
// };

Vector1 Trajectory::MatrixSpeedTestVector(Vector3 const& robot_position, Vector3 const& depth_position, Vector3 const inverse_covariance_vector) const {
	return (robot_position - depth_position).transpose() * inverse_covariance_vector.cwiseProduct(robot_position - depth_position);
	//return (robot_position - depth_position).transpose() *(inverse_covariance_vector.array() * (robot_position - depth_position).array()).matrix();
};

// Vector1 Trajectory::MatrixSpeedTestLLT(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance) {
// 	Eigen::LLT<Matrix3> llt;
// 	llt.compute(covariance);
// 	return (robot_position - depth_position).transpose() * llt.solve(robot_position - depth_position);
// };

// Vector1 Trajectory::MatrixSpeedTestLDLT(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance) {
// 	Eigen::LDLT<Matrix3> ldlt(covariance);
// 	return (robot_position - depth_position).transpose() * ldlt.solve(robot_position - depth_position);
// };