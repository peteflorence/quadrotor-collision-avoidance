#include "trajectory.h"

void Trajectory::TestTrajectory() {
  std::cout << "Printing from inside Trajectory " << std::endl;  
}

void Trajectory::setInitialVelocity(Vector3 const& initial_velocity_to_set) {
  //std::cout << "I'm going to set velocity to " << initial_velocity_to_set << std::endl;
  //std::cout << "before setting velocity " << initial_velocity << std::endl;
  initial_velocity = initial_velocity_to_set;
  //std::cout << "after setting velocity " << initial_velocity << std::endl;
};

void Trajectory::setAcceleration(Vector3 const& acceleration) {
  this->acceleration = acceleration;
};

Vector3 Trajectory::getAcceleration() const{
  return this->acceleration;
}

Vector3 Trajectory::getPosition(Scalar const& t) const {
  //std::cout << "My initial velocity is " << initial_velocity << std::endl;
  //std::cout << "My acceleration is " << acceleration << std::endl;
  return 0.5*acceleration*t*t + initial_velocity*t;

  // do rotation by roll and pitch here

};

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