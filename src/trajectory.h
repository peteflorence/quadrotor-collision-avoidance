#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <Eigen/Dense>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 1, 1> Vector1;

class Trajectory {
public:
 
  void TestTrajectory();

  Trajectory(){};

  Trajectory(Vector3 acceleration, Vector3 initial_velocity) {
  	this->acceleration = acceleration;
  	this->initial_velocity = initial_velocity; 
  	std::cout << "I'm initializing trajecotories " << std::endl;
  };


  void setAccelerationMax(double const& acceleration_max);
  void setAcceleration(Vector3 const& acceleration);
  void setInitialVelocity(Vector3 const& initial_velocity);
  void setInitialAcceleration(Vector3 const& initial_acceleration);
  Vector3 getInitialVelocity() const;

  Vector3 getAcceleration() const;
  Vector3 getPosition(Scalar const& t) const;
  Vector3 getTerminalStopPosition(Scalar const& t) const;
  Vector3 getVelocity(Scalar const& t) const;
  Matrix3 getCovariance(Scalar const& t) const;
  //Vector1 MatrixSpeedTest(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance) const;
  Vector1 MatrixSpeedTestVector(Vector3 const& robot_position, Vector3 const& depth_position, Vector3 const inverse_covariance_vector) const;
  // Vector1 MatrixSpeedTestLLT(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance);
  // Vector1 MatrixSpeedTestLDLT(Vector3 const& robot_position, Vector3 const& depth_position, Matrix3 const covariance);


private:
  
  Vector3 acceleration;
  Vector3 initial_velocity;
  Vector3 initial_acceleration;
  Vector3 jerk;
  Vector3 position_end_of_jerk_time;
  Vector3 velocity_end_of_jerk_time;

  double a_max_horizontal;
  double jerk_time = 0.100;

};

#endif