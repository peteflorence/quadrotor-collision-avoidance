#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <Eigen/Dense>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3 ;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

class Trajectory {
public:
 
  void TestTrajectory();

  Trajectory(){};

  Trajectory(Vector3 acceleration, Vector3 initial_velocity) {
  	this->acceleration = acceleration;
  	this->initial_velocity = initial_velocity; 
  };
  
  void setInitialVelocity(Vector3 const& initial_velocity);

  Vector3 getPosition(Scalar const& t);
  Vector3 getVelocity(Scalar const& t);
  Matrix3 getCovariance(Scalar const& t);


private:
  
  Vector3 acceleration;
  Vector3 initial_velocity;

};

#endif