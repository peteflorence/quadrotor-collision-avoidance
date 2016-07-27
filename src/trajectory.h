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
 
  Trajectory(){};

  Trajectory(Vector3 acceleration, Vector3 initial_velocity) {
  	this->acceleration = acceleration;
  	this->initial_velocity = initial_velocity; 
  	std::cout << "I'm initializing trajecotories " << std::endl;
  };


  void setAccelerationMax(double const& acceleration_max);

  void setAcceleration(Vector3 const& acceleration);
  void setInitialAcceleration(Vector3 const& initial_acceleration);
  void setInitialVelocity(Vector3 const& initial_velocity);
  
  Vector3 getAcceleration() const;
  Vector3 getVelocity(Scalar const& t) const;
  Vector3 getInitialVelocity() const;
  Vector3 getPosition(Scalar const& t) const;
  Vector3 getTerminalStopPosition(Scalar const& t) const;

  void setAccelerationLASER(Vector3 const& acceleration_laser);
  void setInitialAccelerationLASER(Vector3 const& initial_acceleration_laser);
  void setInitialVelocityLASER(Vector3 const& initial_velocity_laser);

  Vector3 getAccelerationLASER() const;
  Vector3 getVelocityLASER(Scalar const& t) const;
  Vector3 getInitialVelocityLASER() const;
  Vector3 getPositionLASER(Scalar const& t) const;
  Vector3 getTerminalStopPositionLASER(Scalar const& t) const;

  void setAccelerationRDF(Vector3 const& acceleration_rdf);
  void setInitialAccelerationRDF(Vector3 const& initial_acceleration_rdf);
  void setInitialVelocityRDF(Vector3 const& initial_velocity_rdf);

  Vector3 getInitialAccelerationRDF() const;
  Vector3 getAccelerationRDF() const;
  Vector3 getVelocityRDF(Scalar const& t) const;
  Vector3 getInitialVelocityRDF() const;
  Vector3 getPositionRDF(Scalar const& t) const;
  Vector3 getTerminalStopPositionRDF(Scalar const& t) const;

  Vector3 getPositionRDF_MonteCarlo(Scalar const& t, Vector3 const& sampled_initial_velcoity) const;
  
private:
  
  Vector3 acceleration;
  Vector3 initial_velocity;
  Vector3 initial_acceleration;
  Vector3 jerk;
  Vector3 position_end_of_jerk_time;
  Vector3 velocity_end_of_jerk_time;

  Vector3 acceleration_laser;
  Vector3 initial_velocity_laser;
  Vector3 initial_acceleration_laser;
  Vector3 jerk_laser;
  Vector3 position_end_of_jerk_time_laser;
  Vector3 velocity_end_of_jerk_time_laser;

  Vector3 acceleration_rdf;
  Vector3 initial_velocity_rdf;
  Vector3 initial_acceleration_rdf;
  Vector3 jerk_rdf;
  Vector3 position_end_of_jerk_time_rdf;
  Vector3 velocity_end_of_jerk_time_rdf;

  double a_max_horizontal;
  double jerk_time = 0.200;

};

#endif