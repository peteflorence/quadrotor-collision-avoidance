#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H


#include <iostream>
#include "trajectory.h"
#include <vector>

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

class TrajectoryLibrary {
public:

  void Initialize2DLibrary();

  void setInitialVelocity(Vector3 const& initialVelocity);

  void setRollPitch(double const& roll, double const& pitch) {
    this->roll = roll;
    this->pitch = pitch;
    updateInitialAcceleration();
  };
  void setThrust(double const& thrust) {
    this->thrust = thrust;
  };

  void updateInitialAcceleration();
  Vector3 getInitialAcceleration() const{
    return initial_acceleration;
  }


  Trajectory getTrajectoryFromIndex(size_t index);
  size_t getNumTrajectories();
  Vector3 getSigmaAtTime(double const& t);
  Vector3 getInverseSigmaAtTime(double const& t);

  std::vector<Trajectory>::const_iterator GetTrajectoryIteratorBegin() const {
	return trajectories.begin(); 
  };

  std::vector<Trajectory>::const_iterator GetTrajectoryIteratorEnd() const {
	return trajectories.end(); 
  };

  std::vector<Trajectory>::iterator GetTrajectoryNonConstIteratorBegin() {
  return trajectories.begin(); 
  };

  std::vector<Trajectory>::iterator GetTrajectoryNonConstIteratorEnd() {
  return trajectories.end(); 
  };

  void setInitialAccelerationLASER(Vector3 const& initial_acceleration_laser_frame);
  void setInitialVelocityLASER(Vector3 const& initial_velocity_laser_frame);

  Vector3 getLASERSigmaAtTime(double const& t);
  Vector3 getLASERInverseSigmaAtTime(double const& t);

  void setInitialAccelerationRDF(Vector3 const& initial_acceleration_laser_frame);
  void setInitialVelocityRDF(Vector3 const& initial_velocity_laser_frame);

  Vector3 getRDFSigmaAtTime(double const& t) const;
  Vector3 getRDFInverseSigmaAtTime(double const& t) const;

  std::vector<Vector3> getRDFSampledInitialVelocity(size_t n);



private:
  
  double a_max = 9.8*2.4;  // 2.4 Thrust to weight ratio

  std::vector<Trajectory> trajectories;
  Trajectory trajectory1;

  Vector3 initial_velocity = Vector3(0,0,0);
  Vector3 initial_acceleration = Vector3(0,0,0);

  Vector3 initial_velocity_laser_frame = Vector3(0,0,0);
  Vector3 initial_acceleration_laser_frame = Vector3(0,0,0);

  Vector3 initial_velocity_rdf_frame = Vector3(0,0,0);
  Vector3 initial_acceleration_rdf_frame = Vector3(0,0,0);

  double roll = 0;
  double pitch = 0;
  double thrust = 0;

  std::vector<Vector3> sampled_velocities;

};

#endif