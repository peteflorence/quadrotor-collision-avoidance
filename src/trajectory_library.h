#include <iostream>
#include "trajectory.h"
#include <vector>

class TrajectoryLibrary {
public:
  void TestLibrary();

  void Initialize2DLibrary(double const& final_time);

  void setInitialVelocityAllTrajectories(Vector3 const& initialVelocity);
  void setInitialVelocityJustOneTrajectory(Vector3 const& initialVelocity);

  void setRollPitch(double const& roll, double const& pitch) {
    this->roll = roll;
    this->pitch = pitch;
  };
  void setThrust(double const& thrust) {
    this->thrust = thrust;
  };

  void updateInitialAcceleration();
  Vector3 getInitialAcceleration() const{
    if (initial_acceleration(0) == 0) {
      std::cout << "I had initial acceleration of 0 and r,p,t were " << roll << " " << pitch << " " << thrust << std::endl; 
    }
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



private:
  
  double a_max = 9.8*2.4;  // 2.4 Thrust to weight ratio

  std::vector<Trajectory> trajectories;
  Trajectory trajectory1;

  Vector3 initial_velocity = Vector3(0,0,0);
  Vector3 initial_acceleration = Vector3(0,0,0);

  double roll = 0;
  double pitch = 0;
  double thrust = 0;

  double final_time;
 
};

