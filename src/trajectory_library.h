#include <iostream>
#include "trajectory.h"
#include <vector>

class TrajectoryLibrary {
public:
  void TestLibrary();

  void Initialize2DLibrary();

  void setInitialVelocity(Vector3 const& initialVelocity);
  void setInitialVelocityJustOneTrajectory(Vector3 const& initialVelocity);

  Trajectory getTrajectoryFromIndex(size_t index);

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
 
};

