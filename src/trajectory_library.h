#include <iostream>
#include "trajectory.h"
#include <vector>

class TrajectoryLibrary {
public:
  void TestLibrary();

  void Initialize2DLibrary();

  std::vector<Trajectory>::const_iterator GetTrajectoryIteratorBegin() const {
	return trajectories.begin(); 
  };

  std::vector<Trajectory>::const_iterator GetTrajectoryIteratorEnd() const {
	return trajectories.end(); 
  };



private:
  

  std::vector<Trajectory> trajectories;
  Trajectory trajectory1;
 
};

