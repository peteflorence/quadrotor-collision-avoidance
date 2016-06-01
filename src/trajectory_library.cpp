#include "trajectory_library.h"

void TrajectoryLibrary::TestLibrary() {

  std::cout << "Printing from inside TrajectoryLibrary " << std::endl;  
  trajectory.TestTrajectory();

  Eigen::Vector3d acceleration, initial_velocity;
  acceleration << 1,1,1;
  initial_velocity << 2,2,2;

  Trajectory trajectory_init = Trajectory(acceleration, initial_velocity); 
  std::cout << trajectory_init.getPosition(10.0) << std::endl << "is position at 10.0" << std::endl;

  std::cout << trajectory_init.getVelocity(10.0) << std::endl << "is velocity at 10.0" << std::endl;

  std::cout << trajectory_init.getCovariance(10.0) << std::endl << "is covariance at 10.0" << std::endl;



}