#include "trajectory_library.h"
#include <chrono>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

void TrajectoryLibrary::TestLibrary() {

  std::cout << "Printing from inside TrajectoryLibrary " << std::endl;  
  trajectory.TestTrajectory();

  Vector3 acceleration, initial_velocity;
  acceleration << 1,1,1;
  initial_velocity << 2,2,0;

  Trajectory trajectory_init = Trajectory(acceleration, initial_velocity); 
  std::cout << trajectory_init.getPosition(10.0) << std::endl << "is position at 10.0" << std::endl;

  std::cout << trajectory_init.getVelocity(10.0) << std::endl << "is velocity at 10.0" << std::endl;

  std::cout << trajectory_init.getCovariance(10.0) << std::endl << "is covariance at 10.0" << std::endl;


  auto t1 = std::chrono::high_resolution_clock::now();
  for (Scalar t = 0; t < 1000000; t++) {
  	trajectory_init.getPosition(t);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "that took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
              << " milliseconds\n";

}