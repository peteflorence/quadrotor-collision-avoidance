#include "trajectory_library.h"
#include <chrono>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 1, 1> Vector1;


void TrajectoryLibrary::Initialize2DLibrary() {
	for (double i = 0; i < 10; i++) {
		trajectories.push_back(Trajectory( Vector3(i,i,i), Vector3(i,i,i) ));
	}
};


void TrajectoryLibrary::TestLibrary() {

  std::cout << "Printing from inside TrajectoryLibrary " << std::endl;  
  trajectory1.TestTrajectory();

  Vector3 acceleration, initial_velocity;
  acceleration << 1,1,1;
  initial_velocity << 2,2,0;

  Trajectory trajectory_init = Trajectory(acceleration, initial_velocity); 
  std::cout << trajectory_init.getPosition(10.0) << std::endl << "is position at 10.0" << std::endl;

  std::cout << trajectory_init.getVelocity(10.0) << std::endl << "is velocity at 10.0" << std::endl;

  std::cout << trajectory_init.getCovariance(10.0) << std::endl << "is covariance at 10.0" << std::endl;

  Vector3 robot_position, depth_position;
  robot_position << 1, 2, 3;
  depth_position << 7, 8, 9;

  Matrix3 covariance;
  covariance << 1, 0, 0, 0, 2, 0, 0, 0, 3;

  Vector3 inverse_covariance_vector;
  inverse_covariance_vector << 1.0/1, 1.0/2, 1.0/3;


  auto t1 = std::chrono::high_resolution_clock::now();
  for (Scalar t = 0; t < 1000000; t++) {
  	trajectory_init.MatrixSpeedTestVector(robot_position, depth_position, inverse_covariance_vector);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "that took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
              << " milliseconds\n";

}