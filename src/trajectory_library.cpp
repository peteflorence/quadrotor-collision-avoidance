#include "trajectory_library.h"
#include <chrono>
#include <math.h>

void TrajectoryLibrary::Initialize2DLibrary(double const& final_time) {
	//double a_max_horizontal = sqrt(a_max*a_max - 9.8*9.8);
	double a_max_horizontal = 9.8*0.3;
	Vector3 zero_initial_velocity = Vector3(0,0,0);

	// Make first trajectory be zero accelerations
	Vector3 acceleration = Vector3(0,0,0);
	trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));

	// Make next 8 trajectories sample around maximum horizontal acceleration
	for (double i = 1; i < 9; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*a_max_horizontal, sin(theta)*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	// Make next 8 trajectories sample around 0.6 * maximum horizontal acceleration
	for (double i = 9; i < 17; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.6*a_max_horizontal, sin(theta)*0.6*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	// Make next 8 trajectories sample around 0.3 * maximum horizontal acceleration
	for (double i = 17; i < 25; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.3*a_max_horizontal, sin(theta)*0.3*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	this->final_time = final_time; 

};

void TrajectoryLibrary::setInitialVelocityAllTrajectories(Vector3 const& initialVelocity) {
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialVelocity(initialVelocity);
	}
	initial_velocity = initialVelocity;
};

void TrajectoryLibrary::setInitialVelocityJustOneTrajectory(Vector3 const& initialVelocity) {
	trajectories.at(0).setInitialVelocity(initialVelocity);
};


Trajectory TrajectoryLibrary::getTrajectoryFromIndex(size_t index) {
	return trajectories.at(index);
};

size_t TrajectoryLibrary::getNumTrajectories() {
	return trajectories.size();
};

Vector3 TrajectoryLibrary::getSigmaAtTime(double const& t) {
	return t*(Vector3(1.5,1.5,1.5) + 0.1*(initial_velocity.array().abs()).matrix());
};

Vector3 TrajectoryLibrary::getInverseSigmaAtTime(double const& t) {
	Vector3 sigma = getSigmaAtTime(t);
	return Vector3(1.0/sigma(0), 1.0/sigma(1), 1.0/sigma(2));
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