#include <iostream>
#include <Eigen/Dense>

class Trajectory {
public:
 
  void TestTrajectory();

  Trajectory(){};

  Trajectory(Eigen::Vector3d acceleration, Eigen::Vector3d initial_velocity) {
  	this->acceleration = acceleration;
  	this->initial_velocity = initial_velocity; 
  };
  
  void setInitialVelocity(Eigen::Vector3d const& initial_velocity);

  Eigen::Vector3d getPosition(double const& t);
  Eigen::Vector3d getVelocity(double const& t);
  Eigen::Matrix3d getCovariance(double const& t);


private:
  
  Eigen::Vector3d acceleration;
  Eigen::Vector3d initial_velocity;

};