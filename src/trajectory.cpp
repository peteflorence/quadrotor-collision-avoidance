#include "trajectory.h"

void Trajectory::TestTrajectory() {
  std::cout << "Printing from inside Trajectory " << std::endl;  
}

void Trajectory::setAccelerationMax(double const& acceleration_max) {
  this->a_max_horizontal = acceleration_max;
};

void Trajectory::setInitialVelocity(Vector3 const& initial_velocity_to_set) {
  initial_velocity = initial_velocity_to_set;
};

void Trajectory::setInitialAcceleration(Vector3 const& initial_acceleration_to_set) {
  initial_acceleration = initial_acceleration_to_set;
  jerk = (acceleration - initial_acceleration) / jerk_time;
  position_end_of_jerk_time = 0.1666*jerk*jerk_time*jerk_time*jerk_time + 0.5*initial_acceleration*jerk_time*jerk_time + initial_velocity*jerk_time;
  velocity_end_of_jerk_time = 0.5*jerk*jerk_time*jerk_time + initial_acceleration*jerk_time + initial_velocity;
};

void Trajectory::setAcceleration(Vector3 const& acceleration) {
  this->acceleration = acceleration;
};

Vector3 Trajectory::getAcceleration() const{
  return this->acceleration;
}

Vector3 Trajectory::getPosition(Scalar const& t) const {
  if (t < jerk_time) {
    return 0.1666*jerk*t*t*t + 0.5*initial_acceleration*t*t + initial_velocity*t;
  }
  else {
    double t_left = t - jerk_time;
    return position_end_of_jerk_time + 0.5*acceleration*t_left*t_left + initial_velocity*t_left;
  }
};

Vector3 Trajectory::getTerminalStopPosition(Scalar const& t) const {
  Vector3 position_end_of_trajectory = getPosition(t);
  Vector3 velocity_end_of_trajectory = getVelocity(t);

  double speed = velocity_end_of_trajectory.norm();
  
  Vector3 stopping_vector = -velocity_end_of_trajectory/speed;
  Vector3 max_stop_acceleration = a_max_horizontal*stopping_vector;
  Vector3 stopping_jerk = (max_stop_acceleration - acceleration) / jerk_time;
  Vector3 position_end_of_jerk_stop = 0.1666*stopping_jerk*jerk_time*jerk_time*jerk_time + 0.5*acceleration*jerk_time*jerk_time + velocity_end_of_trajectory*jerk_time + position_end_of_trajectory;
  Vector3 velocity_end_of_jerk_stop = 0.5*stopping_jerk*jerk_time*jerk_time + acceleration*jerk_time + velocity_end_of_trajectory;

  // check if stopped during jerk time
  if (velocity_end_of_trajectory.dot(velocity_end_of_jerk_stop) < 0) {
    return position_end_of_jerk_stop;
  }

  double realistic_stop_accel = a_max_horizontal*0.65;
  double speed_after_jerk = velocity_end_of_jerk_stop.norm();
  double stop_t_after_jerk = (speed_after_jerk / realistic_stop_accel);
  //double extra_drift = speed_after_jerk*0.200;
  double stopping_distance_after_jerk =  0.5 * -realistic_stop_accel * stop_t_after_jerk*stop_t_after_jerk + speed_after_jerk*stop_t_after_jerk;

  return position_end_of_jerk_stop + stopping_distance_after_jerk*-stopping_vector;

}

Vector3 Trajectory::getInitialVelocity() const {
  return initial_velocity;
};



Vector3 Trajectory::getVelocity(Scalar const& t) const {
  if (t < jerk_time) {
    return 0.5*jerk*t*t + initial_acceleration*t + initial_velocity; 
  }
  else {
    double t_left = t - jerk_time;
    return velocity_end_of_jerk_time + acceleration*t_left;
  }
};

Matrix3 Trajectory::getCovariance(Scalar const& t) const {
  //Vector3 covariances = initial_velocity*t*t;
  //return covariances.asDiagonal();
  return Matrix3::Identity();
};