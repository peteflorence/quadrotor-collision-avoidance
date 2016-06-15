#include <iostream>
#include "trajectory.h"
#include <math.h>

class AttitudeGenerator {
public:
  void TestAttitudeGenerator();
  void setZ(double z);
  void setZsetpoint(double z_setpoint);
  void setZvelocity(double const& z_velocity);
  void setGains(Vector3 const& pid, double const& offset);
  double zPID();

  Vector3 generateDesiredAttitudeThrust(Vector3 const& desired_acceleration);

private:
  
  double z=0.0;
  double z_setpoint = 1.0;
  
  double z_velocity = 0;
  double z_velocity_setpoint = 0;

  double roll;
  double pitch;
  double thrust;


  double _dt = 1/100.0;
  double _max = 0.9;
  double _min = 0.3;
  double _Kp = 1.5;
  double _Ki = 0.6;
  double _Kd = 0.5;
  double _pre_error = 0;
  double _integral = 0;
  double _i_max = 0.07;
  double _offset = 0.69;

};
