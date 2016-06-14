#include <iostream>
#include "trajectory.h"
#include <math.h>

class AttitudeGenerator {
public:
  void TestAttitudeGenerator();
  void setZ(double z);
  void setZsetpoint(double z_setpoint);
  void setGains(Vector3 const& pid, double const& offset);
  double zPID();

  Vector3 generateDesiredAttitudeThrust(Vector3 const& desired_acceleration);

private:
  
  double z;
  double z_setpoint=0.0;

  double roll;
  double pitch;
  double thrust;


  double _dt = 1/30.0;
  double _max = 1;
  double _min = 0;
  double _Kp = 0.5;
  double _Ki = 0.05;
  double _Kd = 0.5;
  double _pre_error = 0;
  double _integral = 0;
  double _i_max = 0.5;
  double _offset = 0.0;

};
