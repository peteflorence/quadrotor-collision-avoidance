#include <iostream>
#include "trajectory.h"
#include <math.h>

class AttitudeGenerator {
public:
  void TestAttitudeGenerator();
  void setZ(double z);
  void setZsetpoint(double z_setpoint);
  double zPID();

  Vector3 generateDesiredAttitudeThrust(Vector3 const& desired_acceleration);

private:
  
  double z;
  double z_setpoint;

  double _dt = 1/30.0;
  double _max = 100;
  double _min = 100;
  double _Kp = 10;
  double _Kd = 1;
  double _Ki = 1;
  double _pre_error = 0;
  double _integral = 0;

};