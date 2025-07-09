#pragma once

#include <math.h>

namespace mtrn3100 {

  struct Velocity {
  float vx;     // linear velocity in x (m/s)
  float vy;     // usually 0 for planar diff drive
  float omega;  // angular velocity (rad/s)
};

class Kinematics  {

public:
    Kinematics(float wheel_radius, float wheel_base)
        : r(wheel_radius), b(wheel_base) {}

    Velocity forwardKinematics(float wL, float wR) {
        Velocity v;
        v.vx = r * (wR + wL) / 2.0;
        v.vy = 0.0;
        v.omega = r * (wR - wL) / b;
        return v;
    }

private:
    float r; 
    float b; 

    
};

}  // namespace mtrn3100
