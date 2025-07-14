#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "Kinematics.hpp"
#include "IMU.hpp"

namespace mtrn3100 {

class MotionController {
public:
    MotionController(Motor& leftMotor, Motor& rightMotor, Kinematics& kin, IMU& imu)
        : motorL(leftMotor), motorR(rightMotor), kinematics(kin), imu(imu) {}

    void startTurnLeft(float angle_deg) {
        imu.update();  
        target_yaw = normalizeAngle(imu.getYaw() - angle_deg);
        turning = true;
    }

    void startTurnRight(float angle_deg) {
        imu.update();
        target_yaw = normalizeAngle(imu.getYaw() + angle_deg);
        turning = true;
    }

    void update() {
        if (!turning) return;

        imu.update();
        float current_yaw = imu.getYaw();
        float error = angleDifference(target_yaw, current_yaw);

        if (abs(error) < tolerance_deg) {
            stop();
            turning = false;
            return;
        }

        float omega = constrain(kp * error * DEG_TO_RAD, -maxOmega, maxOmega);
        auto ws = kinematics.inverseKinematics(0.0f, omega);

        int pwmL = -omegaToPWM(ws.wL);  
        int pwmR = omegaToPWM(ws.wR);

        motorL.setPWM(pwmL);
        motorR.setPWM(pwmR);
    }

    void stop() {
        motorL.setPWM(0);
        motorR.setPWM(0);
        turning = false;
    }

    bool isTurning() const {
        return turning;
    }

private:
    Motor& motorL;
    Motor& motorR;
    Kinematics& kinematics;
    IMU& imu;

    float target_yaw = 0.0f;
    bool turning = false;

    const float kp = 2.0f;
    const float maxOmega = 16.23f;
    const float pwmPerOmega = 255.0f / 16.23f;
    const float tolerance_deg = 3.0f;
    const int minPWM = 30;   
    const int maxPWM = 200; 

  int omegaToPWM(float omega) {
      int pwm = int(omega * pwmPerOmega);

      if (pwm > 0) {
          pwm = max(pwm, minPWM); 
      } else if (pwm < 0) {
          pwm = min(pwm, -minPWM); 
      }

      return constrain(pwm, -maxPWM, maxPWM);
  }

    float normalizeAngle(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    float angleDifference(float target, float current) {
        float diff = target - current;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        return diff;
    }
};

}  // namespace mtrn3100