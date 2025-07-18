#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "Kinematics.hpp"
#include "Lidar.hpp"
#include <MPU6050_light.h>

namespace mtrn3100 {

class MotionController {
public:
    MotionController(Motor& leftMotor, Motor& rightMotor, Encoder& leftEncoder, Encoder& rightEncoder, Kinematics& kinematics, MPU6050& mpu, Lidar& lidar)
        : leftMotor(leftMotor), rightMotor(rightMotor),
          leftEncoder(leftEncoder), rightEncoder(rightEncoder),
          kinematics(kinematics), mpu(mpu), lidar(lidar) {}

    void begin() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    void startTurnRight(float angle_deg) {
        mpu.update();  
        targetYaw = normalizeAngle(mpu.getAngleZ() - angle_deg);
        turning = true;
        yaw_error_sum = 0;
        yaw_error_prev = 0;
        last_update_time = millis();
    }

    void startTurnLeft(float angle_deg) {
        mpu.update();  
        targetYaw = normalizeAngle(mpu.getAngleZ() + angle_deg);
        turning = true;
        yaw_error_sum = 0;
        yaw_error_prev = 0;
        last_update_time = millis();
    }

    void startForward(float distance_cm) {
        leftEncoder.reset();
        rightEncoder.reset();
        targetDistance = distance_cm;
        driving = true;
        drive_error_sum = 0;
        drive_error_prev = 0;
        last_update_time = millis();
    }

    void enableLidarTracking() {
        trackingWithLidar = true;
    }

    void disableLidarTracking() {
        trackingWithLidar = false;
    }

    void update() {
        unsigned long now = millis();
        float dt = (now - last_update_time) / 1000.0f;
        if (dt <= 0.001f) dt = 0.001f;
        last_update_time = now;

        if (turning) {
            mpu.update();
            float currentYaw = mpu.getAngleZ();
            float error = angleDifference(targetYaw, currentYaw);

            if (abs(error) < toleranceDeg) {
                stop();
                return;
            }

            yaw_error_sum += error * dt;
            float d_error = (error - yaw_error_prev) / dt;

            float omega = yaw_kp * error * DEG_TO_RAD
                        + yaw_ki * yaw_error_sum * DEG_TO_RAD
                        + yaw_kd * d_error * DEG_TO_RAD;

            omega = constrain(omega, -maxOmega, maxOmega);

            yaw_error_prev = error;

            auto ws = kinematics.inverseKinematics(0.0f, omega);

            int pwmL = -omegaToPWM(ws.wL);  
            int pwmR = omegaToPWM(ws.wR);

            leftMotor.setPWM(pwmL);
            rightMotor.setPWM(pwmR);

        } else if (driving) {
            float leftDist = abs(leftEncoder.getDistance());
            float rightDist = abs(rightEncoder.getDistance());
            float avgDist = (leftDist + rightDist) / 2.0f;

            float direction = (targetDistance >= 0) ? 1.0f : -1.0f;
            float error = targetDistance * 10.0f - direction * avgDist;  // mm

            if (abs(error) < 2.0f) {
                stop();
                return;
            }

            drive_error_sum += error * dt;
            float d_error = (error - drive_error_prev) / dt;

            float vx_mm_s = drive_kp * error
                          + drive_ki * drive_error_sum
                          + drive_kd * d_error;

            if (error > 0)
                vx_mm_s = constrain(vx_mm_s, 20.0f, 100.0f);
            else
                vx_mm_s = constrain(vx_mm_s, -100.0f, -20.0f);

            float vx = vx_mm_s / 1000.0f;

            drive_error_prev = error;

            auto ws = kinematics.inverseKinematics(vx, 0.0f);
            int pwmL = -omegaToPWM(ws.wL);
            int pwmR = omegaToPWM(ws.wR);
            leftMotor.setPWM(pwmL);
            rightMotor.setPWM(pwmR);

        } else if (trackingWithLidar) {
            float frontDistance = lidar.readDistance(); // mm
            float error = frontDistance - 100.0f ;

            if (abs(error) < lidarToleranceMm) {
                stop();
                return;
            }

            drive_error_sum += error * dt;
            float d_error = (error - drive_error_prev) / dt;

            float vx_mm_s = drive_kp * error
                          + drive_ki * drive_error_sum
                          + drive_kd * d_error;
            vx_mm_s = constrain(vx_mm_s, -100.0f, 100.0f);
            float vx = vx_mm_s / 1000.0f;

            drive_error_prev = error;

            auto ws = kinematics.inverseKinematics(vx, 0.0f);
            int pwmL = -omegaToPWM(ws.wL);
            int pwmR = omegaToPWM(ws.wR);
            leftMotor.setPWM(pwmL);
            rightMotor.setPWM(pwmR);
        }
    }

    void stop() {
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        turning = false;
        driving = false;
        trackingWithLidar = false;
    }

    bool isTurning() const { return turning; }
    bool isDriving() const { return driving; }
    bool isTrackingLidar() const { return trackingWithLidar; }

private:
    Motor& leftMotor;
    Motor& rightMotor;
    Encoder& leftEncoder;
    Encoder& rightEncoder;
    Kinematics& kinematics;
    MPU6050& mpu;
    Lidar& lidar;

    float targetYaw = 0.0f;
    float targetDistance = 0.0f; // cm
    bool turning = false;
    bool driving = false;
    bool trackingWithLidar = false;

    const float pwmPerOmega = 255.0f / 16.23f;
    const float toleranceDeg = 3.0f;
    const int minPWM = 30;
    const int maxPWM = 200;
    const float lidarToleranceMm = 5.0f;

    // PID for yaw
    float yaw_kp = 0.25f;
    float yaw_ki = 0.0f;
    float yaw_kd = 0.1f;
    float yaw_error_sum = 0.0f;
    float yaw_error_prev = 0.0f;

    // PID for drive
    float drive_kp = 1.7f;
    float drive_ki = 0.0f;
    float drive_kd = 0.1f;
    float drive_error_sum = 0.0f;
    float drive_error_prev = 0.0f;

    unsigned long last_update_time = 0;
    const float maxOmega = 16.23f;

    int omegaToPWM(float omega) {
        int pwm = int(omega * pwmPerOmega);
        if (pwm > 0) pwm = max(pwm, minPWM);
        else if (pwm < 0) pwm = min(pwm, -minPWM);
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

} // namespace mtrn3100

