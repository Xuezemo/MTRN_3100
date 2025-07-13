// Headers List needed in main.cpp
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "IMU.hpp" // Not sure exact file name - @Joe
#include "Kinematics.hpp"
#include "RobotHelpers.hpp" // Helper function to change rad/s into PWM for Arduino

// Motor and encoder pin assignments
mtrn3100::Motor leftMotor(?,?);   //  pins
mtrn3100::Motor rightMotor(?,?); //   pins

mtrn3100::Kinematics kin(?,?); // mm radius, mm wheelbase
mtrn3100::PIDController distPID(?,?,?); // Need to Tune in Lab!
mtrn3100::PIDController turnPID(?,?,?); // Need to Tune in Lab!
float maxWheelSpeed = ?f; // max rad/s for the motor

// Function Implementations 
void driveToWall(
    mtrn3100::Kinematics& kin,
    mtrn3100::Motor& leftMotor,
    mtrn3100::Motor& rightMotor,
    mtrn3100::PIDController& distPID,
    float maxWheelSpeedRadPerSec,
    float target_mm = 100.0f,   // Default: 100 mm from wall
    float tol_mm = 5.0f)        // Default: ±5 mm tolerance
{
    // 1. Set up PID target
    float start = getFrontLidarMM(); // Need to write this function 
    distPID.zeroAndSetTarget(start, target_mm);

    // 2. Loop until within tolerance
    while (true) {
        float current = getFrontLidarMM();
        float vx_mm_s = distPID.compute(current); // mm/s

        // Clamp vx for safety, e.g., -100 to +100 mm/s
        vx_mm_s = constrain(vx_mm_s, -100.0f, 100.0f);

        float vx_m_s = vx_mm_s / 1000.0f; // convert to m/s

        // No turning
        mtrn3100::WheelSpeeds ws = kin.inverseKinematics(vx_m_s, 0);

        setWheelSpeeds(ws.wL, ws.wR, leftMotor, rightMotor, maxWheelSpeedRadPerSec);

        // Check stop condition
        if (fabs(distPID.getError()) < tol_mm && fabs(vx_mm_s) < 2.0f) 
          break;
          delay(20);
    }
    // Stop
    setWheelSpeeds(0, 0, leftMotor, rightMotor, maxWheelSpeedRadPerSec);
}

void turnToAngle(
    mtrn3100::Kinematics& kin,
    mtrn3100::Motor& leftMotor,
    mtrn3100::Motor& rightMotor,
    mtrn3100::PIDController& turnPID,
    float maxWheelSpeedRadPerSec,
    float target_relative_rad = M_PI/2, // +90 deg
    float tol_rad = 0.0524f)              // Default: ±3 deg tolerance
{
    float start = getYawRad();    // Need to write this function
    float target = start + target_relative_rad;

    // Keep in -pi to +pi range
    if (target > M_PI) target -= 2*M_PI;
    if (target < -M_PI) target += 2*M_PI;

    turnPID.zeroAndSetTarget(start, target);

    while (true) {
        float curr = getYawRad(); 

        // Compute angular velocity command (rad/s)
        float omega = turnPID.compute(curr);

        // Clamp for safety
        omega = constrain(omega, -1.0f, 1.0f);

        // No forward motion
        mtrn3100::WheelSpeeds ws = kin.inverseKinematics(0, omega);

        setWheelSpeeds(ws.wL, ws.wR, leftMotor, rightMotor, maxWheelSpeedRadPerSec);

        if (fabs(turnPID.getError()) < tol_rad && fabs(omega) < 0.01f) 
          break;

        delay(20);
    }
    setWheelSpeeds(0, 0, leftMotor, rightMotor, maxWheelSpeedRadPerSec);
}

// Arduino Functions 
void setup() {
    // Arduino setup code
}

void loop() {
    // Drive to wall 100mm away
    driveToWall(kin, leftMotor, rightMotor, distPID, maxWheelSpeed, 100.0f, 5.0f);

    delay(500); // Not sure

    // Turn 90 degrees right
    turnToAngle(kin, leftMotor, rightMotor, turnPID, maxWheelSpeed, -M_PI/2, 0.05f);

    while (1); // Stop forever
}

