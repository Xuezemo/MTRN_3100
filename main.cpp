// --- Includes and objects ---
#include "Motor.hpp"
#include "Encoder.hpp"
#include "RobotHelpers.hpp" // Helper function to change rad/s into PWM for Arduino
#include "Kinematics.hpp"
#include "PIDController.hpp"
#include "IMU.hpp" 
#include "Lidar.hpp"
// (Add others if needed)

// Motor and encoder pin assignments
const float KP = 0.7f, KI = 0.01f, KD = 0.03f;
const float WHEEL_RADIUS = 0.017f, WHEEL_BASE = 0.10f, MAX_WHEEL_SPEED = 8.0f;

// Pin numbers
const uint8_t LEFT_PWM_PIN = 5, LEFT_DIR_PIN = 6;
const uint8_t RIGHT_PWM_PIN = 9, RIGHT_DIR_PIN = 10;
const uint8_t LIDAR_XSHUT_PIN = 3, LIDAR_I2C_ADDR = 0x29;

mtrn3100::Motor leftMotor(LEFT_PWM_PIN, LEFT_DIR_PIN);
mtrn3100::Motor rightMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN);
mtrn3100::Kinematics kin(WHEEL_RADIUS, WHEEL_BASE);
mtrn3100::PIDController distPID(KP, KI, KD);
mtrn3100::Lidar frontLidar(LIDAR_XSHUT_PIN, LIDAR_I2C_ADDR);
const float MAX_WHEEL_SPEED_RAD_PER_SEC = 8.0f; // e.g., max speed for your motor

// Helper Function to change speed from rad/s to PWM
void setWheelSpeeds(float wL, float wR) {
    int pwmL = constrain(int((wL / MAX_WHEEL_SPEED) * 255), -255, 255);
    int pwmR = constrain(int((wR / MAX_WHEEL_SPEED) * 255), -255, 255);
    leftMotor.setPWM(pwmL);
    rightMotor.setPWM(pwmR);
}

void setup() {
    // Arduino setup code
    Serial.begin(9600);
    Serial.print("Please input string: ");
    while (Serial.available() < 8);  
    for (int j = 0; j < 8; j++) {
        input_string += (char)Serial.read();
    }
    input_received = true;
}

// Function Implementations
// This function runs ONE STEP. Call it each loop.
bool driveToWall() {
    float frontDistance = frontLidar.readDistance(); // mm
    float vx_mm_s = distPID.compute(frontDistance);
    vx_mm_s = constrain(vx_mm_s, -100.0f, 100.0f);
    float vx_m_s = vx_mm_s / 1000.0f;
    mtrn3100::WheelSpeeds ws = kin.inverseKinematics(vx_m_s, 0);
    setWheelSpeeds(ws.wL, ws.wR);
    if (fabs(distPID.getError()) < 5.0f && fabs(vx_mm_s) < 2.0f) {
        setWheelSpeeds(0, 0);
        Serial.println("Arrived at wall!");
        return true;
    }
    return false;
}

void loop() {
    if (i < 8 && input_received == true) {
        if (input_string[i] == 'l') {
            turnToAngle(kin, leftMotor, rightMotor, turnPID, maxWheelSpeedRadPerSec, -M_PI/2);
        }

        else if (input_string[i] == 'r') {
            turnToAngle(kin, leftMotor, rightMotor, turnPID, maxWheelSpeedRadPerSec, M_PI/2);
        }

        else if (input_string[i] == 'f') {
            mtrn3100::WheelSpeeds ws = kin.inverseKinematics(vx_m_s, 0);
            setWheelSpeeds(ws.wL, ws.wR, leftMotor, rightMotor, maxWheelSpeedRadPerSec);
            // Need to test timeout value here according to time robot should be moving
            setTimeout(300);
            setWheelSpeeds(0, 0, leftMotor, rightMotor, maxWheelSpeedRadPerSec);
        }

        i++;
        // Delay between each action - can be updated upon testing
        delay(5000);
    }

}

