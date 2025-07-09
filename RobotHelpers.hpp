// This Helper Function is for - converts your desired speed in rad/s to a PWM value for the motors.
// Because our motors run on PWM values, not directly on rad/s.
void setWheelSpeeds(float wL, float wR, mtrn3100::Motor& leftMotor, mtrn3100::Motor& rightMotor, float maxWheelSpeedRadPerSec) {
    // wL and wR are in rad/s
    // Convert to PWM (simple proportional mapping)
    // Map -maxWheelSpeedRadPerSec ... +maxWheelSpeedRadPerSec to -255 ... +255
    int pwmL = constrain(int((wL / maxWheelSpeedRadPerSec) * 255), -255, 255);
    int pwmR = constrain(int((wR / maxWheelSpeedRadPerSec) * 255), -255, 255);

    leftMotor.setPWM(pwmL);
    rightMotor.setPWM(pwmR);
}
