#pragma once

#include <Wire.h>
#include "MPU6050.h"

namespace mtrn3100 {

class IMU {
public:
    IMU() : gx_offset(0), gy_offset(0), gz_offset(0),
                roll(0), pitch(0), yaw(0), lastTime(0) {}

    void begin() {
        Wire.begin();
        accelgyro.initialize();
        calibrateGyroBias();
        lastTime = millis();
    }

    void update(int N = 10) {
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        long sum_ax = 0, sum_ay = 0, sum_az = 0;
        long sum_gx = 0, sum_gy = 0, sum_gz = 0;

        for (int i = 0; i < N; i++) {
            accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
            sum_ax += ax_raw; sum_ay += ay_raw; sum_az += az_raw;
            sum_gx += gx_raw; sum_gy += gy_raw; sum_gz += gz_raw;
            delay(2);
        }

        float ax_g = (sum_ax / (float)N) / 16384.0;
        float ay_g = (sum_ay / (float)N) / 16384.0;
        float az_g = (sum_az / (float)N) / 16384.0;

        float gx_dps = (sum_gx / (float)N) / 131.0 - gx_offset;
        float gy_dps = (sum_gy / (float)N) / 131.0 - gy_offset;
        float gz_dps = (sum_gz / (float)N) / 131.0 - gz_offset;

        ax = ax_g; ay = ay_g; az = az_g;
        gx = gx_dps; gy = gy_dps; gz = gz_dps;

        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0; 
        lastTime = currentTime;

        updateAttitude(gx, gy, gz, dt);
    }

    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }

    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }

    float getRoll()  const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw()   const { return yaw; }

private:
    MPU6050 accelgyro;

    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    float gx_offset, gy_offset, gz_offset;

    float roll, pitch, yaw;
    unsigned long lastTime;

    void calibrateGyroBias() {
        const int N = 100;
        long sum_gx = 0, sum_gy = 0, sum_gz = 0;
        int16_t ax, ay, az, gx, gy, gz;

        for (int i = 0; i < N; i++) {
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            delay(20);
        }

        gx_offset = (sum_gx / (float)N) / 131.0;
        gy_offset = (sum_gy / (float)N) / 131.0;
        gz_offset = (sum_gz / (float)N) / 131.0;
    }

    void updateAttitude(float wx, float wy, float wz, float dt) {
        float cosRoll = cos(roll * DEG_TO_RAD);
        float cosPitch = cos(pitch * DEG_TO_RAD);
        float sinRoll = sin(roll * DEG_TO_RAD);
        float tanPitch = tan(pitch * DEG_TO_RAD);

        float rollDot  = wx + (wy * sinRoll + wz * cosRoll) * tanPitch;
        float pitchDot = wy * cosRoll - wz * sinRoll;
        float yawDot   = (wy * sinRoll + wz * cosRoll) / cosPitch;

        roll  += rollDot * dt;
        pitch += pitchDot * dt;
        yaw   += yawDot * dt;

        roll  = normalizeAngle(roll);
        pitch = normalizeAngle(pitch);
        yaw   = normalizeAngle(yaw);
    }
    
    float normalizeAngle(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }
};

} 