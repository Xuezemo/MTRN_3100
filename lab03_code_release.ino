#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Kinematics.hpp"
#include "OLED.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define MOT1PWM 11 // PIN 9 is a PWM pin
#define MOT1DIR 12
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);

#define EN1_A 2 // PIN 2 is an interupt
#define EN1_B 7
mtrn3100::Encoder encoder1(EN1_A, EN1_B);


#define MOT2PWM 9
#define MOT2DIR 10
#define EN2_A   3
#define EN2_B   8
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
mtrn3100::Encoder encoder2(EN2_A, EN2_B);


mtrn3100::PIDController pid1(38.0, 0.0, 1.5);  
mtrn3100::PIDController pid2(38.0, 0.0, 1.5);  

void calibrateGyroBias() {
  const int N = 100;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < N; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(2);
  }

  gx_offset = (sum_gx / (float)N) / 131.0;
  gy_offset = (sum_gy / (float)N) / 131.0;
  gz_offset = (sum_gz / (float)N) / 131.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  encoder1.reset();
  encoder2.reset();

  pid1.zeroAndSetTarget(encoder1.getRotation(), -(10.0f * PI)); 
  pid2.zeroAndSetTarget(encoder2.getRotation(), 10.0f * PI);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); 
  }
  display.clearDisplay();
  display.setTextSize(1);      
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Ready");
  display.drawBitmap(
            (SCREEN_WIDTH - LOGO_WIDTH) / 2,
            (SCREEN_HEIGHT - LOGO_HEIGHT) / 2,
            logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  display.display();

  accelgyro.initialize();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF_CFG = 3 → 44Hz
  Wire.endTransmission();
  delay(500);
}


void loop() {
  const int N = 10;  

  int16_t ax, ay, az, gx, gy, gz;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < N; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum_ax += ax; sum_ay += ay; sum_az += az;
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    delay(2);  
  }

  float ax_g = (sum_ax / N) / 16384.0;
  float ay_g = (sum_ay / N) / 16384.0;
  float az_g = (sum_az / N) / 16384.0;

  float gx_dps = (sum_gx / N) / 131.0;
  float gy_dps = (sum_gy / N) / 131.0;
  float gz_dps = (sum_gz / N) / 131.0;


  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Accel (g):");

  display.setCursor(0, 10);
  display.print("X: "); display.print(ax_g, 2);
  display.print(" Y: "); display.print(ay_g, 2);
  display.print(" Z: "); display.print(az_g, 2);

  display.setCursor(0, 30);
  display.println("Gyro (deg/s):");

  display.setCursor(0, 40);
  display.print("X: "); display.print(gx_dps, 2);
  display.print(" Y: "); display.print(gy_dps, 2);
  display.print(" Z: "); display.print(gz_dps, 2);

  display.display();

  delay(200); 

}


