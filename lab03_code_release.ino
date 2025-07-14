#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Kinematics.hpp"
#include "OLED.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.hpp"
#include "MotionController.hpp"
#include "Lidar.hpp"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL6180X.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 accelgyro;
mtrn3100::IMU imu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

mtrn3100::Kinematics kinematics(0.016, 0.102);

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


mtrn3100::Lidar lidar1(A0, 0x40);  // sensor1 at pin A0, address 0x40
mtrn3100::Lidar lidar2(A1, 0x41);  // sensor2 at pin A1, address 0x41
mtrn3100::Lidar lidar3(A2, 0x42);  // sensor3 at pin A2, address 0x42

// VL6180X sensor2;
int sensor1_pin = A0; //40
int sensor2_pin = A1; //41
int sensor3_pin = A2; //42

// void LidarInitialize(){
//   pinMode(sensor2_pin, OUTPUT);
//   digitalWrite(sensor2_pin, LOW);

//   digitalWrite(sensor2_pin, HIGH);
//   delay(50);
//   sensor2.init();
//   sensor2.configureDefault();
//   sensor2.setTimeout(250);
//   sensor2.setAddress(0x41);
//   delay(50);

// }


mtrn3100::MotionController controller(motor1, motor2, kinematics, imu);



void setup() {
  Serial.begin(115200);
  Wire.begin();

  encoder1.reset();
  encoder2.reset();

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

  imu.begin();
  // LidarInitialize();
  // lidar1.begin();
  lidar2.begin();
  // lidar3.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF_CFG = 3 → 44Hz
  Wire.endTransmission();



  delay(500);
}

bool hasTurned = false;  

void loop() {
  imu.update();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Roll/Pitch/Yaw:");
  display.setCursor(0, 10);
  display.print("R: "); display.print(imu.getRoll(), 1);
  display.print(" P: "); display.print(imu.getPitch(), 1);
  display.print(" Y: "); display.print(imu.getYaw(), 1);

  display.setCursor(0, 30);
  display.print("Dis: "); display.print(lidar2.readDistance(), 1);

  display.display();
  delay(20); 

  

  controller.update();  
  // if (!hasTurned && !controller.isTurning()) {
  //   controller.startTurnRight(90);  
  //   hasTurned = true;               
  // }


  

  delay(200); 

}


