#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Kinematics.hpp"
#include "OLED.hpp"
#include <MPU6050_light.h>
#include "MotionController.hpp"
#include <SPI.h>
#include "Lidar.hpp"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu(Wire);
unsigned long timer = 0;

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
mtrn3100::Lidar lidar3(A2, 0x42); 

mtrn3100::MotionController controller(motor1, motor2, encoder1, encoder2, kinematics, mpu, lidar2);

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

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); 

  // lidar1.begin();
  lidar2.begin();
  // lidar3.begin();
  delay(500);
}
  
bool hasTurned = false; 

void loop() {
  mpu.update();

  if((millis()-timer)>10){ // print data every 10ms
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Roll/Pitch/Yaw:");
    display.setCursor(0, 10);
    display.print("R: "); display.print(mpu.getAngleX(), 1);
    display.print(" P: "); display.print(mpu.getAngleY(), 1);
    display.print(" Y: "); display.print(mpu.getAngleZ(), 1);

    timer = millis();  
  }
  display.setCursor(0, 30);
  display.print("Dis: "); display.print(lidar2.readDistance(), 1);

  display.setCursor(0, 50);
  display.print("Dis: "); display.print(encoder2.getDistance(), 1);


  // if (!hasTurned && !controller.isDriving()) {
  //   controller.startForward(-10);  
  //   hasTurned = true;               
  // } 
  //move a distance here 

  controller.update(); 
  // controller.enableLidarTracking();

  display.display();
  delay(20); 



  delay(200); 

}


