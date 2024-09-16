#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"

//declaration of variables for the MPU6050
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
int16_t ax, ay, az, gx, gy, gz;
int AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ, state = 0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
float elapsedTime, currentTime, previousTime;
int acel_deadzone=6;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int checks=1000; 

MPU6050 mpu(MPU);

//definition of relevant pins for the RGB LED
#define BLUE_LED_PIN 3   //set blue LED pin
#define GREEN_LED_PIN 5 //set yellow LED pin
#define RED_LED_PIN 6   //set red LED pin



void setup() {
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
//  Wire.beginTransmission(0x68);
//  Wire.write(0x1C);
//  Wire.write(0x1);
//  Wire.endTransmission();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
}

void loop() {
  if (state==0){
      digitalWrite(RED_LED_PIN, HIGH);
      Serial.println("\nReading sensors for first time...");
      calculate_IMU_error();
      state++;
      delay(1000);
    }
    
   if (state==1) {
      Serial.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(1000);
    }

    if (state==2) {
      calculate_IMU_error();
      // Print the error values on the Serial Monitor
      Serial.print("AccErrorX: ");
      Serial.println(AccErrorX);
      Serial.print("AccErrorY: ");
      Serial.println(AccErrorY);
      Serial.print("AccErrorZ: ");
      Serial.println(AccErrorZ);
      Serial.print("GyroErrorX: ");
      Serial.println(GyroErrorX);
      Serial.print("GyroErrorY: ");
      Serial.println(GyroErrorY);
      Serial.print("GyroErrorZ: ");
      Serial.println(GyroErrorZ);
      state++;
  }
  
   if (state==3) {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // print real time acceleration
//    Serial.print("AccX: ");
//    Serial.print(ax*9.81/16384);
//    Serial.print(", ");
//    Serial.print("AccY: ");
//    Serial.print(ay*9.81/16384);
//    Serial.print(", ");
//    Serial.print("AccZ: ");
//    Serial.print(az*9.81/16384);
//    Serial.print(", ");
    // get gyro data (yaw / pitch / roll) in degrees
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    gyroAngleX += gx/131.0 * elapsedTime; // deg / s * s = deg
    gyroAngleY += gy/131.0 * elapsedTime; // deg / s * s = deg
    gyroAngleZ += gz/131.0 * elapsedTime; // deg / s * s = deg
//    Serial.print("GyroX: ");
//    Serial.print(gyroAngleX);
//    Serial.print(", ");
//    Serial.print("GyroY: ");
//    Serial.print(gyroAngleY);
//    Serial.print(", ");
//    Serial.print("GyroZ: ");
//    Serial.print(gyroAngleZ);
//    Serial.print(", ");
    // calculate yaw pitch roll from accelerometer
    AccX = ax / 16384.0;
    AccY = ay / 16384.0;
    AccZ = az / 16384.0;
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI); 
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    Serial.print("Roll: ");
    Serial.print(0.2*gyroAngleX + 0.8*accAngleX); // combine gyro and accelerometer data
    Serial.print(", ");
    Serial.print("Pitch: ");
    Serial.print(0.2*gyroAngleY + 0.8*accAngleY); // combine gyro and accelerometer data
    Serial.print(", ");
    Serial.print("Yaw: ");
    Serial.print(gyroAngleZ);
//    Serial.print("AccAngleX: ");
//    Serial.print(accAngleX);
//    Serial.print(", ");
//    Serial.print("AccAngleY: ");
//    Serial.print(accAngleY);
    Serial.println("");
    delay(5);
    }
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 1000 times
  long c=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  while (c < (checks+101)) {
    if (c > 100 && c <= (checks+100)) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
    }
    if (c == (checks+100)) {
      //Divide the sum by the number of checks to get the error value
      AccErrorX = buff_ax/checks;
      AccErrorY = buff_ay/checks;
      AccErrorZ = buff_az/checks;
      GyroErrorX = buff_gx/checks;
      GyroErrorY = buff_gy/checks;
      GyroErrorZ = buff_gz/checks;
    }
    c++;
    delay(2);
  }
}


void calibration() {
  ax_offset=-AccErrorX/8;
  ay_offset=-AccErrorY/8;
  az_offset=(16384-AccErrorZ)/8;

  gx_offset=-GyroErrorX/4;
  gy_offset=-GyroErrorY/4;
  gz_offset=-GyroErrorZ/4;
  
  while (1){
      int ready=0;
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);
  
      mpu.setXGyroOffset(gx_offset);
      mpu.setYGyroOffset(gy_offset);
      mpu.setZGyroOffset(gz_offset);
    
      calculate_IMU_error();
      Serial.println("...");  
  
      if (abs(AccErrorX)<=acel_deadzone) ready++;
      else ax_offset=ax_offset-AccErrorX/acel_deadzone;
  
      if (abs(AccErrorY)<=acel_deadzone) ready++;
      else ay_offset=ay_offset-AccErrorY/acel_deadzone;
  
      if (abs(16384-AccErrorZ)<=acel_deadzone) ready++;
      else az_offset=az_offset+(16384-AccErrorZ)/acel_deadzone;
  
      if (abs(GyroErrorX)<=giro_deadzone) ready++;
      else gx_offset=gx_offset-GyroErrorX/(giro_deadzone+1);
  
      if (abs(GyroErrorY)<=giro_deadzone) ready++;
      else gy_offset=gy_offset-GyroErrorY/(giro_deadzone+1);
  
      if (abs(GyroErrorZ)<=giro_deadzone) ready++;
      else gz_offset=gz_offset-GyroErrorZ/(giro_deadzone+1);
  
      if (ready==6) break;
    }


}
