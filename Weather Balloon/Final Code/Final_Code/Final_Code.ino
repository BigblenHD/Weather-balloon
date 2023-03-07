#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

MPU6050 mpu(Wire);
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();

//definition of relevant pins for the RGB LED
#define BLUE_LED_PIN 3   //set blue LED pin
#define GREEN_LED_PIN 5 //set yellow LED pin
#define RED_LED_PIN 6   //set red LED pin

long timer = 0;
float counter = 0;
const int chipSelect = 4;

void setup() {
  Serial.begin(9600);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  // MPU60500
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
  // BMP280
  Serial.println(F("BMP280 Sensor event test"));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  
  // SD CARD READER
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  SD.remove("datalog.txt");
}

void loop() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  mpu.update();
  if(millis() - timer > 500){ // print data every second
    counter += 0.5;
    Serial.print("Seconds: ");Serial.println(counter);
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
  
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100);
    Serial.println(" hPa");
  
    Serial.print(F("Height = "));
    Serial.print(bmp.readAltitude() + 187.2); // 187.2 = calibration value 
    Serial.println(" m");
  
    Serial.println();

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(counter);
      dataFile.print(",");
      dataFile.print(bmp.readPressure()/100);
      dataFile.print(",");
      dataFile.print(bmp.readAltitude() + 187.2); // 187.2 = calibration value 
      dataFile.print(",");
      dataFile.print(bmp.readTemperature());
      dataFile.print(",");
      dataFile.print(mpu.getAccX());
      dataFile.print(",");
      dataFile.print(mpu.getAccY());
      dataFile.print(",");
      dataFile.print(mpu.getAccZ());
      dataFile.print(",");
      dataFile.print(mpu.getGyroX());
      dataFile.print(",");
      dataFile.print(mpu.getGyroY());
      dataFile.print(",");
      dataFile.print(mpu.getGyroZ());
      dataFile.print(",");
      dataFile.print(mpu.getAccAngleX());
      dataFile.print(",");
      dataFile.print(mpu.getAccAngleY());
      dataFile.print(",");
      dataFile.print(mpu.getAngleX());
      dataFile.print(",");
      dataFile.print(mpu.getAngleY());
      dataFile.print(",");
      dataFile.print(mpu.getAngleZ());
      dataFile.println("");
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      while(1);
    }
    timer = millis();
  }
}
