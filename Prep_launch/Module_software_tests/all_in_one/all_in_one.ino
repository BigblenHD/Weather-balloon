#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>


Adafruit_MS8607 ms8607;

#include "DFRobot_OzoneSensor.h"

#include <Arduino_LSM9DS1.h>

#include <Adafruit_MAX31865.h>

#include <SPI.h>
#include <SD.h>

#include "RTClib.h"

RTC_PCF8523 rtc;

#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3
/*   iic slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70      // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
DFRobot_OzoneSensor Ozone;

int addr = 0x18;
int day,hour,min,sec = 0;
byte buffer[2] = {0,0};

int status = 0;

const int RED = 27;
const int GREEN = 25;
const int BLUE = 23;

bool ms8607_error = false;
bool ozone_sensor_error = false;
bool imu_error = false;
bool sd_error = false;
bool rtd_error = false;
bool rtc_error = false;

Adafruit_MAX31865 thermo = Adafruit_MAX31865(7, 6, 5, 4);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

const int chipSelect = 10;

File dataFile;


void setup() {
  Serial.begin(115200);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  //while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  for(int i = 0; i< 3;i++){
     digitalWrite(GREEN, HIGH);
     delay(500);
     digitalWrite(GREEN, LOW);
     delay(500);
  }
  digitalWrite(BLUE, HIGH);
  initialize_MS8607();
  initialize_ozone();
  initialize_gamma();
  initialize_imu();
  initialize_rtd();
  initialize_sd();
  initialize_rtc();
  digitalWrite(BLUE, LOW);
  for(int i = 0; i< 3; i++){
     digitalWrite(GREEN, HIGH);
     delay(500);
     digitalWrite(GREEN, LOW);
     delay(500);
  }
}

void loop() {
  DateTime now = rtc.now();
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.print(now.toString("YYYY-MM-DD hh:mm:ss"));  dataFile.print(",");
  Serial.println("----------------------------------------------------------------");

  sensors_event_t temp, pressure, humidity;

  if(!ms8607_error){
  ms8607.getEvent(&pressure, &temp, &humidity);
  Serial.print("Temperature: ");Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure); Serial.println(" hPa");
  Serial.print("Humidity: ");Serial.print(humidity.relative_humidity); Serial.println(" %rH");
  Serial.println("");

  dataFile.print(temp.temperature);  dataFile.print(",");  dataFile.print(pressure.pressure);  dataFile.print(",");  dataFile.print(humidity.relative_humidity);  dataFile.print(",");
  }
  else{
    dataFile.print("nan, nan, nan");
  }
  Serial.println("----------------------------------------------------------------");
  if(!ozone_sensor_error){
  int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
  Serial.print("Ozone concentration is ");
  Serial.print(ozoneConcentration);
  Serial.println(" PPB.");

  dataFile.print(ozoneConcentration);  dataFile.print(",");
  }
  else{
    dataFile.print("nan");
  }
  Serial.println("----------------------------------------------------------------");
  
  //Read Statue, Measuring Time, Measuring Value
  Gamma_Mod_Read_Value();  

  Serial.println("----------------------------------------------------------------");

  if(!imu_error){
    IMU_Data();
  }
  else{
    dataFile.print("nan, nan, nan, nan, nan, nan, nan, nan, nan");
  }

  
  Serial.println("----------------------------------------------------------------");
  if(!rtd_error){
  uint16_t rtd = thermo.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));
  dataFile.println(thermo.temperature(RNOMINAL, RREF));

  rtd_read_reset_error();
  }
  else{
    dataFile.println("nan");
  }
  // Log data to the file
  dataFile.close();
  delay(1000);
}

void Gamma_Mod_Read_Value(){
  Gamma_Mod_Read(0xB0); // Read Status
  Gamma_Mod_Read(0xB1); // Read Measuring Time
  Gamma_Mod_Read(0xB2); // Read Measuring Value (10min avg / 1min update)
  Gamma_Mod_Read(0xB3); // Read Measuring Value (1min avg / 1min update)
}

void Gamma_Mod_Read(int cmd){
  /* Begin Write Sequence */
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.endTransmission();
  /* End Write Sequence */
  delay(10);
  /* Begin Read Sequence */
  Wire.requestFrom(addr, 2);
  byte i = 0;
  while(Wire.available())
  {
    buffer[i] = Wire.read();
    i++;
  }
  /* End Read Sequence */
  
  /* View Results */
  Print_Result(cmd);
}

void Print_Result(int cmd){
  float value = 0.0f;
  switch(cmd){
    case 0xA0:
      Serial.print("Reset Response\t\t\t");
      if(buffer[0]== 1) Serial.println("Reset Success.");
      else              Serial.println("Reset Fail(Status - Ready).");
      break;
    case 0xB0:
      Serial.print("Status\t\t\t\t");
      switch(buffer[0]){
        case 0: Serial.println("Ready"); break;
        case 1: Serial.println("10min Waiting"); break;
        case 2: Serial.println("Normal"); break;
      }
      status = buffer[0];
      Serial.print("VIB Status\t\t\t");
      switch(buffer[1]){
        case 0: Serial.println("OFF"); break;
        case 1: Serial.println("ON"); break;
      }
      break;
    case 0xB1:
      if(status > 0){
        sec++;
        Cal_Measuring_Time();
      }
      break;
    case 0xB2:
      
      Serial.print("Measuring Value(10min avg)\t");
      value = buffer[0] + (float)buffer[1]/100;
      Serial.print(value); Serial.println(" uSv/hr");
      dataFile.print(value); dataFile.print(",");
      break;
    case 0xB3:
      Serial.print("Measuring Value(1min avg)\t");
      value = buffer[0] + (float)buffer[1]/100;
      Serial.print(value); Serial.println(" uSv/hr");
      dataFile.print(value); dataFile.print(",");
      break;  
    case 0xB4:
      Serial.print("FW Version\t\t\t");
      Serial.print("V"); Serial.print(buffer[0]);
      Serial.print("."); Serial.println(buffer[1]);
      break;        
  }
}

/*
 *  Calculation Measuring Time
 *  Format :: 0d 00:00:00 ( (day)d (hour):(min):(sec) )
 */
void Cal_Measuring_Time(){
  if(sec == 60)   { sec = 0;  min++;  }
  if(min == 60)   { min = 0;  hour++; }
  if(hour == 24)  { hour = 0; day++;  }

  Serial.print("Measuring Time\t\t\t");
  Serial.print(day); Serial.print("d ");
  if(hour < 10) Serial.print("0");
  Serial.print(hour); Serial.print(":");
  if(min < 10) Serial.print("0");
  Serial.print(min); Serial.print(":"); 
  if(sec < 10) Serial.print("0");
  Serial.println(sec);
}

void IMU_Data(){
    float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.println("Accelerometer: ");    
    Serial.print(x);
    dataFile.print(x);    dataFile.print(",");
    Serial.print('\t');
    Serial.print(y);
    dataFile.print(y);    dataFile.print(",");
    Serial.print('\t');
    Serial.println(z);
    dataFile.print(z);    dataFile.print(",");
  }
  float gx, gy, gz;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    Serial.println("Gyroscope: ");  //Degrees / second
    Serial.print(gx);
    dataFile.print(gx);    dataFile.print(",");
    Serial.print('\t');
    Serial.print(gy);
    dataFile.print(gy);    dataFile.print(","); 
    Serial.print('\t');
    Serial.println(gz);
    dataFile.print(gz);    dataFile.print(",");

  }
  float mx, my, mz;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    Serial.println("Magnteometer: ");   //Magnetic field in uT
    Serial.print(mx);
    Serial.print('\t');
    dataFile.print(mx);    dataFile.print(",");
    Serial.print(my);
    Serial.print('\t');
    dataFile.print(my);    dataFile.print(",");
    Serial.println(mz);
    dataFile.print(mz);    dataFile.print(",");

  }
}

void initialize_MS8607(){
  Serial.println("Adafruit MS8607 test!");
  int counter = 0;
  while (!ms8607.begin() && counter < 100) {
    counter += 1;
    delay(10);
  }
  if(counter == 100){
     Serial.println("Failed to find MS8607 chip");
     ms8607_error = true;
     digitalWrite(GREEN, LOW);
     digitalWrite(RED, HIGH);
  }
  else {
    Serial.println("MS8607 Found!");
  }
    ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_8b);
  Serial.print("Humidity resolution set to ");
  switch (ms8607.getHumidityResolution()){
    case MS8607_HUMIDITY_RESOLUTION_OSR_12b: Serial.println("12-bit"); break;
    case MS8607_HUMIDITY_RESOLUTION_OSR_11b: Serial.println("11-bit"); break;
    case MS8607_HUMIDITY_RESOLUTION_OSR_10b: Serial.println("10-bit"); break;
    case MS8607_HUMIDITY_RESOLUTION_OSR_8b: Serial.println("8-bit"); break;
  }
  // ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096);
  Serial.print("Pressure and Temperature resolution set to ");
  switch (ms8607.getPressureResolution()){
    case MS8607_PRESSURE_RESOLUTION_OSR_256: Serial.println("256"); break;
    case MS8607_PRESSURE_RESOLUTION_OSR_512: Serial.println("512"); break;
    case MS8607_PRESSURE_RESOLUTION_OSR_1024: Serial.println("1024"); break;
    case MS8607_PRESSURE_RESOLUTION_OSR_2048: Serial.println("2048"); break;
    case MS8607_PRESSURE_RESOLUTION_OSR_4096: Serial.println("4096"); break;
    case MS8607_PRESSURE_RESOLUTION_OSR_8192: Serial.println("8192"); break;
  }
  Serial.println("");
}

void initialize_ozone(){
  int counter = 0;
  while(!Ozone.begin(Ozone_IICAddress) && counter < 100) {
    counter += 1;
    delay(10);
  }
  if(counter == 100){
    Serial.println("Ozone sensor error !");
    ozone_sensor_error = true;
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);
    digitalWrite(RED, HIGH);
  }
  else{
    Serial.println("Ozone sensor connect success !");
  } 

  /*   Set iic mode, active mode or passive mode
        MEASURE_MODE_AUTOMATIC            // active  mode
        MEASURE_MODE_PASSIVE              // passive mode
  */
  Ozone.setModes(MEASURE_MODE_PASSIVE);
}

void initialize_gamma(){
  Wire.begin();
  Serial.println("Gamma Sensor Sensing Start");
  //Read Firmware version
  Gamma_Mod_Read(0xB4);
  //Reset before operating the sensor
  Gamma_Mod_Read(0xA0);
}

void initialize_imu(){
  int counter = 0;
  while (!IMU.begin() && counter < 100) {
    counter += 1;
    delay(10);
  }
  if(counter == 100){
    Serial.println("Failed to initialize IMU!");
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, LOW);
  }
  else {
    Serial.println("IMU connection success!");
  }
}

void rtd_read_reset_error(){
    // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    rtd_error = true;
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  Serial.println();
}

void initialize_rtd(){
  thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
}

void initialize_sd(){
  int counter = 0;
    // Initialize SD card
  while (!SD.begin(chipSelect) && counter < 100) {
    counter += 1;
    delay(10);
  }
  if(counter == 100){
    Serial.println("SD card initialization failed!");
    sd_error = true;
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, LOW);
  }
  else {
    Serial.println("SD card initialized successfully.");
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  // Open a new file for writing
  if (!dataFile) {
    Serial.println("Error opening datalog.txt for writing.");
    sd_error = true;
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, LOW);
  }
  else {
  Serial.println("Logging data...");
  dataFile.println("time(s), temp(C), pressure(hPa), humidity(%rH), ozone(PPB), radioactivity_10min(uSv/hr), radioactivity_1min(uSv/hr), ax, ay, az, gx, gy, gz, mx, my, mz, temp_RTC(C)");
  dataFile.close();
  }
}

void initialize_rtc(){
  int counter = 0;
  while (! rtc.begin() && counter < 100) {
    counter += 1;
    delay(10);
    Serial.flush();
  }
  if(counter == 100){
     Serial.println("Couldn't find RTC");
     rtc_error = true;
     digitalWrite(GREEN, LOW);
     digitalWrite(BLUE, LOW);
     digitalWrite(RED, HIGH);
  }
  else{
    Serial.println("RTC connection success!");
  }

 
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }
  
  rtc.start();
}
