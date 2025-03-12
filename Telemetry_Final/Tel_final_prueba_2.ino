#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// WiFi
const char* ssid = "moto_edge_50_pro_JD";  // Enter your Wi-Fi name (cel de diego)
const char* password = "Canada031106";     // Enter Wi-Fi password 

WiFiClient esp32Client;

// WebSocket Server URL
const char* websocket_server = "wss://api.ecovolt.lemi-tec.mx?connectItemType=ESP32&username=Antonio "; // link

WebSocketsClient webSocket;

// Define the RX and TX pins for GPS
#define RXD2 20
#define TXD2 21

#define GPS_BAUD 9600

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

// Define serial pins for MPU6050
#define SDA_PIN 8
#define SCL_PIN 9

// 0: 250 dps (131 LSB/dps); 1: 500 dps (65.5 LSB/dps) ; 2: 1000 dps ( 32.8 LSB/dps) ; 3: 2000 dps (16.4 LSB/dps)
#define GYRO_CONFIG_MODE 0

// 0: 2g (16384 LSB/g); 1: 4g (8192 LSB/g) ; 2: 8g (4096 LSB/g) ; 3: 16g (2048 LSB/g)
#define ACC_CONFIG_MODE 1

#define MPU_POWER_REGISTER 0x6B
#define MPU_MODE 0x6C
#define GYRO_MODE 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_INIT 0x1C
#define ACCEL_CONFIG 0x1D
#define accX_H 0x3B
#define accY_H 0x3D
#define accZ_H 0x3F
#define gyroX_H 0x43
#define gyroY_H 0x45
#define gyroZ_H 0x47

// I2C Address of MPU6050
const uint8_t MPU = 0x68;

// Calibration Time Assigned
unsigned long delay_cal = 3000;

float rawaccX = 0, rawaccY = 0, rawaccZ = 0, rawgyroX = 0, rawgyroY = 0, rawgyroZ = 0;
float gyroX, gyroY, gyroZ, accX, accY, accZ, calgyroX, calgyroY, calgyroZ, calaccX, calaccY, calaccZ;
float lat, lon, speed, alt, sat;

int Led_WIFI = 0;
int Led_DATA = 1;

int sensorC_PIN = 2;
int sensorV_PIN = 3;
float sensibility = 0.066; //sensibility in V/A
float current, voltage, irms, power, sensorCRead, sensorVRead;

hw_timer_t *timer = NULL;

void IRAM_ATTR timerInterrupcion() {
  get_CV_data();
  print_CV_data();
}

void wifiInit() {
  Serial.print("Conectándose a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("Conectado a WiFi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void getInfo_GPS() {
  lat = gps.location.lat();
  lon = gps.location.lng();
  speed = gps.speed.kmph();
  alt = gps.altitude.meters();
  sat = gps.satellites.value();
}

void gpsInfo() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    getInfo_GPS();
    displayInfo();
  } else {
    Serial.println("Inválido");
    getInfo_GPS();
    displayInfo();
  }
}

void displayInfo() {
  Serial.print("LAT: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG: ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("SPEED (km/h) = ");
  Serial.println(gps.speed.kmph());
  Serial.print("ALT (m)= ");
  Serial.println(gps.altitude.meters());
  Serial.print("Satellites = ");
  Serial.println(gps.satellites.value());
  Serial.println("");
}

void i2cWrite(uint8_t address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read 2 Bytes from a given register address i.e H and L
int16_t i2cRead(uint8_t address, int8_t reg) {
  int16_t temp = 0;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 2, true);
  temp = (Wire.read() << 8 | Wire.read());
  return temp;
}

// status implies the calibration mode
void printAcc(bool status) {
  if (status) {
    Serial.printf("Calibrated AccX = ");
    Serial.println(calaccX);
    Serial.printf("Calibrated AccY = ");
    Serial.println(calaccY);
    Serial.printf("Calibrated AccZ = ");
    Serial.println(calaccZ);
  } else {
    Serial.printf("AccX = ");
    Serial.println(accX);
    Serial.printf("AccY = ");
    Serial.println(accY);
    Serial.printf("AccZ = ");
    Serial.println(accZ);
  }
}

void printGyro(bool status) {
  if (status) {
    Serial.printf("Calibrated GyroX = ");
    Serial.println(calgyroX);
    Serial.printf("Calibrated GyroY = ");
    Serial.println(calgyroY);
    Serial.printf("Calibrated GyroZ = ");
    Serial.println(calgyroZ);
  } else {
    Serial.printf("GyroX = ");
    Serial.println(gyroX);
    Serial.printf("GyroY = ");
    Serial.println(gyroY);
    Serial.printf("GyroZ = ");
    Serial.println(gyroZ);
  }
}

void powMan() {
  // Access the Power Management Register 1 (107)
  // Configured at no sleep mode
  i2cWrite(MPU, MPU_POWER_REGISTER, 0);

  // Access the Power Management Register 2 (108)
  // Configured at all readings
  i2cWrite(MPU, MPU_MODE, 0);
}

// 0: 250 dps (131 LSB/dps); 1: 500 dps (65.5 LSB/dps) ; 2: 1000 dps ( 32.8 LSB/dps) ; 3: 2000 dps (16.4 LSB/dps)
void gyroConfig(uint8_t choice) {
  uint8_t val;
  switch (choice) {
    case 0:
      val = 0b00000001;
      break;
    case 1:
      val = 0b00001001;
      break;
    case 2:
      val = 0b00010001;
      break;
    case 3:
      val = 0b00011001;
      break;
    default:
      Serial.printf("Invalid input! By default Case 0 selected\n");
      val = 0b00000001;
      break;
  }

  // Accessing Register 26
  // Selecting Mode 0
  i2cWrite(MPU, GYRO_MODE, 0x00);


  // // Accessing Register 27 (Gryo Config Register)
  // Setting the full scale range to 250dps
  i2cWrite(MPU, GYRO_CONFIG, val);
}

// 0: 2g (16384 LSB/g); 1: 4g (8192 LSB/g) ; 2: 8g (4096 LSB/g) ; 3: 16g (2048 LSB/g)
void accConfig(uint8_t choice) {
  uint8_t val;
  switch (choice) {
    case 0:
      val = 0b00000000;  // 0x00
      break;
    case 1:
      val = 0b00001000;  // 0x80
      break;
    case 2:
      val = 0b00010000;  // 0x10
      break;
    case 3:
      val = 0b00011000;  // 0x11
      break;
    default:
      Serial.printf("Invalid input! By default Case 0 selected\n");
      val = 0b00000000;  // 0x00
      break;
  }

  // Accessing Register 28 (Accel Config Initial)
  i2cWrite(MPU, ACCEL_INIT, val);

  // Accessing Register 29 (Accel Config Filter)
  // Selecting the mode ACCEL_CHOICE = 1 with BW = 1.13kHz
  i2cWrite(MPU, ACCEL_CONFIG, 0b00001000);
}

// Status will detect whether in caliberation mode or not -> true if in setup
void accData(bool status, int choice) {
  float rawVal;
  switch (choice) {
    case 0:
      rawVal = 16384.0;
      break;
    case 1:
      rawVal = 8192.0;
      break;
    case 2:
      rawVal = 4096.0;
      break;
    case 3:
      rawVal = 2048.0;
      break;
  }

  if (status) {
    // Registers 59 - 64: AccX_H, AccX_L, AccY_H, AccY_L, AccZ_H, AccZ_L {They provide raw value}
    unsigned int count = 0;
    unsigned long initTime = millis();
    Serial.printf("Do not move! Starting calibration for %d seconds\n", (delay_cal / 1000));
    // In the loop for 3 seconds
    while (millis() - initTime <= delay_cal) {
      rawaccX += (i2cRead(MPU, accX_H)) / rawVal;
      rawaccY += (i2cRead(MPU, accY_H)) / rawVal;
      rawaccZ += ((i2cRead(MPU, accZ_H)) / rawVal) - 1;
      count++;
    }
    Serial.printf("Count = %d\n", count);
    Serial.printf("Calibration done!\n");
    calaccX = (float)rawaccX / count;
    calaccY = (float)rawaccY / count;
    calaccZ = (float)rawaccZ / count;
    printAcc(true);
  } else {
    // Registers 59 - 64: AccX_H, AccX_L, AccY_H, AccY_L, AccZ_H, AccZ_L {They provide raw value}
    accX = (i2cRead(MPU, accX_H) / rawVal) - calaccX;
    accY = (i2cRead(MPU, accY_H) / rawVal) - calaccY;
    accZ = (i2cRead(MPU, accZ_H) / rawVal) - calaccZ;
    printAcc(false);
  }
}

// 0: 250 dps (131 LSB/dps); 1: 500 dps (65.5 LSB/dps) ; 2: 1000 dps ( 32.8 LSB/dps) ; 3: 2000 dps (16.4 LSB/dps)
void gyroData(bool status, int choice) {
  float rawVal;
  switch (choice) {
    case 0:
      rawVal = 131.0;
      break;
    case 1:
      rawVal = 65.5;
      break;
    case 2:
      rawVal = 32.8;
      break;
    case 3:
      rawVal = 16.4;
      break;
  }
  // Registers 67-72: GyroX_H, GyroX_L, GyroY_H, GyroY_L, GyroZ_H, GyroZ_L {They provide raw value}
  if (status) {
    unsigned int count = 0;
    unsigned long initTime = millis();
    Serial.printf("Do not move! Starting calibration for %d seconds\n", (delay_cal / 1000));
    // In the loop for 3 seconds
    while (millis() - initTime <= delay_cal) {
      rawgyroX += (i2cRead(MPU, gyroX_H)) / rawVal;
      rawgyroY += (i2cRead(MPU, gyroY_H)) / rawVal;
      rawgyroZ += (i2cRead(MPU, gyroZ_H)) / rawVal;
      count++;
    }
    Serial.printf("Count = %d\n", count);
    Serial.printf("Calibration done!\n");
    calgyroX = (float)rawgyroX / count;
    calgyroY = (float)rawgyroY / count;
    calgyroZ = (float)rawgyroZ / count;
    printGyro(true);
  } else {
    gyroX = (i2cRead(MPU, gyroX_H)) / rawVal - calgyroX;
    gyroY = (i2cRead(MPU, gyroY_H)) / rawVal - calgyroY;
    gyroZ = (i2cRead(MPU, gyroZ_H)) / rawVal - calgyroZ;
    printGyro(false);
  }
}

void setup_timer(){
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &timerInterrupcion); // Interruption function
  timerAlarm(timer, 500000, true, 0); // Interruption every 1/2 second
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void get_CV_data(){
  sensorCRead = analogRead(sensorC_PIN) * (5.0 / 1023.0);
  current = (sensorCRead-2.5)/sensibility;
  //irms = current*0.707;
  sensorVRead = analogRead(sensorV_PIN);
  voltage = fmap(sensorVRead, 0, 1023, 0.0, 10.0);
  power = current*voltage; // P=IV watts
}

void print_CV_data(){
  Serial.print("Current (A): ");
  Serial.println(current, 4);
  Serial.print("Irms (A): ");
  Serial.println(irms, 4);
  Serial.print("Power (W): ");
  Serial.println(power, 4);
}

// Send Telemetry
void sendTelemetryData(){
  // Create JSON document
  StaticJsonDocument<200> jsonDoc;
  
  jsonDoc["latitude"] = String(lat);
  jsonDoc["longitude"] =  String(lon);
  jsonDoc["altitude"] = String(alt);
  jsonDoc["speed"] = String(speed);
  jsonDoc["Accelerometer(x)"] = String(accX);
  jsonDoc["Accelerometer(y)"] = String(accY);
  jsonDoc["Accelerometer(z)"] = String(accZ);
  jsonDoc["Gyroscope(x)"] = String(gyroX);
  jsonDoc["Gyroscope(y)"] = String(gyroY);
  jsonDoc["Gyroscope(z)"] = String(gyroZ);
  jsonDoc["Current"] = String(current);
  jsonDoc["Voltage"] = String(voltage);
  jsonDoc["Power"] = String(power);

  // Convert JSON to string
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Send via WebSocket
  webSocket.sendTXT(jsonString);
  Serial.println("Enviado: " + jsonString);
}

// WebSocket Event Handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED:
            Serial.println("Conectado al servidor WebSocket!");
            break;
        case WStype_TEXT:
            Serial.printf("Mensaje recibido: %s\n", payload);
            break;
        case WStype_DISCONNECTED:
            Serial.println("Desconectado del servidor WebSocket!");
            break;
    }
}

void setup() {
  Serial.begin(115200);
  
  setup_timer();
  pinMode(Led_WIFI, OUTPUT);
  pinMode(Led_DATA, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  wifiInit();

  // Setup WebSocket client
  webSocket.begin(websocket_server, 80, "/");
  webSocket.onEvent(webSocketEvent);
  
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  // Using the I2C communication at fast mode (400kHz)
  Wire.setClock(400000);
  // Calling the power management function
  powMan();
  // Selection Mode 0: 250dps
  gyroConfig(GYRO_CONFIG_MODE);
  // Selection Mode 2: +-8g
  accConfig(ACC_CONFIG_MODE);
  // Calibration for Acceleration
  accData(true, ACC_CONFIG_MODE);
  // Calibration for Gyroscope
  gyroData(true, GYRO_CONFIG_MODE);
  delay(3000);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(Led_WIFI, HIGH);
  } else if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(Led_WIFI, LOW);
  }
  webSocket.loop();  // Connection alive
  gpsInfo();
  accData(false, ACC_CONFIG_MODE);
  gyroData(false, GYRO_CONFIG_MODE);
  delay(500);
}
