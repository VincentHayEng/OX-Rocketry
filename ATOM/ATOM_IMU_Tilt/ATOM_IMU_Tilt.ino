/*******************************************************************************
 * ATOM - IMU TILT TRACKING
 * 
 * Hardware:
 * - ESP32-S3
 * - MPU9250 (IMU )
 * - 4x Servo Motors 
 * - External power supply for servos
 * 
 * Connections:
 * MPU9250: VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22
 * Servos: North->GPIO25, East->GPIO26, South->GPIO27, West->GPIO32
 * 
 * Tilt board forward/back -> North/South fins deflect
 * Tilt board left/right   -> East/West fins deflect
 ******************************************************************************/

#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// I2C pins
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int MPU9250_ADDR = 0x68;  // AD0 pin low. Use 0x69 if AD0 is tied high.

// MPU9250 registers
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// Servo pins
const int SERVO_NORTH = 25;
const int SERVO_EAST = 26;
const int SERVO_SOUTH = 27;
const int SERVO_WEST = 32;

// Servo limits (same range as thermal version)
const int SERVO_CENTER = 45;
const int SERVO_MIN = 0;
const int SERVO_MAX = 65;

// Tracking parameters
const float TRACKING_GAIN = 1.0;      // tune this - higher = more sensitive to tilt
const float MAX_TILT_DEG = 45.0;      // tilt angle that maps to full fin deflection

// Smoothing for stable movement (same idea as thermal smoothing)
float smoothPitch = 0.0;
float smoothRoll = 0.0;
const float SMOOTHING = 0.15;         // 0.05-0.6, lower = smoother but slower

// Accel sensitivity for default +/-2g range
const float ACCEL_SENSITIVITY = 16384.0;  // LSB per g

bool imuReady = false;

int lastNorthPos = SERVO_CENTER;
int lastEastPos = SERVO_CENTER;
int lastSouthPos = SERVO_CENTER;
int lastWestPos = SERVO_CENTER;
const int SERVO_DEADBAND = 2;

Servo finNorth, finEast, finSouth, finWest;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_RATE = 20;  // 50 Hz, plenty for a hand-tilt demo

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ATOM - IMU Tilt Tracking System");
  Serial.println("Initializing...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  imuReady = initMPU9250();
  if (imuReady) {
    Serial.println("MPU9250: OK");
  } else {
    Serial.println("MPU9250: FAILED - check wiring / address");
  }

  // Initialize servos
  finNorth.attach(SERVO_NORTH);
  finEast.attach(SERVO_EAST);
  finSouth.attach(SERVO_SOUTH);
  finWest.attach(SERVO_WEST);

  // Center all servos
  finNorth.write(SERVO_CENTER);
  finEast.write(SERVO_CENTER);
  finSouth.write(SERVO_CENTER);
  finWest.write(SERVO_CENTER);

  Serial.println("Servos: OK");
  Serial.println("System ready");
  Serial.println();

  lastUpdate = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= UPDATE_RATE) {
    lastUpdate = now;

    if (imuReady) {
      float rawPitch, rawRoll;
      readTilt(rawPitch, rawRoll);

      // Apply smoothing, same pattern as the thermal target smoothing
      smoothPitch = smoothPitch * (1.0 - SMOOTHING) + rawPitch * SMOOTHING;
      smoothRoll  = smoothRoll  * (1.0 - SMOOTHING) + rawRoll  * SMOOTHING;

      updateServos();
      printStatus();
    }
  }
}

bool initMPU9250() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);  // wake the sensor up (default is sleep mode)
  if (Wire.endTransmission() != 0) return false;
  delay(50);
  return true;
}

void readTilt(float &pitchOut, float &rollOut) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6);

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();

  float ax = rawAX / ACCEL_SENSITIVITY;
  float ay = rawAY / ACCEL_SENSITIVITY;
  float az = rawAZ / ACCEL_SENSITIVITY;

  // Standard accelerometer tilt equations
  pitchOut = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  rollOut  = atan2(ay, az) * 180.0 / PI;
}

void updateServos() {
  // Normalize tilt to -1..1 range based on MAX_TILT_DEG
  float errorPitch = constrain(smoothPitch / MAX_TILT_DEG, -1.0, 1.0);
  float errorRoll  = constrain(smoothRoll / MAX_TILT_DEG, -1.0, 1.0);

  // Same mapping pattern as the thermal version:
  // Tilt forward (nose down) -> deflect North fin
  // Tilt back              -> deflect South fin
  // Tilt right             -> deflect East fin
  // Tilt left              -> deflect West fin
  int northPos = SERVO_CENTER - (int)(errorPitch * TRACKING_GAIN * 20);
  int southPos = SERVO_CENTER + (int)(errorPitch * TRACKING_GAIN * 20);
  int eastPos  = SERVO_CENTER + (int)(errorRoll  * TRACKING_GAIN * 20);
  int westPos  = SERVO_CENTER - (int)(errorRoll  * TRACKING_GAIN * 20);

  // Apply limits
  northPos = constrain(northPos, SERVO_MIN, SERVO_MAX);
  southPos = constrain(southPos, SERVO_MIN, SERVO_MAX);
  eastPos  = constrain(eastPos, SERVO_MIN, SERVO_MAX);
  westPos  = constrain(westPos, SERVO_MIN, SERVO_MAX);

if (abs(northPos - lastNorthPos) >= SERVO_DEADBAND) { finNorth.write(northPos); lastNorthPos = northPos; }
if (abs(eastPos - lastEastPos) >= SERVO_DEADBAND) { finEast.write(eastPos); lastEastPos = eastPos; }
if (abs(southPos - lastSouthPos) >= SERVO_DEADBAND) { finSouth.write(southPos); lastSouthPos = southPos; }
if (abs(westPos - lastWestPos) >= SERVO_DEADBAND) { finWest.write(westPos); lastWestPos = westPos; }
}

void printStatus() {
  Serial.print("Pitch: ");
  Serial.print(smoothPitch, 1);
  Serial.print(" deg | Roll: ");
  Serial.print(smoothRoll, 1);
  Serial.println(" deg");
}
