/*******************************************************************************
 * FIN STABILIZATION 
 * MPU-6050 + 4x Servo Motors 
 * MPU-6050
 * ESP32
 * Power module
 * 9v Adapter 
 * for now below 
 * Servo Motors :
 *   North Fin-> GPIO 25
 *   East Fin-> GPIO 26
 *   South Fin-> GPIO 27  
 *   West Fin-> GPIO 32
 
 ******************************************************************************/

#include <Wire.h>

// Comment out next line when hardware is connected
#define DEBUG_MODE_NO_HARDWARE

 #ifndef DEBUG_MODE_NO_HARDWARE
  #include <MPU6050.h>
  #include <ESP32Servo.h>
 #endif

// Pin Definitions
const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;

const int SERVO_PIN_NORTH = 25;
const int SERVO_PIN_EAST  = 26;
const int SERVO_PIN_SOUTH = 27;
const int SERVO_PIN_WEST  = 32;

// Servo Positions
const int SERVO_NEUTRAL = 90;
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// Control Parameters
const float PITCH_GAIN = 1.0;
const float ROLL_GAIN = 1.0;
const float DEADZONE = 5.0;
const float FILTER_ALPHA = 0.2;

// Global Variables
float pitch = 0.0;
float roll = 0.0;
float pitch_filtered = 0.0;
float roll_filtered = 0.0;

int servoNorth = SERVO_NEUTRAL;
int servoEast = SERVO_NEUTRAL;
int servoSouth = SERVO_NEUTRAL;
int servoWest = SERVO_NEUTRAL;

bool mpuConnected = false;
bool servosConnected = false;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 50;

unsigned long lastDisplay = 0;
const unsigned long DISPLAY_INTERVAL = 1000;

// DEBUG stuff
#ifndef DEBUG_MODE_NO_HARDWARE
  MPU6050 mpu;
  Servo finNorth;
  Servo finEast;
  Servo finSouth;
  Servo finWest;
  
  int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
  int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
#endif

// DEBUG variables
#ifdef DEBUG_MODE_NO_HARDWARE
  float simTime = 0.0;
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println();
  Serial.println("========================================================");
  Serial.println(" FIN STABILIZATION ");
  Serial.println("========================================================");
  Serial.println();
  
  //DEBUG
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.println("MODE: DEBUG");
    Serial.println("This code will simulate sensor data for testing");
    Serial.println();
    Serial.println();
    mpuConnected = false;
    servosConnected = false;
  #else
    Serial.println("MODE: HARDWARE - Initializing sensors and servos");
    Serial.println();
    initializeHardware();
  #endif
  
  Serial.println("========================================================");
  Serial.println("System Ready Starting stabilization");
  Serial.println("========================================================");
  Serial.println();
  
  lastUpdate = millis();
  lastDisplay = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;
    
    #ifdef DEBUG_MODE_NO_HARDWARE
      updateSimulatedData();
    #else
      updateRealData();
    #endif
    
    calculateServoPositions();
    
    #ifndef DEBUG_MODE_NO_HARDWARE
      if (servosConnected) {
        moveServos();
      }
    #endif
  }
  
  if (currentTime - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = currentTime;
    displayTelemetry();
  }
  
  delay(5);
}

#ifndef DEBUG_MODE_NO_HARDWARE
void initializeHardware() {
  Serial.println(" HARDWARE INITIALIZATION");
  Serial.println();
  
  // Initialize I2C
  Serial.print("Initializing I2C bus");
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  Serial.println("OK");
  
  // Initialize MPU-6050 
  Serial.print("Initializing MPU-6050 sensor... ");
  mpu.initialize();
  delay(100);
  
  if (mpu.testConnection()) {
    Serial.println("OK");
    mpuConnected = true;
    
    Serial.println();
    Serial.println("Keep sensor FLAT and STILL for calibration!");
    Serial.println("Calibrating in 3 seconds...");
    delay(3000);
    
    Serial.print("Calibrating MPU-6050... ");
    calibrateMPU();
    Serial.println("DONE");
  } else {
    Serial.println("FAILED");
    Serial.println("WARNING: MPU-6050 not detected.");
    mpuConnected = false;
  }
  
  Serial.println();
  
  // Initialize Servos
  Serial.println("Initializing servo motors...");
  
  Serial.print("  North fin (GPIO 25)... ");
  finNorth.attach(SERVO_PIN_NORTH);
  finNorth.write(SERVO_NEUTRAL);
  Serial.println("OK");
  
  Serial.print("  East fin (GPIO 26)... ");
  finEast.attach(SERVO_PIN_EAST);
  finEast.write(SERVO_NEUTRAL);
  Serial.println("OK");
  
  Serial.print("  South fin (GPIO 27)... ");
  finSouth.attach(SERVO_PIN_SOUTH);
  finSouth.write(SERVO_NEUTRAL);
  Serial.println("OK");
  
  Serial.print("  West fin (GPIO 32)... ");
  finWest.attach(SERVO_PIN_WEST);
  finWest.write(SERVO_NEUTRAL);
  Serial.println("OK");
  
  servosConnected = true;
  
  Serial.println();
  Serial.println("Running servo test sequence...");
  testServos();
  
  Serial.println();
  Serial.println("All systems initialized successfully!");
  Serial.println();
}

void calibrateMPU() {
  //Intializingh
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    delay(5);
  }
  
  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = (az_sum / samples) - 16384;
  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;
  
  Serial.println();
  Serial.print("  Calibration offsets - ");
  Serial.print("AX: "); Serial.pri  #include <MPU6050.h>nt(ax_offset);
  Serial.print(", AY: "); Serial.print(ay_offset);
  Serial.print(", AZ: "); Serial.print(az_offset);
  Serial.print(", GX: "); Serial.print(gx_offset);
  Serial.print(", GY: "); Serial.print(gy_offset);
  Serial.print(", GZ: "); Serial.println(gz_offset);
}

// EVery single fin/servo testing 
void testServos() {
  Serial.println("  Testing North fin...");
  finNorth.write(SERVO_MAX);
  delay(500);
  finNorth.write(SERVO_MIN);
  delay(500);
  finNorth.write(SERVO_NEUTRAL);
  delay(300);
  
  Serial.println("  Testing East fin...");
  finEast.write(SERVO_MAX);
  delay(500);
  finEast.write(SERVO_MIN);
  delay(500);
  finEast.write(SERVO_NEUTRAL);
  delay(300);
  
  Serial.println("  Testing South fin...");
  finSouth.write(SERVO_MAX);
  delay(500);
  finSouth.write(SERVO_MIN);
  delay(500);
  finSouth.write(SERVO_NEUTRAL);
  delay(300);
  
  Serial.println("  Testing West fin...");
  finWest.write(SERVO_MAX);
  delay(500);
  finWest.write(SERVO_MIN);
  delay(500);
  finWest.write(SERVO_NEUTRAL);
  delay(300);
  
  Serial.println("  All servos tested successfully!");
}

void updateRealData() {
  if (!mpuConnected) {
    pitch = 0.0;
    roll = 0.0;
    return;
  }
  
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  
  float raw_pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float raw_roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  
  if (isnan(raw_pitch) || abs(raw_pitch) > 90) raw_pitch = pitch_filtered;
  if (isnan(raw_roll) || abs(raw_roll) > 90) raw_roll = roll_filtered;
  
  pitch_filtered = (FILTER_ALPHA * raw_pitch) + ((1 - FILTER_ALPHA) * pitch_filtered);
  roll_filtered = (FILTER_ALPHA * raw_roll) + ((1 - FILTER_ALPHA) * roll_filtered);
  
  pitch = pitch_filtered;
  roll = roll_filtered;
}

void moveServos() {
  finNorth.write(servoNorth);
  finEast.write(servoEast);
  finSouth.write(servoSouth);
  finWest.write(servoWest);
}
#endif

#ifdef DEBUG_MODE_NO_HARDWARE
void updateSimulatedData() {
  simTime += 0.05;
  
  // Simulate teh tilting motion
  pitch = 15.0 * sin(simTime * 0.5);
  roll = 10.0 * cos(simTime * 0.7);
  
  pitch_filtered = pitch;
  roll_filtered = roll;
}
#endif

// calculating the deadzones and fixing them back
void calculateServoPositions() {
  float corrected_pitch = (abs(pitch) < DEADZONE) ? 0 : pitch;
  float corrected_roll = (abs(roll) < DEADZONE) ? 0 : roll;
  
  int northAngle = SERVO_NEUTRAL + (int)(corrected_pitch * PITCH_GAIN);
  int southAngle = SERVO_NEUTRAL - (int)(corrected_pitch * PITCH_GAIN);
  int eastAngle = SERVO_NEUTRAL + (int)(corrected_roll * ROLL_GAIN);
  int westAngle = SERVO_NEUTRAL - (int)(corrected_roll * ROLL_GAIN);
  
  servoNorth = constrain(northAngle, SERVO_MIN, SERVO_MAX);
  servoSouth = constrain(southAngle, SERVO_MIN, SERVO_MAX);
  servoEast = constrain(eastAngle, SERVO_MIN, SERVO_MAX);
  servoWest = constrain(westAngle, SERVO_MIN, SERVO_MAX);
}

//Just print statemnets for serial monitor 
void displayTelemetry() {
  Serial.println("--- TELEMETRY UPDATE ---");
  Serial.println();
  
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.print("Mode: SIMULATED DATA (Hardware NOT connected)");
    Serial.print(" | Time: ");
    Serial.print(simTime, 1);
    Serial.println("s");
  #else
    Serial.print("Mode: REAL DATA");
    Serial.print(" | MPU: ");
    Serial.print(mpuConnected ? "Connected" : "Disconnected");
    Serial.print(" | Servos: ");
    Serial.println(servosConnected ? "Connected" : "Disconnected");
  #endif
  
  Serial.println();
  
  Serial.println("ORIENTATION:");
  Serial.print("  Pitch: ");
  if (pitch >= 0) Serial.print("+");
  Serial.print(pitch, 2);
  Serial.print(" deg");
  if (pitch > 10) {
    Serial.println("  (NOSE UP)");
  } else if (pitch < -10) {
    Serial.println("  (NOSE DOWN)");
  } else {
    Serial.println("  (LEVEL)");
  }
  
  Serial.print("  Roll:  ");
  if (roll >= 0) Serial.print("+");
  Serial.print(roll, 2);
  Serial.print(" deg");
  if (roll > 10) {
    Serial.println("  (TILT RIGHT)");
  } else if (roll < -10) {
    Serial.println("  (TILT LEFT)");
  } else {
    Serial.println("  (LEVEL)");
  }
  
  Serial.println();
  
  Serial.println("SERVO POSITIONS:");
  Serial.print("  North: ");
  Serial.print(servoNorth);
  Serial.print(" deg");    // will Replace with teh HEX VALUE of degree later 
  Serial.print("  |  East: ");
  Serial.print(servoEast);
  Serial.print(" deg");
  Serial.print("  |  South: ");
  Serial.print(servoSouth);
  Serial.print(" deg");
  Serial.print("  |  West: ");
  Serial.print(servoWest);
  Serial.println(" deg");
  
  Serial.println();
  
  Serial.println("CORRECTIONS:");
  int pitchCorrection = abs(servoNorth - SERVO_NEUTRAL);
  int rollCorrection = abs(servoEast - SERVO_NEUTRAL);
  
  // REPLACE WITH MACROS LATER MAYbe 
  Serial.print("  Pitch correction: ");
  Serial.print(pitchCorrection);
  Serial.print(" deg");
  if (pitchCorrection > 20) {
    Serial.println("  (LARGE)");
  } else if (pitchCorrection > 5) {
    Serial.println("  (MEDIUM)");
  } else {
    Serial.println("  (SMALL)");
  }
  
  Serial.print("  Roll correction:  ");
  Serial.print(rollCorrection);
  Serial.print(" deg");
  if (rollCorrection > 20) {
    Serial.println("  (LARGE)");
  } else if (rollCorrection > 5) {
    Serial.println("  (MEDIUM)");
  } else {
    Serial.println("  (SMALL)");
  }
  
  Serial.println();
  
  Serial.print("STATUS: ");
  if (abs(pitch) < 5 && abs(roll) < 5) {
    Serial.println("STABLE, Minimal correction needed");
  } else if (abs(pitch) > 20 || abs(roll) > 20) {
    Serial.println("HIGH TILT, Active correction in progress");
  } else {
    Serial.println("CORRECTING, Fins adjusting");
  }
  
  Serial.println();
  Serial.println("========================================================");
  Serial.println();
}


//fpr now must work like below
void printStartupInstructions() {
  Serial.println();
  Serial.println("TESTING INSTRUCTIONS:");
  Serial.println("1. TILT FORWARD   -> North fin deflects to push back");
  Serial.println("2. TILT BACKWARD -> South fin deflects to push back");
  Serial.println("3. TILT LEFT     -> East fin deflects to push back");
  Serial.println("4. TILT RIGHT    -> West fin deflects to push back");
  Serial.println();
}