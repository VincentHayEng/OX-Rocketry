/*******************************************************************************
 * PROJECT ATOM - AUTONOMOUS TRACKING ORDNANCE MODULE
 * Integrated Thermal Tracking + Fin Stabilization
 * 
 * Hardware:
 * - ESP32
 * - MPU-6050 (IMU for stabilization)
 * - MLX90640 (Thermal camera for target tracking)
 * - 4x Servo Motors (Canard fins)
 * - Power module
 * - 9V Adapter
 * 
 * CONNECTIONS:
 * 
 * I2C Bus (Shared):
 *   SDA -> GPIO 21
 *   SCL -> GPIO 22
 *   
 * MPU-6050:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SDA -> GPIO 21 (shared)
 *   SCL -> GPIO 22 (shared)
 *   
 * MLX90640:
 *   VIN -> 3.3V
 *   GND -> GND
 *   SDA -> GPIO 21 (shared)
 *   SCL -> GPIO 22 (shared)
 *   **Requires 4.7kΩ pullup resistors on SDA/SCL**
 *   
 * Servo Motors (Canard Fins):
 *   North Fin -> GPIO 25
 *   East Fin  -> GPIO 26
 *   South Fin -> GPIO 27
 *   West Fin  -> GPIO 32
 *   
 ******************************************************************************/

#include <Wire.h>

// Comment out next line when hardware is connected
//#define DEBUG_MODE_NO_HARDWARE

#ifndef DEBUG_MODE_NO_HARDWARE
  #include <MPU6050.h>
  #include <ESP32Servo.h>
  #include <Adafruit_MLX90640.h>
#endif

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

const int SERVO_PIN_NORTH = 25;
const int SERVO_PIN_EAST  = 26;
const int SERVO_PIN_SOUTH = 27;
const int SERVO_PIN_WEST  = 32;

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

const int SERVO_NEUTRAL = 90;
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================

// Stabilization gains (for maintaining level flight)
const float PITCH_STAB_GAIN = 3.0;
const float ROLL_STAB_GAIN = 3.0;

// Tracking gains (for following target)
const float TRACKING_GAIN_X = 2.0;  // Horizontal tracking sensitivity
const float TRACKING_GAIN_Y = 2.0;  // Vertical tracking sensitivity

// Control mode weights (0.0 to 1.0)
float stabilizationWeight = 0.4;  // How much to stabilize
float trackingWeight = 0.6;       // How much to track target

const float DEADZONE = 5.0;
const float FILTER_ALPHA = 0.2;

// ============================================================================
// THERMAL CAMERA CONFIGURATION
// ============================================================================

const int FRAME_WIDTH = 32;
const int FRAME_HEIGHT = 24;
const int TOTAL_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;
const int MLX90640_I2C_ADDR = 0x33;

const float TEMP_THRESHOLD = 30.0;
const float AMBIENT_OFFSET = 5.0;

// ============================================================================
// GLOBAL VARIABLES - IMU
// ============================================================================

float pitch = 0.0;
float roll = 0.0;
float pitch_filtered = 0.0;
float roll_filtered = 0.0;

// ============================================================================
// GLOBAL VARIABLES - THERMAL CAMERA
// ============================================================================

float thermalFrame[TOTAL_PIXELS];
float ambientTemp = 25.0;
float maxTemp = 0.0;
float minTemp = 100.0;
int targetX = -1;
int targetY = -1;
bool targetDetected = false;

// ============================================================================
// GLOBAL VARIABLES - SERVO CONTROL
// ============================================================================

int servoNorth = SERVO_NEUTRAL;
int servoEast = SERVO_NEUTRAL;
int servoSouth = SERVO_NEUTRAL;
int servoWest = SERVO_NEUTRAL;

// ============================================================================
// GLOBAL VARIABLES - SYSTEM STATUS
// ============================================================================

bool mpuConnected = false;
bool thermalCameraConnected = false;
bool servosConnected = false;

unsigned long lastIMUUpdate = 0;
unsigned long lastThermalUpdate = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastDisplay = 0;

const unsigned long IMU_INTERVAL = 50;      // 20 Hz
const unsigned long THERMAL_INTERVAL = 100;  // 10 Hz
const unsigned long SERVO_INTERVAL = 20;     // 50 Hz
const unsigned long DISPLAY_INTERVAL = 1000; // 1 Hz

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

#ifndef DEBUG_MODE_NO_HARDWARE
  MPU6050 mpu;
  Adafruit_MLX90640 mlx;
  
  Servo finNorth;
  Servo finEast;
  Servo finSouth;
  Servo finWest;
  
  int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
  int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
#endif

// ============================================================================
// DEBUG VARIABLES
// ============================================================================

#ifdef DEBUG_MODE_NO_HARDWARE
  float simTime = 0.0;
  int simTargetX = 16;
  int simTargetY = 12;
#endif

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  printHeader();
  printConnectionDiagram();
  
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.println("MODE: DEBUG (Simulated Data)");
    Serial.println("All sensors and actuators simulated for testing");
    Serial.println();
    mpuConnected = false;
    thermalCameraConnected = false;
    servosConnected = false;
  #else
    Serial.println("MODE: HARDWARE");
    Serial.println();
    initializeHardware();
  #endif
  
  Serial.println("========================================================");
  Serial.println("SYSTEM READY - Project ATOM Online");
  Serial.println("========================================================");
  Serial.println();
  
  lastIMUUpdate = millis();
  lastThermalUpdate = millis();
  lastServoUpdate = millis();
  lastDisplay = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Update IMU data
  if (currentTime - lastIMUUpdate >= IMU_INTERVAL) {
    lastIMUUpdate = currentTime;
    
    #ifdef DEBUG_MODE_NO_HARDWARE
      updateSimulatedIMU();
    #else
      updateRealIMU();
    #endif
  }
  
  // Update thermal camera
  if (currentTime - lastThermalUpdate >= THERMAL_INTERVAL) {
    lastThermalUpdate = currentTime;
    
    #ifdef DEBUG_MODE_NO_HARDWARE
      updateSimulatedThermal();
    #else
      updateRealThermal();
    #endif
  }
  
  // Update servo positions
  if (currentTime - lastServoUpdate >= SERVO_INTERVAL) {
    lastServoUpdate = currentTime;
    
    calculateIntegratedControl();
    
    #ifndef DEBUG_MODE_NO_HARDWARE
      if (servosConnected) {
        moveServos();
      }
    #endif
  }
  
  // Display telemetry
  if (currentTime - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = currentTime;
    displayTelemetry();
  }
  
  delay(5);
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void printHeader() {
  Serial.println();
  Serial.println("========================================================");
  Serial.println("  PROJECT ATOM");
  Serial.println("  Autonomous Tracking Ordnance Module");
  Serial.println("========================================================");
  Serial.println();
  Serial.println("  Thermal Tracking + Active Stabilization");
  Serial.println("  Target Acquisition & Guidance System");
  Serial.println();
  Serial.println("========================================================");
  Serial.println();
}

void printConnectionDiagram() {
  Serial.println("WIRING DIAGRAM:");
  Serial.println();
  Serial.println("I2C Bus (Shared):");
  Serial.println("  SDA -> GPIO 21");
  Serial.println("  SCL -> GPIO 22");
  Serial.println("  **Pullup: 4.7kΩ resistors required on both lines**");
  Serial.println();
  Serial.println("MPU-6050 (IMU):");
  Serial.println("  VCC -> 3.3V | GND -> GND");
  Serial.println("  SDA -> GPIO 21 (shared) | SCL -> GPIO 22 (shared)");
  Serial.println();
  Serial.println("MLX90640 (Thermal Camera):");
  Serial.println("  VIN -> 3.3V | GND -> GND");
  Serial.println("  SDA -> GPIO 21 (shared) | SCL -> GPIO 22 (shared)");
  Serial.println();
  Serial.println("Servo Motors (Canard Fins):");
  Serial.println("  North -> GPIO 25 | East  -> GPIO 26");
  Serial.println("  South -> GPIO 27 | West  -> GPIO 32");
  Serial.println();
  Serial.println("========================================================");
  Serial.println();
}

#ifndef DEBUG_MODE_NO_HARDWARE
void initializeHardware() {
  Serial.println("HARDWARE INITIALIZATION");
  Serial.println();
  
  // Initialize I2C bus
  Serial.print("Initializing I2C bus... ");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  Serial.println("OK");
  
  // Initialize MPU-6050
  Serial.print("Initializing MPU-6050 (IMU)... ");
  mpu.initialize();
  delay(100);
  
  if (mpu.testConnection()) {
    Serial.println("OK");
    mpuConnected = true;
    
    Serial.println("  Calibrating MPU-6050...");
    Serial.println("  Keep sensor FLAT and STILL!");
    delay(2000);
    calibrateMPU();
    Serial.println("  Calibration complete");
  } else {
    Serial.println("FAILED");
    Serial.println("  WARNING: MPU-6050 not detected");
    mpuConnected = false;
  }
  
  Serial.println();
  
  // Initialize MLX90640
  Serial.print("Initializing MLX90640 (Thermal Camera)... ");
  
  if (mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
    Serial.println("OK");
    thermalCameraConnected = true;
    
    Serial.println("  Setting camera parameters...");
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_8_HZ);
    Serial.println("  Camera configured");
  } else {
    Serial.println("FAILED");
    Serial.println("  WARNING: MLX90640 not detected at 0x33");
    Serial.println("  Check wiring and pullup resistors!");
    thermalCameraConnected = false;
  }
  
  Serial.println();
  
  // Initialize Servos
  Serial.println("Initializing servo motors...");
  
  finNorth.attach(SERVO_PIN_NORTH);
  finNorth.write(SERVO_NEUTRAL);
  Serial.println("  North fin... OK");
  
  finEast.attach(SERVO_PIN_EAST);
  finEast.write(SERVO_NEUTRAL);
  Serial.println("  East fin... OK");
  
  finSouth.attach(SERVO_PIN_SOUTH);
  finSouth.write(SERVO_NEUTRAL);
  Serial.println("  South fin... OK");
  
  finWest.attach(SERVO_PIN_WEST);
  finWest.write(SERVO_NEUTRAL);
  Serial.println("  West fin... OK");
  
  servosConnected = true;
  
  Serial.println();
  Serial.println("Running servo test...");
  testServos();
  
  Serial.println();
  Serial.println("All systems initialized!");
  Serial.println();
}

void calibrateMPU() {
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
}

void testServos() {
  int testPositions[] = {SERVO_MAX, SERVO_MIN, SERVO_NEUTRAL};
  
  for (int pos : testPositions) {
    finNorth.write(pos);
    finEast.write(pos);
    finSouth.write(pos);
    finWest.write(pos);
    delay(300);
  }
  
  Serial.println("  Servo test complete");
}

void updateRealIMU() {
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

void updateRealThermal() {
  if (!thermalCameraConnected) {
    return;
  }
  
  if (mlx.getFrame(thermalFrame) != 0) {
    return;
  }
  
  processThermalFrame();
  detectTarget();
}

void moveServos() {
  finNorth.write(servoNorth);
  finEast.write(servoEast);
  finSouth.write(servoSouth);
  finWest.write(servoWest);
}
#endif

// ============================================================================
// DEBUG/SIMULATION FUNCTIONS
// ============================================================================

#ifdef DEBUG_MODE_NO_HARDWARE
void updateSimulatedIMU() {
  simTime += 0.05;
  
  // Simulate gentle rocking motion
  pitch = 8.0 * sin(simTime * 0.3);
  roll = 6.0 * cos(simTime * 0.4);
  
  pitch_filtered = pitch;
  roll_filtered = roll;
}

void updateSimulatedThermal() {
  simTime += 0.1;
  
  // Simulate moving target
  simTargetX = 16 + (int)(10 * sin(simTime * 0.2));
  simTargetY = 12 + (int)(8 * cos(simTime * 0.3));
  
  simTargetX = constrain(simTargetX, 0, FRAME_WIDTH - 1);
  simTargetY = constrain(simTargetY, 0, FRAME_HEIGHT - 1);
  
  // Generate simulated thermal frame
  ambientTemp = 25.0;
  
  for (int y = 0; y < FRAME_HEIGHT; y++) {
    for (int x = 0; x < FRAME_WIDTH; x++) {
      int index = y * FRAME_WIDTH + x;
      
      float dx = x - simTargetX;
      float dy = y - simTargetY;
      float distance = sqrt(dx * dx + dy * dy);
      
      if (distance < 4.0) {
        thermalFrame[index] = ambientTemp + 20.0 - (distance * 3.0);
      } else {
        thermalFrame[index] = ambientTemp + random(-15, 15) / 10.0;
      }
    }
  }
  
  processThermalFrame();
  detectTarget();
}
#endif

// ============================================================================
// THERMAL PROCESSING FUNCTIONS
// ============================================================================

void processThermalFrame() {
  maxTemp = -100.0;
  minTemp = 200.0;
  
  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (thermalFrame[i] > maxTemp) maxTemp = thermalFrame[i];
    if (thermalFrame[i] < minTemp) minTemp = thermalFrame[i];
  }
}

void detectTarget() {
  targetDetected = false;
  float threshold = ambientTemp + AMBIENT_OFFSET;
  
  float hottestTemp = threshold;
  int hottestIndex = -1;
  
  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (thermalFrame[i] > hottestTemp) {
      hottestTemp = thermalFrame[i];
      hottestIndex = i;
    }
  }
  
  if (hottestIndex >= 0) {
    targetDetected = true;
    targetX = hottestIndex % FRAME_WIDTH;
    targetY = hottestIndex / FRAME_WIDTH;
  } else {
    targetX = -1;
    targetY = -1;
  }
}

// ============================================================================
// INTEGRATED CONTROL FUNCTION
// ============================================================================

void calculateIntegratedControl() {
  // PART 1: Calculate stabilization corrections
  float corrected_pitch = (abs(pitch) < DEADZONE) ? 0 : pitch;
  float corrected_roll = (abs(roll) < DEADZONE) ? 0 : roll;
  
  int stabNorth = (int)(corrected_pitch * PITCH_STAB_GAIN);
  int stabSouth = (int)(-corrected_pitch * PITCH_STAB_GAIN);
  int stabEast = (int)(corrected_roll * ROLL_STAB_GAIN);
  int stabWest = (int)(-corrected_roll * ROLL_STAB_GAIN);
  
  // PART 2: Calculate tracking corrections
  int trackNorth = 0;
  int trackSouth = 0;
  int trackEast = 0;
  int trackWest = 0;
  
  if (targetDetected) {
    // Calculate offset from center
    int centerX = FRAME_WIDTH / 2;
    int centerY = FRAME_HEIGHT / 2;
    
    float errorX = (targetX - centerX) / (float)centerX;  // -1 to +1
    float errorY = (targetY - centerY) / (float)centerY;  // -1 to +1
    
    // Convert to servo corrections
    // Positive errorY = target below center = pitch DOWN (South fin up)
    trackNorth = (int)(-errorY * TRACKING_GAIN_Y * 20);
    trackSouth = (int)(errorY * TRACKING_GAIN_Y * 20);
    
    // Positive errorX = target to right = roll RIGHT (West fin up)
    trackEast = (int)(errorX * TRACKING_GAIN_X * 20);
    trackWest = (int)(-errorX * TRACKING_GAIN_X * 20);
  }
  
  // PART 3: Blend stabilization and tracking
  int northCorrection = (int)(stabNorth * stabilizationWeight + trackNorth * trackingWeight);
  int southCorrection = (int)(stabSouth * stabilizationWeight + trackSouth * trackingWeight);
  int eastCorrection = (int)(stabEast * stabilizationWeight + trackEast * trackingWeight);
  int westCorrection = (int)(stabWest * stabilizationWeight + trackWest * trackingWeight);
  
  // PART 4: Apply to servos with limits
  servoNorth = constrain(SERVO_NEUTRAL + northCorrection, SERVO_MIN, SERVO_MAX);
  servoSouth = constrain(SERVO_NEUTRAL + southCorrection, SERVO_MIN, SERVO_MAX);
  servoEast = constrain(SERVO_NEUTRAL + eastCorrection, SERVO_MIN, SERVO_MAX);
  servoWest = constrain(SERVO_NEUTRAL + westCorrection, SERVO_MIN, SERVO_MAX);
}

// ============================================================================
// TELEMETRY DISPLAY
// ============================================================================

void displayTelemetry() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║         PROJECT ATOM - TELEMETRY UPDATE                ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // System status
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.print("MODE: SIMULATION | Time: ");
    Serial.print(simTime, 1);
    Serial.println("s");
  #else
    Serial.println("MODE: HARDWARE");
    Serial.print("  IMU: ");
    Serial.print(mpuConnected ? "✓" : "✗");
    Serial.print(" | Thermal: ");
    Serial.print(thermalCameraConnected ? "✓" : "✗");
    Serial.print(" | Servos: ");
    Serial.println(servosConnected ? "✓" : "✗");
  #endif
  
  Serial.println();
  Serial.println("┌─────────────────────────────────────────────────────────┐");
  Serial.println("│ ORIENTATION (IMU)                                       │");
  Serial.println("└─────────────────────────────────────────────────────────┘");
  
  Serial.print("  Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("° | Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  
  Serial.println();
  Serial.println("┌─────────────────────────────────────────────────────────┐");
  Serial.println("│ TARGET TRACKING (Thermal Camera)                        │");
  Serial.println("└─────────────────────────────────────────────────────────┘");
  
  if (targetDetected) {
    Serial.println("  ★ TARGET LOCKED ★");
    Serial.print("  Position: [");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.print("] | Temp: ");
    Serial.print(thermalFrame[targetY * FRAME_WIDTH + targetX], 1);
    Serial.println("°C");
    
    int centerX = FRAME_WIDTH / 2;
    int centerY = FRAME_HEIGHT / 2;
    int offsetX = targetX - centerX;
    int offsetY = targetY - centerY;
    
    Serial.print("  Offset: X=");
    Serial.print(offsetX);
    Serial.print(" Y=");
    Serial.print(offsetY);
    Serial.print(" | ");
    
    if (abs(offsetX) < 3 && abs(offsetY) < 3) {
      Serial.println("CENTERED ✓");
    } else {
      if (offsetX > 0) Serial.print("RIGHT ");
      if (offsetX < 0) Serial.print("LEFT ");
      if (offsetY > 0) Serial.print("DOWN ");
      if (offsetY < 0) Serial.print("UP ");
      Serial.println();
    }
  } else {
    Serial.println("  ○ NO TARGET - Searching...");
    Serial.print("  Temp Range: ");
    Serial.print(minTemp, 1);
    Serial.print("°C to ");
    Serial.print(maxTemp, 1);
    Serial.println("°C");
  }
  
  Serial.println();
  Serial.println("┌─────────────────────────────────────────────────────────┐");
  Serial.println("│ CANARD POSITIONS                                        │");
  Serial.println("└─────────────────────────────────────────────────────────┘");
  
  Serial.print("  N:");
  Serial.print(servoNorth);
  Serial.print("° | E:");
  Serial.print(servoEast);
  Serial.print("° | S:");
  Serial.print(servoSouth);
  Serial.print("° | W:");
  Serial.print(servoWest);
  Serial.println("°");
  
  Serial.println();
  Serial.println("┌─────────────────────────────────────────────────────────┐");
  Serial.println("│ CONTROL MODE                                            │");
  Serial.println("└─────────────────────────────────────────────────────────┘");
  
  Serial.print("  Stabilization: ");
  Serial.print(stabilizationWeight * 100, 0);
  Serial.print("% | Tracking: ");
  Serial.print(trackingWeight * 100, 0);
  Serial.println("%");
  
  if (targetDetected) {
    Serial.println("  Status: ACTIVE TRACKING");
  } else {
    Serial.println("  Status: STABILIZATION ONLY");
  }
  
  Serial.println();
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();
}