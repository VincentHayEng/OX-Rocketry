/*******************************************************************************
 * ATOM - THERMAL TRACKING ONLY
 * 
 * Hardware:
 * - ESP32
 * - MLX90640 (Thermal camera)
 * - 4x Servo Motors 
 * - External power supply for servos
 * 
 * Connections:
 * MLX90640: VIN->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22
 * Servos: North->GPIO25, East->GPIO26, South->GPIO27, West->GPIO32
 * to bring back the fins 
 * 
 ******************************************************************************/

#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <ESP32Servo.h>

// I2C pins
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int MLX90640_ADDR = 0x33;

// Servo pins
const int SERVO_NORTH = 25;
const int SERVO_EAST = 26;
const int SERVO_SOUTH = 27;
const int SERVO_WEST = 32;

// Servo limits
const int SERVO_CENTER = 45 ; //90;  
const int SERVO_MIN = 0; //45;
const int SERVO_MAX = 65; //135;

// Thermal camera settings
const int FRAME_WIDTH = 32;
const int FRAME_HEIGHT = 24;
const int TOTAL_PIXELS = 768;

// Tracking parameters
const float TRACKING_GAIN = 2.5;
const float TEMP_THRESHOLD = 30.0;

// Smoothing for stable tracking 
float smoothTargetX = 16.0;  // Center of frame
float smoothTargetY = 12.0;  // Center of frame
const float SMOOTHING = 0.05  ;  // 0.2-0.6, lower = smoother but slower

// Global variabless
float thermalFrame[TOTAL_PIXELS];
int targetX = -1;
int targetY = -1;
bool targetLocked = false;
bool cameraReady = false;

int lastTargetX = -1;  
int lastTargetY = -1;  
//BACK TO original
unsigned long lastTargetMove = 0;
const unsigned long TARGET_TIMEOUT = 1500; // 0.5 seconds previus was 5000- 5 seconds

Adafruit_MLX90640 mlx;
Servo finNorth, finEast, finSouth, finWest;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_RATE = 10;  // 100 Hz - UPDATED FOR SPEED//previous 100;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(" ATOM - Thermal Tracking System");
  Serial.println("Initializing...");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  
  // Initializingn the thermal cam
  if (mlx.begin(MLX90640_ADDR, &Wire)) {
    Serial.println("Thermal camera: OK");
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_16_HZ);
    cameraReady = true;
  } else {
    Serial.println("Thermal camera: FAILED");
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

//main loop 
void loop() {
  unsigned long now = millis();
  
  if (now - lastUpdate >= UPDATE_RATE) {
    lastUpdate = now;
    
    if (cameraReady) {
      captureThermalFrame();
      detectTarget();
      updateServos();
      printStatus();
    }
  }
}

void captureThermalFrame() {
  if (mlx.getFrame(thermalFrame) != 0) {
    Serial.println("Frame capture failed");
  }
}

void detectTarget() {
  targetLocked = false;
  float maxTemp = TEMP_THRESHOLD;
  int maxIndex = -1;
  
  // Find hottest pixel
  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (thermalFrame[i] > maxTemp) {
      maxTemp = thermalFrame[i];
      maxIndex = i;
    }
  }
  
  if (maxIndex >= 0) {
    targetLocked = true;
    targetX = maxIndex % FRAME_WIDTH;
    targetY = maxIndex / FRAME_WIDTH;
    
    // Apply smoothing
    smoothTargetX = smoothTargetX * (1.0 - SMOOTHING) + targetX * SMOOTHING;
    smoothTargetY = smoothTargetY * (1.0 - SMOOTHING) + targetY * SMOOTHING;

    
    // Check if target has moved
    if (targetX != lastTargetX || targetY != lastTargetY) {
      lastTargetMove = millis();
      lastTargetX = targetX;
      lastTargetY = targetY;
    }
  }
}
void updateServos() {

    // Check if target hasn't moved for too long
  if (targetLocked && (millis() - lastTargetMove > TARGET_TIMEOUT)) {
    targetLocked = false; // Treat as no target
  }
  
  if (!targetLocked) {
    // No target - center all fins
    finNorth.write(SERVO_CENTER);
    finEast.write(SERVO_CENTER);
    finSouth.write(SERVO_CENTER);
    finWest.write(SERVO_CENTER);
    return;
  }
  
  // Calculating teh tracking error
  int centerX = FRAME_WIDTH / 2;
  int centerY = FRAME_HEIGHT / 2;
  
  //float errorX = (targetX - centerX) / (float)centerX;
  //float errorY = (targetY - centerY) / (float)centerY;
  
  //Now Calculating the tracking error - USING SMOOTHED VALUES
  float errorX = (smoothTargetX - centerX) / (float)centerX;
  float errorY = (smoothTargetY - centerY) / (float)centerY;
  
  // Calculating servo positions
  // Target right -> deflect east fin
  // Target left -> deflect west fin
  // Target down -> deflect south fin
  // Target up -> deflect north fin
  
  int northPos = SERVO_CENTER - (int)(errorY * TRACKING_GAIN * 20);
  int southPos = SERVO_CENTER + (int)(errorY * TRACKING_GAIN * 20);
  int eastPos = SERVO_CENTER + (int)(errorX * TRACKING_GAIN * 20);
  int westPos = SERVO_CENTER - (int)(errorX * TRACKING_GAIN * 20);
  
  // Apply limits
  northPos = constrain(northPos, SERVO_MIN, SERVO_MAX);
  southPos = constrain(southPos, SERVO_MIN, SERVO_MAX);
  eastPos = constrain(eastPos, SERVO_MIN, SERVO_MAX);
  westPos = constrain(westPos, SERVO_MIN, SERVO_MAX);
  
  // Move servos
  finNorth.write(northPos);
  finEast.write(eastPos);
  finSouth.write(southPos);
  finWest.write(westPos);
}


//print outs 
void printStatus() {
  Serial.println("--- Status ---");
  
  if (targetLocked) {
    Serial.print("Target locked at [");
    Serial.print(targetX);
    Serial.print(",");
    Serial.print(targetY);
    Serial.print("] ");
    Serial.print(thermalFrame[targetY * FRAME_WIDTH + targetX], 1);
    Serial.println(" C");
    
    int offsetX = targetX - (FRAME_WIDTH / 2);
    int offsetY = targetY - (FRAME_HEIGHT / 2);
    
    Serial.print("Offset: X=");
    Serial.print(offsetX);
    Serial.print(" Y=");
    Serial.println(offsetY);
  } else {
    Serial.println("No target detected");
  }
  
  Serial.println();
}