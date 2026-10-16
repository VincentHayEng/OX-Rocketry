/*******************************************************************************
 * MLX90640 THERMAL CAMERA TEST
 * Project ATOM 
 * 
 * CONNECTIONS:
 * MLX90640 -> ESP32
 * VIN/VDD  -> 3.3V
 * GND      -> GND
 * SDA      -> GPIO 21
 * SCL      -> GPIO 22
 * 
 * Note:  I2C pullup resistors (4.7kohm)
 ******************************************************************************/

#include <Wire.h>

// Comment out next line when hardware is connected
//#define DEBUG_MODE_NO_HARDWARE

#ifndef DEBUG_MODE_NO_HARDWARE
  #include <Adafruit_MLX90640.h>
#endif

// I2C Configuration
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int MLX90640_I2C_ADDR = 0x33;

// Thermal Camera Configuration
const int FRAME_WIDTH = 32;
const int FRAME_HEIGHT = 24;
const int TOTAL_PIXELS = FRAME_WIDTH * FRAME_HEIGHT; // 768 pixels

// Target Detection Parameters
const float TEMP_THRESHOLD = 30.0;  // Temperature threshold for target detection (°C)
const float AMBIENT_OFFSET = 5.0;   // Degrees above ambient to consider as target

// Global Variables
float thermalFrame[TOTAL_PIXELS];
float ambientTemp = 25.0;
float maxTemp = 0.0;
float minTemp = 100.0;
int targetX = -1;
int targetY = -1;
bool targetDetected = false;
bool cameraConnected = false;

unsigned long lastFrameTime = 0;
const unsigned long FRAME_INTERVAL = 500; // 2 FPS (slower for readability)

#ifndef DEBUG_MODE_NO_HARDWARE
  Adafruit_MLX90640 mlx;
#endif

#ifdef DEBUG_MODE_NO_HARDWARE
  float simTime = 0.0;
  int simTargetX = 16;
  int simTargetY = 12;
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println();
  Serial.println("========================================================");
  Serial.println(" MLX90640 THERMAL CAMERA TEST");
  Serial.println(" Project ATOM - Autonomous Tracking Ordnance Module");
  Serial.println("========================================================");
  Serial.println();
  
  printConnectionDiagram();
  
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.println("MODE: DEBUG (Simulated Data)");
    Serial.println("Hardware not required for testing");
    Serial.println();
    cameraConnected = false;
  #else
    Serial.println("MODE: HARDWARE");
    Serial.println();
    initializeCamera();
  #endif
  
  Serial.println("========================================================");
  Serial.println("System Ready - Starting thermal imaging");
  Serial.println("========================================================");
  Serial.println();
  
  lastFrameTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastFrameTime >= FRAME_INTERVAL) {
    lastFrameTime = currentTime;
    
    #ifdef DEBUG_MODE_NO_HARDWARE
      captureSimulatedFrame();
    #else
      captureRealFrame();
    #endif
    
    processFrame();
    detectTarget();
    displayTelemetry();
  }
  
  delay(10);
}

void printConnectionDiagram() {
  Serial.println("WIRING CONNECTIONS:");
  Serial.println();
}

#ifndef DEBUG_MODE_NO_HARDWARE
void initializeCamera() {
  Serial.println("HARDWARE INITIALIZATION");
  Serial.println();
  
  // Initialize I2C
  Serial.print("Initializing I2C bus... ");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz for MLX90640
  delay(100);
  Serial.println("OK");
  
  // Initialize MLX90640
  Serial.print("Initializing MLX90640 thermal camera... ");
  
  if (mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
    Serial.println("OK");
    cameraConnected = true;
    
    // Set refresh rate
    Serial.print("Setting refresh rate to 8Hz... ");
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_8_HZ);
    Serial.println("OK");
    
    Serial.println();
    Serial.println("Camera initialized successfully!");
  } else {
    Serial.println("FAILED");
    Serial.println("WARNING: MLX90640 not detected at address 0x33");
    Serial.println("Check wiring ");
    cameraConnected = false;
  }
  
  Serial.println();
}

void captureRealFrame() {
  if (!cameraConnected) {
    return;
  }
  
  if (mlx.getFrame(thermalFrame) != 0) {
    Serial.println("ERROR: Failed to capture frame");
    return;
  }
}
#endif

#ifdef DEBUG_MODE_NO_HARDWARE
void captureSimulatedFrame() {
  simTime += 0.1;
  
  // Simulate moving target
  simTargetX = 16 + (int)(8 * sin(simTime * 0.3));
  simTargetY = 12 + (int)(6 * cos(simTime * 0.5));
  
  // Keep target within bounds
  simTargetX = constrain(simTargetX, 0, FRAME_WIDTH - 1);
  simTargetY = constrain(simTargetY, 0, FRAME_HEIGHT - 1);
  
  // Generate simulated thermal frame
  ambientTemp = 25.0 + random(-10, 10) / 10.0;
  
  for (int y = 0; y < FRAME_HEIGHT; y++) {
    for (int x = 0; x < FRAME_WIDTH; x++) {
      int index = y * FRAME_WIDTH + x;
      
      // Calculate distance from simulated target
      float dx = x - simTargetX;
      float dy = y - simTargetY;
      float distance = sqrt(dx * dx + dy * dy);
      
      // Create hot spot at target location
      if (distance < 3.0) {
        thermalFrame[index] = ambientTemp + 15.0 - (distance * 2.0);
      } else {
        thermalFrame[index] = ambientTemp + random(-20, 20) / 10.0;
      }
    }
  }
}
#endif

void processFrame() {
  // Find min and max temperatures
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
  
  // Find hottest pixel above threshold
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

void displayTelemetry() {
  Serial.println("--- THERMAL CAMERA TELEMETRY ---");
  Serial.println();
  
  #ifdef DEBUG_MODE_NO_HARDWARE
    Serial.print("Mode: SIMULATED DATA");
    Serial.print(" | Time: ");
    Serial.print(simTime, 1);
    Serial.println("s");
  #else
    Serial.print("Mode: REAL DATA");
    Serial.print(" | Camera: ");
    Serial.println(cameraConnected ? "Connected" : "Disconnected");
  #endif
  
  Serial.println();
  Serial.println("THERMAL DATA:");
  Serial.print("  Ambient: ");
  Serial.print(ambientTemp, 1);
  Serial.print(" °C | Min: ");
  Serial.print(minTemp, 1);
  Serial.print(" °C | Max: ");
  Serial.print(maxTemp, 1);
  Serial.print(" °C | Range: ");
  Serial.print(maxTemp - minTemp, 1);
  Serial.println(" °C");
  
  Serial.println();
  Serial.println("TARGET:");
  
  if (targetDetected) {
    Serial.print("  ★ LOCKED at [");
    Serial.print(targetX);
    Serial.print(",");
    Serial.print(targetY);
    Serial.print("] ");
    
    int targetIndex = targetY * FRAME_WIDTH + targetX;
    Serial.print(thermalFrame[targetIndex], 1);
    Serial.print("°C");
    
    // Calculate offset from center
    int centerX = FRAME_WIDTH / 2;
    int centerY = FRAME_HEIGHT / 2;
    int offsetX = targetX - centerX;
    int offsetY = targetY - centerY;
    
    Serial.print(" | Offset: X");
    if (offsetX >= 0) Serial.print("+");
    Serial.print(offsetX);
    Serial.print(" Y");
    if (offsetY >= 0) Serial.print("+");
    Serial.print(offsetY);
    
    Serial.print(" | ");
    
    if (abs(offsetX) <= 2 && abs(offsetY) <= 2) {
      Serial.println("CENTERED");
    } else {
      if (offsetX > 2) Serial.print("RIGHT ");
      else if (offsetX < -2) Serial.print("LEFT ");
      if (offsetY > 2) Serial.print("DOWN ");
      else if (offsetY < -2) Serial.print("UP ");
      Serial.println();
    }
    
  } else {
    Serial.println("  NO TARGET - Searching...");
  }
  
  Serial.println();
  
  // Full frame ASCII visualization (scaled down)
  Serial.println();
  Serial.println("THERMAL VIEW (Full 32x24 frame, scaled):");
  Serial.println("  ┌────────────────────────────────┐");
  
  // Show every 3rd row and every 2nd column for compact view
  for (int y = 0; y < FRAME_HEIGHT; y += 3) {
    Serial.print("  │");
    for (int x = 0; x < FRAME_WIDTH; x += 2) {
      int index = y * FRAME_WIDTH + x;
      float temp = thermalFrame[index];
      
      // Check if target is in this or adjacent cell
      bool isTarget = false;
      if (targetDetected) {
        for (int dy = 0; dy < 3 && y + dy < FRAME_HEIGHT; dy++) {
          for (int dx = 0; dx < 2 && x + dx < FRAME_WIDTH; dx++) {
            if (x + dx == targetX && y + dy == targetY) {
              isTarget = true;
            }
          }
        }
      }
      
      if (isTarget) {
        Serial.print("XX");
      } else if (temp > ambientTemp + 8) {
        Serial.print("##");
      } else if (temp > ambientTemp + 4) {
        Serial.print("::");
      } else if (temp > ambientTemp + 1) {
        Serial.print("..");
      } else {
        Serial.print("  ");
      }
    }
    Serial.println("│");
  }
  
  Serial.println("  └────────────────────────────────┘");
  Serial.println("  Legend: XX=Target, ##=Very Hot, ::=Hot, ..=Warm, (space)=Cool");
  
  Serial.println();
  Serial.println("========================================================");
  Serial.println();
}