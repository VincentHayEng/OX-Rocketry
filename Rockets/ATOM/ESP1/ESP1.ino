/*******************************************************************************
 * ESP32 #1: ONBOARD ROCKET FLIGHT COMPUTER (SIMULATION MODE)
 * 
 * This code runs WITHOUT hardware and simulates a complete rocket flight
 * with realistic sensor data for testing and debugging
 * 
 * SIMULATED SENSORS:
 * • BMP280 - Altitude (0m → 300m → 0m)
 * • MPU6050 - Acceleration (0G → 8G → 1G)
 * • GPS - Coordinates with drift
 * • Battery - Voltage drain simulation
 * 
 * Compile in Arduino IDE with ESP32 board selected
 ******************************************************************************/

// ============================================================================
// FLIGHT STATES
// ============================================================================
enum FlightState {
  STATE_IDLE,
  STATE_ARMED,
  STATE_LAUNCHED,
  STATE_ASCENT,
  STATE_APOGEE,
  STATE_DESCENT,
  STATE_LANDED
};

FlightState currentState = STATE_IDLE;
const char* stateNames[] = {
  "IDLE", "ARMED", "LAUNCHED", "ASCENT", "APOGEE", "DESCENT", "LANDED"
};

// ============================================================================
// SIMULATED SENSOR DATA
// ============================================================================
float altitude = 0.0;              // Current altitude (m)
float maxAltitude = 0.0;           // Peak altitude reached
float verticalSpeed = 0.0;         // m/s
float acceleration = 0.0;          // G-force
float batteryVoltage = 4.2;        // LiPo voltage
float latitude = 45.4215;          // GPS latitude
float longitude = -75.6972;        // GPS longitude (Ottawa)
int gps_satellites = 0;            // GPS satellite count

// Flight timing
unsigned long flightStartTime = 0;
unsigned long flightTime = 0;
unsigned long lastUpdate = 0;

// Apogee detection
int apogeeConfirmCount = 0;
bool drogueDeployed = false;
bool mainDeployed = false;
bool flightComplete = false;

// Simulation parameters
float simTime = 0.0;               // Simulation time in seconds
const float ASCENT_DURATION = 8.0; // 8 seconds to apogee
const float MAX_ALTITUDE = 300.0;  // 300 meters peak
const float DESCENT_RATE = -5.0;   // -5 m/s descent

// ============================================================================
// SETUP - Initialization
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║   ESP32 ROCKET FLIGHT COMPUTER - SIMULATION MODE         ║");
  Serial.println("║   Onboard Controller (No Hardware Required)              ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // Simulate sensor initialization
  Serial.println("┌─── SENSOR INITIALIZATION ───────────────────────────────┐");
  delay(500);
  Serial.println("│ ✓ BMP280 Barometer............... OK (Simulated)       │");
  delay(300);
  Serial.println("│ ✓ MPU6050 IMU.................... OK (Simulated)       │");
  delay(300);
  Serial.println("│ ✓ GPS Module..................... OK (Simulated)       │");
  delay(300);
  Serial.println("│ ✓ SD Card........................ OK (Simulated)       │");
  delay(300);
  Serial.println("│ ✓ Servo Motors................... OK (Simulated)       │");
  delay(300);
  Serial.println("│ ✓ ESP-NOW Telemetry.............. OK (Simulated)       │");
  Serial.println("└─────────────────────────────────────────────────────────┘");
  Serial.println();
  
  // GPS satellite acquisition simulation
  Serial.println("🛰️  GPS: Acquiring satellites...");
  for (int i = 0; i <= 8; i++) {
    gps_satellites = i;
    Serial.print("   Satellites: ");
    Serial.print(i);
    Serial.println("/8");
    delay(400);
  }
  Serial.println("   ✓ GPS LOCK ACQUIRED");
  Serial.println();
  
  currentState = STATE_ARMED;
  Serial.println("🚀 ROCKET ARMED - Ready for Launch");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
  
  lastUpdate = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Update simulation every 50ms (20Hz)
  if (currentTime - lastUpdate >= 50) {
    float deltaTime = (currentTime - lastUpdate) / 1000.0;
    lastUpdate = currentTime;
    
    // Update simulation time
    if (currentState >= STATE_LAUNCHED && currentState < STATE_LANDED) {
      simTime += deltaTime;
    }
    
    // Run state machine
    switch (currentState) {
      case STATE_IDLE:
        stateIdle();
        break;
        
      case STATE_ARMED:
        stateArmed();
        break;
        
      case STATE_LAUNCHED:
        stateLaunched();
        break;
        
      case STATE_ASCENT:
        stateAscent();
        break;
        
      case STATE_APOGEE:
        stateApogee();
        break;
        
      case STATE_DESCENT:
        stateDescent();
        break;
        
      case STATE_LANDED:
        stateLanded();
        break;
    }
    
    // Display telemetry
    displayTelemetry();
  }
  
  delay(10);
}

// ============================================================================
// STATE: IDLE
// ============================================================================
void stateIdle() {
  // Nothing happening, waiting for initialization
}

// ============================================================================
// STATE: ARMED (Pre-flight checks)
// ============================================================================
void stateArmed() {
  static unsigned long armedTime = 0;
  if (armedTime == 0) armedTime = millis();
  
  // Simulate waiting on launch pad for 5 seconds, then auto-launch
  if (millis() - armedTime > 5000) {
    Serial.println("\n🔥 LAUNCH DETECTED! (Acceleration > 3G)");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    currentState = STATE_LAUNCHED;
    flightStartTime = millis();
    simTime = 0.0;
  }
}

// ============================================================================
// STATE: LAUNCHED (Initial acceleration phase)
// ============================================================================
void stateLaunched() {
  // Simulate rapid acceleration
  acceleration = 8.0;  // 8G during motor burn
  
  // Transition to ascent after 2 seconds
  if (simTime > 2.0) {
    currentState = STATE_ASCENT;
    Serial.println("\n⬆️  MOTOR BURNOUT - Entering Coast Phase");
  }
}

// ============================================================================
// STATE: ASCENT (Coasting upward to apogee)
// ============================================================================
void stateAscent() {
  // Simulate altitude gain (parabolic trajectory)
  float progress = simTime / ASCENT_DURATION;
  altitude = MAX_ALTITUDE * (2 * progress - progress * progress);
  
  // Calculate vertical speed
  verticalSpeed = MAX_ALTITUDE * (2 - 2 * progress) / ASCENT_DURATION;
  
  // Simulate deceleration as we approach apogee
  acceleration = 1.0 - (progress * 0.5);  // 1G → 0.5G
  
  // Track max altitude
  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  }
  
  // Check for apogee (triple confirmation)
  if (detectApogee()) {
    currentState = STATE_APOGEE;
    Serial.println("\n🎯 APOGEE DETECTED!");
    Serial.print("   Maximum Altitude: ");
    Serial.print(maxAltitude, 1);
    Serial.println(" m");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }
}

// ============================================================================
// APOGEE DETECTION ALGORITHM (CRITICAL!)
// ============================================================================
bool detectApogee() {
  bool condition1 = (verticalSpeed < -0.5);           // Speed is negative
  bool condition2 = (altitude < maxAltitude - 0.5);   // Altitude decreasing
  bool condition3 = (acceleration < 0.8);             // Less than 1G
  
  if (condition1 && condition2 && condition3) {
    apogeeConfirmCount++;
    if (apogeeConfirmCount >= 3) {  // 3 consecutive readings (150ms)
      return true;
    }
  } else {
    apogeeConfirmCount = 0;  // Reset if any condition fails
  }
  
  return false;
}

// ============================================================================
// STATE: APOGEE (Deploy drogue chute)
// ============================================================================
void stateApogee() {
  if (!drogueDeployed) {
    Serial.println("\n🪂 DEPLOYING DROGUE PARACHUTE...");
    Serial.println("   Servo 1: 0° → 90° (0.5s)");
    drogueDeployed = true;
    delay(500);  // Simulate servo movement
    Serial.println("   ✓ Drogue Deployed Successfully");
    Serial.println();
  }
  
  // Immediately transition to descent
  currentState = STATE_DESCENT;
  simTime = 0;  // Reset timer for descent phase
}

// ============================================================================
// STATE: DESCENT (Falling with drogue, deploy main at 100m)
// ============================================================================
void stateDescent() {
  // Simulate descent at constant rate
  altitude = maxAltitude - (simTime * abs(DESCENT_RATE));
  if (altitude < 0) altitude = 0;
  
  verticalSpeed = DESCENT_RATE;
  acceleration = 1.0;  // 1G during freefall
  
  // Deploy main chute at 100m
  if (altitude < 100 && !mainDeployed) {
    Serial.println("\n🪂 DEPLOYING MAIN PARACHUTE...");
    Serial.println("   Altitude: 100m - Safe Deployment Height");
    Serial.println("   Servo 2: 0° → 90° (0.5s)");
    mainDeployed = true;
    delay(500);
    Serial.println("   ✓ Main Chute Deployed Successfully");
    Serial.println();
  }
  
  // Check for landing
  if (altitude <= 0) {
    currentState = STATE_LANDED;
    flightTime = millis() - flightStartTime;
    Serial.println("\n🎉 ROCKET LANDED!");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    displayFlightSummary();
  }
}

// ============================================================================
// STATE: LANDED (Recovery mode)
// ============================================================================
void stateLanded() {
  static int beepCount = 0;
  static unsigned long lastBeep = 0;
  
  if (!flightComplete) {
    Serial.println("\n📍 RECOVERY MODE ACTIVE");
    Serial.println("   GPS Coordinates:");
    Serial.print("   Latitude:  ");
    Serial.println(latitude, 6);
    Serial.print("   Longitude: ");
    Serial.println(longitude, 6);
    Serial.println("\n   🔊 Buzzer: Beeping for recovery...");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    flightComplete = true;
  }
  
  // Simulate recovery beeper (every 2 seconds)
  if (millis() - lastBeep > 2000) {
    beepCount++;
    Serial.print("   🔊 BEEP #");
    Serial.print(beepCount);
    Serial.println(" - Locator Active");
    lastBeep = millis();
  }
  
  // Simulate battery drain
  batteryVoltage -= 0.001;
  if (batteryVoltage < 3.0) {
    Serial.println("\n⚠️  LOW BATTERY WARNING!");
    batteryVoltage = 3.0;
  }
}

// ============================================================================
// TELEMETRY DISPLAY
// ============================================================================
void displayTelemetry() {
  static unsigned long lastDisplay = 0;
  
  // Display every 1 second
  if (millis() - lastDisplay < 1000) return;
  lastDisplay = millis();
  
  // Skip telemetry display when landed (already shown)
  if (currentState == STATE_LANDED) return;
  
  Serial.println("┌─────────────────── TELEMETRY ───────────────────────┐");
  Serial.print("│ State: ");
  Serial.print(stateNames[currentState]);
  for (int i = strlen(stateNames[currentState]); i < 42; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ Altitude: ");
  Serial.print(altitude, 1);
  Serial.print(" m");
  for (int i = String(altitude, 1).length(); i < 38; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ Vertical Speed: ");
  Serial.print(verticalSpeed, 1);
  Serial.print(" m/s");
  for (int i = String(verticalSpeed, 1).length(); i < 31; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ Acceleration: ");
  Serial.print(acceleration, 1);
  Serial.print(" G");
  for (int i = String(acceleration, 1).length(); i < 34; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print(" V");
  for (int i = String(batteryVoltage, 2).length(); i < 37; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ GPS Sats: ");
  Serial.print(gps_satellites);
  Serial.print("/8");
  for (int i = String(gps_satellites).length(); i < 38; i++) Serial.print(" ");
  Serial.println("│");
  
  if (currentState >= STATE_LAUNCHED) {
    Serial.print("│ Flight Time: ");
    Serial.print(simTime, 1);
    Serial.print(" s");
    for (int i = String(simTime, 1).length(); i < 35; i++) Serial.print(" ");
    Serial.println("│");
  }
  
  Serial.println("└─────────────────────────────────────────────────────┘");
  Serial.println();
}

// ============================================================================
// FLIGHT SUMMARY
// ============================================================================
void displayFlightSummary() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              FLIGHT SUMMARY - MISSION SUCCESS            ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.print("   Maximum Altitude:    ");
  Serial.print(maxAltitude, 1);
  Serial.println(" m");
  Serial.print("   Flight Duration:     ");
  Serial.print(flightTime / 1000.0, 1);
  Serial.println(" seconds");
  Serial.print("   Drogue Deployed:     ");
  Serial.println(drogueDeployed ? "YES ✓" : "NO ✗");
  Serial.print("   Main Chute Deployed: ");
  Serial.println(mainDeployed ? "YES ✓" : "NO ✗");
  Serial.print("   Final Battery:       ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");
  Serial.println();
  Serial.println("   All Systems Nominal");
  Serial.println("    Flight Data Saved to SD Card (Simulated)");
  Serial.println();
}