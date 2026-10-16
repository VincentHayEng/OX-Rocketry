/*******************************************************************************
 * ESP32 #2: GROUND STATION RECEIVER (SIMULATION MODE)
 * 
 * This code simulates receiving telemetry from the onboard flight computer
 * Displays real-time data as if receiving via ESP-NOW
 * 
 * SIMULATED DISPLAY:
 * • Receives flight telemetry every 100ms
 * • Displays altitude, speed, GPS data
 * • Alerts on apogee and landing
 * • Shows recovery information
 * 
 * Compile in Arduino IDE with ESP32 board selected
 ******************************************************************************/

// ============================================================================
// TELEMETRY DATA STRUCTURE
// ============================================================================
struct TelemetryData {
  int state;              // Flight state (0-6)
  float altitude;         // Current altitude (m)
  float maxAltitude;      // Peak altitude
  float verticalSpeed;    // m/s
  float acceleration;     // G-force
  float batteryVoltage;   // Volts
  float latitude;         // GPS latitude
  float longitude;        // GPS longitude
  int gpsSatellites;      // Satellite count
  float flightTime;       // Seconds
  bool drogueDeployed;
  bool mainDeployed;
};

TelemetryData rocketData;

// Flight states
const char* stateNames[] = {
  "IDLE", "ARMED", "LAUNCHED", "ASCENT", "APOGEE", "DESCENT", "LANDED"
};

// ============================================================================
// SIMULATION VARIABLES
// ============================================================================
float simTime = 0.0;
unsigned long lastUpdate = 0;
unsigned long lastAlert = 0;
bool connectionLost = false;
bool apogeeAlerted = false;
bool landingAlerted = false;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║     ESP32 GROUND STATION - SIMULATION MODE               ║");
  Serial.println("║     Mission Control Receiver                             ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize display
  Serial.println("┌─── SYSTEM INITIALIZATION ────────────────────────────────┐");
  delay(300);
  Serial.println("│ ✓ OLED Display................... OK (Simulated)        │");
  delay(200);
  Serial.println("│ ✓ Buzzer......................... OK (Simulated)        │");
  delay(200);
  Serial.println("│ ✓ ESP-NOW Receiver............... OK (Simulated)        │");
  delay(200);
  Serial.println("│ ✓ SD Card Logger................. OK (Simulated)        │");
  Serial.println("└──────────────────────────────────────────────────────────┘");
  Serial.println();
  
  Serial.println("📡 WAITING FOR ROCKET TELEMETRY...");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
  
  // Initialize telemetry data
  rocketData.state = 1;  // Start at ARMED
  rocketData.altitude = 0.0;
  rocketData.maxAltitude = 0.0;
  rocketData.verticalSpeed = 0.0;
  rocketData.acceleration = 0.0;
  rocketData.batteryVoltage = 4.2;
  rocketData.latitude = 45.4215;
  rocketData.longitude = -75.6972;
  rocketData.gpsSatellites = 8;
  rocketData.flightTime = 0.0;
  rocketData.drogueDeployed = false;
  rocketData.mainDeployed = false;
  
  lastUpdate = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Simulate receiving telemetry every 100ms
  if (currentTime - lastUpdate >= 100) {
    float deltaTime = (currentTime - lastUpdate) / 1000.0;
    lastUpdate = currentTime;
    
    // Simulate receiving data from rocket
    simulateRocketData(deltaTime);
    
    // Display telemetry
    displayGroundStationData();
    
    // Check for alerts
    checkAlerts();
  }
  
  delay(10);
}

// ============================================================================
// SIMULATE RECEIVING ROCKET DATA
// ============================================================================
void simulateRocketData(float deltaTime) {
  const float ASCENT_DURATION = 8.0;
  const float MAX_ALTITUDE = 300.0;
  const float DESCENT_RATE = -5.0;
  
  // Progress through flight states
  if (rocketData.state == 1) {  // ARMED
    simTime += deltaTime;
    if (simTime > 5.0) {
      rocketData.state = 2;  // LAUNCHED
      simTime = 0;
      Serial.println("\n🚀 ROCKET LAUNCHED - TELEMETRY ACTIVE!");
      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }
  }
  else if (rocketData.state == 2) {  // LAUNCHED
    simTime += deltaTime;
    rocketData.acceleration = 8.0;
    rocketData.flightTime = simTime;
    
    if (simTime > 2.0) {
      rocketData.state = 3;  // ASCENT
      simTime = 0;
    }
  }
  else if (rocketData.state == 3) {  // ASCENT
    simTime += deltaTime;
    rocketData.flightTime += deltaTime;
    
    float progress = simTime / ASCENT_DURATION;
    rocketData.altitude = MAX_ALTITUDE * (2 * progress - progress * progress);
    rocketData.verticalSpeed = MAX_ALTITUDE * (2 - 2 * progress) / ASCENT_DURATION;
    rocketData.acceleration = 1.0 - (progress * 0.5);
    
    if (rocketData.altitude > rocketData.maxAltitude) {
      rocketData.maxAltitude = rocketData.altitude;
    }
    
    // Check for apogee
    if (rocketData.verticalSpeed < -0.5 && rocketData.altitude < rocketData.maxAltitude - 0.5) {
      rocketData.state = 4;  // APOGEE
    }
  }
  else if (rocketData.state == 4) {  // APOGEE
    rocketData.drogueDeployed = true;
    rocketData.state = 5;  // DESCENT
    simTime = 0;
  }
  else if (rocketData.state == 5) {  // DESCENT
    simTime += deltaTime;
    rocketData.flightTime += deltaTime;
    
    rocketData.altitude = rocketData.maxAltitude - (simTime * abs(DESCENT_RATE));
    if (rocketData.altitude < 0) rocketData.altitude = 0;
    
    rocketData.verticalSpeed = DESCENT_RATE;
    rocketData.acceleration = 1.0;
    
    // Deploy main at 100m
    if (rocketData.altitude < 100 && !rocketData.mainDeployed) {
      rocketData.mainDeployed = true;
    }
    
    // Check for landing
    if (rocketData.altitude <= 0) {
      rocketData.state = 6;  // LANDED
    }
  }
  else if (rocketData.state == 6) {  // LANDED
    rocketData.altitude = 0;
    rocketData.verticalSpeed = 0;
    rocketData.acceleration = 1.0;
    
    // Simulate battery drain
    rocketData.batteryVoltage -= 0.001;
    if (rocketData.batteryVoltage < 3.0) {
      rocketData.batteryVoltage = 3.0;
    }
  }
}

// ============================================================================
// DISPLAY GROUND STATION DATA
// ============================================================================
void displayGroundStationData() {
  static unsigned long lastDisplay = 0;
  
  // Update display every 1 second
  if (millis() - lastDisplay < 1000) return;
  lastDisplay = millis();
  
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              GROUND STATION MISSION CONTROL              ║");
  Serial.println("╠═══════════════════════════════════════════════════════════╣");
  
  // Flight Status
  Serial.print("║ Status: ");
  Serial.print(stateNames[rocketData.state]);
  for (int i = strlen(stateNames[rocketData.state]); i < 50; i++) Serial.print(" ");
  Serial.println("║");
  
  // Altitude
  Serial.print("║ Altitude:        ");
  char altStr[20];
  sprintf(altStr, "%.1f m", rocketData.altitude);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // Max Altitude
  Serial.print("║ Max Altitude:    ");
  sprintf(altStr, "%.1f m", rocketData.maxAltitude);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // Vertical Speed
  Serial.print("║ Vertical Speed:  ");
  sprintf(altStr, "%.1f m/s", rocketData.verticalSpeed);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // Acceleration
  Serial.print("║ Acceleration:    ");
  sprintf(altStr, "%.1f G", rocketData.acceleration);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // Battery
  Serial.print("║ Battery:         ");
  sprintf(altStr, "%.2f V", rocketData.batteryVoltage);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // GPS
  Serial.print("║ GPS:             ");
  sprintf(altStr, "%d/8 sats", rocketData.gpsSatellites);
  Serial.print(altStr);
  for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
  Serial.println("║");
  
  // Flight Time
  if (rocketData.state >= 2) {
    Serial.print("║ Flight Time:     ");
    sprintf(altStr, "%.1f s", rocketData.flightTime);
    Serial.print(altStr);
    for (int i = strlen(altStr); i < 41; i++) Serial.print(" ");
    Serial.println("║");
  }
  
  // Parachute Status
  Serial.print("║ Drogue Chute:    ");
  Serial.print(rocketData.drogueDeployed ? "DEPLOYED ✓" : "STOWED");
  for (int i = 0; i < (rocketData.drogueDeployed ? 31 : 37); i++) Serial.print(" ");
  Serial.println("║");
  
  Serial.print("║ Main Chute:      ");
  Serial.print(rocketData.mainDeployed ? "DEPLOYED ✓" : "STOWED");
  for (int i = 0; i < (rocketData.mainDeployed ? 31 : 37); i++) Serial.print(" ");
  Serial.println("║");
  
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
}

// ============================================================================
// CHECK FOR ALERTS
// ============================================================================
void checkAlerts() {
  unsigned long currentTime = millis();
  
  // Apogee Alert
  if (rocketData.state == 4 && !apogeeAlerted) {
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║                    🎯 APOGEE REACHED!                     ║");
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    Serial.print("║  Maximum Altitude: ");
    Serial.print(rocketData.maxAltitude, 1);
    Serial.println(" m                              ║");
    Serial.print("║  Time to Apogee:   ");
    Serial.print(rocketData.flightTime, 1);
    Serial.println(" s                                  ║");
    Serial.println("║  Drogue Parachute: DEPLOYED ✓                            ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("🔊 ALARM: BEEP BEEP BEEP!");
    Serial.println();
    apogeeAlerted = true;
    lastAlert = currentTime;
  }
  
  // Landing Alert
  if (rocketData.state == 6 && !landingAlerted) {
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║                  🎉 ROCKET HAS LANDED!                    ║");
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    Serial.print("║  Flight Duration:  ");
    Serial.print(rocketData.flightTime, 1);
    Serial.println(" seconds                          ║");
    Serial.print("║  Max Altitude:     ");
    Serial.print(rocketData.maxAltitude, 1);
    Serial.println(" m                                  ║");
    Serial.println("║  All Systems:      NOMINAL ✓                             ║");
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    Serial.println("║              📍 RECOVERY COORDINATES                      ║");
    Serial.print("║  Latitude:         ");
    Serial.print(rocketData.latitude, 6);
    Serial.println("                     ║");
    Serial.print("║  Longitude:        ");
    Serial.print(rocketData.longitude, 6);
    Serial.println("                    ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("🔊 LOCATOR BEEPING: BEEP... BEEP... BEEP...");
    Serial.println();
    landingAlerted = true;
    lastAlert = currentTime;
  }
  
  // Recovery Beeps (every 2 seconds when landed)
  if (rocketData.state == 6 && currentTime - lastAlert > 2000) {
    Serial.println("🔊 RECOVERY BEEP - Rocket Location Signal Active");
    Serial.print("   GPS: ");
    Serial.print(rocketData.latitude, 6);
    Serial.print(", ");
    Serial.println(rocketData.longitude, 6);
    Serial.println();
    lastAlert = currentTime;
  }
  
  // Connection Loss Warning (simulated - won't happen in this sim)
  if (rocketData.state >= 2 && rocketData.state < 6) {
    // Simulate checking connection every 5 seconds
    // In real code, this would check for missing telemetry packets
  }
}