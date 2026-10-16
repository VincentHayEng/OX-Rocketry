  /*******************************************************************************
 * SIMPLE SERVO TEST - North Fin Only
 * Just sweeps the servo back and forth to verify it works
 * 
 * CONNECTIONS:
 * ------------
 * Servo (North Fin):
 *   BROWN wire  -> ESP32 GND
 *   RED wire    -> ESP32 3.3V  (you said you connected to 3.3V)
 *   ORANGE wire -> ESP32 GPIO 25
 * 
 * This will make the servo sweep slowly back and forth
 * If servo doesn't move, check power or try different GPIO pin
 ******************************************************************************/

#include <ESP32Servo.h>

Servo northServo;

const int SERVO_PIN = 25;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("  SIMPLE SERVO TEST - North Fin");
  Serial.println("========================================");
  Serial.println();
  
  Serial.print("Attaching servo to GPIO 25... ");
  northServo.attach(SERVO_PIN);
  Serial.println("OK");
  
  Serial.println();
  Serial.println("Starting servo sweep test...");
  Serial.println("Watch your servo - it should move!");
  Serial.println();
}

void loop() {
  // Sweep from 0 to 180 degrees
  Serial.println("Moving to 0 degrees");
  northServo.write(0);
  delay(2000);  // Wait 2 seconds
  
  Serial.println("Moving to 45 degrees");
  northServo.write(45);
  delay(2000);
  
  Serial.println("Moving to 90 degrees (center)");
  northServo.write(90);
  delay(2000);
  
  Serial.println("Moving to 135 degrees");
  northServo.write(135);
  delay(2000);
  
  Serial.println("Moving to 180 degrees");
  northServo.write(180);
  delay(2000);
  
  Serial.println();
  Serial.println("--- Sweep complete, restarting ---");
  Serial.println();
  delay(1000);
}