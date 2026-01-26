#include <Arduino.h>

// --- PINS ---
// Adjust these if you used different pins
const int LIFT_UP_PIN = D5;   // Connected to BTS7960 RPWM
const int LIFT_DOWN_PIN = D6; // Connected to BTS7960 LPWM

// --- SPEED SETTINGS ---
// 0 = Stop, 255 = Full Speed
const int LIFT_SPEED = 255; 

void setup() {
  // Configure pins as outputs
  pinMode(LIFT_UP_PIN, OUTPUT);
  pinMode(LIFT_DOWN_PIN, OUTPUT);
  
  // Safety: Start with motor stopped
  stopLift();
  
  // Wait 2 seconds before starting so you can get ready
  delay(2000);
}

void loop() {
  // --- GO UP ---
  // Ensure Down pin is 0 before writing Up pin to avoid shorting the bridge
  analogWrite(LIFT_DOWN_PIN, 0); 
  analogWrite(LIFT_UP_PIN, LIFT_SPEED); 
  delay(5000); // Run for 5 seconds
  
  // --- STOP ---
  stopLift();
  delay(2000); // Wait 2 seconds
  
  // --- GO DOWN ---
  analogWrite(LIFT_UP_PIN, 0); 
  analogWrite(LIFT_DOWN_PIN, LIFT_SPEED);
  delay(5000); // Run for 5 seconds
  
  // --- STOP ---
  stopLift();
  delay(2000); // Wait 2 seconds
}

// Helper function to stop the motor safely
void stopLift() {
  analogWrite(LIFT_UP_PIN, 0);
  analogWrite(LIFT_DOWN_PIN, 0);
}
