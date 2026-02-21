#include <SabertoothSimplified.h>

// Use Serial0 for the Nano ESP32 pins (TX1/D1)
SabertoothSimplified ST(Serial0); 

// --- SAFETY & TUNING ---
unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // Stop if no command received for 500ms

const int DRIVE_SPEED = 40; // Forward speed (0 to 127)
const int TURN_SPEED = 30;  // Turning speed (0 to 127)

void setup() {
  Serial0.begin(9600);   // Talk to Sabertooth
  Serial.begin(115200);  // Talk to Raspberry Pi (Match Python baud rate!)
  
  delay(2000); // Wait for Sabertooth to wake up
  
  // Start with a strict stop
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void loop() {
  // 1. Check for incoming commands from the Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // We received a command, reset the safety timer!
    lastCommandTime = millis(); 

    // 2. Execute the movement
    if (command == "FORWARD") {
      ST.motor(1, DRIVE_SPEED);
      ST.motor(2, DRIVE_SPEED);
    } 
    else if (command == "LEFT") {
      ST.motor(1, -TURN_SPEED); // Reverse left motor
      ST.motor(2, TURN_SPEED);  // Forward right motor
    } 
    else if (command == "RIGHT") {
      ST.motor(1, TURN_SPEED);  // Forward left motor
      ST.motor(2, -TURN_SPEED); // Reverse right motor
    } 
    else if (command == "STOP") {
      ST.motor(1, 0);
      ST.motor(2, 0);
    }
  }

  // --- 3. THE WATCHDOG TIMER ---
  // If 500ms pass without a word from the Pi, assume connection is lost and stop!
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    ST.motor(1, 0);
    ST.motor(2, 0);
  }
}
