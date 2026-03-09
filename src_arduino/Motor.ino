#include <SabertoothSimplified.h>

// Initialize Sabertooth communication on Hardware Serial0 (TX1/D1)
SabertoothSimplified ST(Serial0); 

// System Constraints & Timeouts
unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // ms threshold for serial timeout

// Motor Speed Parameters (Range: 0-127)
const int DRIVE_SPEED = 127; 
const int TURN_SPEED = 30;  

void setup() {
  Serial0.begin(9600);    // Sabertooth communication baud rate
  Serial.begin(115200);   // Raspberry Pi serial communication baud rate
  
  delay(2000); // Sabertooth hardware initialization delay
  
  // Ensure motors are halted on startup
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void loop() {
  // Process incoming serial buffer
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Reset watchdog timer on successful read
    lastCommandTime = millis(); 

    // Parse command and actuate motors
    if (command == "FORWARD") {
      ST.motor(1, DRIVE_SPEED);
      ST.motor(2, DRIVE_SPEED);
    }
    else if (command == "BACKWARD") {
      ST.motor(1, -DRIVE_SPEED); 
      ST.motor(2, -DRIVE_SPEED);
    } 
    else if (command == "LEFT") {
      ST.motor(1, -TURN_SPEED); 
      ST.motor(2, TURN_SPEED);  
    } 
    else if (command == "RIGHT") {
      ST.motor(1, TURN_SPEED);  
      ST.motor(2, -TURN_SPEED); 
    } 
    else if (command == "STOP") {
      ST.motor(1, 0);
      ST.motor(2, 0);
    }
  }

  // Watchdog Timer: Halt motors if command timeout is exceeded
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    ST.motor(1, 0);
    ST.motor(2, 0);
  }
}
