#include <SabertoothSimplified.h>

// Hardware serial configuration
HardwareSerial SaberSerial(2); 
SabertoothSimplified ST(SaberSerial);

// Pin definitions
const int Fan1     = D2;
const int Fan2     = D3;
const int RPWM_PIN = D4;
const int LPWM_PIN = D5;
const int LIDAR_RX = D6;
const int LIDAR_TX = D7;

// Timing and state variables
unsigned long lastCommandTime = 0;
unsigned long lastLidarTime   = 0;
int lidarState = 0;
unsigned long lidarTimer = 0;

const unsigned long WATCHDOG_TIMEOUT = 500; 

// Motor speeds
const int DRIVE_SPEED = 127;
const int TURN_SPEED  = 63;

// Kinematic profiling parameters
int currentSpeedM1 = 0;
int currentSpeedM2 = 0;
int targetSpeedM1  = 0;
int targetSpeedM2  = 0;
int lastSentM1     = -999;
int lastSentM2     = -999;

const int ACCEL_STEP = 5;
const int BRAKE_STEP = 15; 
const int RAMP_INTERVAL = 50;
unsigned long lastRampTime = 0;

// LiDAR request payload
const byte request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
float currentDist = 0;

// Serial command buffer
static char cmdBuf[32];
static int  cmdLen = 0;

void setupLidar() {
  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void pollLidar() {
  if (lidarState == 0 && (millis() - lastLidarTime > 100)) {
    while (Serial1.available()) Serial1.read(); 
    Serial1.write(request, sizeof(request));
    lidarTimer = millis();
    lidarState = 1; 
  }
  else if (lidarState == 1 && (millis() - lidarTimer > 50)) {
    if (Serial1.available() >= 7) {
      byte response[7];
      for (int i = 0; i < 7; i++) response[i] = Serial1.read();
      
      if (response[0] == 0x01 && response[1] == 0x03) {
        int rawCm = (response[3] << 8) | response[4];
        currentDist = rawCm / 30.48; // Convert to feet
      }
    }
    lidarState = 0; 
    lastLidarTime = millis();
  }
}

void setup() {
  SaberSerial.begin(9600, SERIAL_8N1, -1, D8);
  Serial.begin(115200);
  setupLidar();

  pinMode(Fan1,     OUTPUT);  
  pinMode(Fan2,     OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);  
  pinMode(LPWM_PIN, OUTPUT);

  digitalWrite(RPWM_PIN, LOW);  
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(Fan1, HIGH);     
  digitalWrite(Fan2, HIGH);

  delay(2000);
  
  strcpy(cmdBuf, "STOP");
  handleCommand();   
  
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void handleCommand() {
  lastCommandTime = millis();

  if (strcmp(cmdBuf, "FORWARD") == 0) {
    targetSpeedM1 = DRIVE_SPEED;
    targetSpeedM2 = DRIVE_SPEED;
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "BACKWARD") == 0) {
    targetSpeedM1 = -DRIVE_SPEED;
    targetSpeedM2 = -DRIVE_SPEED;
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "LEFT") == 0) {
    targetSpeedM1 = -TURN_SPEED;
    targetSpeedM2 =  TURN_SPEED;
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "RIGHT") == 0) {
    targetSpeedM1 =  TURN_SPEED;
    targetSpeedM2 = -TURN_SPEED;
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "UP") == 0) {
    digitalWrite(LPWM_PIN, LOW);  digitalWrite(RPWM_PIN, HIGH);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
  }
  else if (strcmp(cmdBuf, "DOWN") == 0) {
    digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, HIGH);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
  }
  
  // Standard forward arcs
  else if (strcmp(cmdBuf, "ARC_LEFT") == 0) {
    targetSpeedM1 = TURN_SPEED;  
    targetSpeedM2 = DRIVE_SPEED; 
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "ARC_RIGHT") == 0) {
    targetSpeedM1 = DRIVE_SPEED; 
    targetSpeedM2 = TURN_SPEED;  
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  
  // Tight forward arcs
  else if (strcmp(cmdBuf, "TIGHT_ARC_LEFT") == 0) {
    targetSpeedM1 = 0;           
    targetSpeedM2 = TURN_SPEED;  
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "TIGHT_ARC_RIGHT") == 0) {
    targetSpeedM1 = TURN_SPEED;  
    targetSpeedM2 = 0;           
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }

  // Standard reverse arcs
  else if (strcmp(cmdBuf, "ARC_REV_LEFT") == 0) {
    targetSpeedM1 = -TURN_SPEED;  
    targetSpeedM2 = -DRIVE_SPEED; 
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "ARC_REV_RIGHT") == 0) {
    targetSpeedM1 = -DRIVE_SPEED; 
    targetSpeedM2 = -TURN_SPEED;  
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  
  // Tight reverse arcs
  else if (strcmp(cmdBuf, "TIGHT_ARC_REV_LEFT") == 0) {
    targetSpeedM1 = 0;            
    targetSpeedM2 = -TURN_SPEED;  
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "TIGHT_ARC_REV_RIGHT") == 0) {
    targetSpeedM1 = -TURN_SPEED;  
    targetSpeedM2 = 0;            
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  
  else if (strcmp(cmdBuf, "FAN") == 0) {
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "STOP") == 0) {
    targetSpeedM1 = 0;
    targetSpeedM2 = 0;
    digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, LOW);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
  }
}

void loop() {
  // Process incoming serial commands
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand();
        cmdLen = 0;
      }
    }
    else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
    else {
      cmdLen = 0; 
    }
  }

  // Dual-channel kinematic profiling
  if (millis() - lastRampTime > RAMP_INTERVAL) {
    lastRampTime = millis();
    
    // Determine acceleration vs. braking step values
    int stepM1 = ACCEL_STEP;
    if ((currentSpeedM1 > 0 && targetSpeedM1 < currentSpeedM1) || 
        (currentSpeedM1 < 0 && targetSpeedM1 > currentSpeedM1)) {
      stepM1 = BRAKE_STEP;
    }

    int stepM2 = ACCEL_STEP;
    if ((currentSpeedM2 > 0 && targetSpeedM2 < currentSpeedM2) || 
        (currentSpeedM2 < 0 && targetSpeedM2 > currentSpeedM2)) {
      stepM2 = BRAKE_STEP;
    }

    // Apply speed adjustments for Motor 1
    if (currentSpeedM1 < targetSpeedM1) {
      currentSpeedM1 += stepM1;
      if (currentSpeedM1 > targetSpeedM1) currentSpeedM1 = targetSpeedM1;
    }
    else if (currentSpeedM1 > targetSpeedM1) {
      currentSpeedM1 -= stepM1;
      if (currentSpeedM1 < targetSpeedM1) currentSpeedM1 = targetSpeedM1;
    }
    
    // Apply speed adjustments for Motor 2
    if (currentSpeedM2 < targetSpeedM2) {
      currentSpeedM2 += stepM2;
      if (currentSpeedM2 > targetSpeedM2) currentSpeedM2 = targetSpeedM2;
    }
    else if (currentSpeedM2 > targetSpeedM2) {
      currentSpeedM2 -= stepM2;
      if (currentSpeedM2 < targetSpeedM2) currentSpeedM2 = targetSpeedM2;
    }

    // Transmit updated speeds to Sabertooth controller
    if (currentSpeedM1 != lastSentM1 || currentSpeedM2 != lastSentM2) {
      ST.motor(1, currentSpeedM1);
      ST.motor(2, currentSpeedM2);
      
      lastSentM1 = currentSpeedM1;
      lastSentM2 = currentSpeedM2;
    }
  }

  // Watchdog timeout implementation (Currently disabled)
  /*
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    targetSpeedM1 = 0;
    targetSpeedM2 = 0;
    digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, LOW);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
    lastCommandTime = millis();
  }
  */

  pollLidar();
}
