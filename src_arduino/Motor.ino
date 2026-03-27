#include <SabertoothSimplified.h>
// 1. Create a new hardware serial port
HardwareSerial SaberSerial(2); 

// 2. Tell the Sabertooth library to use this new port
SabertoothSimplified ST(SaberSerial);

const int Fan1     = D2;
const int Fan2     = D3;
const int RPWM_PIN = D4;
const int LPWM_PIN = D5;
const int LIDAR_RX = D6;
const int LIDAR_TX = D7;

unsigned long lastCommandTime        = 0;
unsigned long lastLidarTime          = 0;
const unsigned long WATCHDOG_TIMEOUT = 500;

const int DRIVE_SPEED = 127;
const int TURN_SPEED  = 63;

const byte request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
float currentDist = 0;

// Only change from your working code: fixed char buffer instead of String.
// String calls malloc/free on every read and fragments the ESP32 heap over
// time until it crashes. This char buffer has zero heap involvement.
static char cmdBuf[32];
static int  cmdLen = 0;

void setupLidar() {
  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void readLidar() {
  while (Serial1.available()) Serial1.read();
  Serial1.write(request, sizeof(request));
  delay(50);
  if (Serial1.available() >= 7) {
    byte response[7];
    for (int i = 0; i < 7; i++) response[i] = Serial1.read();
    if (response[0] == 0x01 && response[1] == 0x03) {
      int rawCm   = (response[3] << 8) | response[4];
      currentDist = rawCm / 30.48;
      Serial.print("LIDAR Distance: ");
      Serial.println(currentDist, 2);
    }
  }
}

void setup() {
  SaberSerial.begin(9600, SERIAL_8N1, -1, D8);
  Serial.begin(115200);
  setupLidar();

  pinMode(Fan1,     OUTPUT);  pinMode(Fan2,     OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);  pinMode(LPWM_PIN, OUTPUT);

  digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);

  delay(2000);
  strcpy(cmdBuf, "STOP");
  handleCommand();   
  ST.motor(1, 0);
  ST.motor(2, 0);
}

void handleCommand() {
  lastCommandTime = millis();

  if (strcmp(cmdBuf, "FORWARD") == 0) {
    ST.motor(1,  DRIVE_SPEED);
    ST.motor(2,  DRIVE_SPEED);
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "BACKWARD") == 0) {
    ST.motor(1, -DRIVE_SPEED);
    ST.motor(2, -DRIVE_SPEED);
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "LEFT") == 0) {
    ST.motor(1, -TURN_SPEED);
    ST.motor(2,  TURN_SPEED);
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "RIGHT") == 0) {
    ST.motor(1,  TURN_SPEED);
    ST.motor(2, -TURN_SPEED);
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
  else if (strcmp(cmdBuf, "FAN") == 0) {
    digitalWrite(Fan1, LOW);  digitalWrite(Fan2, LOW);
  }
  else if (strcmp(cmdBuf, "STOP") == 0) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, LOW);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
  }
}

void loop() {
  // Non-blocking char-by-char parser — replaces Serial.readStringUntil()
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
      cmdLen = 0; // oversized packet — discard
    }
  }

  // Watchdog — direct ST.motor(0), no ramp delay
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    ST.motor(1, 0);
    ST.motor(2, 0);
    digitalWrite(RPWM_PIN, LOW);  digitalWrite(LPWM_PIN, LOW);
    digitalWrite(Fan1, HIGH);     digitalWrite(Fan2, HIGH);
    lastCommandTime = millis();
  }

  // LiDAR poll
  if (millis() - lastLidarTime > 100) {
    lastLidarTime = millis();
    readLidar();
  }
}
