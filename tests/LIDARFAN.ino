#include <Arduino.h>

// --- PINS ---
const int LIDAR_RX = D3;
const int LIDAR_TX = D4;
const int FAN1_RELAY = D2; // Fan 1 Relay Input
const int FAN2_RELAY = D7; // Fan 2 Relay Input (New)
const int STATUS_LED = LED_BUILTIN;

// --- MODBUS COMMANDS ---
// Command: Read Distance (Address 0x01, Register 0x0000, Length 1)
// CRC is pre-calculated as 0x840A
const byte request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};

// Variables
unsigned long lastQuery = 0;
int currentDist = 0;

void setup() {
  // Setup Hardware
  pinMode(FAN1_RELAY, OUTPUT);
  pinMode(FAN2_RELAY, OUTPUT); // Configure second fan
  pinMode(STATUS_LED, OUTPUT);
  
  // Default State: OFF (Active Low Relay, so HIGH = OFF)
  digitalWrite(FAN1_RELAY, HIGH); 
  digitalWrite(FAN2_RELAY, HIGH); 
  digitalWrite(STATUS_LED, LOW);

  // Start LiDAR Serial (115200 matches datasheet)
  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  
  // Blink twice to show "System Ready"
  delay(500);
  digitalWrite(STATUS_LED, HIGH); delay(200); digitalWrite(STATUS_LED, LOW); delay(200);
  digitalWrite(STATUS_LED, HIGH); delay(200); digitalWrite(STATUS_LED, LOW);
}

void loop() {
  // --- 1. SEND QUERY EVERY 100ms ---
  if (millis() - lastQuery > 100) {
    lastQuery = millis();
    
    // Clear any old junk from buffer
    while(Serial1.available()) Serial1.read();
    
    // Send the "Read Distance" command
    Serial1.write(request, sizeof(request));
  }

  // --- 2. READ RESPONSE ---
  // Expected Response (7 bytes): [ID] [Func] [Bytes] [DistH] [DistL] [CRCL] [CRCH]
  
  if (Serial1.available() >= 7) {
    // Check Headers (ID=01, Func=03, Bytes=02)
    if (Serial1.read() == 0x01) {
      if (Serial1.read() == 0x03) {
        if (Serial1.read() == 0x02) {
          
          // Read Distance Bytes
          int distHigh = Serial1.read();
          int distLow = Serial1.read();
          
          // Eat the CRC bytes
          Serial1.read(); 
          Serial1.read();
          
          // Calculate Logic
          currentDist = (distHigh * 256) + distLow;
          
          // --- 3. CONTROL FANS ---
          // Virtual Wall at 183cm (6ft)
          // Ignore 0 (Error) and >1200 (Out of range)
          if (currentDist > 0 && currentDist < 183) {
            // --- OBJECT DETECTED: FANS ON ---
            digitalWrite(FAN1_RELAY, LOW);  // Fan 1 ON
            digitalWrite(FAN2_RELAY, LOW);  // Fan 2 ON
            digitalWrite(STATUS_LED, HIGH); // LED ON
          } 
          else {
            // --- CLEAR PATH: FANS OFF ---
            digitalWrite(FAN1_RELAY, HIGH); // Fan 1 OFF
            digitalWrite(FAN2_RELAY, HIGH); // Fan 2 OFF
            digitalWrite(STATUS_LED, LOW);  // LED OFF
          }
        }
      }
    }
  }
}
