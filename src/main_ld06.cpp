#include <Arduino.h>
#include "ld06.h"

// Use Serial1 for the LD06 (with RX on pin 16)
LD06 ld06(Serial1);

// For demonstration, we use SIGNAL_PIN to output a decision (e.g., LED or digital output)
const int SIGNAL_PIN = 13; // Change to your appropriate output pin
const uint16_t OBSTACLE_THRESHOLD = 300; // threshold distance in mm

// Toggle builtin LED to show that the board is alive.
void toggleBuiltinLed() {
#ifdef LED_BUILTIN
  static bool ledState = false;
  static uint32_t ref = millis();
  if (millis() - ref > 100) {
    pinMode(LED_BUILTIN, OUTPUT);
    ref = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
#endif
}

// Function to decide motion direction based on front and back obstacle detection
int decideMotion() {
  float forwardSum = 0;
  int forwardCount = 0;
  float backwardSum = 0;
  int backwardCount = 0;
  
  uint16_t n = ld06.getNbPointsInScan();
  for (uint16_t i = 0; i < n; i++) {
    DataPoint* p = ld06.getPoints(i);
    if (!p)
      continue;
    float angle = p->angle;
    if (angle < 0) angle += 360;
    
    // Forward region: 0°-30° and 330°-360° (adjust as needed)
    if ((angle >= 60 && angle <= 120)) {
      forwardSum += p->distance;
      forwardCount++;
    }
    // Backward region: 150°-210° (adjust as needed)
    else if (angle >= 240 && angle <= 300) {
      backwardSum += p->distance;
      backwardCount++;
    }
  }
  
  float forwardAvg = (forwardCount > 0) ? (forwardSum / forwardCount) : 10000;
  float backwardAvg = (backwardCount > 0) ? (backwardSum / backwardCount) : 10000;
  
  Serial.print("Forward avg: ");
  Serial.print(forwardAvg);
  Serial.print("   Backward avg: ");
  Serial.println(backwardAvg);
  
  if (forwardAvg < OBSTACLE_THRESHOLD && backwardAvg >= OBSTACLE_THRESHOLD) {
    return -1;  // Obstacle in front → go backward
  }
  else if (backwardAvg < OBSTACLE_THRESHOLD && forwardAvg >= OBSTACLE_THRESHOLD) {
    return 1;   // Obstacle in back → go forward
  }
  else if (forwardAvg < OBSTACLE_THRESHOLD && backwardAvg < OBSTACLE_THRESHOLD) {
    // If both are obstructed, choose the one with more clearance (i.e., higher average)
    if (forwardAvg < backwardAvg)
      return -1;
    else
      return 1;
  }
  else {
    return 0; // No obstacles detected
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  // Start Serial1 for LD06 (RX on pin 16)
  Serial1.begin(230400, SERIAL_8N1, 16);
  while (!Serial1) { ; }
  
  // Initialize Serial2 for inter-ESP32 communication.
  // Here, we do not use an RX pin (set to -1) and use TX on GPIO17.
  Serial2.begin(115200, SERIAL_8N1, -1, 17);
  
  ld06.init();
  
  // Points filtering configuration
  ld06.enableFiltering();
  ld06.setIntensityThreshold(200);
  ld06.setDistanceRange(100, 1000);
  ld06.setAngleRange(0, 360);
  
  // Uncomment if you want full-scan mode:
  // ld06.enableFullScan();
  
  // Set up the signal output (optional, e.g., for an LED)
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW);
}

void loop() {
  toggleBuiltinLed();
  
  if (ld06.readScan()) {
    // ld06.printScanTeleplot(Serial); // for debugging
    
    int decision = decideMotion();
    if (decision == -1) {
      Serial.println("BACKWARD");
      digitalWrite(SIGNAL_PIN, HIGH);
      // Transmit decision to the other ESP32 via Serial2
      Serial2.println("BACKWARD_L");
    }
    else if (decision == 1) {
      Serial.println("FORWARD");
      digitalWrite(SIGNAL_PIN, LOW);
      // Transmit decision to the other ESP32 via Serial2
      Serial2.println("FORWARD_L");
    }
    else if (decision == 0) {
      Serial.println("STILL");
      digitalWrite(SIGNAL_PIN, LOW);
      // Transmit decision to the other ESP32 via Serial2
      Serial2.println("STILL");
    }
  }
}
