#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

//#include "BluetoothSerial.h"

/*
INCLUDE, SerialBT, SETUP - ENABLE
*/

// private includes
#include "pid.h"
#include "gpid.h"

MPU6050 mpu;
//BluetoothSerial SerialBT;

// pin deines
#define ENA 14  // PWM pin for speed control
#define IN1 27  // Control pin for direction
#define IN2 26  // Control pin for direction


#define ENB 25  // PWM pin for speed control
#define INB1 33  // Control pin for direction
#define INB2 32  // Control pin for direction

#define ENCODER_PIN_A 35  //Yellow Encoder signal A connected to GPIO34
#define ENCODER_PIN_B 34  // Encoder signal B connected to GPIO35

#define ENCODER_B_PIN_A 19  //Yellow Encoder signal A connected to GPIO36
#define ENCODER_B_PIN_B 18  // Encoder signal B connected to GPIO39


#define TRIG_PIN 16 // Trigger pin connected to D2
#define ECHO_PIN 17 // Echo pin connected to D4

#define wheelRadius 0.5 // 0.45 gives me the distance in ft

//Voltage Sensor
#define VOLTAGE_SENSOR_PIN 2  // GPIO35 for voltage sensor signal
#define VOLTAGE_DIVIDER_RATIO 9.09  // Module's voltage divider ratio // 10.52 -> 9.14, 3.3 -> 9.16, 13.22 -> 8.93
#define ADC_RESOLUTION 4095.0  // ESP32 ADC resolution (12-bit)
#define REF_VOLTAGE 3.3  // Reference voltage for ESP32 (3.3V)


// Position hold parameters
float homeX = 0.0;                // Will store the initial forward/backward position
bool positionHoldEnabled = true;  // Enable position hold when no motion command is active
const float POSITION_TOLERANCE = 0.5;  // Allowable error in meters (5 cm tolerance)






float stationaryAngle = 0.0;
float forwardAngle = -1.25;
float backwardAngle = 1.0;

bool emergencyBrakeActive = false;
unsigned long emergencyBrakeStartTime = 0;






float targetAngle = stationaryAngle;
int dir = 0;
float turnAdjustment = 0.0;

// Flag and target for a commanded turn
bool turning = false;
float targetTurnAngle = 0.0;
float turnStartYaw = 0.0;
int turnAngle = 0;

float start_x = 0.0;
float start_y = 0.0;
static float pos_x = 0.0, pos_y = 0.0;
bool reverseExecuted = false;

// Obstacle marker globals.
bool obstacleDetected = false;
float obst_x = 0.0;
float obst_y = 0.0;

bool obstacleAvoidanceActive = false;
unsigned long obstacleAvoidanceStartTime = 0;
const unsigned long OBSTACLE_AVOIDANCE_DURATION = 2000; // 2 seconds




// BLE STUFF ======================================================================================================

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0" // Custom BLE Service UUID
#define CHARACTERISTIC_UUID "87654321-4321-6789-4321-abcdef012345" // Custom BLE Characteristic UUID

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// Callback class to handle connection events
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device Connected!");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device Disconnected! Restarting advertisement...");
        BLEDevice::startAdvertising(); // Restart advertising so other devices can find it
    }
};

float normalizeAngle(float angle);
float ypr[3]; 

PID motorAPID(0.5, 10.75, 0.01);
PID motorBPID(0.5, 10.75, 0.01);
float kp = 2.82;
float ki = 10.09;
float kd = 0.24;

float forward_kp = 1.5;
float forward_ki = 9.5;
float forward_kd = 0.21;

GPID gyro(kp, ki, kd); 

// Create a PID instance for position control (tune these gains for your robot)
float posKp = 0.05;
float posKi = 0.0;
float posKd = 0.01;
PID posPID(posKp, posKi, posKd);

float targetposx = 1.0;
float targetposy = 0.0;
static bool masterLock = false;

float tune_turn = 0.0;


class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            Serial.print("Received Data (Decimal): ");
            for (char c : rxValue) {
                Serial.print((uint8_t)c);  // Directly print decimal values
                Serial.print(" ");
            }
            Serial.println();

            if (rxValue.find("T,") == 0) {
                float newTargetX, newTargetY;
                sscanf(rxValue.c_str(), "T,%f,%f", &newTargetX, &newTargetY);
                targetposx = newTargetX;
                targetposy = newTargetY;
                start_x = pos_x;
                start_y = pos_y;
                // Optionally, reset masterLock for new navigation.
                masterLock = false;
                Serial.printf("New target received: %.2f, %.2f\n", targetposx, targetposy);
                return;
              }

            // Ensure there's meaningful data before processing
            if (rxValue.length() > 1) {
                uint8_t L_command = (uint8_t)rxValue[6];  // Ignore first byte if it's a header
                uint8_t R_command = (uint8_t)rxValue[5];  // Ignore first byte if it's a header


                // Serial.print("Interpreted Command: ");
                // Serial.println(command);

                // Example: Control LED based on received command
                if (L_command == 1) {
                    Serial.println("UP");
                    dir = 1;
                    targetAngle = forwardAngle;
                    turnAdjustment = 0;
                    gyro.setTunings(forward_kp, forward_ki, forward_kd);


                } else if (L_command == 4) {
                    Serial.println("LEFT");
                    dir = 0;
                    // targetAngle = -(abs(stationaryAngle) + (abs(forwardAngle - stationaryAngle) * 0.3));
                    turnAdjustment = -tune_turn;

                    // posKi -= 0.5;
                    // posPID.setKi(posKi);

                } else if (L_command == 8) {
                    Serial.println("RIGHT");
                    dir = 0;
                    // targetAngle = -(abs(stationaryAngle) - (abs(stationaryAngle - backwardAngle) * 0.15));
                    turnAdjustment = tune_turn;

                    // posKi += 0.5;
                    // posPID.setKi(posKi);

                } else if (L_command == 2) {
                    Serial.println("DOWN");
                    dir = -1;
                    targetAngle = backwardAngle;
                    turnAdjustment = 0;
                    gyro.setTunings(forward_kp, forward_ki, forward_kd);

                //#####################################

                } else if (R_command == 4) {
                    Serial.println("R_UP");
                    // if (!turning) {
                    //     turning = true;
                    //     turnStartYaw = ypr[0]; // Record the current yaw
                    //     // Compute the target yaw (90° turn to the right) and normalize it
                    //     turnAngle = 45;
                    // }
                    posKp += 0.01;
                    posPID.setKp(posKp);

                    
                } else if (R_command == 32) {
                    Serial.println("R_LEFT");
                    // if (!turning) {
                    //     turning = true;
                    //     turnStartYaw = ypr[0]; // Record the current yaw
                    //     // Compute the target yaw (90° turn to the right) and normalize it
                       
                    //     turnAngle = -90;
                    // }

                    // posKd -= 0.01;
                    // posPID.setKd(posKd);

                    tune_turn += 0.01;



                } else if (R_command == 16) {
                    Serial.println("R_DOWN");
                    // if (!turning) {
                    //     turning = true;
                    //     turnStartYaw = ypr[0]; // Record the current yaw
                    //     // Compute the target yaw (90° turn to the right) and normalize it
                    //     turnAngle = -45;
                    // }
                    posKp -= 0.01;
                    posPID.setKp(posKp);


                } else if (R_command == 8) {
                    Serial.println("R_RIGHT");
                    // // Only trigger if not already turning
                    // if (!turning) {
                    //     turning = true;
                    //     turnStartYaw = ypr[0]; // Record the current yaw
                    //     // Compute the target yaw (90° turn to the right) and normalize it
                    //     turnAngle = 90;
                    //     targetAngle = forwardAngle;
                    // }

                    // posKd += 0.01;
                    // posPID.setKd(posKd);

                    tune_turn += 0.01;

                //#####################################
                

                } else if (L_command == 0 || R_command == 0) {
                    Serial.println("Released");
                    dir = 0;
                    targetAngle = stationaryAngle;
                    turnAdjustment = 0;
                    gyro.setTunings(kp, ki, kd);
                } else {
                    Serial.println("Unmapped Command");
                }
            }
        }
    }
};

// ================================================================================================================


float set_speed = 10;
bool isObstacleDetected = false;

volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoReceived = false;
unsigned long lastUltrasonicTime = 0;


// PID motorAPID(0.5, 10.75, 0.01);
// PID motorBPID(0.5, 10.75, 0.01);
/*
8/12/24
M 0.5 10.75 0.01
G 4.5 0 0.175

For MOVING Forwards and Backward
G 2.5 4 0.15

12V -> WORKS!!
1) G 1.5 9.5 0.165

9.6V -> WORKS!
1) G 1.5 9.6 0.165

*/


// 11.4 - 8.8 to 9 V
// 10.42 from 0.6v battery
//G 3.75 0 0.195 ===========================================

//G 2.1 7.5 0.2


// 16-17 at -5 deg


// M 0.5 2.55 0 
// M 0.85 0.75 0 

//Carpet
//M 0.85 0.75 0 
//G 5.5 0 0.25

//M 0.75 2.75 0 
//G 1.5 0 0.25


// Target angle for balance
// GPID gyro(2.82, 10.09, 0.24); // works moderately w/ no motor PID

// GPID turn_pid(3.5, 5, 0.275);

// 01/03/2025
//G 2.82 10.09 0.24




// GPID gyro(3.75, 5, 0.15); // works moderately w/ no motor PID
// GPID gyro(3.75, 9.5, 0.35); // works moderately w/ no motor PID
// GPID gyro(3.5, 5, 0.275);


//G 5.5 0 0.25


/*
While tuning kp130 ki 800 kd 4, set point 178, offset angle 0.1 and
Speed factor left 0.48 and right 0.35
*/

// variables
volatile int encoderCount = 0;  // Variable to store encoder pulse count
int prevEncoderCount = 0;       // Variable to store previous encoder count


volatile int encoderCount_B = 0;  // Variable to store encoder pulse count
int prevEncoderCount_B = 0;       // Variable to store previous encoder count

unsigned long prevTime = 0;     // To store the previous time for velocity calculation

int pulsesPerRevolution = 11;  // https://www.aliexpress.com/item/1005005954131759.html#nav-specification

float outputA;
float outputB;

////////
#define INTERRUPT_PIN 4  // Change to your actual interrupt pin for ESP32
bool dmpReady = false;  // Set true if DMP init was successful
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
// float ypr[3];           // Array to store Yaw, Pitch, Roll
//////


// Timer and distance variables
hw_timer_t *timer = NULL;
volatile bool distanceReady = false;
volatile long duration = 0;
float distance;

// Normalize an angle to the range -PI to PI
float normalizeAngle(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}


void updatePIDViaSerial() {
 // Check if there is any data available on Serial
 if (Serial.available() > 0) {
     // Read the incoming data as a string
     String input = Serial.readStringUntil('\n');
  
     // Parse command
     char commandType;
     float kp, ki, kd;
     int result = sscanf(input.c_str(), "%c %f %f %f", &commandType, &kp, &ki, &kd);
  
     if (result == 4) {
         // Update Motor PID values
         if (commandType == 'M') {  // 'M' for Motor PID
             motorAPID.setKp(kp);
             motorAPID.setKi(ki);
             motorAPID.setKd(kd);
             motorBPID.setKp(kp);
             motorBPID.setKi(ki);
             motorBPID.setKd(kd);
          
             Serial.println("Motor PID values updated:");
             Serial.print("Kp: "); Serial.print(kp);
             Serial.print(" Ki: "); Serial.print(ki);
             Serial.print(" Kd: "); Serial.println(kd);
         }
         // Update Gyroscope PID values
         else if (commandType == 'G') {  // 'G' for Gyroscope PID
             gyro.setKp(kp);
             gyro.setKi(ki);
             gyro.setKd(kd);
          
             Serial.println("Gyro PID values updated:");
             Serial.print("Kp: "); Serial.print(kp);
             Serial.print(" Ki: "); Serial.print(ki);
             Serial.print(" Kd: "); Serial.println(kd);
         }
     } else {
         Serial.println("Invalid command format. Use 'M Kp Ki Kd' for Motor PID or 'G Kp Ki Kd' for Gyro PID.");
     }
 }
}


void updatePID(float newKp, float newKi, float newKd, float set_speed_new) {
motorAPID.setKp(newKp);
motorAPID.setKi(newKi);
motorAPID.setKd(newKd);
motorBPID.setKp(newKp);
motorBPID.setKi(newKi);
motorBPID.setKd(newKd);
set_speed = set_speed_new;
}


void parseCommand(String command) {
char commandType;
float kp, ki, kd, set_speed_new;
sscanf(command.c_str(), "%c %f %f %f %f", &commandType, &kp, &ki, &kd, &set_speed_new);
if (commandType == 'P') { // Example: Send "P 1.0 0.1 0.05" to set PID values
   updatePID(kp, ki, kd, set_speed_new);
}
}
// functions
void forward(int set_speed_A, int set_speed_B) {
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
analogWrite(ENA, ((set_speed_A) / 100.0 ) * 255);

digitalWrite(INB1, LOW);
digitalWrite(INB2, HIGH);
analogWrite(ENB, ((set_speed_B) / 100.0 ) * 255);
}

void backward(int set_speed_A, int set_speed_B) {
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
analogWrite(ENA, ((set_speed_A) / 100.0 ) * 255);

digitalWrite(INB1, HIGH);
digitalWrite(INB2, LOW);
analogWrite(ENB, ((set_speed_B) / 100.0 ) * 255);
}

void stop(){
analogWrite(ENA, 0);
analogWrite(ENB, 0);
set_speed = gyro.compute(0, 0);
outputA = motorAPID.compute(0, 0); // hard coded to stop integral increasing indefinitely
outputB = motorBPID.compute(0, 0);
}


// Interrupt detection routine
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
mpuInterrupt = true;
}
float angle;
////////

// Interrupt Service Routine (ISR) for Encoder
void IRAM_ATTR updateEncoder() {
// Read both encoder pins
int stateA = digitalRead(ENCODER_PIN_A);
int stateB = digitalRead(ENCODER_PIN_B);


// Check the direction based on the phase of A and B signals
if (stateA == stateB) {
encoderCount++;  // Clockwise (CW)
} else {
encoderCount--;  // Counterclockwise (CCW)
}
}


void IRAM_ATTR updateEncoder_B() {
// Read both encoder pins
int stateA = digitalRead(ENCODER_B_PIN_A);
int stateB = digitalRead(ENCODER_B_PIN_B);


// Check the direction based on the phase of A and B signals
if (stateA == stateB) {
encoderCount_B++;  // Clockwise (CW)
} else {
encoderCount_B--;  // Counterclockwise (CCW)
}
}


//=====================================================================
// void IRAM_ATTR onTimer() {
//    distanceReady = true;  // Set flag to start ultrasonic measurement
// }

// void IRAM_ATTR echoISR() {
//    if (digitalRead(ECHO_PIN) == HIGH) {
//        echoStartTime = micros();
//    } else {
//        echoEndTime = micros();
//        echoReceived = true;
//    }
// }
// void triggerUltrasonic() {
//    digitalWrite(TRIG_PIN, LOW);
//    delayMicroseconds(2);
//    digitalWrite(TRIG_PIN, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(TRIG_PIN, LOW);
// }

// float get_distance(){
//   if (distanceReady) {
//       distanceReady = false;
//       distance = duration * 0.034 / 2;  // Calculate distance in cm
//       // Serial.print("Distance: ");
//       //Serial.print(distance);
//       //Serial.println("");
//   }
//   return distance;
// }
//=====================================================================


// Linear interpolation function
float lerp(float start, float end, float ratio) {
    return start + (end - start) * ratio;
}



void setup() {

//////
Serial.begin(921600);
Serial2.begin(115200, SERIAL_8N1, 16, -1);
//SerialBT.begin("ESP32_RPM");
Wire.begin();
// Initialize the MPU6050 and configure DMP



// Initialize BLE =============================================================================
BLEDevice::init("ESP32_BLE");

// Create BLE Server
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

// Create BLE Service
BLEService *pService = pServer->createService(SERVICE_UUID);

pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
);
pCharacteristic->addDescriptor(new BLE2902());
pCharacteristic->setCallbacks(new MyCallbacks());
pCharacteristic->setValue("Hello from ESP32!");


// Start the service
pService->start();

// Start advertising
BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
pAdvertising->addServiceUUID(SERVICE_UUID);
pServer->getAdvertising()->start();

Serial.println("BLE Server is ready and advertising!");

// =============================================================================

mpu.initialize();
pinMode(INTERRUPT_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

if (mpu.testConnection()) {
   Serial.println("MPU6050 connection successful.");
} else {
   Serial.println("MPU6050 connection failed.");
   while (1); // Halt execution if MPU fails
}

// Initialize the DMP
uint8_t devStatus = mpu.dmpInitialize();

// If successful, enable DMP
if (devStatus == 0) {
   mpu.setDMPEnabled(true);
   mpu.CalibrateGyro(6);
   mpuIntStatus = mpu.getIntStatus();
   dmpReady = true;
   packetSize = mpu.dmpGetFIFOPacketSize();
} else {
   Serial.print("DMP Initialization failed (code ");
   Serial.print(devStatus);
   Serial.println(")");
   while (1); // Halt execution if DMP fails
}


//    pinMode(ECHO_PIN, INPUT);
//    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

// Set motor control pins as outputs
pinMode(ENA, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);


pinMode(ENB, OUTPUT);
pinMode(INB1, OUTPUT);
pinMode(INB2, OUTPUT);


// Set encoder pins as inputs
pinMode(ENCODER_PIN_A, INPUT);
pinMode(ENCODER_PIN_B, INPUT);

pinMode(ENCODER_B_PIN_A, INPUT);
pinMode(ENCODER_B_PIN_B, INPUT);


pinMode(VOLTAGE_SENSOR_PIN, INPUT);


// Attach interrupt for encoder signal A
attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), updateEncoder_B, RISING);

/////////
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
//   // Initialize timer to trigger every 100ms
//   timer = timerBegin(0, 80, true);  // 80 prescaler for 1us ticks
//   timerAttachInterrupt(timer, &onTimer, true);
//   timerAlarmWrite(timer, 100000, true);  // 100000us = 100ms
//   timerAlarmEnable(timer);
///////

// Initialize time
prevTime = millis();

stop();

}


void loop() {
// Ensure DMP is ready
if (!dmpReady) return;

// Wait for MPU interrupt to signal data is ready
if (mpuInterrupt) {
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();
   // Check for available DMP packet
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
       // Extract Yaw, Pitch, Roll from DMP packet
       Quaternion q;
       VectorFloat gravity;
       mpu.dmpGetQuaternion(&q, fifoBuffer);
       mpu.dmpGetGravity(&gravity, &q);
       mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

       // Display YPR values in degrees
       // Serial.print("Yaw: ");
       // Serial.print(ypr[0] * 180/M_PI);
       // Serial.print("\tPitch: ");
       // Serial.print(ypr[1] * 180/M_PI);
       // Serial.print("\tRoll: ");
       // Serial.println(ypr[2] * 180/M_PI);
       angle = (ypr[1] * 180/M_PI); // -ve sign :: temp fix to invert direction of wheel
    }
}



// Calculate velocity every 100ms
unsigned long currentTime = millis();
unsigned long timeDifference = currentTime - prevTime;


if (Serial2.available() > 0) {
  String receivedData = Serial2.readStringUntil('\n');
  receivedData.trim(); // Remove any extra whitespace or newline characters
  if (receivedData == "BACKWARD_L") { // lidar esp32 tells there is obstacle behind
    // targetAngle = forwardAngle;
    Serial.println(" cmd BACKWARD");
    obstacleDetected = false;

  } else if (receivedData == "FORWARD_L") { // lidar esp32 tells there is obstacle in front
    obstacleDetected = true;

    dir = 0;

    // obstacleAvoidanceActive = true;            // Begin obstacle avoidance mode.
    // dir = 1;                                     // Command forward drive.
    // turning = true;                              // Initiate turning.
    // turnStartYaw = ypr[0];                         // Record the current yaw.
    // turnAngle = -45;                             // Set turn angle to -90° (i.e. turn left).
    // targetAngle = stationaryAngle;               // While turning, keep tilt neutral.
    // gyro.setTunings(kp, ki, kd);                 // Use standard tunings.
    float obsDist = 3;                           // Example fixed distance for obstacle marker.
    obst_x = pos_x + obsDist * cos(ypr[0]);       // Calculate obstacle marker X.
    obst_y = pos_y + obsDist * sin(ypr[0]);       // Calculate obstacle marker Y.
    
    Serial.println("cmd FORWARD_L: Obstacle detected, initiating turn (targetAngle = stationary while turning)");

} else if (receivedData == "STILL") {
    dir = 0;
    targetAngle = stationaryAngle;
    turnAdjustment = 0;
    obstacleDetected = false;

  } else if (receivedData == "FORWARD"){
    dir = 1;
    targetAngle = forwardAngle;
    turnAdjustment = 0;

  } else if (receivedData == "BACKWARD"){
    dir = -1;
    targetAngle = backwardAngle;
    turnAdjustment = 0;

  } else if (receivedData == "LEFT"){
    dir = 0;
    targetAngle = -(abs(stationaryAngle) + (abs(forwardAngle - stationaryAngle) * 0.3));
    turnAdjustment = -2.0;

  } else if (receivedData == "RIGHT"){
    dir = 0;
    targetAngle = -(abs(stationaryAngle) - (abs(stationaryAngle - backwardAngle) * 0.15));
    turnAdjustment = 2.0;

   } else {
    Serial.println("NOT VALID");
  }
}


if (timeDifference >= 50) {  // 100ms interval
    lastUltrasonicTime = currentTime;
 //    triggerUltrasonic();
// Calculate the number of pulses in the last 100ms
int pulseDifference = encoderCount - prevEncoderCount;
int pulseDifference_B = encoderCount_B - prevEncoderCount_B;

// Calculate velocity (in RPM)
float pulsesPerSecond = pulseDifference * (1000.0 / (timeDifference));
float rpm = ((pulsesPerSecond / pulsesPerRevolution) * 60.0) / 10; // 10 is magic number to convet to RPM (530 RPM Motor)

float pulsesPerSecond_B = pulseDifference_B * (1000.0 / (timeDifference));
float rpm_B = ((pulsesPerSecond_B / pulsesPerRevolution) * 60.0) / 10; // 10 is magic number to convet to RPM

float vel_A = (rpm * 2 * PI * wheelRadius) / 60.0;  // in meters per second
float vel_B = (rpm_B * 2 * PI * wheelRadius) / 60.0;
float avg_vel = (vel_A + vel_B) / 2.0;

// Estimate distance traveled (average speed * time)
float distance_traveled = ((vel_A + vel_B) / 2.0) * (timeDifference / 1000.0) * (0.45/0.5) ; // wheel radius 0.45

// Update X, Y position using Dead Reckoning
float theta_rad = ypr[0]; // Yaw in radians
float delta_x = distance_traveled * cos(theta_rad);
float delta_y = distance_traveled * sin(theta_rad);


pos_x += delta_x;
pos_y += delta_y;


// Define target coordinates (global or local as needed)


if (deviceConnected) {
    // Transmit current position, target position, average velocity, targetAngle, and obstacle marker.
    // When no obstacle is detected, obst_x and obst_y will be 0.
    char posData[90];
    sprintf(posData, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, %.2f", 
            pos_x, pos_y, targetposx, targetposy, avg_vel, targetAngle, 
            obstacleDetected ? obst_x : 0.0, obstacleDetected ? obst_y : 0.0, ypr[0] * 180 / M_PI);
    pCharacteristic->setValue(posData);
    pCharacteristic->notify();
    Serial.println(posData);
}



// Tolerances (tune as needed)
const float POSITION_TOLERANCE_DIST = 0.5; // meters
const float HEADING_TOLERANCE = 2.0;       // degrees
const float MASTER_LOCK_DIST = 0.5;
const float OUTER_CIRCLE_DIST = 5.0; // Adjust this distance (in meters) as needed.
bool lock_angle = false;





// ##############################################################################################################

// // Only apply position hold when no manual drive command is active
// if (dir == 0 && positionHoldEnabled) {
//     // Assume pos_x represents the forward/backward displacement (in meters)
//     float posError = targetposx - pos_x;
//     if (fabs(posError) > POSITION_TOLERANCE) {
//         // Compute a correction from the position PID (positive correction means drive forward)
//         // float posKp = 0.15;
//         // float posKi = 0.0;
//         // float posKd = 0.03;
//         float posCorrection = posPID.compute(targetposx, pos_x);
//         // Adjust targetAngle: adding a positive correction tilts the robot forward,
//         // causing it to drive forward toward home, and vice versa.
//         targetAngle = stationaryAngle - posCorrection;
        

        
        
//         gyro.setTunings(forward_kp, forward_ki, forward_kd);
//         Serial.printf("Pos error: %.2f, Correction: %.2f, New targetAngle: %.2f\n", posError, posCorrection, targetAngle);
//     } else {
//         // Within tolerance; use the normal stationary angle
//         targetAngle = stationaryAngle;
//         gyro.setTunings(forward_kp, forward_ki, forward_kd);
//         // Optionally, if your PID class has a reset, you could reset its integral here.
//         // posPID.reset();   
//     }
// }

// ##############################################################################################################


// // keep the robot going forward for 2 seconds to avoid obstacle
// static unsigned long forwardMoveStartTime = 0;
//     if (obstacleAvoidanceActive) {
//         // While turning, keep a neutral tilt.
//         if (turning) {
//             targetAngle = stationaryAngle;
//             gyro.setTunings(kp, ki, kd);
//         } else {
//             // When turning is complete, if we haven't started moving forward yet, record the time.
//             if (forwardMoveStartTime == 0) {
//                 forwardMoveStartTime = millis();
//             }
//             // For 2 seconds, command forward movement.
//             if (millis() - forwardMoveStartTime < 1000) {
//                 targetAngle = -1.0;
//                 gyro.setTunings(kp, ki, kd);
//             } else {
//                 // After 2 seconds, exit obstacle avoidance.
//                 obstacleAvoidanceActive = false;
//                 forwardMoveStartTime = 0;
//             }
//         }
// }


if (dir == 0 && positionHoldEnabled) {
    // Calculate differences in x and y.


    float dx = targetposx - pos_x;
    float dy = targetposy - pos_y;
    // Distance error (hypotenuse)
    float distance_error = sqrt(dx * dx + dy * dy);
    // Compute desired heading in degrees (atan2 returns radians)
    float desired_heading = atan2(dy, dx) * 180.0 / PI;
    // Current heading from sensor: assuming ypr[0] is yaw in radians.
    float current_heading = ypr[0] * 180.0 / PI;
    
    // Compute heading error and normalize to -180°..180°
    float heading_error = desired_heading - current_heading;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;

    

    // Check if target is behind: if the absolute heading error is greater than 90°,
    // then consider the target as being behind.
    bool reverseDrive = (fabs(heading_error) > 90.0);
    
 

    // Once the robot is very close to the target, lock turning permanently.
    if (distance_error < MASTER_LOCK_DIST) {
        masterLock = true;
    } 

    if (distance_error > OUTER_CIRCLE_DIST){
        lock_angle = true;
    } else {
        lock_angle = false;
    }

    
    if (masterLock) {
        turning = false;
        turnAdjustment = 0;
        // Compute total distance from the recorded start to the target.
        float totalDist = sqrt(pow(targetposx - start_x, 2) + pow(targetposy - start_y, 2));
        // Project the current position onto the vector from start to target.

        float currentProj = (((pos_x - start_x) * (targetposx - start_x)) +
                             ((pos_y - start_y) * (targetposy - start_y))) / totalDist;

       
        // The signed error: positive if not reached, negative if overshot.
        float signedError = totalDist - currentProj;

       
        
        // Use the signed error in your posPID.
        posPID.setTunings(0.15, 0.0, 0.03);
        float posCorrection = posPID.compute(0, signedError);
        targetAngle = stationaryAngle + posCorrection;
        if (reverseExecuted) {
            targetAngle = -targetAngle;
        }
        
        gyro.setTunings(kp, ki, kd);
        
        Serial.printf("Master lock: totalDist=%.2f, currentProj=%.2f, signedError=%.2f, posCorrection=%.2f, targetAngle=%.2f\n",
                      totalDist, currentProj, signedError, posCorrection, targetAngle);
    }
     else {
    // Check if we are still far from the target.
    if (distance_error > POSITION_TOLERANCE_DIST) {


        if (reverseDrive) {
            // Fold the heading error into [-90, 90]:
            float effective_heading_error = heading_error;
            if (heading_error > 90.0) {
                effective_heading_error = heading_error - 180.0;
            } else if (heading_error < -90.0) {
                effective_heading_error = heading_error + 180.0;
            }
            // If the effective heading error is significant (> 5°), allow turning:
            if (fabs(effective_heading_error) > HEADING_TOLERANCE) {
                turning = true;
                // Use the effective heading error for turning.
                turnStartYaw = ypr[0];
                turnAngle = effective_heading_error;
               
                float hold_dx = pos_x - start_x;
                float hold_dy = pos_y - start_y;
                float hold_error = sqrt(hold_dx * hold_dx + hold_dy * hold_dy);
                posPID.setTunings(0.2, 0.0, 0.03);
                float holdCorrection = posPID.compute(0, -hold_error);
                // This hold correction keeps the robot near the previous target position.
                targetAngle = stationaryAngle + holdCorrection;
                gyro.setTunings(kp, ki, kd);

                Serial.printf("Reverse turning: desired=%.2f, current=%.2f, effective_error=%.2f\n",
                              desired_heading, current_heading, effective_heading_error);
            } else {
                // If the effective error is small, command reverse drive.
                turning = false;
                reverseExecuted = true;
                if (lock_angle){
                    targetAngle = backwardAngle;
                } else {
                    posPID.setTunings(0.25, 0.0, 0.03);
                    float posCorrection = posPID.compute(0, -distance_error);
                    // In reverse mode, targetAngle is between stationaryAngle and backwardAngle.
                    targetAngle = stationaryAngle + posCorrection;
                }
                gyro.setTunings(kp, ki, kd);
                // Serial.printf("Reverse driving: distErr=%.2f, posCorr=%.2f, targetAngle=%.2f\n",
                //               distance_error, posCorrection, targetAngle);
            }
        }
        else {

        // Use a hysteresis approach for heading:
        // If the absolute heading error exceeds the tolerance, we command a turn.
        // Once in turning mode, we only clear it if the error drops below half the tolerance.
        static bool wasTurning = false;
        reverseExecuted = false;
        if (fabs(heading_error) > HEADING_TOLERANCE) {
            turning = true;
            wasTurning = true;
            turnStartYaw = ypr[0]; // Record the current yaw
            if (reverseDrive){
                if (heading_error > 90.0) {
                    heading_error -= 180.0;
                } else if (heading_error < -90.0) {
                    heading_error += 180.0;
                }
            }
            turnAngle = heading_error;
            
            float hold_dx = pos_x - start_x;
            float hold_dy = pos_y - start_y;
            float hold_error = sqrt(hold_dx * hold_dx + hold_dy * hold_dy);
            posPID.setTunings(0.2, 0.0, 0.03);
            float holdCorrection = posPID.compute(0, hold_error);
            // This hold correction keeps the robot near the previous target position.
            targetAngle = stationaryAngle + holdCorrection;
            gyro.setTunings(kp, ki, kd);

            // Serial.printf("Turning: desired=%.2f, current=%.2f, error=%.2f\n",
            //               desired_heading, current_heading, heading_error);
        } else {
            // If the error is low but we were turning and the error is not very low, remain in turning mode.
            if (wasTurning && fabs(heading_error) > (HEADING_TOLERANCE / 2)) {
                turning = true;
                Serial.printf("Maintaining turn: error=%.2f (still above half tolerance)\n", heading_error);
            } else {
                turning = false;
                wasTurning = false;
                turnAdjustment = 0;

            }
            if (!turning) {

                

                // Heading is aligned well enough. Use the distance error for forward drive.
                

                if (lock_angle){
                    targetAngle = -1.25;
                } else {
                    posPID.setTunings(0.25, 0.0, 0.03);
                    float posCorrection = posPID.compute(0, distance_error);
                    targetAngle = stationaryAngle + posCorrection;
                }
                
                
                
        
                turnAdjustment = 0;
                gyro.setTunings(kp, ki, kd);
                
                // Cap targetAngle between a safe forward tilt and neutral.
                
                // Serial.printf("Driving: distance_error=%.2f, Correction=%.2f, targetAngle=%.2f\n",
                //               distance_error, posCorrection, targetAngle);
            }
        }
    } 
    }

    }
}


// Update previous values
prevEncoderCount = encoderCount;
prevEncoderCount_B = encoderCount_B;
prevTime = currentTime;

   // Check distance from the ultrasonic sensor
   if (isObstacleDetected) {
       stop();  // Stop motors if obstacle is detected
   } else {
       // outputA = motorAPID.compute(set_speed, vel_A);
       // outputB = motorBPID.compute(set_speed, vel_B);
        
        if (avg_vel > 15.0){
            targetAngle = backwardAngle; // BRAKING !!!
        }

        if (avg_vel < -15.0){
            targetAngle = forwardAngle; // BRAKING !!!
        }

      
        set_speed = gyro.compute(targetAngle, angle); // Compute set_speed based on angle error
        // set_speed = 0;

        // Inside your loop(), before computing motor outputs:
        if (turning) {
            // Calculate the yaw error between the target and current yaw
            float yawError = normalizeAngle(targetTurnAngle - ypr[0]);

            targetTurnAngle = normalizeAngle(turnStartYaw + (turnAngle * M_PI / 180.0));

            // Tolerance: allow a small error (here, 5° converted to radians)
            const float tolerance = 0.5 * M_PI / 180.0;
            // Optional: require stability for a short duration (e.g., 100ms)
            static unsigned long stableStartTime = 0;
            if (fabs(yawError) < tolerance) {
                if (stableStartTime == 0)
                    stableStartTime = millis();
                else if (millis() - stableStartTime >= 50) {
                    // Turn is complete: clear turning state and reset turnAdjustment
                    turning = false;
                    turnAdjustment = 0;
                    stableStartTime = 0;
                }
            } else {
                // Reset stable timer if error increases
                stableStartTime = 0;
                // Map the yaw error (in degrees) to turnAdjustment
                // For example: a 90° error should yield a turnAdjustment of 2.0 (as before)
                float errorDegrees = yawError * (180.0 / M_PI);
                const float K_turn = 2.8 / 90.0;  // Gain: adjust as needed
                turnAdjustment = K_turn * errorDegrees;
                
                    if (turnAdjustment > 0.20){
                        turnAdjustment = 0.20;
                    }

                    if (turnAdjustment < -0.20){
                        turnAdjustment = -0.20;
                    
                    }}
                
        }


        const float TURN_GAIN = 2.5;  // Tune this value as needed
        float turnTerm = turnAdjustment * TURN_GAIN;

        // Optionally, if the robot is moving in reverse, invert the turn term so that the differential remains consistent.
      

         outputA = motorAPID.compute(set_speed - turnTerm, vel_A);
         outputB = motorBPID.compute(set_speed + turnTerm, vel_B);

        //  Serial.printf("Target Angle: %.2f, OutputA: %.2f, OutputB: %.2f, TurnTerm: %.2f\n", targetAngle, outputA, outputB, turnTerm);

       if (outputA > 0 && outputB > 0) {
           forward(outputA, outputB);
       } else if (outputA < 0 && outputB < 0) {
           backward(-outputA, -outputB);
       } else if (outputA >= 0 && outputB < 0) {
            // In case outputs differ in sign, you can choose to turn in place:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, ((outputA) / 100.0) * 255);
            digitalWrite(INB1, HIGH);
            digitalWrite(INB2, LOW);
            analogWrite(ENB, ((-outputB) / 100.0) * 255);
        } else if (outputA < 0 && outputB >= 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, ((-outputA) / 100.0) * 255);
            digitalWrite(INB1, LOW);
            digitalWrite(INB2, HIGH);
            analogWrite(ENB, ((outputB) / 100.0) * 255);
        }
   }

  int rawADC = analogRead(VOLTAGE_SENSOR_PIN);

  // Calculate voltage at the module output
  float moduleOutputVoltage = (rawADC / ADC_RESOLUTION) * REF_VOLTAGE;

  // Calculate input voltage based on divider ratio
  float inputVoltage = moduleOutputVoltage * VOLTAGE_DIVIDER_RATIO;

}


updatePIDViaSerial();

// Process echo if received
//    if (echoReceived) {
//        noInterrupts();
//        unsigned long duration = echoEndTime - echoStartTime;
//        echoReceived = false;
//        interrupts();

//        float distance = duration * 0.034 / 2.0;
//        // Serial.print("DIS: ");
//        // Serial.println(distance);

//        // Use the distance value as needed
//        isObstacleDetected = (distance < 11);
//    }

 delay(1);  // Delay for readability in the serial monitor
}