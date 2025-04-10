#include "pid.h"
#include <Arduino.h>




#define MAX_OUTPUT 100
#define MAX_RPM_F 650
#define MAX_VEL_R 29
#define MAX_RPM_R 570




// Constructor to initialize the PID parameters and reset variables
PID::PID(float kp, float ki, float kd) {
   this->kp = kp;
   this->ki = ki;
   this->kd = kd;
   this->prev_error = 0;
   this->integral = 0;
   this->last_time = millis();
}


// Compute the PID output
float PID::compute(float setpoint, float measured_value) {


   float error = setpoint - measured_value;


   unsigned long current_time = millis();
   float delta_time = (current_time - last_time) / 1000.0; // Convert ms to seconds


   if (delta_time > 0) {
       if (setpoint != 0) {
           integral += error * delta_time;
       } else {
           integral = 0; // Reset integral term when motor should be stopped
       }
       float derivative = (error - prev_error) / delta_time;


       // PID formula
       float output = (kp * error) + (ki * integral) + (kd * derivative);


       // Update previous values for next iteration
       prev_error = error;
       last_time = current_time;


       // Determine scaling factor based on forward or reverse direction
       float max_vel = (setpoint >= 0) ? -100 : 100;
      
       // Scale output to range [0, 100] based on maximum velocity for the direction
       //output = (output / max_vel) * 100;


       // Constrain output to be within 0 to 100
       output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

    //    Serial.printf("P : %.3f I: %.3f D: %.3f\n", kp, ki, kd);


    //    Serial.print("error: ");
    //    Serial.print(kp*error);
    //    Serial.print("Integral: ");
    //    Serial.print(ki*integral);

    //     Serial.print(" output: ");
    //     Serial.println(output);


       return output;
   }
   return 0;
}


// Setters for tuning parameters
void PID::setKp(float kp) { this->kp = kp; }
void PID::setKi(float ki) { this->ki = ki; }
void PID::setKd(float kd) { this->kd = kd; }

void PID::reset() {
    prev_error = 0;
    integral = 0;
    last_time = millis();
}

// Add this method in your GPID class definition in gpid.h (or its implementation file)
void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    // Optionally reset internal variables to avoid wind-up or sudden jumps:
    // prev_error = 0;
    // integral = 0;
    // last_time = millis();
}

