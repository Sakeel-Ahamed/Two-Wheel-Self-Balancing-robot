#include "gpid.h"
#include <Arduino.h>


#define MAX_OUTPUT 40
#define MAX_OUTPUT_MOTOR 100


// Constructor to initialize the PID parameters and reset variables
GPID::GPID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->prev_error = 0;
    this->integral = 0;
    this->last_time = millis();
}

// Compute the PID output
float GPID::compute(float setpoint, float measured_value) {

    float error = setpoint - measured_value;

    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0; // Convert ms to seconds

    // Serial.print("delta_time: ");
    // Serial.println(delta_time);

    if (delta_time > 0) {

        float output = kp * error;

        if (output > -MAX_OUTPUT && output < MAX_OUTPUT) {
            integral += error * delta_time;
        }

        float maxIntegral = MAX_OUTPUT / ki;
        integral = constrain(integral, -maxIntegral, maxIntegral);

        float derivative = (error - prev_error) / delta_time;

        // PID formula
        output += (ki * integral) + (kd * derivative);

        output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

        // Update previous values for next iteration
        prev_error = error;
        last_time = current_time;

        // Scale output to range [-34, 34] based on max output range
        

        // Serial.printf("P : %.3f I: %.3f D: %.3f\n", kp, ki, kd);

        
        // Serial.print("Error : ");
        // Serial.print(error);
        // Serial.print("P : ");
        // Serial.print(kp * error);
        // Serial.print(" Integral: ");
        // Serial.print(ki * integral);
        // Serial.print(" derivative: ");
        // Serial.print(kd * derivative);
        // Serial.print(" output: ");
        // Serial.println(output);
        

        return output;
    }
    return 0;
}

// Setters for tuning parameters
void GPID::setKp(float kp) { this->kp = kp; }
void GPID::setKi(float ki) { this->ki = ki; }
void GPID::setKd(float kd) { this->kd = kd; }

// Add this method in your GPID class definition in gpid.h (or its implementation file)
void GPID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    // Optionally reset internal variables to avoid wind-up or sudden jumps:
    // prev_error = 0;
    // integral = 0;
    // last_time = millis();
}

void GPID::reset() {
    prev_error = 0;
    integral = 0;
    last_time = millis();
}
