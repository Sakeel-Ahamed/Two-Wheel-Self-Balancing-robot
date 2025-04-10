#ifndef GPID_H
#define GPID_H

class GPID {
public:
    GPID(float kp, float ki, float kd);
    float compute(float setpoint, float measured_value);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    unsigned long last_time;
};

#endif // GPID_H