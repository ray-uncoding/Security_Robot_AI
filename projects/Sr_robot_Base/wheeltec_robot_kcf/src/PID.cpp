#include "PID.h"
#include <iostream>
using namespace std;
PID::PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    //Calculation error
    float error = target - current;
    //Accumulation of errors
    intergral += error;
    //The difference between this error and the previous error
    derivative = error - prevError;
    //Apply the formula of pid
    targetpoint = kp * error + ki * intergral + kd * derivative;
    //Record the last error
    prevError = error;
    return targetpoint;
}

void PID::reset() {
    targetpoint = 0;
    intergral = 0;
    derivative = 0;
    prevError = 0;
}

void PID::Set_PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
