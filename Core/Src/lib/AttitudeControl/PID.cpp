#include "PID.h"

PID::PID(){
    Kp = 1;
    Ki = 0;
    Kd = 0;

    Imax = -1;
    last_error = 0;
    dt = 0.02;
    integrator = 0;
}

PID::PID(float kp, float ki, float kd, float imax):Kp(kp), Ki(ki), Kd(kd), Imax(imax){
    last_error = 0;
    dt = 0.02;
    integrator = 0;
}

void PID::set_kp(float _kp){
	Kp = _kp;
}

void PID::set_ki(float _ki){
	Ki = _ki;
}

void PID::set_kd(float _kd){
	Kd = _kd;
}

void PID::set_dt(float _dt){
	dt = _dt;
}

void PID::reset_I(){
	integrator = 0;
}

void PID::set_output_limits(float _Imax){
	Imax = _Imax;
}

float PID::update(float error) {

    float proportional = Kp * error;

    integrator += error * dt;
    float integralTerm = Ki * integrator;
    if(Imax != -1) integralTerm = integralTerm > Imax ? Imax : integralTerm;

    float derivative = Kd * (error - last_error) / dt;

    float output = proportional + integralTerm + derivative;

    last_error = error;

    return output;
}
