#pragma once

class PID {
private:
    float Kp;
    float Ki;
    float Kd;

    float Imax;
    float filtering_frequency;
    float last_error;
    float dt;
    float integrator;

public:
    PID();
    PID(float kp, float ki, float kd, float imax);

    void set_kp(float _kp);
    void set_ki(float _ki);
    void set_kd(float _kd);
    void set_dt(float _dt);
    void set_output_limits(float _Imax);
    void reset_I();

    float update(float error);
};
