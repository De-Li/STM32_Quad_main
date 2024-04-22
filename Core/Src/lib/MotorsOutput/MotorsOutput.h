#pragma once
#include <stdio.h>
#include "PWMChannel.h"
#include "../Math/Math.h"

#define AXIS 4
#define ROLL_CONTROL_RATE 0.3
#define PITCH_CONTROL_RATE 0.3
#define YAW_CONTROL_RATE 0.3

class MotorsOutput {
public:
	MotorsOutput(){}
	void init();
	void motors_output();
	void set_attitude_to_motors(float uc_throttle, float uc_roll, float uc_pitch, float uc_yaw);
	void conver_output_to_pwm(float* moter_output);
	void motor_correction();

private:
	void motors_start();
	void motors_stop();

	int PWM_min;
	int PWM_max;
	bool start_flag;
	PWMChannel motors_thrust_out[AXIS];
};
