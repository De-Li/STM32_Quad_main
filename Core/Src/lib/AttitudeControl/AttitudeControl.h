#pragma once
#include "PID.h"
#include "../SensorHandler/SensorHandler.h"
#include "../MotorsOutput/MotorsOutput.h"
#include "../PilotCommand/PilotCommand.h"
#include "../Math/Math.h"

class AttitudeControl{
public:
	AttitudeControl();

	void init(MotorsOutput* _motors, PilotCommand* _pilotcommand, SensorHandler* _ins);
	void update();
	void run_rate_control(float rc_throttle, float rc_roll, float rc_pitch, float rc_yaw);
	void set_output_to_motors(float uc_throttle, float uc_roll, float uc_pitch, float uc_yaw);
	void run_angle_control(float rc_throttle, float rc_roll, float rc_pitch, float rc_yaw);

	PID pid_roll_rate{0.4, 0.02, 0, 2.0};
	PID pid_pitch_rate{0.4, 0.02, 0, 2.0};
	PID pid_yaw_rate{1, 0.02, 0, 2.0};

	PID p_roll_angle{3.2, 0, 0, 1.0};
	PID p_pitch_angle{3.6, 0, 0, 1.0};

	float rate_max[3] = {1.2, 1.2, 1.2};

private:
	MotorsOutput* motors;
	PilotCommand* pilotcommand;
	SensorHandler* ins;

	bool take_off_flag;
};
