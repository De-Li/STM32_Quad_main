#pragma once
#include <stdio.h>

#include "../Copter/defines.h"
#include "../lib/Common/Common.h"
#include "../lib/Scheduler/Scheduler.h"
#include "../lib/SensorHandler/SensorHandler.h"
#include "../lib/MotorsOutput/MotorsOutput.h"
#include "../lib/PilotCommand/PilotCommand.h"
#include "../lib/AttitudeControl/AttitudeControl.h"
#include "../lib/ArmingCheck/ArmingCheck.h"

class HexCopter {
public:
	HexCopter() {

	}

	void setup();
	void loop();

	SensorHandler ins;
	MotorsOutput motors;
	PilotCommand pilotcommand;
	ArmingCheck armingcheck;

private:
	static const Scheduler::Task scheduler_tasks[];

    void get_scheduler_tasks(const Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit);

	void read_AHRS();
	void run_rate_controller();
    void motors_output();
	void hz_check();
	void arm_motors_check();
	void prearm_check();
	void motor_arm_check();

	Scheduler scheduler;
	AttitudeControl control;
};

extern HexCopter copter;
extern UART_HandleTypeDef huart3;
