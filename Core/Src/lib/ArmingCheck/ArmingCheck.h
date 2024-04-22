//#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../Copter/Copter.h"
#include "../PilotCommand/PilotCommand.h"
#include "../SensorHandler/SensorHandler.h"

#ifndef ARMINGCHECK
#define ARMINGCHECK

//10 hz checking rate->1 second
#define ARM_DELAY 20
#define DISARM_DELAY 20

class ArmingCheck{
public:

	friend class HexCopter;


	typedef enum
	{
		DISARM = 0,
		ARM = 1
	}status;

	ArmingCheck(){};
	void arm_check();
	bool prearm_check();
	bool get_arming_status();
	uint32_t get_failure_mask();

protected:

	bool check_sensors();
	bool check_rc();

private:

	uint8_t arming_status;
	uint32_t failure_mask = 0;
	enum arming_check_list{
		arming_check_ins = (1U << 0),
		arming_check_rc = (1U << 1),
		//arming_check_voltage = (1U << 3),
		//arming_check_safety_switch= (1U << 0),
		//arming_check_compass= (1U << 0),
	};
};
#endif
