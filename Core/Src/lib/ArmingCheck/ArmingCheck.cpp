#include "ArmingCheck.h"

extern HexCopter copter;

void ArmingCheck::arm_check()
{
	static int16_t arming_counter;

	//ch1:roll, ch2:pitch, ch3:throttle, ch4: yaw.
	uint16_t temp[14];
	memcpy(temp, copter.pilotcommand.get_rc(), sizeof(temp));

	if(copter.pilotcommand.emergency_event()){
		if(arming_status == ArmingCheck::status::DISARM) return;
		arming_status = ArmingCheck::status::DISARM;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
		printf("DISARMED\n");
	}

	if(temp[2] > 1225 || (temp[0] == 0 && temp[1]==0))
	{
		arming_counter = 0;
		return;
	}

	if(temp[3]>2000)
	{
		if(arming_counter <= ARM_DELAY)
		{
			arming_counter++;
		}
		if(arming_counter == ARM_DELAY && arming_status== 0)
		{
			arming_status = ArmingCheck::status::ARM;
			arming_counter = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
			printf("ARMED\n");
		}

	}
	else if(temp[3]<1230)
	{
		if(arming_counter <= DISARM_DELAY)
		{
			arming_counter++;
		}
		if(arming_counter == DISARM_DELAY && arming_status== 1)
		{
			arming_status = ArmingCheck::status::DISARM;
			arming_counter = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
			printf("DISARMED\n");
		}

	}
	else
	{
		arming_counter = 0;
	}
	//printf("arming_status: %d, arming counter: %d\n", arming_status, arming_counter);
}

bool ArmingCheck::check_sensors()
{
	bool sensors_state = copter.ins.check_new_input();
	if(sensors_state == 1)
	{
		failure_mask = failure_mask & ~ArmingCheck::arming_check_ins;
		//printf("sensors_state \n");
		return true;
	}
	else
	{
		failure_mask = failure_mask | ArmingCheck::arming_check_ins;
		printf("sensors connection failure\n");
		return false;
	}
}

bool ArmingCheck::check_rc()
{
	bool rc_state = copter.pilotcommand.has_new_input();
	if(rc_state == 1)
	{
		//printf("RC connection successful\n");
		failure_mask = failure_mask & ~ArmingCheck::arming_check_rc;
		return true;
	}
	else
	{
		failure_mask = failure_mask | ArmingCheck::arming_check_rc;
		printf("RC connection failure\n");
		return false;
	}
	//return rc_state;
}

bool ArmingCheck::prearm_check()
{
	get_failure_mask();

	if(get_arming_status() == ArmingCheck::status::ARM)
	{
		return 1;
	}
	return check_rc()&&check_sensors();
	//return check_rc();
}

bool ArmingCheck::get_arming_status()
{
	return arming_status;
}

uint32_t ArmingCheck::get_failure_mask()
{
	//printf("Failure_mask : %ld \n", failure_mask);
	return failure_mask;
}
