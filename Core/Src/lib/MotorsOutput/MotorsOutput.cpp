#include "MotorsOutput.h"
#include "main.h"

#include "../../Copter/Copter.h"

//#define CORRECTION

extern HexCopter copter;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void MotorsOutput::init(){
	PWM_min = 1000;
	PWM_max = 2000;

#if AXIS == 4
	motors_thrust_out[0].init(TIM2, 1);
	motors_thrust_out[1].init(TIM2, 2);
	motors_thrust_out[2].init(TIM3, 1);
	motors_thrust_out[3].init(TIM3, 2);
#elif AXIS == 6
	motors_thrust_out[0].init(TIM2, 1);
	motors_thrust_out[1].init(TIM2, 2);
	motors_thrust_out[2].init(TIM3, 1);
	motors_thrust_out[3].init(TIM3, 2);
	motors_thrust_out[4].init(TIM3, 3);
	motors_thrust_out[5].init(TIM3, 4);
#endif

	start_flag = false;
}

void MotorsOutput::motors_output(){

#ifdef CORRECTION
	if(!start_flag) motors_start();
	motor_correction();
	return;
#endif

	if(copter.armingcheck.get_arming_status() == ArmingCheck::status::DISARM)
	{
		if(start_flag) motors_stop();
		return;
	}
	if(!start_flag) motors_start();
#ifdef HITL
	uint16_t out[AXIS];
	for(int idx = 0; idx < AXIS; idx++){
		out[idx] = motors_thrust_out[idx].pwm;
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)out, AXIS*2, 100);

#else

#endif
}

void MotorsOutput::conver_output_to_pwm(float* moter_output){
	for(int idx = 0; idx < AXIS; idx++){
		motors_thrust_out[idx].setPWM(map(moter_output[idx], 0, 1, PWM_min, PWM_max));
		printf("%d,", motors_thrust_out[idx].pwm);
		//motors_thrust_out[idx] = map(moter_output[idx], -1, 1, 1000, 2000);
	}
	printf("\n");
}

void MotorsOutput::motors_start(){
#if AXIS == 4
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
#elif AXIS == 6
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif
	start_flag = true;
}

void MotorsOutput::motors_stop(){
	for(int idx = 0; idx < AXIS; idx++){
		motors_thrust_out[idx].setPWM(1000);
	}
#if AXIS == 4
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
#elif AXIS == 6
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
#endif
	start_flag = false;
}

void MotorsOutput::set_attitude_to_motors(float uc_throttle, float uc_roll, float uc_pitch, float uc_yaw){
#ifdef CORRECTION
	return;
#endif

	float moter_output[AXIS];
	float roll_rate = ROLL_CONTROL_RATE * uc_roll;
	float pitch_rate = PITCH_CONTROL_RATE * uc_pitch;
	float yaw_rate = YAW_CONTROL_RATE * uc_yaw;

#if AXIS == 4
	moter_output[0] = uc_throttle;
	moter_output[1] = uc_throttle;
	moter_output[2] = uc_throttle;
	moter_output[3] = uc_throttle;

	//printf("%f, %f, %f\n", roll_rate, pitch_rate, yaw_rate);

	if(uc_throttle < 0.01){
		conver_output_to_pwm(moter_output);
		return;
	}

	if(pitch_rate >= 0){
		if(moter_output[0] + pitch_rate > 1 || moter_output[1] + pitch_rate > 1){
			moter_output[2] -= pitch_rate;
			moter_output[3] -= pitch_rate;
		}else{
			moter_output[0] += pitch_rate;
			moter_output[1] += pitch_rate;
		}
	}else{
		if(moter_output[2] - pitch_rate > 1 || moter_output[3] - pitch_rate > 1){
			moter_output[0] += pitch_rate;
			moter_output[1] += pitch_rate;
		}else{
			moter_output[2] -= pitch_rate;
			moter_output[3] -= pitch_rate;
		}
	}


	if(roll_rate >= 0){
		if(moter_output[0] + roll_rate > 1 || moter_output[2] + roll_rate > 1){
			moter_output[1] -= roll_rate;
			moter_output[3] -= roll_rate;
		}else{
			moter_output[0] += roll_rate;
			moter_output[2] += roll_rate;
		}
	}else{
		if(moter_output[1] - roll_rate > 1 || moter_output[3] - roll_rate > 1){
			moter_output[0] += roll_rate;
			moter_output[2] += roll_rate;
		}else{
			moter_output[1] -= roll_rate;
			moter_output[3] -= roll_rate;
		}
	}

	if(yaw_rate >= 0){
		if(moter_output[1] + yaw_rate > 1 || moter_output[2] + yaw_rate > 1){
			moter_output[0] -= yaw_rate;
			moter_output[3] -= yaw_rate;
		}else{
			moter_output[1] += yaw_rate;
			moter_output[2] += yaw_rate;
		}
	}else{
		if(moter_output[0] - yaw_rate > 1 || moter_output[3] - yaw_rate > 1){
			moter_output[1] += yaw_rate;
			moter_output[2] += yaw_rate;
		}else{
			moter_output[0] -= yaw_rate;
			moter_output[3] -= yaw_rate;
		}
	}

#elif AXIS == 6
	moter_output[0] = uc_throttle + uc_pitch + uc_roll * 0.5 + uc_yaw;
	moter_output[1] = uc_throttle + uc_pitch - uc_roll * 0.5 - uc_yaw;
	moter_output[2] = uc_throttle + uc_roll - uc_yaw;
	moter_output[3] = uc_throttle - uc_roll + uc_yaw;
	moter_output[4] = uc_throttle - uc_pitch + uc_roll * 0.5 + uc_yaw;
	moter_output[5] = uc_throttle - uc_pitch - uc_roll * 0.5 - uc_yaw;
#endif

	conver_output_to_pwm(moter_output);
}

void MotorsOutput::motor_correction(){
	if(copter.armingcheck.get_arming_status() == ArmingCheck::status::ARM && motors_thrust_out[0].pwm != 2000){
		for(int idx = 0; idx < AXIS; idx++){
			motors_thrust_out[idx].setPWM(2000);
		}
	}else if(copter.armingcheck.get_arming_status() == ArmingCheck::status::DISARM && motors_thrust_out[0].pwm != 1000){
		for(int idx = 0; idx < AXIS; idx++){
			motors_thrust_out[idx].setPWM(1000);
		}
	}
}
