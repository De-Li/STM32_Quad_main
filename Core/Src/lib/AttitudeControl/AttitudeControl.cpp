#include "AttitudeControl.h"

#include "../../Copter/Copter.h"

extern HexCopter copter;

AttitudeControl::AttitudeControl(){

}

void AttitudeControl::init(MotorsOutput* _motors, PilotCommand* _pilotcommand, SensorHandler* _ins){
	motors = _motors;
	pilotcommand = _pilotcommand;
	ins = _ins;
	take_off_flag = false;
}

void AttitudeControl::update(){
	if(copter.armingcheck.get_arming_status() == ArmingCheck::status::DISARM)
	{
		take_off_flag = false;
		return;
	}

	float pitch, roll;
	float yaw = pilotcommand->get_pilot_yaw_rate();
	float throttle = pilotcommand->get_pilot_throttle();

	if(!take_off_flag){
		if(throttle <= 0.02) return;
		pid_roll_rate.reset_I();
		pid_pitch_rate.reset_I();
		pid_yaw_rate.reset_I();
		take_off_flag = true;
	}

	if(pilotcommand->flightmode == pilotcommand->mode::acro){
		pilotcommand->get_pilot_command_rate(roll, pitch);
		run_rate_control(throttle, roll, pitch, yaw);
	}else{
		pilotcommand->get_pilot_command_angle(roll, pitch);
		run_angle_control(throttle, roll, pitch, yaw);
	}
}

void AttitudeControl::run_rate_control(float rc_throttle, float rc_roll, float rc_pitch, float rc_yaw){
	float feedback_roll = ins->get_roll();
	float feedback_pitch = ins->get_pitch();
	float feedback_yaw = ins->get_yaw();

	float uc_roll = pid_roll_rate.update(rc_roll - feedback_roll);
	float uc_pitch = pid_pitch_rate.update(rc_pitch - feedback_pitch);
	float uc_yaw = pid_yaw_rate.update(rc_yaw - feedback_yaw);

	//printf("rate_control:%f, %f, %f\n", uc_roll, uc_pitch, uc_yaw);

	set_output_to_motors(rc_throttle, uc_roll, uc_pitch, uc_yaw);
}

void AttitudeControl::run_angle_control(float rc_throttle, float rc_roll, float rc_pitch, float rc_yaw){
	float feedback_phi = ins->get_phi();
	float feedback_theta = ins->get_theta();

	float uc_roll = p_roll_angle.update(rc_roll - feedback_phi);
	float uc_pitch = p_pitch_angle.update(rc_pitch - feedback_theta);

	//printf("angle_control:%f, %f\n", uc_roll, uc_pitch);

	run_rate_control(rc_throttle, uc_roll, uc_pitch, rc_yaw);
}

void AttitudeControl::set_output_to_motors(float uc_throttle, float uc_roll, float uc_pitch, float uc_yaw){
	if(uc_roll > rate_max[0]) uc_roll = rate_max[0];
	else if(uc_roll < -rate_max[0]) uc_roll = -rate_max[0];
	uc_roll = map(uc_roll, -rate_max[0], rate_max[0], -1, 1);

	if(uc_pitch > rate_max[1]) uc_pitch = rate_max[1];
	else if(uc_pitch < -rate_max[1]) uc_pitch = -rate_max[1];
	uc_pitch = map(uc_pitch, -rate_max[1], rate_max[1], -1, 1);

	if(uc_yaw > rate_max[2]) uc_yaw = rate_max[2];
	else if(uc_yaw < -rate_max[2]) uc_yaw = -rate_max[2];
	uc_yaw = map(uc_yaw, -rate_max[2], rate_max[2], -1, 1);

	motors->set_attitude_to_motors(uc_throttle, uc_roll, uc_pitch, uc_yaw);
}

