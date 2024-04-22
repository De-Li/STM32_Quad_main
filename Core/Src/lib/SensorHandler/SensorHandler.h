#pragma once
#include "MS5611/MS5611.h"
#include <math.h>
#include "MPU6050/mpu6050.h"

class SensorHandler{
public:
	SensorHandler();

	void init();

	//static SensorHandler* get_singleton();

	// update gyro and accel values from accumulated samples
	void update();

	float get_roll() { return roll; }
	float get_pitch() { return pitch; }
	float get_yaw() { return yaw; }
	float get_phi() { return phi; }
	float get_theta() { return theta; }

	bool check_new_input();

#ifdef HITL
	uint8_t RxBuf[12];
	uint8_t busy_flag;
	uint8_t data[20];
#endif

private:
	void imuUpdate();
	void baroUpdate();
	void HITLUpdate();

	MPU6050 imu;
	MS5611 baro;

	int baro_tick; //氣壓計解析

	float roll, pitch, yaw;
	float phi, theta;

	bool new_data_flag_imu = 0;
};

extern UART_HandleTypeDef huart3;
