#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "../../Math/Math.h"

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

class MPU6050{
private:
	// MPU6050 structure
	typedef struct {

	    int16_t Accel_X_RAW;
	    int16_t Accel_Y_RAW;
	    int16_t Accel_Z_RAW;
	    double Ax;
	    double Ay;
	    double Az;

	    int16_t Gyro_X_RAW;
	    int16_t Gyro_Y_RAW;
	    int16_t Gyro_Z_RAW;
	    double Gx;
	    double Gy;
	    double Gz;

	    float Temperature;

	    double KalmanAngleX;
	    double KalmanAngleY;
	} MPU6050_t;


	// Kalman structure
	typedef struct {
	    double Q_angle;
	    double Q_bias;
	    double R_measure;
	    double angle;
	    double bias;
	    double P[2][2];
	} Kalman_t;

	Kalman_t KalmanX = {
	        .Q_angle = 0.001f,
	        .Q_bias = 0.003f,
	        .R_measure = 0.03f
	};

	Kalman_t KalmanY = {
	        .Q_angle = 0.001f,
	        .Q_bias = 0.003f,
	        .R_measure = 0.03f,
	};

	const uint16_t i2c_timeout = 100;
	const double Accel_Z_corrector = 14418.0;
	uint32_t timer;

	double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

public:
	MPU6050_t DataStruct;

	double gyro_offset[3] = {-5, -3.7, 0.5};
	double acc_offset[3] = {0.03, -0.01, 0.06};

	uint8_t Init();

	void Read_Accel();

	void Read_Gyro();

	void Read_Temp();

	void Read_All();

	void Raw2Real();

	void Kalman_filter();
};

