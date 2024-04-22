#include "../SensorHandler/SensorHandler.h"
#include <cstring>
#include "../Math/Math.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart5_rx;

SensorHandler::SensorHandler(){
	baro_tick = 0;
}

void SensorHandler::init(){
#ifdef HITL
	busy_flag = HAL_UART_Receive_IT(&huart3, RxBuf, sizeof(RxBuf));
	//__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
#else
	imu.Init();
	//baro.init();
#endif
}

void SensorHandler::update(){
	imuUpdate();
	//baroUpdate();
}


void SensorHandler::imuUpdate(){
	// Prepare imu data
	/*if(imu.dataReady()){
	}
	else
	{
		new_data_flag_imu = 0;
	}*/
#ifdef HITL
	HITLUpdate();
#else
	imu.Read_All();
#endif
	imu.Kalman_filter();

#ifdef HITL
	roll = imu.DataStruct.Gx;
	pitch = imu.DataStruct.Gy;
	yaw = imu.DataStruct.Gz;

	theta = imu.DataStruct.KalmanAngleY;
	phi = imu.DataStruct.KalmanAngleX;
#else
	roll = imu.DataStruct.Gy;
	pitch = imu.DataStruct.Gx;
	yaw = -imu.DataStruct.Gz;

	theta = imu.DataStruct.KalmanAngleX;
	phi = imu.DataStruct.KalmanAngleY;
#endif
	//printf("x:%f, y:%f, z:%f\n", imu.DataStruct.Ax, imu.DataStruct.Ay, imu.DataStruct.Az);
	//printf("x:%f, y:%f, z:%f\n", roll* RAD_TO_DEG, pitch* RAD_TO_DEG, yaw* RAD_TO_DEG);
	//printf("x:%f, y:%f, z:%f\n", phi * RAD_TO_DEG, theta * RAD_TO_DEG, yaw * RAD_TO_DEG);
	new_data_flag_imu = 1;
}

void SensorHandler::baroUpdate(){
	baro_tick++;
	switch (baro_tick){
	case 45:
		baro.convertOSR(MS5611_CMD_CONVERT_D2_1024);
		break;
	case 46:
		baro.read_temp();
		break;
	case 47:
		baro.convertOSR(MS5611_CMD_CONVERT_D1_1024);
		break;
	case 48:
		baro.read_press();
		break;
	case 49:
		baro.calculate();
		baro_tick = 0;
		break;
	}
}
#ifdef HITL
void SensorHandler::HITLUpdate(){
	uint16_t combinedData = ((uint32_t)data[1] << 8) | (uint32_t)data[0];
	memcpy(&imu.DataStruct.Gyro_X_RAW, &combinedData, sizeof(uint16_t));

	combinedData = ((uint32_t)data[3] << 8) | (uint32_t)data[2];
	memcpy(&imu.DataStruct.Gyro_Y_RAW, &combinedData, sizeof(uint16_t));

	combinedData = ((uint32_t)data[5] << 8) | (uint32_t)data[4];
	memcpy(&imu.DataStruct.Gyro_Z_RAW, &combinedData, sizeof(uint16_t));

	combinedData = ((uint32_t)data[7] << 8) | (uint32_t)data[6];
	memcpy(&imu.DataStruct.Accel_X_RAW, &combinedData, sizeof(uint16_t));

	combinedData = ((uint32_t)data[9] << 8) | (uint32_t)data[8];
	memcpy(&imu.DataStruct.Accel_Y_RAW, &combinedData, sizeof(uint16_t));

	combinedData = ((uint32_t)data[11] << 8) | (uint32_t)data[10];
	memcpy(&imu.DataStruct.Accel_Z_RAW, &combinedData, sizeof(uint16_t));

    imu.Raw2Real();

	if(busy_flag == 0) {
		new_data_flag_imu = 1;
		//printf("%d, %d, %d, %d, %d, %d\n", imu.DataStruct.Gyro_X_RAW, imu.DataStruct.Gyro_Y_RAW, imu.DataStruct.Gyro_Z_RAW, imu.DataStruct.Accel_X_RAW, imu.DataStruct.Accel_Y_RAW, imu.DataStruct.Accel_Z_RAW);
	}else{
		new_data_flag_imu = 0;
	}
}
#endif

bool SensorHandler::check_new_input(){
	return new_data_flag_imu;
}
