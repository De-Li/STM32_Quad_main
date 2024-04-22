#include "MS5611.h"

extern I2C_HandleTypeDef hi2c3;

#define MS5611_DEBUG 1

MS5611::MS5611()
{

	//init handle point
	ms5611_handle = (struct MS5611_t *)malloc(sizeof(struct MS5611_t));

	ms5611_handle->C[0] = 0;
	ms5611_handle->C[1] = 0;
	ms5611_handle->C[2] = 0;
	ms5611_handle->C[3] = 0;
	ms5611_handle->C[4] = 0;
	ms5611_handle->C[5] = 0;
	ms5611_handle->C[6] = 0;
	ms5611_handle->C[7] = 0;

	ms5611_handle->D[0] = 0;
	ms5611_handle->D[1] = 0;

	ms5611_handle->TEMP = 0;
	ms5611_handle->P = 0;
	ms5611_handle->SENS = 0;
	ms5611_handle->OFF = 0;
}

void MS5611::init()
{
	HAL_StatusTypeDef ret_V;

	uint8_t PROM = 0xA0;
	uint8_t REST = 0x1E;

	ret_V = HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, &REST, 1, 0xff);
	if (ret_V != HAL_OK){
		printf("Baro reset error\n");
	}

	HAL_Delay(10);

	for(uint8_t i =0; i < 8; i++){
		ret_V = HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, &PROM, 1, 0xff);
		if (ret_V != HAL_OK){
			printf("Baro C%i Transmit error: %i\n", i, ret_V);
		}
		uint8_t buffer[2];
		ret_V = HAL_I2C_Master_Receive(&hi2c3, MS5611_SLAVE_ADDR << 1, buffer, 2, 0xff);
		ms5611_handle->C[i] = (uint8_t)buffer[0]<<8|(uint8_t)buffer[1];
		if (ret_V != HAL_OK){
			printf("Baro C%i Receive error: %i\n", i, ret_V);
		}else{
			printf("Baro C%i : %i\n", i, ms5611_handle->C[i]);
		}
		PROM += 2;
	}
	//HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, (unsigned char *)MS5611_CMD_CONVERT_D1_1024, 1, 0xff);
}

void MS5611::convertOSR(uint8_t OSR){
	HAL_StatusTypeDef ret_V;
	ret_V = HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, &OSR, 1, 0xff);
	if (ret_V != HAL_OK){
		printf("Baro Transmit Convert error: %i\n", ret_V);
	}
}

/*
 * 讀取溫度，轉換經度4096
*/
void MS5611::read_temp()
{
	HAL_StatusTypeDef ret_V;
	uint8_t READ = 0x00;
	uint8_t buffer[3];
	ret_V = HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, &READ, 1, 0xff);
	if (ret_V != HAL_OK){
		printf("Baro Transmit Temp0x00 error: %i\n", ret_V);
	}
	ret_V = HAL_I2C_Master_Receive(&hi2c3, MS5611_SLAVE_ADDR << 1, buffer, 3, 0xff);
	ms5611_handle->D[1] = (unsigned int)buffer[0]<<16|(unsigned int)buffer[1]<<8|(unsigned int)buffer[2];
	if (ret_V != HAL_OK){
		printf("Baro Transmit Temp error: %i\n", ret_V);
	}
}

/*
 * 讀取氣壓，轉換經度4096
*/
void MS5611::read_press()
{
	HAL_StatusTypeDef ret_V;
	uint8_t READ = 0x00;
	uint8_t buffer[3];
	ret_V = HAL_I2C_Master_Transmit(&hi2c3, MS5611_SLAVE_ADDR << 1, &READ, 1, 0xff);
	if (ret_V != HAL_OK){
		printf("Baro Transmit Press0x00 error: %i\n", ret_V);
	}
	ret_V = HAL_I2C_Master_Receive(&hi2c3, MS5611_SLAVE_ADDR << 1, buffer, 3, 0xff);
	ms5611_handle->D[0] = (unsigned int)buffer[0]<<16|(unsigned int)buffer[1]<<8|(unsigned int)buffer[2];
	if (ret_V != HAL_OK){
		printf("Baro Transmit Press error: %i\n", ret_V);
	}
}

/*
 * 修正氣壓溫度
*/
void MS5611::calculate()
{

	ms5611_handle->dT = ms5611_handle->D[1] - ((long long)ms5611_handle->C[5]<<8);
	ms5611_handle->TEMP = 2000 + (((long long)ms5611_handle->dT * ms5611_handle->C[6])>>23);
	ms5611_handle->OFF = ((long long)ms5611_handle->C[2] << 16) + ((long long)ms5611_handle->C[4] * ms5611_handle->dT >> 7);
	ms5611_handle->SENS = ((long long)ms5611_handle->C[1] << 15) + ((long long)ms5611_handle->C[3] * ms5611_handle->dT >> 8);
	ms5611_handle->P = (((long long)ms5611_handle->D[0] * ms5611_handle->SENS >> 21) - ms5611_handle->OFF) >> 15;

	/*signed long long dT = 0,TEMP = 0,T2 = 0,OFF = 0,OFF2 = 0,SENS2 = 0,SENS = 0;

	dT = ms5611_handle->D[1] - ((int)(ms5611_handle->C[4])<<8);
	TEMP = 2000 + ((signed long long) (dT*(ms5611_handle->C[5]))>>23);
	printf("TEMP: %l --- dT: %l\n", TEMP, dT);
	//低于20°时：
	if(TEMP < 2000 && TEMP > -1500)
	{
		T2 = ( dT*dT )>>31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;
	}
	OFF = (((__int64_t)(ms5611_handle->C[1])) << 16) + (((ms5611_handle->C[3]) * dT) >> 7);
	SENS = (((__int64_t)(ms5611_handle->C[0])) << 15) + (((ms5611_handle->C[2]) * dT) >> 8);*/

	printf("Baro TEMP: %.1f C --- PRES: %.1f mbar\n", (ms5611_handle->TEMP * 0.01), (ms5611_handle->P * 0.01));
}
