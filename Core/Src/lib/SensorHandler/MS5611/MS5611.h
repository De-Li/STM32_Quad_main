#pragma once
#ifndef _MS5611_H
#define _MS5611_H
#include <cstdlib>
#include <stdio.h>
#include "main.h"
//MS5611 ADDR CSB接地
#define MS5611_SLAVE_ADDR 0x77

//COMMAND
#define MS5611_CMD_REST 0x1E
#define MS5611_CMD_CONVERT_D1_256 0x40
#define MS5611_CMD_CONVERT_D1_512 0x42
#define MS5611_CMD_CONVERT_D1_1024 0x44
#define MS5611_CMD_CONVERT_D1_2048 0x46
#define MS5611_CMD_CONVERT_D1_4096 0x48
#define MS5611_CMD_CONVERT_D2_256 0x50
#define MS5611_CMD_CONVERT_D2_512 0x52
#define MS5611_CMD_CONVERT_D2_1024 0x54
#define MS5611_CMD_CONVERT_D2_2048 0x56
#define MS5611_CMD_CONVERT_D2_4096 0x58

#define MS6511_ADC_READ 0x00

#define MS5611_PROM_READ_0 0xA0
#define MS5611_PROM_READ_1 0xA2
#define MS5611_PROM_READ_2 0xA4
#define MS5611_PROM_READ_3 0xA6
#define MS5611_PROM_READ_4 0xA8
#define MS5611_PROM_READ_5 0xAA
#define MS5611_PROM_READ_6 0xAC
#define MS5611_PROM_READ_7 0xAE

class MS5611{
public:
	MS5611();

	struct MS5611_t
	{
		uint16_t C[8];	//用于补偿温度和气压数据
		unsigned int D[2];	//存放读取的气压，温度数据
		int dT;	//Difference between actual and reference temperature
		long long OFF;	//Offset at actual temperature
		long long SENS;	//Sensitivity at actual temperature
		int TEMP;	//Actual temperature
		signed int P;	//温度补充气压
	};

	MS5611_t* ms5611_handle;

	void init();
	void convertOSR(uint8_t OSR);
	void read_temp();
	void read_press();
	void calculate();
};

#endif
