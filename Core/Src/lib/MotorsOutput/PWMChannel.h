#pragma once

#include "main.h"

class PWMChannel{
public:
	PWMChannel(){}
	void init(TIM_TypeDef* _timer, uint8_t _channel);
	void setPWM(uint32_t _pwm);

	uint16_t pwm;
private:
	uint8_t channel;
	TIM_TypeDef* timer;
};
