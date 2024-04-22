#include "PWMChannel.h"
#include <stdio.h>

void PWMChannel::init(TIM_TypeDef* _timer, uint8_t _channel){
	timer = _timer;
	channel = _channel;
	switch(_channel){
	case 1:
		timer->CCR1 = 1000 - 1;
		break;
	case 2:
		timer->CCR2 = 1000 - 1;
		break;
	case 3:
		timer->CCR3 = 1000 - 1;
		break;
	case 4:
		timer->CCR4 = 1000 - 1;
		break;
	}
	*(&timer->CCR1 + channel - 1) = 1000 - 1;

	pwm = 1000;
}

void PWMChannel::setPWM(uint32_t _pwm){
	*(&timer->CCR1 + channel - 1) = _pwm - 1;
	pwm = _pwm;
}
