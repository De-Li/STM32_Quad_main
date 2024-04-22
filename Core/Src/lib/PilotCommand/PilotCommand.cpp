#include "PilotCommand.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

PilotCommand::PilotCommand()
{

}

void PilotCommand::init()
{
	HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	RC_ROLL_RANGE = RC_ROLL_MAX - RC_ROLL_MIN;
	RC_ROLL_MID = (RC_ROLL_MAX + RC_ROLL_MIN) / 2;

	RC_PITCH_RANGE = RC_PITCH_MAX - RC_PITCH_MIN;
	RC_PITCH_MID = (RC_PITCH_MAX + RC_PITCH_MIN) / 2;

	RC_THROTTLE_RANGE = RC_THROTTLE_MAX - RC_THROTTLE_MIN;
	RC_THROTTLE_MID = (RC_THROTTLE_MAX + RC_THROTTLE_MIN) / 2;

	RC_YAW_RANGE = RC_YAW_MAX - RC_YAW_MIN;
	RC_YAW_MID = (RC_YAW_MAX + RC_YAW_MIN) / 2;

}
void PilotCommand::read_rc_input()
{
	memcpy((uint8_t *)main_buffer, temp_buffer, RXBUF_SIZE);
	for(int i =0; i < RXBUF_SIZE-25; i++)
	{
	  if(main_buffer[i] == 0x0F && (main_buffer[i+24]==0x04 || main_buffer[i+24]==0x14 || main_buffer[i+24]==0x24 || main_buffer[i+24]==0x34))
	  {
		  if(main_buffer[i+24] != previous_stop_byte)
		  {
			  //printf("stopbit: %d, previousstopbit: %d",main_buffer[i+24],  previous_stop_bit);
			  memcpy ((uint8_t *)sbus_data, main_buffer+i, SBUSBUF_SIZE);
			  parse_sbus();
			  update_flight_mode();
			  previous_stop_byte = main_buffer[i+24];

			  /*for(int i =0; i< 6; i++)
			  {
				  printf("%d, ", rc_input[i]);
			  }
			  printf("\n");*/

			  /*SBUS_received_count++;
			  if(SBUS_received_count % 10 == 0)
			  {
				  for(int i =0; i< 25; i++)
				  {
					  printf("%d, ", sbus_data[i]);
				  }
				  printf("\n");
				  uint32_t current_time = HAL_GetTick();
				  float SBUS_received_frequency = ((float)SBUS_received_count*1000)/(current_time-start_time);
				  //float callback_received_frequency = ((float)callback_received_count*1000)/(current_time-start_time);
				  printf("SBUS Frequency: %f\n", SBUS_received_frequency);
				  //printf("callback Frequency: %f\n", callback_received_frequency);
			  }*/
		  }
	  }
	}
}

void PilotCommand::parse_sbus()
{
	uint16_t parsed_sbus[16];
	parsed_sbus[0] = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
	parsed_sbus[1] = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
	parsed_sbus[2] = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
	parsed_sbus[3] = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
	parsed_sbus[4] = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
	parsed_sbus[5] = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
	parsed_sbus[6] = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
	parsed_sbus[7] = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
	parsed_sbus[8] = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
	parsed_sbus[9] = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
	parsed_sbus[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
	parsed_sbus[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
	parsed_sbus[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
	parsed_sbus[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
	parsed_sbus[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
	parsed_sbus[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
	decode_channels(parsed_sbus);
}

void PilotCommand::decode_channels(uint16_t* parsed_sbus)
{
	for(int i =0 ; i< 6; i++)
	{
		rc_input[i] = (1000 + (parsed_sbus[i]*1000)/SBUS_RANGE_RANGE);
	}
}

void PilotCommand::update_flight_mode()
{
	if(rc_input[4] < 1500)
	{
		flightmode = mode::acro;
	}
	else if(rc_input[4] > 1500)
	{
		flightmode = mode::stabilize;
	}
}

bool PilotCommand::emergency_event(){
	bool result = false;
	if(rc_input[5] < 1500)
	{
		result = true;
	}
	return result;
}

void PilotCommand::get_pilot_command_rate(float &roll_rate_out, float &pitch_rate_out)
{

	if(abs(rc_input[0] - RC_ROLL_MID) < 20)
	{
		roll_rate_out = 0;
	}
	else
	{
		if(rc_input[0] < RC_ROLL_MIN) rc_input[0] = RC_ROLL_MIN;
		if(rc_input[0] > RC_ROLL_MAX) rc_input[0] = RC_ROLL_MAX;
		roll_rate_out = ((float)(rc_input[0] - RC_ROLL_MID) / RC_ROLL_RANGE) * ROLL_RATE_MAX * 2;
	}
	if(abs(rc_input[1] - RC_PITCH_MID) < 20)
	{
		pitch_rate_out = 0;
	}
	else
	{
		if(rc_input[1] < RC_PITCH_MIN) rc_input[1] = RC_PITCH_MIN;
		if(rc_input[1] > RC_PITCH_MAX) rc_input[1] = RC_PITCH_MAX;
		pitch_rate_out = ((float)(rc_input[1] - RC_PITCH_MID) / RC_PITCH_RANGE) * PITCH_RATE_MAX * 2;
	}
}

void PilotCommand::get_pilot_command_angle(float &roll_angle_out, float &pitch_angle_out)
{
	if(abs(rc_input[0] - RC_ROLL_MID) < 20)
	{
		roll_angle_out = 0;
	}
	else
	{
		if(rc_input[0] < RC_ROLL_MIN) rc_input[0] = RC_ROLL_MIN;
		if(rc_input[0] > RC_ROLL_MAX) rc_input[0] = RC_ROLL_MAX;
		roll_angle_out = ((float)(rc_input[0] - RC_ROLL_MID) / RC_ROLL_RANGE) * ROLL_ANGLE_MAX * 2;
	}
	if(abs(rc_input[1] - RC_PITCH_MID) < 20)
	{
		pitch_angle_out = 0;
	}
	else
	{
		if(rc_input[1] < RC_PITCH_MIN) rc_input[1] = RC_PITCH_MIN;
		if(rc_input[1] > RC_PITCH_MAX) rc_input[1] = RC_PITCH_MAX;
		pitch_angle_out = ((float)(rc_input[1] - RC_PITCH_MID) / RC_PITCH_RANGE) * PITCH_ANGLE_MAX * 2;
	}
}

float PilotCommand::get_pilot_yaw_rate()
{
	if(abs(rc_input[3] - RC_YAW_MID) < 20)
	{
		return 0;
	}
	else
	{
		if(rc_input[3] < RC_YAW_MIN) rc_input[3] = RC_YAW_MIN;
		if(rc_input[3] > RC_YAW_MAX) rc_input[3] = RC_YAW_MAX;
		return ((float)(rc_input[3] - RC_YAW_MID) / RC_YAW_RANGE) * YAW_RATE_MAX * 2;
	}
}

float PilotCommand::get_pilot_throttle()
{
	if(abs(rc_input[2] - RC_THROTTLE_MID) < 20)
	{
		return 0.5;
	}
	else
	{
		if(rc_input[2] <= RC_THROTTLE_MIN) return 0;
		if(rc_input[2] >= RC_THROTTLE_MAX) return 1;
		return (float)(rc_input[2] - RC_THROTTLE_MIN) / RC_THROTTLE_RANGE;
	}
}

bool PilotCommand::has_new_input()
{
	//printf("previous_rx_buffer[24]:%d,_rx_buffer[24]:%d\n", previous_rx_buffer[24], rx_buffer[24]);
	return (sbus_data[23] == 2 || sbus_data[23] == 0) ? true:false;
}

uint16_t* PilotCommand::get_rc()
{
	return rc_input;
}
