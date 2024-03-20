/*
 * lidar.c
 *
 *  Created on: Oct 27, 2023
 *      Author: alan1
 */


#include "lidar.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	memcpy(received_data, UART6_rxBuffer, UART_buffer_size);
	osStatus_t ret = osSemaphoreRelease(UART6_Handle);
}

int processData(uint8_t num_bytes_received,
				uint16_t *distance, uint16_t *strength){
	int checksum_ok = 0;
	uint16_t message_sum = 0;
	for(int i = 0; i < num_bytes_received; i++){
	  message_sum += received_data[i];
	}
	checksum = received_data[num_bytes_received];
	// Checksum is lower 8 bits of message sum
	checksum_ok = ((message_sum & 0xFF) == checksum) ? 1 : 0;

	// Process message if header ok and checksum ok
	if((received_data[0] << 8 | received_data[1]) == frameheader && checksum_ok){
		*distance = (uint16_t)(received_data[3] << 8 | received_data[2]);
		*strength = (uint16_t)(received_data[5] << 8 | received_data[4]);
		return 1;
	}
	else{
		return 0;
	}
}
