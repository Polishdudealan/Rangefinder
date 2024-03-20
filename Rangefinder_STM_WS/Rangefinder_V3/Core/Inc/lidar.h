/*
 * lidar.h
 *
 *  Created on: Oct 27, 2023
 *      Author: alan1
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include "cmsis_os.h"
#include <string.h>

#define UART_buffer_size 9

extern uint8_t UART6_rxBuffer[UART_buffer_size];
extern uint8_t received_data[UART_buffer_size];
extern UART_HandleTypeDef huart6;
extern osSemaphoreId_t UART6_Handle;

uint16_t frameheader = 0x5959;
uint8_t checksum;

// Returns 1 on success, 0 on failure
int processData(uint8_t num_bytes_received, uint16_t *distance, uint16_t *strength);

#endif /* INC_LIDAR_H_ */
