/*
 * RS_485.c
 *
 *  Created on: 30 июн. 2023 г.
 *      Author: Sokolov EvgenII
 */


#include "RS_485.h"
#include "ACC_init.h"
#include "string.h"
#include "stdio.h"

RS_DATA_STRUCT	rs;

void RS_Send(UART_HandleTypeDef *uart){

    char			buffer[50]; //44
    HAL_StatusTypeDef	result;

    if(rs.RS_DataReady){

		sprintf(buffer, "X_axis: %d\tY_axis: %d\tZ_axis: %d\r\n", (int16_t)OUT.X.all, (int16_t)OUT.Y.all, (int16_t)OUT.Z.all);

		result = HAL_UART_Transmit(uart, (uint8_t*) buffer, strlen(buffer), 10);
		if(result == HAL_OK){

			rs.RS_X_axis_data = 0;
			rs.RS_Y_axis_data = 0;
			rs.RS_Z_axis_data = 0;

			rs.RS_DataSended = 1;
		    rs.RS_DataReady = 0;
		}
		else
			rs.RS_DataSended = 0;

	//HAL_UART_Transmit(uart, (uint8_t*) "\033[0;0H", 6 , 5);
	//HAL_UART_Transmit(uart, (uint8_t*) "\033[2J", 4, 5);

    }

}
