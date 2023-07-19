/*
 * RS_485.c
 *
 *  Created on: 30 июн. 2023 г.
 *      Author: Sokolov EvgenII
 */


#include <RS_Functions.h>
#include "ACC_init.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

RS_DATA_STRUCT			rs;
extern OUT_DATA			OUT;
OUT_DATA			RX_CAN_Data;
// RS485_DE Data Enable, Active High
// RS485_RE Receive En, Active Low
#define ENABLE_TRANSMIT() do { \
    /* Disable Receiver */ \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, SET); \
    /* Enable Transmitter */ \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET); \
    } while(0)

#define ENABLE_RECEIVE() do { \
    /* Enable Receiver */ \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, RESET); \
    /* Disable Transmitter */ \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET); \
    } while(0)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	rs.RS_DataSended = 1;
	rs.RS_DataReady = 0;

}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
	rs.RS_X_axis_data = 0;
	rs.RS_Y_axis_data = 0;
	rs.RS_Z_axis_data = 0;
}

void RS_Send(UART_HandleTypeDef *uart){

    char		buffer[50]; //44
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

void CAN_Recieve(UART_HandleTypeDef *uart, CAN_RxHeaderTypeDef *RxBuff, uint8_t *rx_ml){
	char 			buffer[50];
	uint8_t			nnn[1];
	HAL_UART_StateTypeDef	status;

	ENABLE_TRANSMIT();
	if(rs.RS_DataReady){

	    RX_CAN_Data.X.bit.LO = rx_ml[0];// & 0x0f;
	    RX_CAN_Data.X.bit.HI = rx_ml[1];// & 0x0f;
	    RX_CAN_Data.Y.bit.LO = rx_ml[2];// & 0x0f;
	    RX_CAN_Data.Y.bit.HI = rx_ml[3];// & 0x0f;
	    RX_CAN_Data.Z.bit.LO = rx_ml[4];// & 0x0f;
	    RX_CAN_Data.Z.bit.HI = rx_ml[5];// & 0x0f;

	    sprintf(buffer, "X_axis: %d\tY_axis: %d\tZ_axis: %d\r\n", (int16_t)RX_CAN_Data.X.all, (int16_t)RX_CAN_Data.Y.all, (int16_t)RX_CAN_Data.Z.all);
	    HAL_UART_Transmit_IT(uart, (uint8_t*)buffer, strlen(buffer));
		ENABLE_RECEIVE();
//			if((__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE) ? SET : RESET) == RESET){
//			    HAL_UART_Receive_IT(uart, smt, 1);
//			}
	    //result = HAL_UART_Transmit(uart, (uint8_t*) buffer, strlen(buffer), 10);
//			if(result == HAL_OK){
//				rs.RS_X_axis_data = 0;
//				rs.RS_Y_axis_data = 0;
//				rs.RS_Z_axis_data = 0;
//
//				rs.RS_DataSended = 1;
//				rs.RS_DataReady = 0;
//				}
	}
	else
	    rs.RS_DataSended = 0;

    status = HAL_UART_GetState(uart);
}
