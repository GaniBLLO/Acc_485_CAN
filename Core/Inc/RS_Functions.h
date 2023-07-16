/*
 * RS_485.h
 *
 *  Created on: 30 июн. 2023 г.
 *      Author: Sokolov EvgenII
 */

#ifndef INC_RS_FUNCTIONS_H_
#define INC_RS_FUNCTIONS_H_

#include "main.h"

typedef struct{

    int	RS_DataReady;		//Данные собраны и готовы к отправке
    int RS_DataSended;		//Данные отправлены
    int	CAN_buffer[6];		//Для сбора и передачи данных по шине CAN

    int RS_X_axis_data;
    int RS_Y_axis_data;
    int RS_Z_axis_data;
}RS_DATA_STRUCT;


void RS_Send(UART_HandleTypeDef *uart);
void RS_Receive(UART_HandleTypeDef *uart);
void SPI_Send(CAN_TxHeaderTypeDef *TxBuff);
void CAN_Recieve(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxBuff, uint8_t *rx_ml);

#endif /* INC_RS_FUNCTIONS_H_ */
