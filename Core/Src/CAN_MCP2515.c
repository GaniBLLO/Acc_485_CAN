/*
 * CAN_MCP2515.c
 *
 *  Created on: Jul 6, 2023
 *      Author: AnastasII
 */
#include "main.h"
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;


extern CAN_HandleTypeDef can;

void init_MCP2515(){

    uint32_t RX_msg = 0;
    uint8_t data[3];

    data[0] = 0x02;
    data[1] = 0x0F;
    data[2] = 0x80;

	HAL_CAN_AddTxMessage(&can, &TxHeader, data, &RX_msg);
	HAL_Delay(200);
}

void init_peripherial(){
	init_MCP2515();
}



