/*
 * ACC_init.c
 *
 *  Created on: Jun 28, 2023
 *      Author: Sokolov EvgenII
 */

#include "ACC_init.h"
#include "RS_485.h"

#define ACC_ADDR	0x18 << 1
#define COMMAND_CTRL_REG1	0x20
#define COMMAND_CTRL_REG2	0x21
#define COMMAND_CTRL_REG4	0x23
#define COMMAND_STATUS_REG	0x27
#define COMMAND_X_LO		0x28
#define COMMAND_X_HI		0x29
#define COMMAND_Y_LO		0x2A
#define COMMAND_Y_HI		0x2B
#define COMMAND_Z_LO		0x2C
#define COMMAND_Z_HI		0x2D

ACC_SETTING	ACC_set = ACC_SETTING_DEFAULT;
OUT_DATA	OUT 	= OUT_DATA_XYZ_DEFAULT;
extern RS_DATA_STRUCT	rs;

void update_ACC_data(I2C_HandleTypeDef *i2c){

	uint8_t		command[1];

	HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_STATUS_REG, 1, &command[0], 1, 10);		//Обнвляем статус регистров
	ACC_set.STATUS_REG.all = command[0];

	if(rs.RS_DataSended && ACC_set.STATUS_REG.bit.ZYXDA){
		read_x_axis(i2c);
		read_y_axis(i2c);
		read_z_axis(i2c);
	}
}

void ACC_init(I2C_HandleTypeDef *i2c){

    ACC_init_addr(ACC_ADDR, i2c);
    ACC_setting(ACC_ADDR, i2c);

    rs.RS_DataSended = 1;
    //ACC_check_settings(ACC_ADDR, i2c);
}

void ACC_init_addr (uint8_t address, I2C_HandleTypeDef *i2c){
    HAL_StatusTypeDef result;

    for(;;){
	result = HAL_I2C_IsDeviceReady(i2c, address, 1, 150);
	    if(result == HAL_OK)
	    	break;
	}

    uint8_t command_arr[1];
    uint8_t receive_arr[1];

    command_arr[0] = 0xF;
    //WHO_AM_I
    HAL_I2C_Master_Transmit(i2c, address, command_arr, sizeof(command_arr), 10);
    HAL_I2C_Master_Receive(i2c, address, receive_arr, sizeof(receive_arr), 10);

    GPIOC->BSRR |= GPIO_BSRR_BS13;
    HAL_Delay(1000);
    GPIOC->BSRR |= GPIO_BSRR_BR13;
    HAL_Delay(1000);
    GPIOC->BSRR |= GPIO_BSRR_BS13;
}

void ACC_setting(uint8_t address, I2C_HandleTypeDef *i2c){

    uint8_t 			command_arr[3];

    HAL_I2C_Mem_Write(i2c, address, COMMAND_CTRL_REG1, 1, &ACC_set.CTRL_REG1.all, 1, 10);	//Отправка данных структур (настроек) в память.
    HAL_I2C_Mem_Write(i2c, address, COMMAND_CTRL_REG2, 1, &ACC_set.CTRL_REG2.all, 1, 10);
    HAL_I2C_Mem_Write(i2c, address, COMMAND_CTRL_REG4, 1, &ACC_set.CTRL_REG4.all, 1, 10);

    HAL_I2C_Mem_Read(i2c, address, COMMAND_CTRL_REG1, 1, &command_arr[0], 1, 50);			//Для проверки, что данные записались верно
    HAL_I2C_Mem_Read(i2c, address, COMMAND_CTRL_REG2, 1, &command_arr[1], 1, 50);
    HAL_I2C_Mem_Read(i2c, address, COMMAND_CTRL_REG4, 1, &command_arr[2], 1, 50);
}


void read_x_axis(I2C_HandleTypeDef *i2c){

    uint8_t data_LO_RX[1], data_HI_RX[1];

    if(ACC_set.STATUS_REG.bit.XOR || ACC_set.STATUS_REG.bit.XDA){

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_X_HI, 1, &data_HI_RX[0], 1, 10);		//Считали данные с регистра и записали в структуру
		OUT.X.bit.HI = data_HI_RX[0] & 0xff;

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_X_LO, 1, &data_LO_RX[0], 1, 10);
		OUT.X.bit.LO = data_LO_RX[0] & 0xff;

		rs.RS_X_axis_data = 1;															//Данные готовы к отправке
	}
}


void read_y_axis(I2C_HandleTypeDef *i2c){

    uint8_t data_LO_RX[1], data_HI_RX[1];

    if(ACC_set.STATUS_REG.bit.YOR || ACC_set.STATUS_REG.bit.YDA){

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_Y_HI, 1, &data_HI_RX[0], 1, 10);
		OUT.Y.bit.HI = data_HI_RX[0] & 0xff;

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_Y_LO, 1, &data_LO_RX[0], 1, 10);
		OUT.Y.bit.LO = data_LO_RX[0] & 0xff;

	    rs.RS_Y_axis_data = 1;
    }

}


void read_z_axis(I2C_HandleTypeDef *i2c){

    uint8_t data_LO_RX[1], data_HI_RX[1];
    if(ACC_set.STATUS_REG.bit.ZOR || ACC_set.STATUS_REG.bit.ZDA){

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_Z_HI, 1, &data_HI_RX[0], 1, 10);
		OUT.Z.bit.HI = data_HI_RX[0] & 0xff;

		HAL_I2C_Mem_Read(i2c, ACC_ADDR, COMMAND_Z_LO, 1, &data_LO_RX[0], 1, 10);
		OUT.Z.bit.LO = data_LO_RX[0] & 0xff;

		rs.RS_Z_axis_data = 1;
    }

    if(rs.RS_Z_axis_data && rs.RS_X_axis_data && rs.RS_Y_axis_data)
    	rs.RS_DataReady = 1;

}

