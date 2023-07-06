/*
 * ACC_init.h
 *
 *  Created on: Jun 28, 2023
 *      Author: Sokolov EvgenII
 */

#ifndef INC_ACC_INIT_H_
#define INC_ACC_INIT_H_

#include "main.h"

typedef union{
	uint8_t all;
	struct{
		uint8_t	Xen : 1;		//Вкл. оси Х
		uint8_t	Yen : 1;		//Вкл. оси У
		uint8_t	Zen : 1;		//Вкл. оси Z
		uint8_t	DR_0 : 1;		//Скорость передачи данных
		uint8_t	DR_1 : 1;		//и частота среза низких частот
		uint8_t	PM_0 : 1;		//Режим питания и работы
		uint8_t	PM_1 : 1;		// 001 - normal mode
		uint8_t	PM_2 : 1;
	}bit;
}CONTROL_REG_POWER;

typedef union{
	uint8_t all;
	struct{
		uint8_t	HPCF0 : 1;		//2 бита настраивают частоту среза...
		uint8_t	HPCF1 : 1;		//...фильтра высоких частот
		uint8_t	HPen1 : 1;		//Фильтр высоких частот по прерыванию 1. Вкл. фильтра
		uint8_t	HPen2 : 1;		//Фильтр высоких частот по прерыванию 2. Вкл. фильтра
		uint8_t	FDS  : 1;		//Выбор фильтрованных данных.
		uint8_t	HPM0 : 1;		//Биты для конфигурации режима фильтра..
		uint8_t	HPM1 : 1;		//...высоких частот
		uint8_t	BOOT : 1;		//Бит для перезагрузки содержимого в памяти
	}bit;
}CONTROL_REG_FILTERS;

typedef union{
	uint8_t all;
	struct{
		uint8_t	SIM : 1;		//Бит для SPI
		uint8_t	ST : 1;			//Бит включения самопроверки
		uint8_t	reserv : 1;		//Пустышка
		uint8_t	STsign : 1;		//Бит самодиагностики
		uint8_t	FS0 : 1;		//Диапозон измерений 00 = 2g, 01 = 4g
		uint8_t	FS1: 1;			//11 = 9g
		uint8_t	BLE : 1;		//??
		uint8_t	BDU : 1;		//Обновление данных блока. 1 - пока регистры не буду прочетны/ 0 - данные постоянно обновляются
	}bit;
}CONTROL_REG_UPDATE;

typedef union{
	uint8_t all;
	struct{
		uint8_t	XDA : 1;		//Статус о доступе новых данных по оси X
		uint8_t	YDA : 1;		//Статус о доступе новых данных по оси Y
		uint8_t	ZDA : 1;		//Статус о доступе новых данных по оси Z
		uint8_t	ZYXDA : 1;		//Пришли новые данные по осям
		uint8_t	XOR : 1;		//Бит говорящий о том, что новые данные записались поверх старых
		uint8_t	YOR: 1;			//0 - перезаписи небыло, 1 - новые данные записались поверх\вместо старых
		uint8_t	ZOR : 1;		//см. выше
		uint8_t	ZYXOR : 1;		//если 1 - Новые данные по осям были перезаписаны поверх старых. 0 - перезаписи небыло.
	}bit;
}STATUS_REG;

typedef struct{

	CONTROL_REG_POWER	CTRL_REG1;
	CONTROL_REG_FILTERS	CTRL_REG2;
	CONTROL_REG_UPDATE	CTRL_REG4;
	STATUS_REG			STATUS_REG;

}ACC_SETTING;

#define ACC_SETTING_DEFAULT		{{0x37}, {0x0}, {0x80}, {0x0}}// 1 - 0x3f/37, 3 - 0x30/90




typedef union{

	uint16_t all;
	struct{
		uint8_t LO	:8;
		uint8_t HI	:8;
	}bit;
}X_AXIS;

typedef union{

	uint16_t all;
	struct{
		uint8_t LO	:8;
		uint8_t HI	:8;
	}bit;
}Y_AXIS;

typedef union{

	uint16_t all;
	struct{
		uint8_t LO	:8;
		uint8_t HI	:8;
	}bit;
}Z_AXIS;

typedef struct{

    X_AXIS X;
    Y_AXIS Y;
    Z_AXIS Z;

}OUT_DATA;



#define OUT_DATA_XYZ_DEFAULT		{{0}, {0}, {0}}

void ACC_init(I2C_HandleTypeDef *i2c);
void ACC_init_addr (uint8_t address, I2C_HandleTypeDef *i2c);
void ACC_setting(uint8_t address, I2C_HandleTypeDef *i2c);
void update_ACC_data(I2C_HandleTypeDef *i2c);
void read_x_axis(I2C_HandleTypeDef *i2c);
void read_y_axis(I2C_HandleTypeDef *i2c);
void read_z_axis(I2C_HandleTypeDef *i2c);



#endif /* INC_ACC_INIT_H_ */
