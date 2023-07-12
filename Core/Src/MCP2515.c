/*
 * MCP2515.c
 *
 *  Created on: Jul 2, 2023
 *      Author: AnastasII
 */


#include "MCP2515.h"
#include "main.h"
#include "ACC_init.h"
#include "string.h"
#include "stdio.h"
#include "RS_Functions.h"

/* Pin 설정에 맞게 수정필요. Modify below items for your SPI configurations */
extern 	SPI_HandleTypeDef       hspi1;
extern	CAN_RxHeaderTypeDef	RxHeader;
extern 	UART_HandleTypeDef 	huart1;
#define SPI_CAN                 &hspi1
#define SPI_TIMEOUT             10

#define MCP2515_CS_HIGH()   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
#define MCP2515_CS_LOW()    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)

/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

extern OUT_DATA	OUT;
extern RS_DATA_STRUCT	rs;
SPI_CONFIG_REG	CANCTRL;

ctrl_status_t		ctrl_status;
ctrl_rx_status_t	re;


/* MCP2515 초기화 */
bool MCP2515_Initialize(void)
{
  MCP2515_CS_HIGH();

  uint8_t loop = 10;

  do {
    /* SPI Ready 확인 */
    if(HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
      return true;

    loop--;
  } while(loop > 0);

  return false;
}

/* MCP2515 를 설정모드로 전환 */

bool MCP2515_SetConfigMode(void)
{
  /* CANCTRL Register Configuration */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);

  uint8_t loop = 10;

  do {
    /* 모드전환 확인 */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x80)
      return true;

    loop--;
  } while(loop > 0);

  return false;
}

/* MCP2515 를 Normal모드로 전환 */


bool MCP2515_SetNormalMode(void)
{
  /* CANCTRL Register Normal 모드 설정 */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);

  uint8_t loop = 10;

  do {
    /* 모드전환 확인 */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x00)
      return true;

    loop--;
  } while(loop > 0);

  return false;
}

/* MCP2515 를 Sleep 모드로 전환 */
bool MCP2515_SetSleepMode(void)
{
  /* CANCTRL Register Sleep 모드 설정 */
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);

  uint8_t loop = 10;

  do {
    /* 모드전환 확인 */
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
      return true;

    loop--;
  } while(loop > 0);

  return false;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(void)
{
  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_RESET);

  MCP2515_CS_HIGH();
}

/* 1바이트 읽기 */
uint8_t MCP2515_ReadByte (uint8_t address)
{
  uint8_t retVal;

  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();

  MCP2515_CS_HIGH();

  return retVal;
}

/* Sequential Bytes 읽기 */
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP2515_CS_LOW();

  SPI_Tx(instruction);
  SPI_RxBuffer(data, length);

  MCP2515_CS_HIGH();
}

/* 1바이트 쓰기 */
void MCP2515_WriteByte(uint8_t address, uint8_t data)
{
  MCP2515_CS_LOW();
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);
  MCP2515_CS_HIGH();
}

/* Sequential Bytes 쓰기 */
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{
  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(startAddress);
  SPI_TxBuffer(data, (endAddress - startAddress + 1));

  MCP2515_CS_HIGH();
}

/* TxBuffer에 Sequential Bytes 쓰기 */
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{
  MCP2515_CS_LOW();

  SPI_Tx(instruction);
  SPI_TxBuffer(idReg, 4);
  SPI_Tx(dlc);
  SPI_TxBuffer(data, dlc);

  MCP2515_CS_HIGH();
}

/* TxBuffer에 1 Bytes 쓰기 */
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP2515_CS_LOW();

  SPI_Tx(instruction);
  SPI_Tx(data);

  MCP2515_CS_HIGH();
}

/* RTS 명령을 통해서 TxBuffer 전송 */
void MCP2515_RequestToSend(uint8_t instruction)
{
  MCP2515_CS_LOW();

  SPI_Tx(instruction);

  MCP2515_CS_HIGH();
}

/* MCP2515 Status 확인 */
uint8_t MCP2515_ReadStatus(void)
{
  uint8_t retVal;

  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_READ_STATUS);
  retVal = SPI_Rx();

  MCP2515_CS_HIGH();

  return retVal;
}

/* MCP2515 RxStatus 레지스터 확인 */
uint8_t MCP2515_GetRxStatus(void)
{
  uint8_t retVal;

  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_RX_STATUS);
  retVal = SPI_Rx();

  MCP2515_CS_HIGH();

  return retVal;
}

/* 레지스터 값 변경 */
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{
  MCP2515_CS_LOW();

  SPI_Tx(MCP2515_BIT_MOD);
  SPI_Tx(address);
  SPI_Tx(mask);
  SPI_Tx(data);

  MCP2515_CS_HIGH();
}

/* SPI Tx Wrapper 함수 */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);
}

/* SPI Tx Wrapper 함수 */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);
}

/* SPI Rx Wrapper 함수 */
static uint8_t SPI_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx Wrapper 함수 */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}


void setting_CNFx(){

/*https://github.com/eziya/STM32_SPI_MCP2515/blob/master/Src/CANSPI.c#L202*/
/*http://microsin.net/adminstuff/hardware/mcp2515-stand-alone-can-controller-with-spi-interface.html*/

 /*
 * * Tq = 2(x+1)/Fosc = (1.11us + 36MHz)/2 - 1 = 0.998 => 1
 * Tbit = SyncSeg + PropSeg + PhSeg1+ PhSeg2 = 16(18)
 * Tbit = 1tq + (7tq + 7tq) + 2tq = 17 => 88.235%
 * BRP = 1
 * SJW = 00 => 1*Tq */

    MCP2515_WriteByte(MCP2515_CNF1, 0x1);		//Настройка тактирования
    MCP2515_WriteByte(MCP2515_CNF2, 0xFF);
    MCP2515_WriteByte(MCP2515_CNF3, 0x82);

    MCP2515_WriteByte(MCP2515_TXB0CTRL, 0x0);		//Настройка состояния, приоритетности поступаемых сообщений в буфер
    MCP2515_WriteByte(MCP2515_TXB1CTRL, 0x0);
    MCP2515_WriteByte(MCP2515_TXB2CTRL, 0x0);

    MCP2515_WriteByte(MCP2515_RXB0CTRL, 0x0);
    MCP2515_WriteByte(MCP2515_RXB1CTRL, 0x0);

    MCP2515_WriteByte(MCP2515_CANINTE, 0x0);
    MCP2515_WriteByte(MCP2515_CANINTF, 0x0);

    RXF0 RXF0reg;
    RXF1 RXF1reg;
    RXF2 RXF2reg;
    RXF3 RXF3reg;
    RXF4 RXF4reg;
    RXF5 RXF5reg;
    RXM0 RXM0reg;
    RXM1 RXM1reg;

    /* Intialize Rx Mask values */
	RXM0reg.RXM0SIDH = 0xF;
	RXM0reg.RXM0SIDL = 0xE3;
	RXM0reg.RXM0EID8 = 0xf;
	RXM0reg.RXM0EID0 = 0xf;

	RXM1reg.RXM1SIDH = 0xF;
	RXM1reg.RXM1SIDL = 0xE3;
	RXM1reg.RXM1EID8 = 0xf;
	RXM1reg.RXM1EID0 = 0xf;

	/* Intialize Rx Filter values */
	RXF0reg.RXF0SIDH = 0xF;
	RXF0reg.RXF0SIDL = 0xE0;      //Starndard Filter
	RXF0reg.RXF0EID8 = 0xF;
	RXF0reg.RXF0EID0 = 0xF;

	RXF1reg.RXF1SIDH = 0xF;
	RXF1reg.RXF1SIDL = 0xE0;      //Exntended Filter
	RXF1reg.RXF1EID8 = 0xF;
	RXF1reg.RXF1EID0 = 0xF;

	RXF2reg.RXF2SIDH = 0xF;
	RXF2reg.RXF2SIDL = 0xE0;
	RXF2reg.RXF2EID8 = 0xF;
	RXF2reg.RXF2EID0 = 0xF;

	RXF3reg.RXF3SIDH = 0xF;
	RXF3reg.RXF3SIDL = 0xE0;
	RXF3reg.RXF3EID8 = 0xF;
	RXF3reg.RXF3EID0 = 0xF;

	RXF4reg.RXF4SIDH = 0xF;
	RXF4reg.RXF4SIDL = 0xE0;
	RXF4reg.RXF4EID8 = 0xF;
	RXF4reg.RXF4EID0 = 0xF;

	RXF5reg.RXF5SIDH = 0xF;
	RXF5reg.RXF5SIDL = 0xE0;		//08
	RXF5reg.RXF5EID8 = 0xF;
	RXF5reg.RXF5EID0 = 0xF;

    MCP2515_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
    MCP2515_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));


/*Setting normal mode*/
    while(MCP2515_SetNormalMode() != true);
}

void MCP_settings(){

    HAL_SPI_StateTypeDef	result;

    result = HAL_SPI_GetState(&hspi1);
     if(result == HAL_SPI_STATE_READY)
         while(MCP2515_SetConfigMode() != true);

    setting_CNFx();
    GPIOC->BSRR |= GPIO_BSRR_BS13;
    HAL_Delay(1000);
    GPIOC->BSRR |= GPIO_BSRR_BR13;
    HAL_Delay(1000);
    GPIOC->BSRR |= GPIO_BSRR_BS13;
}


void SPI_Send(){
    uint8_t	res;
    uCAN_MSG	struct_of_msg;

    ctrl_status.ctrl_status = MCP2515_ReadStatus();

    if(ctrl_status.TXB0REQ != 1){

	res = HAL_SPI_GetState(&hspi1);
	if(res == HAL_SPI_STATE_READY){

	    struct_of_msg.frame.idType = 0x0;
	    struct_of_msg.frame.id = 0x66 << 4;
	    struct_of_msg.frame.dlc = 0x06;
	    struct_of_msg.frame.data0 = OUT.X.bit.LO;
	    struct_of_msg.frame.data1 = OUT.X.bit.HI;
	    struct_of_msg.frame.data2 = OUT.Y.bit.LO;
	    struct_of_msg.frame.data3 = OUT.Y.bit.HI;
	    struct_of_msg.frame.data4 = OUT.Z.bit.LO;
	    struct_of_msg.frame.data5 = OUT.Z.bit.HI;

	    MCP2515_CS_LOW();
	    SPI_Tx(MCP2515_LOAD_TXB0SIDH);
	    SPI_TxBuffer( &(struct_of_msg.frame.idType), 4);
	    SPI_Tx(struct_of_msg.frame.dlc);
	    SPI_TxBuffer( &(struct_of_msg.frame.data0), struct_of_msg.frame.dlc);
	    MCP2515_CS_HIGH();

	    MCP2515_RequestToSend(MCP2515_RTS_TX0);
	}
    }
}


void CAN_Recieve(CAN_HandleTypeDef *hcan){
	uint8_t			RX_mailbox[6];
	int16_t			axis_x_data[1], axis_y_data[1], axis_z_data[1];
	char 			buffer[50];
	HAL_StatusTypeDef	status;

	re.ctrl_rx_status = MCP2515_GetRxStatus();

	status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RX_mailbox);
	if(status == HAL_OK){
		axis_x_data[0] = (RX_mailbox[0]<<8) | RX_mailbox[1];
		axis_y_data[0] = (RX_mailbox[2]<<8) | RX_mailbox[3];
		axis_z_data[0] = (RX_mailbox[4]<<8) | RX_mailbox[5];

		sprintf(buffer, "X_axis: %d\tY_axis: %d\tZ_axis: %d\r\n", axis_x_data[0], axis_y_data[0], axis_z_data[0]);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 10);

		rs.RS_X_axis_data = 0;
		rs.RS_Y_axis_data = 0;
		rs.RS_Z_axis_data = 0;

		rs.RS_DataSended = 1;
		rs.RS_DataReady = 0;

	    }
	else
	    rs.RS_DataSended = 0;
}
