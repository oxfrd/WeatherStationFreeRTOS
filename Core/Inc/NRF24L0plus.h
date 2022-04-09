/*
 * NRF24L0plus.h
 *
 *  Created on: Apr 7, 2022
 *      Author: oxford
 */

#ifndef INC_NRF24L0PLUS_H_
#define INC_NRF24L0PLUS_H_

#include "spi.h"
#include "main.h"

/***********************************************************/
//						EDIT BEFORE USE
/***********************************************************/
#define NRF24_SPI 		&hspi1
/*
#define NRF24_CE_GPIO_Port	//comment it if CE pin is named
#define NRF24_CE_PIN		//identically as here
*/

/**********************************************************/
//						COMMANDS
/**********************************************************/

#define R_REGISTER    	0b00000000
#define W_REGISTER    	0b00100000
#define R_RX_PAYLOAD	0b01100001
#define W_TX_PAYLOAD	0b10100000
#define FLUSH_TX      	0b11100001
#define FLUSH_RX      	0b11100010
#define REUSE_TX_PL   	0b11100011
#define R_RX_PL_WID		0b01100000
#define W_ACK_PAYLOAD 	0b10101000
#define R_TX_PAYLOAD_NOACK 	0b10110000
#define NOP           	0b11111111

/**********************************************************/
//						REGISTERS MAP
/**********************************************************/
#define CONFIG      0b00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

typedef enum CONFIG_BITS{
	PRIM_RX,
	PWR_UP,
	CRCO,
	EN_CRC,
	MASK_MAX_RT,
	MASK_TX_DS,
	MASK_RX_DR
};

typedef enum {
	ENAA_P0,  // R/W Enable auto acknowledgement data pipe 5
	ENAA_P1,  // R/W Enable auto acknowledgement data pipe 4
	ENAA_P2,  // R/W Enable auto acknowledgement data pipe 3
	ENAA_P3,  // R/W Enable auto acknowledgement data pipe 2
	ENAA_P4,  // R/W Enable auto acknowledgement data pipe 1
	ENAA_P5
}EN_AA_BITS;

typedef enum
{
	PIPE0_EN,
	PIPE1_EN,
	PIPE2_EN,
	PIPE3_EN,
	PIPE4_EN,
	PIPE5_EN,
}EN_RXADDR_BITS;

typedef enum
{
	RF_PWR01=1,
	RF_PWR02,
	RF_DR_HIGH,
	PLL_LOCK,
	RF_DR_LOW,
	CONT_WAVE =7
}RF_SETUP_BITS;

typedef enum
{
	TX_FULL_STATUS,	//1: TX FIFO full
	RX_P_NO_01,
	RX_P_NO_02,
	RX_P_NO_03,
	MAX_RT,
	TX_DS,
	RX_DR
}STATUS_BITS;

typedef enum
{
	RX_EMPTY,
	RX_FULL,
	TX_EMPTY =4,
	TX_FULL_FIFO,
	TX_REUSE
}FIFO_STATUS_BITS;

typedef enum
{
	DPL_P0,
	DPL_P1,
	DPL_P2,
	DPL_P3,
	DPL_P4,
	DPL_P5,
}DYNPD_BITS;

typedef enum
{
	EN_DYN_ACK,	//Enables the W_TX_PAYLOAD_NOACK command
	EN_ACK_PAY,	//Enables Payload with ACK
	EN_DPL		//Enables Dynamic Payload Length
}FEATURE_BITS;

void nrf24_CE_enable();
void nrf24_CE_disable();
static void CS_ON(void);
static void CS_OFF(void);
HAL_StatusTypeDef nrf24_WriteReg (uint8_t regAdr, uint8_t data);
HAL_StatusTypeDef nrf24_WriteMultileReg (uint8_t regAdr, uint8_t *data, uint8_t size);
HAL_StatusTypeDef nrf24_ReadReg (uint8_t regAdr, uint8_t *received);
uint8_t nrf24_GetStatus(void);

#endif /* INC_NRF24L0PLUS_H_ */
