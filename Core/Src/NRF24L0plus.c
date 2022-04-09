/*
 * NRF24L0plus.c
 *
 *  Created on: Apr 8, 2022
 *      Author: oxford
 */

#include "NRF24L0plus.h"

#include "FreeRTOS.h"

void nrf24_CE_enable()
{
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port,NRF24_CE_Pin, GPIO_PIN_SET);
}


void nrf24_CE_disable()
{
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port,NRF24_CE_Pin, GPIO_PIN_RESET);
}

static void CS_ON(void)
{
	HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin,GPIO_PIN_SET);
}
static void CS_OFF(void)
{
	HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin,GPIO_PIN_RESET);
}



HAL_StatusTypeDef nrf24_WriteReg (uint8_t regAdr, uint8_t data)
{

	HAL_StatusTypeDef errorCode = HAL_OK;
	uint8_t buffer[2];
	buffer[0] = regAdr | (1<<5);
	buffer[1] = data;
	CS_ON();
	errorCode = HAL_SPI_Transmit(NRF24_SPI, buffer, 2, 1000);
	CS_OFF();
return	errorCode;
}







HAL_StatusTypeDef nrf24_WriteMultileReg (uint8_t regAdr, uint8_t *data, uint8_t size)
{
	HAL_StatusTypeDef errorCode = HAL_OK;
	uint8_t buffer[2];
	buffer[0] = regAdr | (1<<5);

	CS_ON();
	errorCode = HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);
	CS_OFF();
return	errorCode;
}







HAL_StatusTypeDef nrf24_ReadReg (uint8_t regAdr, uint8_t *received)
{
	HAL_StatusTypeDef errorCode = HAL_OK;

	CS_ON();
	errorCode = HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)regAdr,
						1, 1000);
	if(errorCode != HAL_OK)
	{
		CS_OFF();
		return errorCode;
	}
	errorCode = HAL_SPI_Receive(NRF24_SPI, (uint8_t *)received,
						1, 1000);
	CS_OFF();

return	errorCode;
}






uint8_t nrf24_GetStatus(void)
{
	uint8_t status=0;

	if(HAL_OK !=nrf24_ReadReg(NOP, &status))
	{
		return 0b10000000;
	}
	return status;
}




