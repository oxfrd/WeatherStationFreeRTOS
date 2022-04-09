/*
 * DHT11.c
 *
 *  Created on: Nov 9, 2021
 *      Author: oxford
 */
#include "DHT11.h"
#include "cmsis_os.h"


uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

/*float Temperature = 0;
float Humidity = 0;*/
uint8_t echo;

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	DWT_Delay_us(18000);   // wait for 18ms
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	DWT_Delay_us(20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response(void) {
	uint8_t Response = 0;
	uint32_t currentTicks = HAL_GetTick();

	DWT_Delay_us(40);	// wait for 40 us

	if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
		DWT_Delay_us(80);

		if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
			Response = 1;
		else
			Response = -1; // 255
	}

	while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
	{
		if((HAL_GetTick()-currentTicks)>DELAY2FAULT)
			return 0; 	//end process if it last too long
		// wait for the pin to go low
	}


	return Response;
}

uint8_t DHT11_Read(void) {
	uint8_t i, j;
	uint32_t currentTicks = HAL_GetTick();

	for (j = 0; j < 8; j++) {
		while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
		{
			if((HAL_GetTick()-currentTicks)>DELAY2FAULT)
				return 0; 	//end process if it last too long
				// wait for the pin to go high
		}

		DWT_Delay_us(40);  // wait for 40 us
		if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i &= ~(1 << (7 - j));   // write 0
		} else
		{
			i |= (1 << (7 - j));  // if the pin is high, write 1
		}

		while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
		{
			if((HAL_GetTick()-currentTicks)>DELAY2FAULT)
				return 0; 	//end process if it last too long
			// wait for the pin to go low
		}
	}
	return i;
}

void DHT11_TempAndHumidity(uint16_t *Temperature, uint16_t *Humidity)
{
	//DHT11_Start();
	echo = DHT11_Check_Response();
	Rh_byte1 = DHT11_Read();
	Rh_byte2 = DHT11_Read();
	Temp_byte1 = DHT11_Read();
	Temp_byte2 = DHT11_Read();
	SUM = DHT11_Read();

	*Temperature = Temp_byte1;
	*Humidity = Rh_byte1;
}


