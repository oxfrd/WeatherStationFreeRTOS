/*
 * DHT11.h
 *
 *  Created on: Nov 9, 2021
 *      Author: oxford
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "main.h"
#include "dwt_stm32_delay.h"

#define DHT11_PORT GPIOD
#define DHT11_PIN GPIO_PIN_0

#define DELAY2FAULT 1	//generate fault after ... ms



//void delayinginus (uint16_t time);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
void DHT11_TempAndHumidity(uint16_t *Temperature, uint16_t *Humidity);

#endif /* INC_DHT11_H_ */
