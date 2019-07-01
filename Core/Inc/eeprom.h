/*
 * eeprom.h
 *
 *  Created on: May 4, 2019
 *      Author: rinaldo
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

// 	eeprom 24cxxx library

#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

#define		_EEPROM_SIZE_KBIT							256
#define		_EEPROM24XX_I2C								hi2c1
#define		_EEPROM_FREERTOS_IS_ENABLE					0
#define		_EEPROM_USE_WP_PIN							0

#if (_EEPROM_USE_WP_PIN==1)
//#define		_EEPROM_WP_GPIO							EE_WP_GPIO_Port
//#define		_EEPROM_WP_PIN							EE_WP_Pin
#endif


bool		EEPROM24XX_IsConnected(void);
bool		EEPROM24XX_Save(uint16_t Address,void *data,size_t size_of_data);
bool		EEPROM24XX_Read(uint16_t Address,void *data,size_t size_of_data);

#endif /* INC_EEPROM_H_ */
