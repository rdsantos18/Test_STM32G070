/*
 * adc.h
 *
 *  Created on: 7 de mai de 2019
 *      Author: rinaldo
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32g0xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define NUM_AMOSTRA		16

void Read_ADC(void);

#endif /* INC_ADC_H_ */
