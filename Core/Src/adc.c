/*
 * adc.c
 *
 *  Created on: 7 de mai de 2019
 *      Author: rinaldo
 */

#include "adc.h"

extern ADC_HandleTypeDef hadc1;

uint32_t adc_ch0, adc_ch1;

void Read_ADC(void)
{
	uint8_t x;
	uint32_t ch1, ch2;

	// Zera Variaveis
	ch1 = 0;
	ch2 = 0;

	for(x = 0; x < NUM_AMOSTRA; x++) {
		// Start ADC
		HAL_ADC_Start(&hadc1);
		// Leitura Channel 0
		HAL_ADC_PollForConversion(&hadc1, 100);
		ch1 += HAL_ADC_GetValue(&hadc1);
		// Leitura Channel 1
		HAL_ADC_PollForConversion(&hadc1, 100);
		ch2 += HAL_ADC_GetValue(&hadc1);

		// Stop ADC
		HAL_ADC_Stop (&hadc1);
	}
		// Tira Média de 16 Leituras
		// Disable INT´s
		__disable_irq();
		adc_ch0 = (ch1 / NUM_AMOSTRA);				// CH0
		adc_ch1 = (ch2 / NUM_AMOSTRA);				// CH1

		// Enable INT´s
		__enable_irq();
}
