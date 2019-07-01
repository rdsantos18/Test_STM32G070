/*
 * misc.c
 *
 *  Created on: 10 de mai de 2019
 *      Author: rinaldo
 */


#include "misc.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;

extern char buffer[];
extern uint8_t enc_rot, enc_detect;
extern uint16_t enc_count, enc_previous;

uint8_t aVal = 0, bVal = 0, CLKLast = 0, bCW = 0, CCW = 0, CLKLast1 = 0;
uint32_t encoderPosCount = 0;

void my_print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc)
{
    if(level >= LV_LOG_LEVEL) {
      //Show the log level if you want
      if(level == LV_LOG_LEVEL_TRACE)  {
         HAL_UART_Transmit(&huart2, (uint8_t*)"\nTrace:", 7, HAL_MAX_DELAY);
      }
      else if(level == LV_LOG_LEVEL_INFO) {
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"\nInfo:", 6, HAL_MAX_DELAY);
      }
      else if(level == LV_LOG_LEVEL_WARN) {
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"\nWarn:", 6, HAL_MAX_DELAY);
      }
      else if(level == LV_LOG_LEVEL_ERROR) {
    	  HAL_UART_Transmit(&huart2, (uint8_t*)"\nERROR:", 7, HAL_MAX_DELAY);
      }
      else {
       	  HAL_UART_Transmit(&huart2, (uint8_t*)"\nERRO:", 6, HAL_MAX_DELAY);
      }

      HAL_UART_Transmit(&huart2, (uint8_t*)dsc, strlen(dsc), HAL_MAX_DELAY);
      //You can write 'file' and 'line' too similary if required.
    }
}

/*
uint16_t Read_Encoder(void)
{
	if(enc_detect) {
		enc_detect = 0;
		if(enc_rot) {
			enc_count++;
			if(enc_count > 10) enc_count = 10;
		}
		else if(!enc_rot) {
			if(enc_count >= 1) enc_count--;
		}
	}
	enc_previous = enc_count;

	// LOG
	sprintf(buffer, "Read-Encoder - Count: %d\n", enc_count);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	return enc_count;
}
*/
uint16_t Read_Encoder_A(void)
{
	/* BEGIN – Code for encoder */
	aVal = HAL_GPIO_ReadPin(ENC_D_GPIO_Port, ENC_D_Pin);
	if (aVal != CLKLast)
	{ // Means the knob is rotating
	  // if the knob is rotating, we need to determine direction
	  // We do that by reading pin B.
//		if(!aVal)
//		{ // aVal is false or 0 then proceed. This prevents double incrementation.
			if (HAL_GPIO_ReadPin(ENC_C_GPIO_Port, ENC_C_Pin) != aVal)
			{ // Means pin A Changed first – We’re Rotating Clockwise
				encoderPosCount++;
				if(encoderPosCount >= 16) encoderPosCount = 16;
				bCW = true;
			}
			else {// Otherwise B changed first and we’re moving CCW
				if(encoderPosCount >= 1) encoderPosCount--;
				bCW = false;
			}
			sprintf(buffer, "Encoder - aVal: %d DT: %d, EncoderPosition: %ld\n", aVal, HAL_GPIO_ReadPin(ENC_C_GPIO_Port, ENC_C_Pin), encoderPosCount);
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//		}
	}
	CLKLast = aVal;
	return aVal;
}

uint16_t Read_Encoder_B(void)
{
	/* BEGIN – Code for encoder */
	bVal = HAL_GPIO_ReadPin(ENC_C_GPIO_Port, ENC_C_Pin);
	if (bVal != CLKLast1)
	{ // Means the knob is rotating
	  // if the knob is rotating, we need to determine direction
	  // We do that by reading pin B.

		if(!bVal)
		{ // aVal is false or 0 then proceed. This prevents double incrementation.
			if (HAL_GPIO_ReadPin(ENC_D_GPIO_Port, ENC_D_Pin) != bVal)
			{ // Means pin A Changed first – We’re Rotating Clockwise
				encoderPosCount++;
				CCW = true;
			}
			else {// Otherwise B changed first and we’re moving CCW
				encoderPosCount--;
				CCW = false;
			}
			sprintf(buffer, "Encoder - aVal: %d DT: %d, EncoderPosition: %ld\n", aVal, HAL_GPIO_ReadPin(ENC_D_GPIO_Port, ENC_D_Pin), encoderPosCount);
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		}
	}
	CLKLast1 = bVal;
	return bVal;
}
