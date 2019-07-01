/*
 * misc.h
 *
 *  Created on: 10 de mai de 2019
 *      Author: rinaldo
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

#include "stm32g0xx_hal.h"
#include "lvgl/lvgl.h"

void my_print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc);
uint16_t Read_Encoder_A(void);
uint16_t Read_Encoder_B(void);

#endif /* INC_MISC_H_ */
