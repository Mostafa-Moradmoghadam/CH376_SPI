//POORIA
//http://electronics-tech.mihanblog.com/
#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f1xx_hal.h"

void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#endif





























