#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32l476g_discovery.h"

uint32_t readADC();
void displayADC();
void ADC_Init(void);

#endif
