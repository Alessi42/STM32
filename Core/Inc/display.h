#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32l4xx.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"

/*
#define FREQ 0
#define MPH 1
#define KMH 2
#define MPS 3  */

void LCDinit(void);
void ValueDisplay(double, int);
int UserInterface(void);
void display(void);

#endif

