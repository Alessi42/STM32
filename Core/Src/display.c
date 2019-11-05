#include "display.h"

int menuChoice = -1;

void LCDinit(void)
{
	// initialise joystick
	__HAL_RCC_GPIOA_CLK_ENABLE();

	BSP_JOY_Init(JOY_MODE_GPIO);
}

void display(void) {
	 char display[6] = {'T','o','o','B','i','g'};
	 BSP_LCD_GLASS_DisplayString(&display);
}

void ValueDisplay(double value, int choice)
{
	char lcd[6];

	int digit1;
	int digit2;
	int digit3;
	int digit4;


	//too large number
	if(value >= 1500)
	{
		lcd[0] = (uint8_t) ('T');
		lcd[1] = (uint8_t) ('O');
		lcd[2] = (uint8_t) ('O');
		lcd[3] = (uint8_t) ('B');
		lcd[4] = (uint8_t) ('I');
		lcd[5] = (uint8_t) ('G');
		BSP_LCD_GLASS_DisplayString(&lcd);
		return;
	}
	else
	{
		switch(menuChoice)
			{
			case 1:
				digit1 = value / 1000;
				digit2 = (value - digit1 * 1000) / 100;
				digit3 = (value - digit1 * 1000 - digit2 * 100) / 10;
				digit4 = (value - digit1 * 1000 - digit2 * 100 - digit3 * 10) / 1;
				digit1 += 48;
				digit2 += 48;
				digit3 += 48;
				digit4 += 48;
				lcd[0] = (uint8_t) (digit1);
				lcd[1] = (uint8_t) (digit2);
				lcd[2] = (uint8_t) (digit3);
				lcd[3] = (uint8_t) (digit4);
				lcd[4] = (uint8_t) ('H');
				lcd[5] = (uint8_t) ('z');
				break;

			case 2:
				value = value * 2.23694 * 0.0141683;
				digit1 = value / 10;
				digit2 = value - digit1 * 10;
				digit1 += 48;
				digit2 += 48;
				lcd[0] = (uint8_t) (digit1);
				lcd[1] = (uint8_t) (digit2);
				lcd[2] = (uint8_t) ('M');
				lcd[3] = (uint8_t) ('P');
				lcd[4] = (uint8_t) ('H');
				lcd[5] = (uint8_t) (' ');
				break;

			case 3:
				value = value * 3.6 * 0.0141683;
				digit1 = value / 10;
				digit2 = value - digit1 * 10;
				digit1 += 48;
				digit2 += 48;
				lcd[0] = (uint8_t) (digit1);
				lcd[1] = (uint8_t) (digit2);
				lcd[2] = (uint8_t) ('K');
				lcd[3] = (uint8_t) ('M');
				lcd[4] = (uint8_t) ('H');
				lcd[5] = (uint8_t) (' ');
				break;

			default:
				value = 0.0141683 * value;
				digit1 = value / 10;
				digit2 = value - digit1 * 10;
				digit1 += 48;
				digit2 += 48;
				lcd[0] = (uint8_t) (digit1);
				lcd[1] = (uint8_t) (digit2);
				lcd[2] = (uint8_t) ('M');
				lcd[3] = (uint8_t) ('/');
				lcd[4] = (uint8_t) ('S');
				lcd[5] = (uint8_t) (' ');
				break;
			}
	}

	BSP_LCD_GLASS_DisplayString(&lcd);
	//return;
}

int UserInterface(void)
{
	switch(BSP_JOY_GetState())
	{
	case JOY_NONE:

		return -1;
		break;

	case JOY_UP:
		// Display MPH
		menuChoice = 2;
		return 2;
		break;

	case JOY_DOWN:
		// Display M/S
		menuChoice = -1;
		return -1;
		break;

	case JOY_LEFT:
		// Display KMH
		menuChoice = 3;
		return 3;
		break;

	case JOY_RIGHT:
		// Display Hz
		menuChoice = 1;
		return 1;
		break;

//	case JOY_SEL:
//		// Display M/S
//		menuChoice = 0;
//		return 0;
//		break;
	}
}

