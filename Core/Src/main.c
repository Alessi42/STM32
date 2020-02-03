/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "usb_host.h"
#include "display.h"
#include "adc.h"
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// ADC handler that gets setup in main
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

LCD_HandleTypeDef hlcd;

QSPI_HandleTypeDef hqspi;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

TIM_HandleTypeDef Timer4Handle;

/* USER CODE BEGIN PV */
//#define MAX_FREQ 50.0
#define TRANSMIT_BUFFER_UART 0
#define TRANSMIT_BDC_VALUES 1

#define SAMPLE_RATE 1000.0 //(2.0 * MAX_FREQ)
#define NUM_OF_FFT_SAMPLES 512
#define FFT_SIZE (NUM_OF_FFT_SAMPLES / 2)

// Current ADC value being read
int g_ADCValue = 0;

// which buffer we're currently writing to
_Bool activeBuffer = 0;
_Bool bufferReady = 0;
_Bool bufferReading = 0;

// Double buffers the length of the fourier transform length
uint32_t ADCBuffer0[NUM_OF_FFT_SAMPLES];
uint32_t ADCBuffer1[NUM_OF_FFT_SAMPLES];

arm_cfft_radix4_instance_f32 S; /* ARM CFFT module */

char snum[10*NUM_OF_FFT_SAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LCD_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void addToDoubleBuffer(uint32_t value) {
	static int bufferIndex = 0;
	// setup a double buffer so that values do not get overwritten and are continuous
	// could also do a double length buffer and detect which half we're in
	if (!activeBuffer) {
		ADCBuffer0[bufferIndex] = value;
	} else {
		ADCBuffer1[bufferIndex] = value;
	}
	bufferIndex++;
	if (bufferIndex > NUM_OF_FFT_SAMPLES) {
		// reset the index to write from the start and change to the next buffer

		bufferIndex = 0;
		if (!bufferReading) {
			// currently not reading from buffer so rewrite other buffer
			bufferReady = 1;
			activeBuffer = !activeBuffer;
		}

	}
}

void TIM4_IRQHandler(void) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

	// read adc value
	g_ADCValue = readADC();

	// add value to double buffer
	addToDoubleBuffer(g_ADCValue);

	// clear the flag for resetting the timer
	__HAL_TIM_CLEAR_FLAG(&Timer4Handle, TIM_FLAG_UPDATE);
}


//void timer() {
//	// fires every second
//	int changeCounter;
//
//	frequency = changeCounter / period;
//
//	// reset change counter
//	changeCounter = 0;
//}
//
//void pinInterrupt() {
//	// fires every time the pin changes from high to low
//	changeCounter++;
//}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	//MX_I2C1_Init();
	//MX_I2C2_Init();
	MX_LCD_Init();
	//MX_QUADSPI_Init();
	//MX_SAI1_Init();
	//MX_SPI2_Init();
	MX_USART1_UART_Init();
	//MX_USB_HOST_Init();
	ADC_Init();
	MX_TIM4_Init();

	/* USER CODE BEGIN 2 */

	BSP_LCD_GLASS_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	float hertz = 0;

	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	initFFT();

	long start_tick, duration;

	while (1) {
		/* USER CODE END WHILE */
		start_tick = HAL_GetTick();

		//createData(&Input);


		if (bufferReady) {
			//hertz = getPeakFrequency();
			// set the buffer ready flag to false
			bufferReady = 0;


			// create float buffer from adc buffer
			// this will then be passed into the fft
			bufferReading = 1;
			if (activeBuffer) {
				createFFTBuffer(&ADCBuffer0);
			} else {
				createFFTBuffer(&ADCBuffer1);
			}
			bufferReading = 0;


			// transmit the input buffer over UART
			// must be done before it is passed through the fft
			if (TRANSMIT_BUFFER_UART) {
				transmitInputBuffer();
			}

			// calculate the fft of the buffer

			/* Process the data through the CFFT/CIFFT module */
			getMaxHertz(&hertz);

			if (TRANSMIT_BUFFER_UART) {
				transmitOutputBuffer();
			}

			if (TRANSMIT_BDC_VALUES) {
				// send value to display board
				TransmitValues(0xd,0xc);
			}

		}

		UserInterface();

		//duration = HAL_GetTick() - start_tick;
		ValueDisplay(hertz, 1);

		//displayADC();

		//TransmitBuffer(&Output, FFT_SIZE, 'o');
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}



float ADCToVoltage(int ADCValue) {
	return 3.3 * (float) ADCValue / 4096.0;
}


void TransmitValues(uint8_t MSDigit, uint8_t LSDigit) {
	// send two 4-bit binary coded decimal numbers with the most
	//significant BCD digit being transmitted in the most significant bits of the serial data.
	uint8_t sendValue = (MSDigit << 4)|LSDigit;

	HAL_UART_Transmit(&huart1, &sendValue, sizeof(sendValue), HAL_MAX_DELAY); //HAL_MAX_DELAY
}

void transmitBufferADC(uint32_t *buffer, int length, char type) {
	static int write = 0;
	int i = 0;

	char *pos = snum;

	memset(snum, 0, sizeof(snum));

	pos += sprintf(pos, "%c\n\r", type);

	for(i=0;i<length;i++) {
		// convert 123 to string [buf]
		pos += sprintf(pos, "%d,",buffer[i]);
	}
	write = 0;
	pos += sprintf(pos, "\n");
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, snum, sizeof(snum), HAL_MAX_DELAY); //HAL_MAX_DELAY
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
}

void TransmitBuffer(float32_t *buffer, int length, char type) {
	static int write = 0;
	int increment = 1;
	int i = 0;

	char *pos = snum;

	char formatString[] = "%f,";

	memset(snum, 0, sizeof(snum));

	pos += sprintf(pos, "%c\n\r", type);

	if (type =='i') {
		formatString[1] = 'd';
	}

	for(i=0;i<length;i+=increment) {
			// convert 123 to string [buf]
		pos += sprintf(pos, formatString ,buffer[i]);
	}
	write = 0;
	pos += sprintf(pos, "\n");
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, snum, sizeof(snum), HAL_MAX_DELAY); //HAL_MAX_DELAY
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
}

void TransmitUART() {
	int num = g_ADCValue;
	char snum[5] = "    ,";
	// convert 123 to string [buf]
	itoa(num, snum, 10);

	// print with end line
//	snum[3] = '\n';
//	snum[4] = '\r';
	// print comma separated
	snum[4] = '\n';

	HAL_UART_Transmit(&huart1, snum, sizeof(snum), HAL_MAX_DELAY);
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_I2C1
			| RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK
			| RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
	Timer4Handle.Instance = TIM4;
	Timer4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer4Handle.Init.ClockDivision = 0;
	Timer4Handle.Init.Prescaler = 1; // should be 1 for normal operation
	Timer4Handle.Init.Period = 4000 / 0.8018; // 4Mhz/SAMPLING_RATE = 4000000/1000
	__HAL_RCC_TIM4_CLK_ENABLE();
	HAL_TIM_Base_Init(&Timer4Handle);
	HAL_TIM_Base_Start_IT(&Timer4Handle);
	HAL_NVIC_SetPriority(TIM4_IRQn, 7, 0); // middle priority
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

}

/**
 * @brief LCD Initialization Function
 * @param None
 * @retval None
 */
static void MX_LCD_Init(void) {

	/* USER CODE BEGIN LCD_Init 0 */

	/* USER CODE END LCD_Init 0 */

	/* USER CODE BEGIN LCD_Init 1 */

	/* USER CODE END LCD_Init 1 */
	hlcd.Instance = LCD;
	hlcd.Init.Prescaler = LCD_PRESCALER_1;
	hlcd.Init.Divider = LCD_DIVIDER_16;
	hlcd.Init.Duty = LCD_DUTY_1_4;
	hlcd.Init.Bias = LCD_BIAS_1_4;
	hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
	hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
	hlcd.Init.DeadTime = LCD_DEADTIME_0;
	hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
	hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
	hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
	hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
	hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
	if (HAL_LCD_Init(&hlcd) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LCD_Init 2 */

	/* USER CODE END LCD_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void) {

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}



/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 57600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, AUDIO_RST_Pin | LD_G_Pin | XL_CS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | LD_R_Pin | M3V3_REG_ON_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_VBUS_GPIO_Port, OTG_FS_VBUS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : AUDIO_RST_Pin */
	GPIO_InitStruct.Pin = AUDIO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_IRQ_OUT_Pin OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin | OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 MAG_INT_Pin MAG_DRDY_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | MAG_INT_Pin | MAG_DRDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
	GPIO_InitStruct.Pin = JOY_LEFT_Pin | JOY_RIGHT_Pin | JOY_UP_Pin
			| JOY_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MFX_WAKEUP_Pin */
	GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 M3V3_REG_ON_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | M3V3_REG_ON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_R_Pin */
	GPIO_InitStruct.Pin = LD_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_G_Pin */
	GPIO_InitStruct.Pin = LD_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin | OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : EXT_RST_Pin GYRO_INT1_Pin */
	GPIO_InitStruct.Pin = EXT_RST_Pin | GYRO_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_CS_Pin */
	GPIO_InitStruct.Pin = GYRO_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_INT2_Pin */
	GPIO_InitStruct.Pin = GYRO_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GYRO_INT2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL_CS_Pin */
	GPIO_InitStruct.Pin = XL_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XL_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL_INT_Pin */
	GPIO_InitStruct.Pin = XL_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(XL_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
