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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "lwip.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "udp_client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_BUFFER_LENGTH 500
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned long timestamp;
uint16_t encoder;
uint8_t adc_buffer[ADC_BUFFER_LENGTH];
const unsigned char marker[10] = {0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01};
unsigned char timestamp_char[4];
unsigned char encoder_char[2];
unsigned char information_byte = 0x81;
unsigned char adc_char[2*ADC_BUFFER_LENGTH];
unsigned char crc_input[4+1+2+2*ADC_BUFFER_LENGTH];
unsigned char crc_char[4];
unsigned char message_buff[10+4+1+2+2*ADC_BUFFER_LENGTH+4];
uint32_t crc_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vprint(const char *fmt, va_list argp);
void serial_printf(const char *fmt, ...);
uint16_t getAMTPos();
unsigned long changed_endian_4Bytes(unsigned long num);
int16_t changed_endian_2Bytes(int16_t value);
uint32_t rc_crc32(uint32_t crc, unsigned char *buf, size_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i;
	for (i = 0; i < ADC_BUFFER_LENGTH; i++){
		adc_char[2*i+1] = 0x00;
	}
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
  MX_LWIP_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  udp_client_init(); // initialize udp client
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MX_LWIP_Process();

	  /* Get Time Stamp */
	  timestamp = HAL_GetTick();
	  //serial_printf("Timestamp %d\r\n ", timestamp);
	  //serial_printf("Size of timestamp %d\r\n", sizeof(timestamp));
	  timestamp = changed_endian_4Bytes(timestamp);
	  memcpy(timestamp_char, (unsigned char *)&timestamp, sizeof(timestamp_char));

	  /* Use PA15 as TX Pin */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

	  /* Start ADC Sampling */
	  for (int i = 0; i < ADC_BUFFER_LENGTH; i++){
		  HAL_ADC_PollForConversion(&hadc1, 10);
		  adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
	  }

	  /* Read Encoder */
	  encoder = getAMTPos();
	  //serial_printf("Encoder Reading: %d\r\n", encoder);
	  encoder = changed_endian_2Bytes(encoder);
	  memcpy(encoder_char, &encoder, sizeof(encoder_char));

	  /* Pad ADC Data */
	  for (i = 0; i < ADC_BUFFER_LENGTH; i++){
		  uint8_t temp = adc_buffer[i];
		  memcpy(adc_char+i*2, &temp, sizeof(temp));
	  }

	  /* combine input for crc */
	  memcpy(crc_input, timestamp_char, sizeof(timestamp_char));
	  memcpy(crc_input+sizeof(timestamp_char), &information_byte, sizeof(information_byte));
	  memcpy(crc_input+sizeof(timestamp_char)+sizeof(information_byte), encoder_char, sizeof(encoder_char));
	  memcpy(crc_input+sizeof(timestamp_char)+sizeof(information_byte)+sizeof(encoder_char), adc_char, sizeof(adc_char));

	  /* calculate crc 32 */
	  crc_result = rc_crc32(0, crc_input, sizeof(crc_input));
	  crc_result = changed_endian_4Bytes(crc_result);
	  memcpy(crc_char, (unsigned char *)&crc_result, sizeof(crc_char));

	  /* combine everything into message */
	  memcpy(message_buff, marker, sizeof(marker));
	  memcpy(message_buff+sizeof(marker), crc_input, sizeof(crc_input));
	  memcpy(message_buff+sizeof(marker)+sizeof(crc_input), crc_char, sizeof(crc_char));

	  /* send udp pack */
	  char *me = "Hello World";
	  udp_client_send(me);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void serial_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

uint16_t getAMTPos() {
	//var to help track status and commands
	uint16_t abs_position = 0;
	uint8_t temp[2];
	uint8_t rx_buff = 0xA5;
	uint8_t READPOS = 0x10;		//read command
	uint8_t NOP = 0x00;				//NOP command

	//send read command
	GPIOA->BSRR = 0x100000U;
	HAL_SPI_TransmitReceive(&hspi1, &READPOS, &rx_buff, 1, 0xFFFF);
	GPIOA->BSRR = 0x0010U;
	HAL_Delay(1);

	//keep read until get 10 (ready)
	while (rx_buff != 0x10) {
		GPIOA->BSRR = 0x100000U;
		HAL_SPI_TransmitReceive(&hspi1, &NOP, &rx_buff, 1, 0xFFFF);
		GPIOA->BSRR = 0x0010U;
		HAL_Delay(1);
	}

	//now is ready: get two bytes that is the 12 bit position
	GPIOA->BSRR = 0x100000U;
	HAL_SPI_TransmitReceive(&hspi1, &NOP, &rx_buff, 1, 0xFFFF);
	GPIOA->BSRR = 0x0010U;
	temp[0] = rx_buff;

	GPIOA->BSRR = 0x100000U;
	HAL_SPI_TransmitReceive(&hspi1, &NOP, &rx_buff, 1, 0xFFFF);
	GPIOA->BSRR = 0x0010U;
	temp[1] = rx_buff;

	temp[0] &= ~0xF0;
	abs_position = temp[0] << 8;
	abs_position += temp[1];

	return abs_position;
}

int16_t changed_endian_2Bytes(int16_t value){
    return ((value >> 8) & 0x00ff) | ((value & 0x00ff) << 8);
}

unsigned long changed_endian_4Bytes(unsigned long num){
    int byte0, byte1, byte2, byte3;
    byte0 = (num & 0x000000FF) >> 0 ;
    byte1 = (num & 0x0000FF00) >> 8 ;
    byte2 = (num & 0x00FF0000) >> 16 ;
    byte3 = (num & 0xFF000000) >> 24 ;
    return((byte0 << 24) | (byte1 << 16) | (byte2 << 8) | (byte3 << 0));
}

uint32_t rc_crc32(uint32_t crc, unsigned char *buf, size_t len){
	static uint32_t table[256];
	static int have_table = 0;
	uint32_t rem;
	uint8_t octet;
	int i, j;
	unsigned char *p, *q;
	/* This check is not thread safe; there is no mutex. */
	if (have_table == 0) {
        //fprintf(stdout, "Table\n");
		/* Calculate CRC table. */
		for (i = 0; i < 256; i++) {
			rem = i;  /* remainder from polynomial division */
			for (j = 0; j < 8; j++) {
				if (rem & 1) {
					rem >>= 1;
					rem ^= 0xedb88320;
				} else
					rem >>= 1;
			}
			table[i] = rem;
		}
		have_table = 1;
	}
	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc >> 8) ^ table[(crc & 0xff) ^ octet];
	}
	return ~crc;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
