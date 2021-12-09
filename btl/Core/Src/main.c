/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "printf.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	ENCODER_MODE_X1 = 1,
	ENCODER_MODE_X4 = 4
}
encoderMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_TIME 1.0 // PID sample time in ms
#define PWM_PERIOD	1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t rx_buf[64];
volatile uint8_t tx_buf[16];

PidType pid;
volatile encoderMode enc_mode = ENCODER_MODE_X4;
volatile int32_t enc_pulse = 0;
volatile float Kp = 60, Ki = 10, Kd = 0.7;
volatile float setpoint = 0, input = 0, output = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_duty_cycle(int32_t dutyCycle);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)rx_buf, sizeof(rx_buf));

  PID_Init(&pid,
		  (float *)&input, (float *)&output, (float *)&setpoint,
		  (float)Kp, (float)Ki, (float)Kd,
		  PID_Direction_Direct);
  PID_SetSampleTime(&pid, SAMPLE_TIME);
  PID_SetOutputLimits(&pid, -PWM_PERIOD, PWM_PERIOD);

  TIM3->CNT = 32768;
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
		uint32_t i = 0;
		while(i < Size)
		{
			uint8_t cmdByte = rx_buf[i];
			i += 2;

			float param = 0;
			float sign = 1;
			float factor = 1;
			bool point_found = false;

			if(rx_buf[i] == '-')
			{
				sign = -1;
				i++;
			}
			while(i < Size)
			{
				if(rx_buf[i] == '.')
				{
					point_found = true;
					i++;
				}
				if(rx_buf[i] >= '0' && rx_buf[i] <= '9')
					param = param * 10 + (float)(rx_buf[i] - '0');
				else
					break;
				if(point_found)
					factor *= 10;
				i++;
			}
			param = sign * param / factor;

			encoderMode tmp_mode = (encoderMode)param;
			switch(cmdByte)
			{
				case 'P':
					Kp = param;
					break;
				case 'I':
					Ki = param;
					break;
				case 'D':
					Kd = param;
					break;
				case 'S':
					if(enc_mode == ENCODER_MODE_X1)
						setpoint = param * 4;
					else if(enc_mode == ENCODER_MODE_X4)
						setpoint = param;
					break;
				case 'M':
					if(tmp_mode != enc_mode && (tmp_mode == ENCODER_MODE_X1 || tmp_mode == ENCODER_MODE_X4))
					{
//						if(tmp_mode == ENCODER_MODE_X1)
//						{
////							HAL_TIM_Encoder_DeInit(&htim3);
////
////							GPIO_InitStruct.Pin = GPIO_PIN_4;
////							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
////							GPIO_InitStruct.Pull = GPIO_NOPULL;
////							HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
////							HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
////							HAL_NVIC_EnableIRQ(EXTI4_IRQn);
////
////							GPIO_InitStruct.Pin = GPIO_PIN_5;
////							GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////							GPIO_InitStruct.Pull = GPIO_NOPULL;
////							HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//							Kp *= 4;
//							Ki *= 4;
//							Kd *= 4;
//						}
//						else if(tmp_mode == ENCODER_MODE_X4)
//						{
//							//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
//							//MX_TIM3_Init();
//							//TIM3->CNT = 32768;
//							//HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//
//							Kp /= 4;
//							Ki /= 4;
//							Kd /= 4;
//						}
//						else
//							break;
						enc_mode = tmp_mode;
						enc_pulse = 0;
						setpoint = 0;
					}
					break;
				default:
					break;
			}
			while(rx_buf[i] == '\r' || rx_buf[i] == '\n')
				i++;
		}

		PID_SetTunings(&pid, Kp, Ki, Kd);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)rx_buf, sizeof(rx_buf));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		enc_pulse += ((int32_t)TIM3->CNT - 32768);
		TIM3->CNT = 32768;

		input = (float)enc_pulse;
		PID_Compute(&pid);
		set_duty_cycle((int32_t)output);

		int32_t tmp_pulse = enc_mode == ENCODER_MODE_X4 ? enc_pulse : enc_pulse / 4;

		uint16_t len = snprintf((char *)tx_buf, sizeof(tx_buf), "%d\r\n", (int)tmp_pulse);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf, len);
	}
}

void set_duty_cycle(int32_t dutyCycle)
{
	bool dir = dutyCycle > 0 ? 1 : 0;
	dutyCycle = abs(dutyCycle);

	if(dir == 1)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_PERIOD - (uint32_t)dutyCycle);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM_PERIOD);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_PERIOD);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM_PERIOD - (uint32_t)dutyCycle);
	}
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
