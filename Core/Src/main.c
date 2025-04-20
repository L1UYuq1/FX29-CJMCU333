/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_CHANNEL_COUNT 6
uint16_t adc_dma_buffer[ADC_CHANNEL_COUNT];
float voltage[ADC_CHANNEL_COUNT];
float kg1[3];
float kg2[3];
float vref[ADC_CHANNEL_COUNT];
char buffer[100];

float filter_alpha = 0.5; // 可以放到循环外
static float filtered_voltage[ADC_CHANNEL_COUNT] = {0}; // 只需声明一次
//串口打印
void send_string(char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_CHANNEL_COUNT);

  // 建议加延时或第一次采样等待
  HAL_Delay(1000);

  for (int j = 0; j < 10; j++) {
      for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
          vref[i] += ((float)adc_dma_buffer[i]) / 4095.0f * 3.3f;
      }
      HAL_Delay(50);
  }

  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
      vref[i] /= 10.0f;
  }


  // 启动 ADC
    //HAL_ADC_Start(&hadc1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



//	  	  	HAL_ADC_Start(&hadc1);  // 每次都重启 ADC（保险）
//	    	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
//	        adc_raw = HAL_ADC_GetValue(&hadc1);
//	       // HAL_ADC_Stop(&hadc1);
//
//
//            voltage = (((float)adc_raw) / 4095.0f )* 3.38f - vref;
//	        HAL_Delay(1);  // 给 ADC 一点稳定时间，可选
//
//
//	        float kg =  voltage/2.0f * 45.36f*3.0f;
//
//
//	        snprintf(buffer, sizeof(buffer), " Voltage = %.5f v   Poids = %.3f kg \r\n",voltage,kg);
//	        send_string(buffer);
//
//	        HAL_Delay(500);  // 每秒2次，防止刷屏太快



	      // 依次获取三个通道的值
	  // ★ 滑动平均滤波处理每路原始ADC数据
	      for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	          float raw_v = ((float)adc_dma_buffer[i]) / 4095.0f * 3.3f;
	          filtered_voltage[i] = filter_alpha * raw_v + (1.0f - filter_alpha) * filtered_voltage[i];
	          voltage[i] = filtered_voltage[i];
	      }

	      // ↓↓↓ 下面继续你原来的 vref 校准、kg 计算和串口输出 ↓↓↓
	      for (int i = 0; i < 3; i++) {
	          float weight1 = (voltage[i] - vref[i]) * 45.36f / 2.0f;
	          float weight2 = (voltage[i+3] - vref[i+3]) * 45.36f / 2.0f;
	          if (weight1 < 0) weight1 = 0;
	          kg1[i] = weight1;
	          if (weight2 < 0) weight2 = 0;
			  kg2[i] = weight2;


	      }

	      for (int i = 0; i < 3; i++) {

	      	          snprintf(buffer, sizeof(buffer), "V[%d]=%.3f V  Poids[%d]=%.3f kg\r\n", i, voltage[i]-vref[i], i, kg1[i]);
	      	          send_string(buffer);
	      	      }
	      for (int i = 0; i < 3; i++) {

	      	      	          snprintf(buffer, sizeof(buffer), "V[%d]=%.3f V  Poids[%d]=%.3f kg\r\n", i+3, voltage[i+3]-vref[i+3], i+3, kg2[i]);
	      	      	          send_string(buffer);
	      	      	      }

	      float total1 = kg1[0] + kg1[1] + kg1[2];
	      float total2 = kg2[0] + kg2[1] + kg2[2];
	      snprintf(buffer, sizeof(buffer), "Poids total1 = %.3f kg\r\n\r\n", total1);
	      send_string(buffer);
	      snprintf(buffer, sizeof(buffer), "Poids total2 = %.3f kg\r\n\r\n", total2);
	      send_string(buffer);

	      HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
