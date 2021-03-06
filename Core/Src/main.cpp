/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "MotorsStruct/MotorsStruct.hpp"
#include <cstring>
#include <atomic>
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

constexpr uint16_t NEUTRAL = 1000 * 6;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
std::atomic<int> TicCount = 0;

enum SPIStatus {
    OFF = 0,
    WORK = 1,
    DONE = 2
} SPIOneStatus = OFF, SPITwoStatus = OFF;


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
        SPIOneStatus = DONE;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
        TicCount += 1;
    }
}

uint16_t CreatePacket(uint16_t gas, bool isNeedTelemetry) {
    uint16_t packet = gas + 47;

    packet <<= 1;

    if (isNeedTelemetry)
        ++packet;

    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    return (packet << 4) | csum;
}

[[noreturn]]
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    uint8_t msg[MotorsStructLenMessage * 2]{};

    HAL_SPI_Receive_IT(&hspi1, msg, MotorsStructLenMessage * 2);

    for (;;) {

        MotorsStruct motorsStructData;

        if (SPIOneStatus != DONE) {
            HAL_SPI_Receive_IT(&hspi1, msg, MotorsStructLenMessage * 2);
        }

        if (SPIOneStatus == DONE) {

            SPIOneStatus = OFF;

            for (size_t i = 0; i < MotorsStructLenMessage; i++) {
                if (msg[i] == 's' && msg[i + 1] == 's' && msg[i + 2] == 's' && msg[i + 3] == 's') {
                    memcpy(&motorsStructData, &msg[i + 4], MotorsStructLen);
                    i = MotorsStructLenMessage * 2;
                }
            }

            TIM1->CCR1 = motorsStructData.PacketArray[0] * 6 + 9000;
            TIM1->CCR2 = motorsStructData.PacketArray[1] * 6 + 9000;
            TIM1->CCR3 = motorsStructData.PacketArray[2] * 6 + 9000;
            TIM1->CCR4 = motorsStructData.PacketArray[3] * 6 + 9000;

            TIM3->CCR1 = motorsStructData.PacketArray[4] * 6 + 9000;
            TIM3->CCR2 = motorsStructData.PacketArray[5] * 6 + 9000;
            TIM3->CCR3 = motorsStructData.PacketArray[6] * 6 + 9000;
            TIM3->CCR4 = motorsStructData.PacketArray[7] * 6 + 9000;

            TIM4->CCR1 = motorsStructData.PacketArray[8] * 6 + 9000;
            TIM4->CCR2 = motorsStructData.PacketArray[9] * 6 + 9000;
            TIM4->CCR3 = motorsStructData.PacketArray[10] * 6 + 9000;
            TIM4->CCR4 = motorsStructData.PacketArray[11] * 6 + 9000;

            TIM2->CCR1 = motorsStructData.PWM[0];
            TIM2->CCR2 = motorsStructData.PWM[1];
            TIM2->CCR3 = motorsStructData.PWM[2];
            TIM2->CCR4 = motorsStructData.PWM[3];

            TicCount = 0;
        }

        if (TicCount == 500) {
            TIM1->CCR1 = NEUTRAL + 9000;
            TIM1->CCR2 = NEUTRAL + 9000;
            TIM1->CCR3 = NEUTRAL + 9000;
            TIM1->CCR4 = NEUTRAL + 9000;

            TIM3->CCR1 = NEUTRAL + 9000;
            TIM3->CCR2 = NEUTRAL + 9000;
            TIM3->CCR3 = NEUTRAL + 9000;
            TIM3->CCR4 = NEUTRAL + 9000;

            TIM4->CCR1 = NEUTRAL + 9000;
            TIM4->CCR2 = NEUTRAL + 9000;
            TIM4->CCR3 = NEUTRAL + 9000;
            TIM4->CCR4 = NEUTRAL + 9000;

            TIM2->CCR1 = 0;
            TIM2->CCR2 = 0;
            TIM2->CCR3 = 0;
            TIM2->CCR4 = 0;
        }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
    while (1) {
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
