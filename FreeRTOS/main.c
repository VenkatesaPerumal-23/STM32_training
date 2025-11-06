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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include <stdio.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

extern void initialise_monitor_handles(void);

QueueHandle_t xEventQueue;
SemaphoreHandle_t xBinSem;
SemaphoreHandle_t xPrintMutex;
TimerHandle_t xTimer;


static void safe_printf(char * string){

	xSemaphoreTakeRecursive(xPrintMutex,portMAX_DELAY);

	printf("%s\n",string);

	xSemaphoreGiveRecursive(xPrintMutex);

}

static void vTimerCallback(TimerHandle_t xTimer){

    (void)xTimer;

    safe_printf("System alive\n");
}

static void vPeriodicTask(void *pvParameters){

	for(;;){

        safe_printf("Periodic Task running...\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

static void vHandlerTask(void *pvParameters){

	char msg[50];

	for(;;){

		if(xSemaphoreTake(xBinSem,portMAX_DELAY)==pdTRUE){

			if(xQueueReceive(xEventQueue,msg,0)==pdTRUE){

				safe_printf(msg);

				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    char msg[50];

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

    if(GPIO_Pin==GPIO_PIN_9){

        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_RESET){

        	snprintf(msg,sizeof(msg),"Object Detected\n");
        }

        else{

        	snprintf(msg,sizeof(msg),"Object Cleared\n");
        }

        xQueueSendFromISR(xEventQueue,msg,&xHigherPriorityTaskWoken);

        xSemaphoreGiveFromISR(xBinSem,&xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    }
}

int main(void)
{

  initialise_monitor_handles();

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  xEventQueue=xQueueCreate(10,sizeof(char[50]));

  xBinSem=xSemaphoreCreateBinary();

  xPrintMutex=xSemaphoreCreateRecursiveMutex();

  xTimer=xTimerCreate("Status",pdMS_TO_TICKS(5000),pdTRUE,NULL,vTimerCallback);

  safe_printf("=== IR Sensor using FreeRTOS ===\n");

  if(xEventQueue && xBinSem && xPrintMutex){

      xTaskCreate(vHandlerTask, "Handler", 128, NULL, 3, NULL);

      xTaskCreate(vPeriodicTask, "Periodic", 128, NULL, 1, NULL);

      xTimerStart(xTimer, 0);

      vTaskStartScheduler();
  }

  for(;;);
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


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
#ifdef USE_FULL_ASSERT
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
