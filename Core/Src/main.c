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

#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "programCtrl.h"
#include "scheduler.h"
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
uint8_t rxBuf[RX_BUFFER_SIZE + 1];
uint8_t rxSaveBuf[RX_BUFFER_SIZE];

__IO uint8_t rxBufIdx = 0;
__IO uint8_t rxFlag = 0;
__IO uint8_t rxDone = 0;
__IO uint32_t rxTick = 0;

unsigned short keyValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RGB(uint8_t R, uint8_t G, uint8_t B);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Scheduler_Init();  // initialize scheduler
  HAL_UART_Receive_IT(&huart1, rxBuf, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("System running...\r\n");
  while (1) {
    Scheduler_Run();   // run scheduler
    User_Task_Ctrl();  // run user task
  }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief control RGB led
 * @param uint8_t RGBstat
 * @retval None
 * @note RGBstat = 0x00, RGB off
 **/
void RGB(uint8_t R, uint8_t G, uint8_t B) {
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, !R);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, !G);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, !B);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    rxFlag = 1;
    rxTick = HAL_GetTick();
    (rxBuf + 1)[rxBufIdx++] = rxBuf[0];
    if (rxBufIdx >= RX_BUFFER_SIZE) {
      HAL_UART_Transmit_IT(&huart1, rxBuf + 1, rxBufIdx);
      rxFlag = 0;
      rxBufIdx = 0;
    }
    HAL_UART_Receive_IT(&huart1, rxBuf, 1);
  }
}
/**
 * @brief 串口超时检查
 */
void Uart_Overtime_100Hz(void) {
  if (rxFlag && HAL_GetTick() - rxTick > 10) {
    memcpy(rxSaveBuf, (rxBuf + 1), rxBufIdx);
    rxDone = 1;
    rxFlag = 0;
    rxBufIdx = 0;
  }
  return;
}

/**
 * @brief 测试用，串口遥控器
 */
void Uart_Controller_20Hz(void) {
  static uint8_t controlWord = 0;
  static uint8_t userMode = 0;
  if (rxDone) {
    rxDone = 0;
    controlWord = rxSaveBuf[0];
    switch (userMode) {
      case 0:  //模式切换
        switch (controlWord) {
          case '?':
            printf(
                "\r\n-------菜单-------\r\n1:电机控制\r\n2:PWM控制\r\n3:"
                "按键读取\r\n4:LED控制\r\n5:用户事件控制\r\n");
            break;
          case '1':
            userMode = 1;
            printf(
                "\r\n>>开始电机控制\r\n>>控制对象:MOTOR_1\r\n>>编码器:ENC_"
                "3\r\n");
            HAL_Delay(1000);
            break;
          case '2':
            userMode = 2;
            printf("\r\n>>开始PWM控制\r\n>>控制对象:MOTOR_2 CHAN_1\r\n");
            break;
          case '3':
            userMode = 3;
            printf("\r\n>>开始按键读取\r\n");
            Enable_SchTask(KEY_CHECK_TASK_ID);
            Enable_SchTask(KEY_READ_TASK_ID);
            break;
          case '4':
            userMode = 4;
            printf("\r\n>>开始LED控制\r\n");
            break;
          case '5':
            userMode = 5;
            printf("\r\n>>开始用户事件控制\r\n");
            break;
          default:
            printf("\r\n输入无效\r\n");
            break;
        }
        break;
      case 1:  // TODO:电机控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>电机控制结束\r\n");
            userMode = 0;
            break;
        }
        break;
      case 2:  // TODO: PWM控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>PWM控制结束\r\n");
            userMode = 0;
            break;
        }
        break;
      case 3: 
        if (controlWord == 'e') {
          printf("\r\n>>按键读取结束\r\n");
          Disable_SchTask(KEY_CHECK_TASK_ID);
          Disable_SchTask(KEY_READ_TASK_ID);
          userMode = 0;
          break;
        }
        break;
      case 4:  // LED控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>LED控制结束\r\n");
            userMode = 0;
            break;
          case 's':
            RGB(rxSaveBuf[1] - '0', rxSaveBuf[2] - '0', rxSaveBuf[3] - '0');
            break;
        }
        break;
      case 5:  //用户事件控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>用户事件控制结束\r\n");
            userMode = 0;
            break;
          case '1':
            user_task_ctrl_word.runFlag = 1;
            break;
          case '2':
            user_task_ctrl_word.runFlag = 0;
            break;
          case 'c':
            user_task_ctrl_word.continueFlag = 1;
            break;
          case 'b':
            user_task_ctrl_word.breakFlag = 1;
            break;
          case 'r':
            Reset_User_Task();
          case '?':
            printf("ID:%d RUN:%d DONE:%d \r\n ", user_task_ctrl_word.taskId,
                   user_task_ctrl_word.taskRunning,
                   user_task_ctrl_word.taskListDone);
            if (user_task_ctrl_word.taskListDone)
              user_task_ctrl_word.taskListDone = 0;  //清除标志位
            break;
        }
        break;
    }
  }
  return;
}

/**
 * @brief check keys
 */
void Key_Check_1000Hz(void) {
  key_check_all_loop_1ms();
  return;
}

/**
 * @brief read keys
 */
void Key_Read_100Hz(void) {
  keyValue = key_read_value();
  switch (keyValue) {
    case KEY1_SHORT:
      printf("\r\n>>KEY1 短按\r\n");
      break;
    case KEY1_LONG:
      printf("\r\n>>KEY1 长按\r\n");
      break;
    case KEY1_DOUBLE:
      printf("\r\n>>KEY1 双击\r\n");
      break;
    case KEY2_SHORT:
      printf("\r\n>>KEY2 短按\r\n");
      break;
    case KEY2_LONG:
      printf("\r\n>>KEY2 长按\r\n");
      break;
    case KEY2_DOUBLE:
      printf("\r\n>>KEY2 双击\r\n");
      break;
  }
  return;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
