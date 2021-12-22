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

#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "pid.h"
#include "programCtrl.h"
#include "scheduler.h"
#include "stdio.h"
#include "string.h"
#include "uartPack.h"
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
uart_o_ctrl_t uart_1;
unsigned short keyValue;
motor_t motor_1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RGB(uint8_t R, uint8_t G, uint8_t B);
void PWM_Cfg_HighDutyAcc(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t freq, float duty);
void PWM_Cfg_HighFreqAcc(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t freq, uint16_t duty);
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
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Scheduler_Init();  // initialize scheduler
  Enable_Uart_O_Control(&huart1, &uart_1);
  Motor_Setup(&motor_1, &htim5, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\r\n--- System running ---\r\n");
  printf("float: %f\r\n", 3.1415926);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
    Uart_O_Data_Process(&huart1, &uart_1);
  }
}

/**
 * @brief 串口超时处理
 */
void Uart_Overtime_100Hz(void) {
  Uart_O_Timeout_Check(&huart1, &uart_1);
  return;
}

/**
 * @brief 测试用，串口控制
 */
void Uart_Controller_20Hz(void) {
  static uint8_t userMode = 0;
  static float goDeg = 0;
  static float setDeg = 0;
  static float duty = 0;
  static float speed = 0;
  static float setPosKp = 0;
  static float setPosKi = 0;
  static float setPosKd = 0;
  static float setSpdKp = 0;
  static float setSpdKi = 0;
  static float setSpdKd = 0;
  uint8_t controlWord = 0;
  if (uart_1.rxSaveFlag) {
    uart_1.rxSaveFlag = 0;
    controlWord = uart_1.rxSaveBuf[0];
    switch (userMode) {
      case 0:  //模式切换
        switch (controlWord) {
          case '?':
            printf(
                "\r\n-------MENU-------\r\n1:Pos + Spd PID Control\r\n2:Spd "
                "PID Control\r\n3:Manual Control\r\n");
            break;
          case '1':
            userMode = 1;
            printf("\r\n---Pos + Spd PID Control---\r\n");
            HAL_Delay(1000);
            __MOTOR_RESET_ENCODER(motor_1);
            HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
            Enable_SchTask(MOTOR_POS_PID_TASK_ID);
            Enable_SchTask(MOTOR_SPD_PID_TASK_ID);
            Enable_SchTask(PARAM_REPORT_TASK_ID);
            break;
          case '2':
            userMode = 2;
            printf("\r\n---Spd PID Control---\r\n");
            HAL_Delay(1000);
            __MOTOR_RESET_ENCODER(motor_1);
            HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
            Enable_SchTask(PARAM_REPORT_TASK_ID);
            Enable_SchTask(MOTOR_SPD_PID_TASK_ID);
            break;
          case '3':
            userMode = 3;
            printf("\r\n---Manual Control---\r\n");
            HAL_Delay(1000);
            __MOTOR_RESET_ENCODER(motor_1);
            Enable_SchTask(PARAM_REPORT_TASK_ID);
            HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
            break;
          default:
            printf("\r\n>>Invalid command\r\n");
            break;
        }
        break;
      case 1:  //位置PID控制
        if (controlWord == 'e') {
          printf("\r\n>>Motor control exit\r\n");
          HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
          Disable_SchTask(MOTOR_POS_PID_TASK_ID);
          Disable_SchTask(MOTOR_SPD_PID_TASK_ID);
          Disable_SchTask(PARAM_REPORT_TASK_ID);
          __MOTOR_PWM_SETZERO(motor_1);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "s:%f", &speed) == 1) {
          motor_1.targetSpeed = speed;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "pp:%f", &setPosKp) == 1) {
          motor_1.posPID.proportion = setPosKp;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "pi:%f", &setPosKi) == 1) {
          motor_1.posPID.integral = setPosKi;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "pd:%f", &setPosKd) == 1) {
          motor_1.posPID.derivative = setPosKd;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "sp:%f", &setSpdKp) == 1) {
          motor_1.spdPID.proportion = setSpdKp;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "si:%f", &setSpdKi) == 1) {
          motor_1.spdPID.integral = setSpdKi;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "sd:%f", &setSpdKd) == 1) {
          motor_1.spdPID.derivative = setSpdKd;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "gdeg:%f", &goDeg) == 1) {
          __MOTOR_GO_DEGREE(motor_1, goDeg);
        } else if (sscanf((char *)uart_1.rxSaveBuf, "sdeg:%f", &setDeg) == 1) {
          __MOTOR_SET_DEGREE(motor_1, setDeg);
        } else {
          printf("\r\n>>Invalid command\r\n");
        }
        break;
      case 2:  //转速PID控制
        if (controlWord == 'e') {
          printf("\r\n>>Motor control exit\r\n");
          HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
          Disable_SchTask(MOTOR_SPD_PID_TASK_ID);
          Disable_SchTask(PARAM_REPORT_TASK_ID);
          __MOTOR_PWM_SETZERO(motor_1);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "s:%f", &speed) == 1) {
          __MOTOR_SET_SPEED(motor_1, speed);
        } else if (sscanf((char *)uart_1.rxSaveBuf, "sp:%f", &setSpdKp) == 1) {
          motor_1.spdPID.proportion = setSpdKp;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "si:%f", &setSpdKi) == 1) {
          motor_1.spdPID.integral = setSpdKi;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "sd:%f", &setSpdKd) == 1) {
          motor_1.spdPID.derivative = setSpdKd;
        } else if (controlWord == '?') {
          printf("PID Param:\r\nSPD:Kp=%.3f,Ki=%.3f,Kd=%.3f\r\n",
                 motor_1.spdPID.proportion, motor_1.spdPID.integral,
                 motor_1.spdPID.derivative);
          HAL_Delay(1000);
        } else {
          printf("\r\n>>Invalid command\r\n");
        }
        break;
      case 3:  //手动控制
        if (controlWord == 'e') {
          printf("\r\n>>Motor control exit\r\n");
          HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
          Disable_SchTask(PARAM_REPORT_TASK_ID);
          __MOTOR_PWM_SETZERO(motor_1);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "d:%f", &duty) == 1) {
          if (duty > 0) {
            __MOTOR_PWM_SETFWD(motor_1, duty);
          } else {
            __MOTOR_PWM_SETREV(motor_1, -duty);
          }
          motor_1.pwmDuty = duty;
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
      printf("\r\n>>KEY1 SHORT\r\n");
      HAL_Delay(50);
      break;
    case KEY1_LONG:
      printf("\r\n>>KEY1 LONG\r\n");
      break;
    case KEY1_DOUBLE:
      printf("\r\n>>KEY1 DOUBLE\r\n");
      break;
    case KEY2_SHORT:
      printf("\r\n>>KEY2 SHORT\r\n");
      break;
    case KEY2_LONG:
      printf("\r\n>>KEY2 LONG\r\n");
      break;
    case KEY2_DOUBLE:
      printf("\r\n>>KEY2 DOUBLE\r\n");
      break;
  }
  return;
}

/**
 * @brief Read ADC value
 */
void ADC_Read_50Hz(void) {
  double calcValue = 0;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 50);  //等待转换完成
  if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
    calcValue = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
  }
  HAL_ADC_Stop(&hadc1);
  printf("ADC:%.4f\r\n", calcValue);
  return;
}

/**
 * @brief Set specified pwm output frequency and duty, high duty ratio
 * accuracy
 * @param  htim             Timer
 * @param  channel          Channel
 * @param  freq             Frequency
 * @param  duty             Duty
 */
void PWM_Cfg_HighDutyAcc(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t freq, float duty) {
  __HAL_TIM_SET_PRESCALER(htim, (SYSTEM_CLOCK_FREQ_HZ / 1000 / freq) - 1);
  __HAL_TIM_SET_AUTORELOAD(htim, 1000 - 1);
  __HAL_TIM_SET_COMPARE(htim, channel, duty * 10);
}

/**
 * @brief Set specified pwm output frequency and duty, high frequency
 * accuracy
 * @param  htim             Timer
 * @param  channel          Channel
 * @param  freq             Frequency
 * @param  duty             Duty
 */
void PWM_Cfg_HighFreqAcc(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t freq, uint16_t duty) {
  __HAL_TIM_SET_PRESCALER(htim, (SYSTEM_CLOCK_FREQ_HZ / 100 / freq) - 1);
  __HAL_TIM_SET_AUTORELOAD(htim, 100 - 1);
  __HAL_TIM_SET_COMPARE(htim, channel, duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim7) {  // 50Hz,20ms,用于精确读取转速
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    Motor_Get_Speed(&motor_1, 50);
  }
}

void Motor_Pos_PID_20Hz(void) {
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  Motor_Pos_PID_Run(&motor_1);
}

void Motor_Spd_PID_40Hz(void) {
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

  Motor_Spd_PID_Run(&motor_1);
}

void Param_Report_40Hz(void) {
  printf("M1:%f,%f,%f,%ld\r\n", __MOTOR_GET_DEGREE(motor_1), motor_1.speed,
         motor_1.pwmDuty,__MOTOR_GET_POS(motor_1));  //输出位置，转速，PWM占空比
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
