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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "candy.h"
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
uart_o_ctrl_t uart_s;
unsigned short keyValue;
motor_t motor_1;
static uint8_t userMode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  RGB(1, 0, 0);
  Scheduler_Init();  // initialize scheduler
  Enable_Uart_O_Control(&huart1, &uart_1);
  Enable_Uart_O_Control(&huart2, &uart_s);
  Motor_Setup(&motor_1, &htim5, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
  screen("%srest%s", S_END_BIT, S_END_BIT);  // clear screen
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\r\n--- System running ---\r\n");
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
  } else if (huart->Instance == USART2) {
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    Uart_O_Data_Process(&uart_1);
  } else if (huart->Instance == USART2) {
    Uart_O_Data_Process(&uart_s);
  }
}

/**
 * @brief 串口超时处理
 */
void Task_Uart_Overtime(void) {
  Uart_O_Timeout_Check(&uart_1);
  Uart_O_Timeout_Check(&uart_s);
  return;
}

/**
 * @brief 测试用，串口控制
 */
void Task_Screen_Controller(void) {
  static uint8_t init = 0;
  static uint8_t controlWord = 0;
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
  if (__RX_DONE(uart_s)) {
    __RX_DONE(uart_s) = 0;
    controlWord = __RX_DATA(uart_s)[0];
    printf("\r\n>> %s\r\n", __RX_DATA(uart_s));
    if (init) {
      switch (userMode) {
        case 0:  //控制选择
          switch (controlWord) {
            case '1':
              userMode = 1;
              __MOTOR_RESET_ENCODER(motor_1);
              HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
              Enable_SchTask(MOTOR_POS_PID_TASK_ID);
              Enable_SchTask(MOTOR_SPD_PID_TASK_ID);
              Enable_SchTask(PARAM_REPORT_TASK_ID);
              HAL_Delay(100);
              break;
            case '2':
              userMode = 2;
              HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
              Enable_SchTask(PARAM_REPORT_TASK_ID);
              Enable_SchTask(MOTOR_SPD_PID_TASK_ID);
              HAL_Delay(100);
              break;
            case '3':
              userMode = 3;
              Enable_SchTask(PARAM_REPORT_TASK_ID);
              HAL_TIM_Base_Start_IT(&htim7);  //开转速计算
              HAL_Delay(100);
              break;
            default:
              break;
          }
          break;
        case 1:  //位置PID控制
          if (controlWord == 'e') {
            HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
            Disable_SchTask(MOTOR_POS_PID_TASK_ID);
            Disable_SchTask(MOTOR_SPD_PID_TASK_ID);
            Disable_SchTask(PARAM_REPORT_TASK_ID);
            __MOTOR_PWM_SETZERO(motor_1);
            userMode = 0;
            break;
          }
          if (sscanf((char *)__RX_DATA(uart_s), "pp:%f", &setPosKp) == 1) {
            setPosKp /= 10000.0f;
            motor_1.posPID.proportion = setPosKp;
          } else if (sscanf((char *)__RX_DATA(uart_s), "pi:%f", &setPosKi) ==
                     1) {
            setPosKi /= 10000.0f;
            motor_1.posPID.integral = setPosKi;
          } else if (sscanf((char *)__RX_DATA(uart_s), "pd:%f", &setPosKd) ==
                     1) {
            setPosKd /= 10000.0f;
            motor_1.posPID.derivative = setPosKd;
          } else if (sscanf((char *)__RX_DATA(uart_s), "sp:%f", &setSpdKp) ==
                     1) {
            setSpdKp /= 10000.0f;
            motor_1.spdPID.proportion = setSpdKp;
          } else if (sscanf((char *)__RX_DATA(uart_s), "si:%f", &setSpdKi) ==
                     1) {
            setSpdKi /= 10000.0f;
            motor_1.spdPID.integral = setSpdKi;
          } else if (sscanf((char *)__RX_DATA(uart_s), "sd:%f", &setSpdKd) ==
                     1) {
            setSpdKd /= 10000.0f;
            motor_1.spdPID.derivative = setSpdKd;
          } else if (sscanf((char *)__RX_DATA(uart_s), "gdeg:%f", &goDeg) == 1)
            __MOTOR_GO_DEGREE(motor_1, goDeg);
          else if (sscanf((char *)__RX_DATA(uart_s), "sdeg:%f", &setDeg) == 1)
            __MOTOR_SET_DEGREE(motor_1, setDeg);
          break;
        case 2:  //转速PID控制
          if (controlWord == 'e') {
            HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
            Disable_SchTask(MOTOR_SPD_PID_TASK_ID);
            Disable_SchTask(PARAM_REPORT_TASK_ID);
            __MOTOR_PWM_SETZERO(motor_1);
            userMode = 0;
            break;
          }
          if (sscanf((char *)__RX_DATA(uart_s), "s:%f", &speed) == 1)
            __MOTOR_SET_SPEED(motor_1, speed);
          else if (sscanf((char *)__RX_DATA(uart_s), "sp:%f", &setSpdKp) == 1) {
            setPosKp /= 10000.0f;
            motor_1.spdPID.proportion = setSpdKp;
          } else if (sscanf((char *)__RX_DATA(uart_s), "si:%f", &setSpdKi) ==
                     1) {
            setPosKi /= 10000.0f;
            motor_1.spdPID.integral = setSpdKi;
          } else if (sscanf((char *)__RX_DATA(uart_s), "sd:%f", &setSpdKd) ==
                     1) {
            setPosKd /= 10000.0f;
            motor_1.spdPID.derivative = setSpdKd;
          }
          break;
        case 3:  //手动控制
          if (controlWord == 'e') {
            HAL_TIM_Base_Stop_IT(&htim7);  //停转速计算
            Disable_SchTask(PARAM_REPORT_TASK_ID);
            __MOTOR_PWM_SETZERO(motor_1);
            userMode = 0;
            break;
          }
          if (sscanf((char *)__RX_DATA(uart_s), "d:%f", &duty) == 1) {
            duty /= 10.0f;
            duty = fmap(duty, 0.0f, 100.0f, 50.0f, 100.0f);
            __MOTOR_PWM_SETFWD(motor_1, duty);
            motor_1.pwmDuty = duty;
          }
          break;
      }
      RGB(0, 0, 0);
    } else {
      if (controlWord == 'o') {
        RGB(0, 0, 1);
        HAL_Delay(1000);
        screen("in.en=1%s", S_END_BIT);
        HAL_Delay(100);
        screen("sp=%d%ssi=%d%ssd=%d%spp=%d%spi=%d%spd=%d%s",
               (int)(motor_1.spdPID.proportion * 10000.0f), S_END_BIT,
               (int)(motor_1.spdPID.integral * 10000.0f), S_END_BIT,
               (int)(motor_1.spdPID.derivative * 10000.0f), S_END_BIT,
               (int)(motor_1.posPID.proportion * 10000.0f), S_END_BIT,
               (int)(motor_1.posPID.integral * 10000.0f), S_END_BIT,
               (int)(motor_1.posPID.derivative * 10000.0f), S_END_BIT);
        // screen("thsp=15%sthup=1%susup=1%s", S_END_BIT, S_END_BIT,
        // S_END_BIT);kkeil
        RGB(1, 0, 1);
        HAL_Delay(1000);
        RGB(0, 1, 0);
        screen("ok.en=1%s", S_END_BIT);
        HAL_Delay(200);
        init = 1;
        RGB(0, 0, 0);
      }
    }
  } else if (__RX_DONE(uart_1)) {
    __RX_DONE(uart_1) = 0;
    controlWord = __RX_DATA(uart_1)[0];
    if (controlWord == '>') {
      screen("%s", __RX_DATA(uart_1) + 1);
    }
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim7) {  // 50Hz,20ms,用于精确读取转速
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    Motor_Get_Speed(&motor_1, 50);
  }
}

void Task_Motor_Pos_PID(void) {
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  Motor_Pos_PID_Run(&motor_1);
}

void Task_Motor_Spd_PID(void) {
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  Motor_Spd_PID_Run(&motor_1);
}

void Task_Param_Report(void) {
  static uint8_t calc = 0;
  static uint8_t calc2 = 0;
  calc = (uint8_t)fmap(motor_1.speed, -120, 120, 0, 80);
  // printf("M1:%f,%f,%f,%ld\r\n", __MOTOR_GET_DEGREE(motor_1), motor_1.speed,
  //        motor_1.pwmDuty,
  //        __MOTOR_GET_POS(motor_1));  //输出位置，转速，PWM占空比
  switch (userMode) {
    case 3:
      screen("add map.id,0,%d%srpm.val=%d%s", calc, S_END_BIT,
             (int)(motor_1.speed * 10), S_END_BIT);
      break;
    case 1:
      calc2 = (uint8_t)fmap(__MOTOR_GET_DEGREE(motor_1), -720, 720, 0, 80);
      screen("add map.id,0,%d%sadd map.id,1,%d%srpm.val=%d%sdeg.val=%d%s", calc,
             S_END_BIT, calc2, S_END_BIT, (int)(motor_1.speed * 10), S_END_BIT,
             (int)(__MOTOR_GET_DEGREE(motor_1) * 100), S_END_BIT);
      break;
    case 2:
      screen("add map.id,0,%d%srpm.val=%d%sdeg.val=%d%s", calc, S_END_BIT,
             (int)(motor_1.speed * 10), S_END_BIT,
             (int)(__MOTOR_GET_DEGREE(motor_1) * 100), S_END_BIT);
      break;
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
  /* User can add his own implementation to report the HAL error return
   * state */
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
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
