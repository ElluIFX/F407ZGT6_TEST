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
#include "programCtrl.h"
#include "scheduler.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  __IO float setpoint;    //设定目标
  __IO float sum_error;   //误差累计
  __IO float proportion;  //比例常数
  __IO float integral;    //积分常数
  __IO float derivative;  //微分常数
  __IO float last_error;  // e[-1]
  __IO float prev_error;  // e[-2]
} m_pid_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SPEEDRATIO 30
#define ENCODER_RESOLUTION 13
#define Kp 0.18f   // P参数
#define Ti 0.02f   // I参数
#define Td 0.012f  // D参数
#define T 0.05
#define Ki Kp *(T / Ti)
#define Kd Kp *(Td / T)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uart_ctrl_t uart_1;
__IO float motor1Spd = 0;
unsigned short keyValue;
m_pid_t PID1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RGB(uint8_t R, uint8_t G, uint8_t B);
void PWM_Set_Freq_Duty(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq,
                       float duty);
float PID_calc(m_pid_t *PIDx, float NextPoint);
void PID_Param_Init(m_pid_t *);
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
  HAL_UART_Receive_IT(&huart1, uart_1.rxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("System running...\r\n");
  printf("float:%f\r\n", 3.1415926);
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
    uart_1.rxStartFlag = 1;
    uart_1.rxTick = HAL_GetTick();
    uart_1.rxBuf[uart_1.rxBufIndex++] = uart_1.rxData[0];
    if (uart_1.rxBufIndex >= RX_BUFFER_SIZE) {
      memcpy(uart_1.rxSaveBuf, uart_1.rxBuf, uart_1.rxBufIndex);
      uart_1.rxCounter = uart_1.rxBufIndex;
      uart_1.rxSaveBuf[uart_1.rxBufIndex] = 0;
      uart_1.rxEndFlag = 1;
      uart_1.rxStartFlag = 0;
      uart_1.rxBufIndex = 0;
    }
    HAL_UART_Receive_IT(&huart1, uart_1.rxData, 1);
  }
}
/**
 * @brief 串口超时处理
 */
void Uart_Overtime_100Hz(void) {
  if (uart_1.rxStartFlag && HAL_GetTick() - uart_1.rxTick > 10) {
    memcpy(uart_1.rxSaveBuf, uart_1.rxBuf, uart_1.rxBufIndex);
    uart_1.rxCounter = uart_1.rxBufIndex;
    uart_1.rxSaveBuf[uart_1.rxBufIndex] = 0;
    uart_1.rxEndFlag = 1;
    uart_1.rxStartFlag = 0;
    uart_1.rxBufIndex = 0;
  }
  return;
}

/**
 * @brief 测试用，串口控制
 */
void Uart_Controller_20Hz(void) {
  static uint8_t controlWord = 0;
  static uint8_t userMode = 0;
  static uint32_t freq = 0;
  static float duty = 0;
  static uint16_t speed = 0;
  static uint16_t pFreq = 0;
  float DACvoltage = 0;

  if (uart_1.rxEndFlag) {
    uart_1.rxEndFlag = 0;
    controlWord = uart_1.rxSaveBuf[0];
    switch (userMode) {
      case 0:  //模式切换
        switch (controlWord) {
          case '?':
            printf(
                "\r\n-------MENU-------\r\n1:Motor control\r\n2:PWM "
                "control\r\n3:Key reading\r\n4:LED control\r\n5:User task "
                "control\r\n6:ADC reading\r\n7:DAC output\r\n");
            break;
          case '1':
            userMode = 1;
            printf(
                "\r\n>>Motor control started\r\n>>Target: MOTOR_1 ENC_3\r\n");
            HAL_Delay(1000);
            PWM_Set_Freq_Duty(&htim1, TIM_CHANNEL_1, 20000, 60);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
            HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
            HAL_TIM_Base_Start_IT(&htim7);
            PID_Param_Init(&PID1);
            PID1.setpoint = 200;
            Enable_SchTask(MOTOR_PID_TASK_ID);
            break;
          case '2':
            userMode = 2;
            printf("\r\n>>PWM control started\r\n>>Target: MOTOR_3 CHA_1\r\n");
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
            break;
          case '3':
            userMode = 3;
            printf("\r\n>>Key reading started\r\n");
            Enable_SchTask(KEY_CHECK_TASK_ID);
            Enable_SchTask(KEY_READ_TASK_ID);
            break;
          case '4':
            userMode = 4;
            printf("\r\n>>LED control started\r\n");
            break;
          case '5':
            userMode = 5;
            printf("\r\n>>User task control started\r\n");
            break;
          case '6':
            userMode = 6;
            printf("\r\n>>ADC reading started\r\n");
            HAL_Delay(1000);
            Enable_SchTask(ADC_READ_TASK_ID);
            break;
          case '7':
            userMode = 7;
            printf("\r\n>>DAC output started\r\n");
            break;
          default:
            printf("\r\n>>Invalid command\r\n");
            break;
        }
        break;
      case 1: 
        if (controlWord == 'e') {
          printf("\r\n>>Motor control exit\r\n");
          HAL_TIM_Base_Stop_IT(&htim7);
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
          Disable_SchTask(MOTOR_PID_TASK_ID);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "s:%hd", &speed) == 1) {
          // printf("\r\n>>Set speed: %d rpm\r\n", speed);
          PID1.setpoint = speed;
        } else if (sscanf((char *)uart_1.rxSaveBuf, "f:%hd", &pFreq) == 1) {
          Set_SchTask_Freq(MOTOR_PID_TASK_ID, pFreq);
        }else{
          printf("\r\n>>Invalid command\r\n");
        }
        break;
      case 2:  // PWM控制
        if (controlWord == 'e') {
          printf("\r\n>>PWM control exit\r\n");
          HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "f:%ld,d:%f", &freq, &duty) != 2) {
          printf("\r\n>>Invalid command\r\n");
          break;
        } else {
          printf("\r\n>>Set Freq:%ldHz\r\n>>Set Duty:%f%%\r\n", freq, duty);
          PWM_Set_Freq_Duty(&htim8, TIM_CHANNEL_1, freq, duty);
        }
        break;
      case 3:  // 按键读取
        if (controlWord == 'e') {
          printf("\r\n>>Key reading exit\r\n");
          Disable_SchTask(KEY_CHECK_TASK_ID);
          Disable_SchTask(KEY_READ_TASK_ID);
          userMode = 0;
          break;
        }
        break;
      case 4:  // LED控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>LED control exit\r\n");
            userMode = 0;
            RGB(0, 0, 0);
            break;
          case 's':
            RGB(uart_1.rxSaveBuf[2] - '0', uart_1.rxSaveBuf[3] - '0',
                uart_1.rxSaveBuf[4] - '0');
            break;
        }
        break;
      case 5:  //用户事件控制
        switch (controlWord) {
          case 'e':
            printf("\r\n>>User task control exit\r\n");
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
      case 6:  // ADC读取
        if (controlWord == 'e') {
          printf("\r\n>>ADC reading exit\r\n");
          Disable_SchTask(ADC_READ_TASK_ID);
          userMode = 0;
          break;
        }
        break;
      case 7:  // DAC输出
        if (controlWord == 'e') {
          printf("\r\n>>DAC output exit\r\n");
          HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
          userMode = 0;
          break;
        }
        if (sscanf((char *)uart_1.rxSaveBuf, "s:%f", &DACvoltage) == 1) {
          HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                           (uint32_t)(DACvoltage / 3.3 * 4095));
          HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
          printf("\r\n>>Set DAC output: %f V\r\n", DACvoltage);
        } else {
          printf("\r\n>>Invalid command\r\n");
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
  static uint8_t str[20];
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 50);  //等待转换完成
  if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
    calcValue = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
  }
  HAL_ADC_Stop(&hadc1);
  sprintf((char *)str, "%.4f", calcValue);
  printf("ADC:%s\r\n", str);
  return;
}

/**
 * @brief Set specified pwm output frequency and duty
 * @param  htim             Timer
 * @param  channel          Channel
 * @param  freq             Frequency
 * @param  duty             Duty
 */
void PWM_Set_Freq_Duty(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq,
                       float duty) {
  const uint32_t SYSTEM_CLOCK = 168000000;
  static uint32_t prescaler = 0;
  static uint32_t period = 0;
  static uint32_t pulse = 0;
  prescaler = (SYSTEM_CLOCK / 1000 / freq) - 1;
  period = 1000 - 1;
  pulse = duty * 10;
  __HAL_TIM_SET_PRESCALER(htim, prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim, period);
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);

  return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint8_t cnt = 0;
  if (htim == &htim7) {
    cnt = !cnt;
    RGB(cnt, 0, 0);
    motor1Spd = ((float)((short)__HAL_TIM_GET_COUNTER(&htim5)) * 3000.0f /
                 ENCODER_RESOLUTION / SPEEDRATIO);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
  }
}

void Motr_PID_40Hz(void) {
  static uint8_t cnt = 0;
  static float pwm1Duty;
  static uint32_t counter = 0;
  counter++;
  cnt = !cnt;
  RGB(0, 0, cnt);
  PID1.sum_error += PID_calc(&PID1, motor1Spd);
  if (PID1.sum_error > 100 * 20) PID1.sum_error = 100 * 20;
  pwm1Duty = PID1.sum_error / 20.0;
  PWM_Set_Freq_Duty(&htim1, TIM_CHANNEL_1, 20000, pwm1Duty);
  if (counter % 2 == 0) printf("M1:%f,%f\r\n", motor1Spd, pwm1Duty);
}

void PID_Param_Init(m_pid_t *PIDx) {
  PIDx->last_error = 0;  // Error[-1]
  PIDx->setpoint = 0;    // 设定目标Desired Value
  PIDx->sum_error = 0;
  PIDx->prev_error = 0;
  PIDx->proportion = Kp;
  PIDx->integral = Ki;
  PIDx->derivative = Kd;
}

float PID_calc(m_pid_t *PIDx, float nextpoint) {
  const float MAX_I = 200;
  float iError, iincpid;
  iError = PIDx->setpoint - nextpoint;  //当前误差
  /*iincpid=                                               //增量计算
  PIDx->proportion*iError                //e[k]项
  -PIDx->integral*PIDx->last_error          //e[k-1]
  +PIDx->derivative*PIDx->prev_error;//e[k-2]
  */
  iincpid =  //增量计算
      PIDx->proportion * (iError - PIDx->last_error) + PIDx->integral * iError +
      PIDx->derivative * (iError - 2 * PIDx->last_error + PIDx->prev_error);
  if (iincpid < -MAX_I) iincpid = -MAX_I;
  if (iincpid > MAX_I) iincpid = MAX_I;
  PIDx->prev_error = PIDx->last_error;  //存储误差，便于下次计算
  PIDx->last_error = iError;
  return (iincpid);
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
