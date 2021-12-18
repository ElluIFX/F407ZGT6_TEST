/**
 * @file scheduler.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-11
 *
 * THINK DIFFERENTLY
 */

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "main.h"
//  defines
#define UART_CONTROLLER_TASK_ID 0
#define UART_OVERTIME_TASK_ID 1
#define KEY_READ_TASK_ID 2
#define KEY_CHECK_TASK_ID 3
#define ADC_READ_TASK_ID 4
#define MOTOR_PID_TASK_ID 5

// typedef
typedef struct {
  void (*task)(void);
  uint16_t rateHz;     // task rate
  uint16_t periodMs;   // task period
  uint32_t lastRunMs;  // last run time
  uint8_t enable;      // enable or disable
} scheduler_task_t;
// private variables
// private functions
__weak void Uart_Controller_20Hz(void);
__weak void Uart_Overtime_100Hz(void);
__weak void Key_Read_100Hz(void);
__weak void Key_Check_1000Hz(void);
__weak void ADC_Read_50Hz(void);
__weak void Motr_PID_40Hz(void);

void Scheduler_Init(void);
void Scheduler_Run(void);
void Enable_SchTask(uint8_t taskId);
void Disable_SchTask(uint8_t taskId);
void Set_SchTask_Freq(uint8_t taskId, uint16_t freq);
#endif  // _SCHEDULER_H_