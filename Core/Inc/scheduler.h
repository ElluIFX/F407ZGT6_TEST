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
void Task_1Hz(void);
void Task_2Hz(void);
void Task_10Hz(void);
__weak void Uart_Controller_20Hz(void);
__weak void Uart_Overtime_100Hz(void);
__weak void Key_Read_100Hz(void);
__weak void Key_Check_1000Hz(void);
void Task_1000Hz(void);

void Scheduler_Init(void);
void Scheduler_Run(void);
void Enable_SchTask(uint8_t taskId);
void Disable_SchTask(uint8_t taskId);
#endif  // _SCHEDULER_H_