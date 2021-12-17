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
void Task_1000Hz(void);

void Scheduler_Init(void);
void Scheduler_Run(void);

#endif  // _SCHEDULER_H_