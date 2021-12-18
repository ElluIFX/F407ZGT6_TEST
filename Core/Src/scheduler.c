/**
 * @file scheduler.c
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-11
 *
 * THINK DIFFERENTLY
 */

#include <scheduler.h>
// variables
/************************ scheduler tasks ************************/

// task lists
static scheduler_task_t schTaskList[] = {
    {Uart_Controller_20Hz, 20, 0, 0, 1},
    {Uart_Overtime_100Hz, 100, 0, 0, 1},
    {Key_Read_100Hz, 100, 0, 0, 0},
    {Key_Check_1000Hz, 1000, 0, 0, 0},
    {ADC_Read_50Hz, 50, 0, 0, 0},
};
const uint8_t SCH_TASK_COUNT = sizeof(schTaskList) / sizeof(scheduler_task_t);

__weak void Uart_Controller_20Hz(void) { return; }
__weak void Uart_Overtime_100Hz(void) { return; }
__weak void Key_Read_100Hz(void) { return; }
__weak void Key_Check_1000Hz(void) { return; }
__weak void ADC_Read_50Hz(void) { return; }

// @note !redefined in main.c

/************************ scheduler tasks end ************************/

// scheduler task control functions

/**
 * @brief Initialize tasklist
 * @retval None
 **/
void Scheduler_Init(void) {
  for (uint8_t i = 0; i < SCH_TASK_COUNT; i++) {
    schTaskList[i].periodMs = 1000 / schTaskList[i].rateHz;
    if (schTaskList[i].periodMs == 0) {
      schTaskList[i].periodMs = 1;
    }
  }
}

/**
 * @brief scheduler runner, call in main loop
 * @retval None
 **/
void Scheduler_Run(void) {
  for (uint8_t i = 0; i < SCH_TASK_COUNT; i++) {
    uint32_t currentTime = HAL_GetTick();
    if (schTaskList[i].enable &&
        (currentTime - schTaskList[i].lastRunMs >= schTaskList[i].periodMs)) {
      schTaskList[i].lastRunMs = currentTime;
      schTaskList[i].task();
    }
  }
}

/**
 * @brief Enable a task
 * @param  taskId           Target task id
 */
void Enable_SchTask(uint8_t taskId) {
  if (taskId < SCH_TASK_COUNT) {
    schTaskList[taskId].enable = 1;
  }
}

/**
 * @brief Disable a task
 * @param  taskId            Target task id
 */
void Disable_SchTask(uint8_t taskId) {
  if (taskId < SCH_TASK_COUNT) {
    schTaskList[taskId].enable = 0;
  }
}

/**
 * @brief Set a task's rate
 * @param  taskId           Task ID
 * @param  freq             Freq
 */
void Set_SchTask_Freq(uint8_t taskId, uint16_t freq) {
  if (taskId < SCH_TASK_COUNT) {
    schTaskList[taskId].rateHz = freq;
    schTaskList[taskId].periodMs = 1000 / schTaskList[taskId].rateHz;
    if (schTaskList[taskId].periodMs == 0) {
      schTaskList[taskId].periodMs = 1;
    }
  }
}