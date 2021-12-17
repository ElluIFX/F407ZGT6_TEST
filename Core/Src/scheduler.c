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
    {Task_1Hz, 1, 0, 0, 1},
    {Task_2Hz, 2, 0, 0, 1},
    {Task_10Hz, 10, 0, 0, 1},
    {Uart_Controller_20Hz, 20, 0, 0, 1},
    {Uart_Overtime_100Hz, 100, 0, 0, 1},
    {Task_1000Hz, 1000, 0, 0, 0},
};
const uint8_t SCH_TASK_COUNT = sizeof(schTaskList) / sizeof(scheduler_task_t);

/**
 * @brief scheduler task 2Hz
 * @retval None
 * @note Nothing here
 **/
void Task_1Hz(void) { return; }

/**
 * @brief scheduler task 5Hz
 * @retval None
 * @note Nothing here
 **/
void Task_2Hz(void) { return; }

/**
 * @brief scheduler task 10Hz
 * @retval None
 * @note Nothing here
 **/
void Task_10Hz(void) { return; }

/**
 * @brief scheduler task 20Hz
 * @retval None
 * @note !redefined in main.c
 **/
__weak void Uart_Controller_20Hz(void) { return; }

/**
 * @brief scheduler task 100Hz
 * @retval None
 * @note !redefined in main.c
 **/
__weak void Uart_Overtime_100Hz(void) { return; }

/**
 * @brief scheduler task 1000Hz
 * @retval None
 * @note Nothing here
 **/
void Task_1000Hz(void) { return; }

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
