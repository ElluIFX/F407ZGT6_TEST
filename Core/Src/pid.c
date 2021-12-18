/**
 * @file pid.c
 * @brief PID control functions
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-18
 *
 * THINK DIFFERENTLY
 */

#include "pid.h"

//增量PID

/**
 * @brief Initialize PID struct
 * @param  PIDx             Target
 */
void Inc_PID_Param_Init(inc_pid_t *PIDx) {
  PIDx->error_1 = 0;
  PIDx->setPoint = 0;
  PIDx->sumError = 0;
  PIDx->error_2 = 0;
  PIDx->Kp = INC_KP;
  PIDx->integral = INC_KI;
  PIDx->derivative = INC_KD;
  PIDx->max_inc = INC_MAX_INC;
}

/**
 * @brief Calculate PID increment
 * @param  PIDx             Target
 * @param  nextpoint        Current input
 * @return float           PID increment
 */
float Inc_PID_Calc(inc_pid_t *PIDx, float nextpoint) {
  float error_0, inc;
  error_0 = PIDx->setPoint - nextpoint;
  inc =  //增量计算
      PIDx->Kp * (error_0 - PIDx->error_1) + PIDx->integral * error_0 +
      PIDx->derivative * (error_0 - 2 * PIDx->error_1 + PIDx->error_2);
  if (PIDx->max_inc > 0) {
    if (inc < -PIDx->max_inc) inc = -PIDx->max_inc;
    if (inc > PIDx->max_inc) inc = PIDx->max_inc;
  }
  PIDx->error_2 = PIDx->error_1;
  PIDx->error_1 = error_0;
  return (inc);
}

/**
 * @brief Reset PID sum error
 * @param  PIDx             Target
 */
void Inc_PID_Clear(inc_pid_t *PIDx) {
  PIDx->error_1 = 0;
  PIDx->sumError = 0;
  PIDx->error_2 = 0;
}

//位置环pid
