/**
 * @file pid.h
 * @brief PID controller
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-18
 *
 * THINK DIFFERENTLY
 */

#ifndef PID_H
#define PID_H
#include <main.h>

// constants
//增量式PID
#define INC_KP 0.18f                     // 比例项系数
#define INC_TI 0.02f                     // 积分周期
#define INC_TD 0.012f                    // 微分周期
#define INC_MAX_INC 0.0f               // 单周期最大增量
#define INC_KI INC_KP *(PID_T / INC_TI)  // 积分项系数
#define INC_KD INC_KP *(INC_TD / PID_T)  // 微分项系数
//通用
#define PID_T 1.0 / 40.0                 // 控制周期

// typedef

typedef struct {          //增量式pid结构体
  __IO float setPoint;    //设定目标
  __IO float sumError;    //误差累计
  __IO float Kp;          //比例常数
  __IO float integral;    //积分常数
  __IO float derivative;  //微分常数
  __IO float error_1;     // e[-1]
  __IO float error_2;     // e[-2]
  __IO float max_inc;     // 单周期最大增量
} inc_pid_t;

// function prototypes

float Inc_PID_Calc(inc_pid_t *PIDx, float NextPoint);
void Inc_PID_Param_Init(inc_pid_t *);
void Inc_PID_Clear(inc_pid_t *PIDx);
#endif