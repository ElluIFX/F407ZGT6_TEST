/**
 * @file pid.h
 * @brief See pid.c for details.
 * @author Ellu (lutaoyu@163.com)
 * @version 2.0
 * @date 2021-12-18
 *
 * THINK DIFFERENTLY
 */

#ifndef PID_H
#define PID_H
#include <main.h>
#include <tim.h>

// constants
//电机参数相关
#define SPEED_RATIO 30         //齿轮组减速比
#define ENCODER_RESOLUTION 13  //编码器线数
#define PULSE_PER_ROTATION \
  (ENCODER_RESOLUTION * SPEED_RATIO)  //每圈编码器脉冲数
#define WHEEL_PERIMETER 0.065         //轮子周长
#define SPEED_PWM_RATIO 20  // 电机转速与PWM输出换算比例（空载）

//功能相关
#define COUNTER_NEUTRAL_POSITION 1073741824  //计数器中位点
#define MOTOR_PWM_FREQ 20000                 //电机PWM频率

//增量式PID
#define INC_KP 0.18f      // 比例项系数
#define INC_KI 0.225f     // 积分项系数
#define INC_KD 0.0864f    // 微分项系数
#define INC_MAX_INC 0.0f  // 单周期最大增量

//速度环PID
#define SPD_KP 50.0f               // 比例项系数
#define SPD_KI 8.5f                // 积分项系数
#define SPD_KD 0.0f                // 微分项系数
#define SPD_DEAD_BAND 0.2f         // 死区
#define SPD_MAX_I_Multipler 10.0f  // 积分上限乘子
#define SPD_INIT_TARGET 0.0f       // 初始目标速度

//位置环PID
#define POS_KP 0.01f                              // 比例项系数
#define POS_KI 0.0f                               // 积分项系数
#define POS_KD 0.08f                              // 微分项系数
#define POS_DEAD_BAND 50l                         // 死区
#define POS_MAX_I 1000l                           // 积分上限
#define POS_INIT_TARGET COUNTER_NEUTRAL_POSITION  // 初始目标位置
#define POS_INIT_TARGET_SPEED 100.0f              // 初始目标速度

// typedef

typedef struct {          //增量式PID结构体
  __IO float setPoint;    //设定目标
  __IO float sumError;    //误差累计
  __IO float proportion;  //比例常数
  __IO float integral;    //积分常数
  __IO float derivative;  //微分常数
  __IO float error_1;     // e[-1]
  __IO float error_2;     // e[-2]
  __IO float max_inc;     // 单周期最大增量
} inc_pid_t;

typedef struct {          //速度环PID结构体
  __IO float setPoint;    //设定目标
  __IO float sumError;    //误差累计
  __IO float proportion;  //比例常数
  __IO float integral;    //积分常数
  __IO float derivative;  //微分常数
  __IO float error_1;     // e[-1]
  __IO float error_2;     // e[-2]
  __IO float deadBand;    // 速度死区
  __IO float maxIMult;    // 积分上限乘数（与速度）
} spd_pid_t;

typedef struct {          //位置环PID结构体
  __IO int32_t setPoint;  //设定目标
  __IO int32_t sumError;  //误差累计
  __IO float proportion;  //比例常数
  __IO float integral;    //积分常数
  __IO float derivative;  //微分常数
  __IO int32_t error_1;   // e[-1]
  __IO int32_t error_2;   // e[-2]
  __IO int32_t deadBand;  // 位置死区
  __IO int32_t maxI;      // 积分上限
} pos_pid_t;

typedef struct {                  //电机闭环控制结构体
  float speed;                    //速度
  uint32_t pos;                   //位置
  uint32_t lastPos;               //上一次位置
  spd_pid_t spdPID;               //速度环PID
  pos_pid_t posPID;               //位置环PID
  float targetSpeed;              //位置环-速度环目标速度
  TIM_HandleTypeDef *timEncoder;  //编码器定时器
  TIM_HandleTypeDef *timPWM;      // PWM定时器
  uint32_t forwardChannel;        //正向通道
  uint32_t reverseChannel;        //反向通道
} motor_t;

// define function
#define __MOTOR_SET_SPEED(motor, speed) motor.spdPID.setpoint = speed
#define __MOTOR_GO_POS(motor, pos) motor.posPID.setPoint += pos
#define __MOTOR_STOP(motor) \
  HAL_TIM_PWM_Stop(motor.timPWM, motor.forwardChannel || motor.reverseChannel)
#define __MOTOR_START(motor) \
  HAL_TIM_PWM_Start(motor.timPWM, motor.forwardChannel || motor.reverseChannel)

// function prototypes

float Inc_PID_Calc(inc_pid_t *PIDx, float NextPoint);
void Inc_PID_Param_Init(inc_pid_t *);
void Inc_PID_Clear(inc_pid_t *PIDx);

float Spd_PID_Calc(spd_pid_t *PIDx, float NextPoint);
void Spd_PID_Param_Init(spd_pid_t *);
void Spd_PID_Clear(spd_pid_t *PIDx);

float Pos_PID_Calc(pos_pid_t *PIDx, int32_t NextPoint);
void Pos_PID_Param_Init(pos_pid_t *);
void Pos_PID_Clear(pos_pid_t *PIDx);

void Motor_Setup(motor_t *motor, TIM_HandleTypeDef *timEncoder,
                 TIM_HandleTypeDef *timPWM, uint32_t forwardChannel,
                 uint32_t reverseChannel);
void Motor_Get_Speed(motor_t *motor, float runTimeHz);
void Motor_Pos_PID_Run(motor_t *motor);
void Motor_Spd_PID_Run(motor_t *motor);

#endif  // __MOTOR_H