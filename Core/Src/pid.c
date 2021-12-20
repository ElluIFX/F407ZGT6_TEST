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

/****************** Position PID Functions ******************/

/**
 * @brief Initialize Position PID struct
 * @param  PIDx             Target
 */
void Pos_PID_Param_Init(pos_pid_t *PIDx) {
  PIDx->setPoint = 0;
  PIDx->sumError = 0;
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->proportion = POS_KP;
  PIDx->integral = POS_KI;
  PIDx->derivative = POS_KD;
  PIDx->deadBand = POS_DEAD_BAND;
  PIDx->maxI = POS_MAX_I;
}

/**
 * @brief Calculate Position PID
 * @param  PIDx             Target
 * @param  nextPoint        Current input
 * @retval float           PID output
 */
float Pos_PID_Calc(pos_pid_t *PIDx, int32_t nextPoint) {
  static int32_t error_0 = 0;
  static float output = 0;
  output = 0;
  error_0 = PIDx->setPoint - nextPoint;
  /* Dead band */
  if (error_0 > PIDx->deadBand || error_0 < -PIDx->deadBand) {
    /* Proportion */
    output += PIDx->proportion * error_0;
    /* Integral */
    PIDx->sumError += error_0;
    if (PIDx->sumError > PIDx->maxI) {
      PIDx->sumError = PIDx->maxI;
    } else if (PIDx->sumError < -PIDx->maxI) {
      PIDx->sumError = -PIDx->maxI;
    }
    output += PIDx->integral * PIDx->sumError;
  } else {
    PIDx->sumError = 0;
    error_0 = 0;
  }
  /* Derivative */
  output += PIDx->derivative * (error_0 - PIDx->error_1);

  PIDx->error_2 = PIDx->error_1;
  PIDx->error_1 = error_0;
  return output;
}

/**
 * @brief Reset Position PID error
 * @param  PIDx             Target
 */
void Pos_PID_Clear(pos_pid_t *PIDx) {
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->sumError = 0;
}

/****************** Speed PID Functions ******************/

/**
 * @brief Initialize Speed PID struct
 * @param  PIDx             Target
 */
void Spd_PID_Param_Init(spd_pid_t *PIDx) {
  PIDx->setPoint = 0;
  PIDx->sumError = 0;
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->proportion = SPD_KP;
  PIDx->integral = SPD_KI;
  PIDx->derivative = SPD_KD;
  PIDx->deadBand = SPD_DEAD_BAND;
  PIDx->maxIMult = SPD_MAX_I_Multipler;
}

/**
 * @brief Calculate Speed PID
 * @param  PIDx             Target
 * @param  nextPoint        Current input
 * @retval float           PID output
 */
float Spd_PID_Calc(spd_pid_t *PIDx, float nextPoint) {
  static float error_0 = 0;
  static float output = 0;
  output = 0;
  error_0 = PIDx->setPoint - nextPoint;
  /* Dead band */
  if (error_0 < PIDx->deadBand || error_0 > -PIDx->deadBand) {
    error_0 = 0;
  }
  /* Proportion */
  PIDx->sumError += error_0;
  output += PIDx->proportion * error_0;
  /* Integral */
  if (PIDx->sumError > PIDx->maxIMult * PIDx->setPoint) {
    PIDx->sumError = PIDx->maxIMult * PIDx->setPoint;
  } else if (PIDx->sumError < -PIDx->maxIMult * PIDx->setPoint) {
    PIDx->sumError = -PIDx->maxIMult * PIDx->setPoint;
  }
  output += PIDx->integral * PIDx->sumError;
  /* Derivative */
  output += PIDx->derivative * (error_0 - PIDx->error_1);

  PIDx->error_2 = PIDx->error_1;
  PIDx->error_1 = error_0;
  return output;
}

/**
 * @brief Reset Speed PID error
 * @param  PIDx             Target
 */
void Spd_PID_Clear(spd_pid_t *PIDx) {
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->sumError = 0;
}

/****************** Incremental PID Functions ******************/

/**
 * @brief Initialize Incremental PID struct
 * @param  PIDx             Target
 */
void Inc_PID_Param_Init(inc_pid_t *PIDx) {
  PIDx->setPoint = 0;
  PIDx->sumError = 0;
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->proportion = INC_KP;
  PIDx->integral = INC_KI;
  PIDx->derivative = INC_KD;
  PIDx->max_inc = INC_MAX_INC;
}

/**
 * @brief Calculate Incremental PID
 * @param  PIDx             Target
 * @param  nextPoint        Current input
 * @retval float           PID increment
 */
float Inc_PID_Calc(inc_pid_t *PIDx, float nextPoint) {
  float error_0, inc;
  error_0 = PIDx->setPoint - nextPoint;
  inc =  //增量计算
      PIDx->proportion * (error_0 - PIDx->error_1) + PIDx->integral * error_0 +
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
 * @brief Reset Incremental PID error
 * @param  PIDx             Target
 */
void Inc_PID_Clear(inc_pid_t *PIDx) {
  PIDx->error_1 = 0;
  PIDx->error_2 = 0;
  PIDx->sumError = 0;
}

/****************** All PID Functions End ******************/

/**
 * @brief Setup motor structure and configure Timers.
 * @param  motor            Target
 * @param  timEncoder       Timer Handle for encoder
 * @param  timPWM           Timer Handle for PWM
 * @param  forwardChannel   PWM channel for forward
 * @param  reverseChannel   PWM channel for reverse
 */
void Motor_Setup(motor_t *motor, TIM_HandleTypeDef *timEncoder,
                 TIM_HandleTypeDef *timPWM, uint32_t forwardChannel,
                 uint32_t reverseChannel) {
  Pos_PID_Param_Init(&motor->posPID);
  Spd_PID_Param_Init(&motor->spdPID);
  motor->timEncoder = timEncoder;
  motor->timPWM = timPWM;
  motor->forwardChannel = forwardChannel;
  motor->reverseChannel = reverseChannel;
  motor->targetSpeed = POS_INIT_TARGET_SPEED;
  HAL_TIM_Encoder_Start(timEncoder, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(timEncoder, COUNTER_NEUTRAL_POSITION);
  __HAL_TIM_SET_PRESCALER(timPWM,
                          (SYSTEM_CLOCK_FREQ_HZ / 1000 / MOTOR_PWM_FREQ) - 1);
  __HAL_TIM_SET_AUTORELOAD(timPWM, 1000 - 1);
  __HAL_TIM_SET_COMPARE(timPWM, forwardChannel, 0);
  __HAL_TIM_SET_COMPARE(timPWM, reverseChannel, 0);
  HAL_TIM_PWM_Start(timPWM, forwardChannel);
  HAL_TIM_PWM_Start(timPWM, reverseChannel);
}

/**
 * @brief Get motor speed and position
 * @param  motor            Target
 * @param  runTimeHz        How fast this function is called
 */
void Motor_Get_Speed(motor_t *motor, float runTimeHz) {
  motor->pos = __HAL_TIM_GET_COUNTER(motor->timEncoder);
  motor->speed = (float)(motor->pos - motor->lastPos) * 60.0f * runTimeHz /
                 ENCODER_RESOLUTION / SPEED_RATIO;
  motor->lastPos = motor->pos;
}

/**
 * @brief Calculate motor position PID, will set speed PID setpoint!
 * @param  motor            Target
 */
void Motor_Pos_PID_Run(motor_t *motor) {
  float tempSpeed = Pos_PID_Calc(&motor->posPID, motor->pos);
  // Set target speed
  if (tempSpeed > motor->targetSpeed) {
    tempSpeed = motor->targetSpeed;
  } else if (tempSpeed < -motor->targetSpeed) {
    tempSpeed = -motor->targetSpeed;
  }
  motor->spdPID.setPoint = tempSpeed;
}

/**
 * @brief Calculate motor speed PID, will change motor speed!
 * @param  motor            Target
 */
void Motor_Spd_PID_Run(motor_t *motor) {
  float pwmDuty = Spd_PID_Calc(&motor->spdPID, motor->speed) / SPEED_PWM_RATIO;
  if (pwmDuty > 100) pwmDuty = 100;
  if (pwmDuty < -100) pwmDuty = -100;
  // Set PWM Output
  if (pwmDuty > 0) {
    __HAL_TIM_SET_COMPARE(motor->timPWM, motor->forwardChannel, pwmDuty);
    __HAL_TIM_SET_COMPARE(motor->timPWM, motor->reverseChannel, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor->timPWM, motor->forwardChannel, 0);
    __HAL_TIM_SET_COMPARE(motor->timPWM, motor->reverseChannel, -pwmDuty);
  }
}