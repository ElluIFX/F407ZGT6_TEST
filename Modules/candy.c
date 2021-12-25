/**
 * @file candy.c
 * @brief 一些有用的小函数
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-23
 *
 * THINK DIFFERENTLY
 */

#include "candy.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay_us(uint16_t us)
{
    uint16_t differ=0xffff-us-5;


    HAL_TIM_Base_Start(&_DELAY_TIM);
    __HAL_TIM_SetCounter(&_DELAY_TIM,differ);
    while(differ < 0xffff-5)
    {
        differ = __HAL_TIM_GetCounter(&_DELAY_TIM);
    }
    HAL_TIM_Base_Stop(&_DELAY_TIM);

}