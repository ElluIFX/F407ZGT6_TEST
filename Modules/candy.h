/**
 * @file candy.h
 * @brief see candy.c for details.
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-23
 *
 * THINK DIFFERENTLY
 */

#ifndef CANDY_H
#define CANDY_H
#include <main.h>

#include "tim.h"

#define S_END_BIT "\xff\xff\xff"
#define screen(x, ...) printft(&huart2, x, __VA_ARGS__)
#define _DELAY_TIM htim6

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
void delay_us(uint16_t us);
void WS2812_SendBit(uint8_t* data, uint16_t le);
#define WS2812_PIN WS2812_GPIO_Port, WS2812_Pin
#define __DELAY_350NS for (uint8_t i = 0; i < 8; i++)
#define __DELAY_600NS for (uint8_t i = 0; i < 13; i++)
#define __DELAY_700NS for (uint8_t i = 0; i < 20; i++)
#define __DELAY_800NS for (uint8_t i = 0; i < 19; i++)
#define __2812_HIGH_BIT                          \
  HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_SET);   \
  __DELAY_700NS;                                 \
  HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_RESET); \
  __DELAY_600NS
#define __2812_LOW_BIT                           \
  HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_SET);   \
  __DELAY_350NS;                                 \
  HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_RESET); \
  __DELAY_800NS
#define __2812_RESET delay_us(60)
#define __SET_2812(r,g,b) WS2812_SendBit((uint8_t[]){g,r,b}, 1) 
#endif