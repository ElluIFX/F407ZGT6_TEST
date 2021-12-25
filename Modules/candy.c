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

/**
 * @brief Linear mapping input to the specified range
 * @param  x                input value
 * @param  in_min           input range min
 * @param  in_max           input range max
 * @param  out_min          output range min
 * @param  out_max          output range max
 * @retval Mapping result
 */
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Delay_us, usint a timer prescale to 1Mhz, reloadRegister = 1
 * @param  us               delay time in us
 */
void delay_us(uint16_t us) {
  uint16_t differ = 0xffff - us - 5;
  HAL_TIM_Base_Start(&_DELAY_TIM);
  __HAL_TIM_SetCounter(&_DELAY_TIM, differ);
  while (differ < 0xffff - 5) {
    differ = __HAL_TIM_GetCounter(&_DELAY_TIM);
  }
  HAL_TIM_Base_Stop(&_DELAY_TIM);
}

/**
 * @brief Send data bits to WS2812, can be used only when MCU's SYSCLK is
 * 168MHz.
 * @param  data             uint8_t color array in G R B order, total length is
 * 3 * number of LEDs, the first LED in left.
 * @param  len              the number of LEDs
 */
void WS2812_SendBit(uint8_t* data, uint16_t len) {
  __2812_RESET;
  len *= 24;
  for (uint16_t i = 0; i < len; i++) {
    if (data[i / 8] & (1 << (7 - i % 8))) {
      __2812_HIGH_BIT;
    } else {
      __2812_LOW_BIT;
    }
  }
}