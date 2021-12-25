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
#define screen(x,...) printft(&huart2,x,__VA_ARGS__)
#define _DELAY_TIM htim6

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
#endif