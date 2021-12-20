/**
 * @file uartPack.c
 * @brief UART functions
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-19
 *
 * THINK DIFFERENTLY
 */

#include "uartPack.h"

#include "string.h"
#include "usart.h"

//重定向printf
#include "stdio.h"
//修正标准库流
struct __FILE {
  int handle;
};
FILE __stdout;
//定义_sys_exit()禁用半主机模式
void _sys_exit(int x) { x = x; }
//重定向write方法
int _write(int fd, char *ch, int len) {
  HAL_UART_Transmit_IT(&_REDIRECT_UART_PORT, (uint8_t *)ch, len);
  while (huart1.gState != HAL_UART_STATE_READY) {
  }
  return len;
}
// END 重定向printf

/**
 * @brief Enable a Overtime UART controller, call once before using UART
 * @param  huart            target UART handle
 * @param  ctrl             target UART controller
 */
void Enable_Uart_O_Control(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl) {
  //设置串口接收中断
  HAL_UART_Receive_IT(huart, ctrl->rxData, 1);
  if (ctrl->rxTimeout == 0) {
    ctrl->rxTimeout = _RX_DEFAILT_TIMEOUT;
  }
}

/**
 * @brief Enable a Single Ending Bit UART controller, call once before using
 * UART
 * @param  huart            target UART handle
 * @param  ctrl             target UART controller
 */
void Enable_Uart_E_Control(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl) {
  //设置串口接收中断
  HAL_UART_Receive_IT(huart, ctrl->rxData, 1);
  if (ctrl->rxEndBit == 0) {
    ctrl->rxEndBit = _RX_DEFAILT_ENDBIT;
  }
}

/**
 * @brief Process Overtime UART data, call in HAL_UART_RxCpltCallback
 * @param  huart            target UART handle
 * @param  ctrl             target UART controller
 * @retval 1: data overflow, 0: no data overflow
 */
uint8_t Uart_O_Data_Process(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl) {
  ctrl->rxFlag = 1;
  ctrl->rxTick = HAL_GetTick();
  ctrl->rxBuf[ctrl->rxBufIndex++] = ctrl->rxData[0];
  if (ctrl->rxBufIndex >= RX_BUFFER_SIZE - 1) {
    memcpy(ctrl->rxSaveBuf, ctrl->rxBuf, ctrl->rxBufIndex);
    ctrl->rxSaveCounter = ctrl->rxBufIndex;
    ctrl->rxSaveBuf[ctrl->rxBufIndex] = 0;
    ctrl->rxSaveFlag = 1;
    ctrl->rxFlag = 0;
    ctrl->rxBufIndex = 0;
    return 1;
  }
  HAL_UART_Receive_IT(huart, ctrl->rxData, 1);
  return 0;
}

/**
 * @brief Overtime UART timeout check, call in scheduler
 * @param  huart            target UART handle
 * @param  ctrl             target UART controller
 * @retval 1: timeout, 0: not timeout
 */
uint8_t Uart_O_Timeout_Check(UART_HandleTypeDef *huart, uart_o_ctrl_t *ctrl) {
  if (ctrl->rxFlag && HAL_GetTick() - ctrl->rxTick > 10) {
    memcpy(ctrl->rxSaveBuf, ctrl->rxBuf, ctrl->rxBufIndex);
    ctrl->rxSaveCounter = ctrl->rxBufIndex;
    ctrl->rxSaveBuf[ctrl->rxBufIndex] = 0;
    ctrl->rxSaveFlag = 1;
    ctrl->rxFlag = 0;
    ctrl->rxBufIndex = 0;
    return 1;
  }
  return 0;
}

/**
 * @brief Process Single Ending Bit UART data, call in HAL_UART_RxCpltCallback
 * @param  huart            target UART handle
 * @param  ctrl             target UART controller
 * @retval 1: end bit, 0: not end bit
 */
uint8_t Uart_E_Data_Process(UART_HandleTypeDef *huart, uart_e_ctrl_t *ctrl) {
  ctrl->rxFlag = 1;
  ctrl->rxBuf[ctrl->rxBufIndex++] = ctrl->rxData[0];
  if (ctrl->rxBufIndex >= RX_BUFFER_SIZE - 1 ||
      ctrl->rxData[0] == ctrl->rxEndBit) {
    memcpy(ctrl->rxSaveBuf, ctrl->rxBuf, ctrl->rxBufIndex);
    ctrl->rxSaveCounter = ctrl->rxBufIndex;
    ctrl->rxSaveBuf[ctrl->rxBufIndex] = 0;
    ctrl->rxSaveFlag = 1;
    ctrl->rxFlag = 0;
    ctrl->rxBufIndex = 0;
    return 1;
  }
  HAL_UART_Receive_IT(huart, ctrl->rxData, 1);
  return 0;
}