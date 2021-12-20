# F407ZGT6_TEST
## What's this
本项目用于测试基地任务的F407ZGT6板子各项功能，测试项目全写在main.c里，正在逐步将新增功能拆成别的模块文件，方便移植
## 文件结构
> ---
> 1. Src
>       - main.c 主程序
>       - key.c 封装多功能按键处理 [作者：霍宏鹏](https://blog.csdn.net/huohongpeng/article/details/60118467)
>       - uartPack.c 封装串口通信函数
>       - schduler.c 封装时分调度器
>       - programCtrl.c 封装用户任务控制
>       - pid.c 封装PID算法与电机闭环控制相关
> 2. Inc
>       - 包含上述对应封装的必须参量、结构体和一些带参宏
> ---


## **下面是第一次写markdown，拿来练手的东西**   
## 关于PID

常见的位置套速度环，加了点方便的宏

### PID移植
1. #include "pid.h"
2. 按电机个数用motor_t开结构
3. 调用Motor_Setup依次初始化
4. 开定时中断，调用Motor_Get_Speed定时获取速度（频率大于等于速度环）
5. 开PID调度事件，分别调用Motor_Pos_PID_Run和Motor_Spd_PID_Run，速度环频率最好是位置环的两倍以上，避免速度震荡。
6. 不想用位置环直接不管它就行
7. 没了

## 关于Printf重定向
如果重定向完卡sys某个函数里，调试反而有输出，那就是没干掉半主机模式

如下

```C
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
```
## 关于Printf输出浮点为空的问题
STM32的标准流浮点输出默认不使能，需要在编译参数中指定软件浮点或是硬件浮点才能使能转换，F4刚好有硬件DFU，跑浮点比软件快很多

用ssprint也没用，当然可以用微库，但如果不想用的话，可以用下面的方法

以Makefile为例，编译参数如下
```makefile
CFLAGS  += -mfloat-abi=hard
LDFLAGS += -u _printf_float -u _scanf_float
```
>keil可以在Target->C/C++->Options->Floating-point->Floating-point model里开启浮点