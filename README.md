# F407ZGT6_TEST
### What's this
本项目用于测试基地任务的F407ZGT6板子各项功能，测试项目全写在main.c里，正在逐步将新增功能拆成别的模块文件，方便移植
### 文件结构
> 1. Src
>       - main.c 主程序
>       - key.c 封装多功能按键处理 [作者：霍宏鹏](https://blog.csdn.net/huohongpeng/article/details/60118467)
>       - uartPack.c 封装串口通信函数
>       - schduler.c 封装时分调度器
>       - programCtrl.c 封装用户任务控制
>       - pid.c 封装PID算法与电机闭环控制相关
> 2. Inc
>       - 包含上述对应封装的必须参量、结构体和一些带参宏
#### 关于PID
PID模块经过一次重构，从原来功能单一的增量PID改成常见的位置套速度环
