# M3508驱动

## 1.0.0
### 基本结构
1. 基于CAN通信进行控制，电机控制的操作对象一般为整个CAN总线（默认8个电机，使用枚举状态判断电机是否启用），初始化的操作对象一般为单个电机
2. 电机配置有速度环PID，位置环PID，可以使用模式切换函数切换PID的模式，每个PID句柄有一个单独的低通滤波器

### 建议的操作顺序
1. **初始化**
    1. 总线初始化`M3508_CAN_Init`
    2. PID初始化`M3508_SpeedPID_Init, M3508_PositionPID_Init`
    3. 设置PID模式`M3508_PIDMode_Switch`
    4. 设置积分限幅`M3508_PID_SetIntLim`
    5. 设置滤波参数`M3508_IIRFilter_SetAlpha`
2. **PID更新**
    1. 设置目标值`M3508_SetPositionTarget`
    2. 调用PID更新`M3508_PID_Update`

## 1.0.1
### 更新内容
1. 增加PID和发送的解耦，PID现在可以以任意频率更新，发送恒定为1KHz

### 函数用法改变
1. `M3508_SetCurrent`参数只剩句柄，m3508总线句柄中添加了电流数组，循环中直接访问并解算发送

### 调用顺序改变
1. 1KHz循环调用`M3508_CAN_CurrentUpdate`
2. 其他频率循环调用`M3508_PID_Update`
3. 建议搭配FreeRTOS使用