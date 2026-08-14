# M3508驱动

## 基本结构
1. 基于CAN通信进行控制，电机控制的操作对象一般为整个CAN总线（默认8个电机，使用枚举状态判断电机是否启用），初始化的操作对象一般为单个电机
2. 电机配置有速度环PID，位置环PID，可以使用模式切换函数切换PID的模式，每个PID句柄有一个单独的低通滤波器

## 建议的初始化操作顺序
1. **初始化**
    1. 总线初始化`M3508_CAN_Init`
    2. PID初始化`M3508_SpeedPID_Init, M3508_PositionPID_Init`
    3. 设置PID模式`M3508_PIDMode_Switch`
    4. 设置积分限幅`M3508_PID_SetIntLim`
    5. 设置滤波参数`M3508_IIRFilter_SetAlpha`
2. **PID更新**
    1. 设置目标值`M3508_SetPositionTarget`
    2. 调用PID更新`M3508_PID_Update`