# GO-M8010-6 电机驱动库（GO8010_driver）

基于 STM32 HAL 的宇树 GO-M8010-6 关节电机驱动库，RS-485 半双工、非阻塞收发，提供打包/发送/接收/解析/超时检测等完整接口。

## 1. 特性

- **对象化**：一个电机一个 `GO8010_Motor_t` 实例，支持总线 0~14 多个电机
- **硬件可配置**：UART 句柄、方向脚（DE）引脚在初始化时传入，换板子只改 `Init`
- **非阻塞**：发送不等待回包，回包由中断异步接收，`Poll` 轮询取结果
- **超时检测**：`IsTimeout` 判断电机是否失联
- **纯协议驱动**：不依赖具体控制算法，速度环/位置环/PID 可在此之上扩展

## 2. 硬件要求

| 项 | 要求 |
|----|------|
| MCU | STM32（HAL 库，示例用 STM32H723） |
| 串口 | RS-485 半双工，**4Mbps**，8N1 |
| 收发器 | 需支持 4Mbps（如 SP3485/MAX3485，10Mbps 级；MAX485 只有 2.5Mbps 不行） |
| 方向脚 | DE（高=发送 / 低=接收），接一个 GPIO |
| 电机 | GO-M8010-6，ID 0~14 |

## 3. 文件结构

```
Core/Inc/GO8010_driver.h   驱动库头文件（本库）
Core/Src/GO8010_driver.c   驱动库实现
Core/Inc/gom_protocol.h    协议结构体（宇树原版，勿改）
Core/Src/gom_protocol.c    打包函数 modify_data
Core/Inc/crc_ccitt.h       CRC 校验
Core/Src/crc_ccitt.c       CRC 实现
```

## 4. 快速开始

```c
#include "GO8010_driver.h"

GO8010_Motor_t motor;   // 一个电机一个实例

int main(void)
{
    /* 1. 初始化：传 UART 句柄、DE 脚端口/引脚、电机ID */
    GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);

    /* 2. 设置命令：速度模式，输出轴匀速 6.28 rad/s（1 圈/秒） */
    GO8010_Motor_SetCmd(&motor, 1, 0.0f, 6.28f * MOTOR_GEAR_RATIO, 0.0f, 0.0f, 0.05f);

    /* 3. 循环：Poll 读状态 → Send 发命令 */
    while (1)
    {
        GO8010_Motor_Poll(&motor);    // 有回包则更新 motor.data
        GO8010_Motor_Send(&motor);    // 发送命令（非阻塞）
        HAL_Delay(1);                 // 1ms 周期
    }
}
```

## 5. API 参考

### 5.1 数据结构

```c
typedef struct {
    UART_HandleTypeDef *huart;    // UART 句柄
    GPIO_TypeDef       *de_port;  // 方向脚端口
    uint16_t            de_pin;   // 方向脚引脚
    uint8_t             id;       // 电机ID

    MotorCmd_t  cmd;              // 命令（浮点参数）
    MotorData_t data;             // 反馈（解析后）

    uint8_t          rx_buf[16];  // 接收缓冲区
    volatile uint8_t rx_done;     // 接收完成标志
    uint32_t         tx_count;    // 发送计数
    uint32_t         last_rx_tick;// 最后收到回包时刻（超时检测）
} GO8010_Motor_t;
```

### 5.2 函数

| 函数 | 说明 |
|------|------|
| `GO8010_Motor_Init(motor, huart, de_port, de_pin, id)` | 初始化（方向脚默认接收态） |
| `GO8010_Motor_SetCmd(motor, mode, T, W, Pos, K_P, K_W)` | 设置控制命令 |
| `GO8010_Motor_Send(motor)` | 打包+发送+准备接收，非阻塞，返回 0 成功 |
| `GO8010_Motor_Poll(motor)` | 查回包标志并解析，返回 1=有新反馈 |
| `GO8010_Motor_IsTimeout(motor, timeout_ms)` | 超时检测，返回 1=失联 |
| `GO8010_Motor_GetPos(motor)` | 读位置（rad，转子） |
| `GO8010_Motor_GetVel(motor)` | 读速度（rad/s，转子） |
| `GO8010_Motor_GetTorque(motor)` | 读力矩（N·m） |
| `GO8010_Motor_GetTemp(motor)` | 读温度（℃） |
| `GO8010_Motor_GetError(motor)` | 读错误码（0 正常） |
| `GO8010_Motor_IsAlive(motor)` | 最近一次反馈 CRC 是否正确 |

### 5.3 命令参数

混合控制公式：`τ = T + K_P×(Pos − p) + K_W×(W − ω)`

| 参数 | 含义 | 范围 | 定标 |
|------|------|------|------|
| `mode` | 0=锁定，1=FOC，2=编码器校准 | — | — |
| `T` | 前馈力矩 | -127.99~127.99 N·m | ×256 |
| `W` | 期望角速度（转子） | -804~804 rad/s | ÷2π×256 |
| `Pos` | 期望位置（转子） | rad | ÷2π×32768 |
| `K_P` | 位置刚度 | 0~25.599 | ×1280 |
| `K_W` | 速度阻尼 | 0~25.599 | ×1280 |

> **注意**：`W`/`Pos` 是**减速前转子**的量。输出轴 = 转子 ÷ 减速比（6.33）。
> 给输出轴目标要乘 `MOTOR_GEAR_RATIO`（6.33f）。

## 6. 控制模式示例

```c
/* 力矩模式：输出 0.05 N·m */
GO8010_Motor_SetCmd(&motor, 1, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f);

/* 速度模式：输出轴 6.28 rad/s */
GO8010_Motor_SetCmd(&motor, 1, 0.0f, 6.28f * MOTOR_GEAR_RATIO, 0.0f, 0.0f, 0.05f);

/* 位置模式：输出轴停在 3.14 rad */
GO8010_Motor_SetCmd(&motor, 1, 0.0f, 0.0f, 3.14f * MOTOR_GEAR_RATIO, 0.2f, 0.0f);

/* 锁定/停机 */
GO8010_Motor_SetCmd(&motor, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
```

## 7. 完整示例：带超时检测的控制循环

```c
while (1)
{
    /* 收：有回包则更新状态 */
    if (GO8010_Motor_Poll(&motor)) {
        // motor.data 已是最新状态
    }
    /* 超时：超过 10ms 没回包 = 失联 */
    else if (GO8010_Motor_IsTimeout(&motor, 10)) {
        // 停机 + 清 PID 积分 + 报警
        GO8010_Motor_SetCmd(&motor, 0, 0, 0, 0, 0, 0);
    }

    /* PID 计算（示例） */
    float err = target - GO8010_Motor_GetPos(&motor);
    float out = pid_compute(err);

    /* 发 */
    GO8010_Motor_SetCmd(&motor, 1, out, 0.0f, 0.0f, 0.0f, 0.0f);
    GO8010_Motor_Send(&motor);

    HAL_Delay(1);
}
```

## 8. 注意事项

1. **命令要持续发**：电机有通讯看门狗，一段时间收不到命令会掉使能。所以必须周期 `Send`（1kHz 合适）。
2. **Poll 先于 Send**：`Send` 会清 `rx_done` 标志，所以循环里要**先 `Poll` 解析、再 `Send`**，否则丢回包。
3. **发送间隔 > 回包时间（约 200µs）**：间隔太短会 `AbortReceive` 截断上一包。1kHz（1ms）没问题。
4. **单在途命令**：半双工，同一时刻只能有一个电机在收发，多电机要轮流 `Send`→`Poll`。
5. **UART 中断回调**：驱动内部定义了 `HAL_UART_RxCpltCallback`，若还有别的串口要用接收中断，需在回调里一起处理。
6. **定标**：`W`/`Pos` 是转子量，输出轴要乘/除减速比 6.33。

## 9. 协议摘要

- **命令帧 17 字节**：`0xFE 0xEE` + 模式字节(ID:4bit/模式:3bit/保留:1bit) + 12 字节参数(力矩/速度/位置/K_P/K_W) + CRC16
- **回包 16 字节**：`0xFD 0xEE` + 模式字节 + 11 字节反馈(力矩/速度/位置/温度/错误码/足端力) + CRC16
- **CRC**：CRC-CCITT（XMODEM 变体，init=0）
- **波特率**：4Mbps，8N1，小端
