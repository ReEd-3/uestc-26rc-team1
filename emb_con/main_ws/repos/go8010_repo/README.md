# GO-M8010-6 电机驱动库 使用说明

STM32(HAL) 通过 RS-485（4Mbps 半双工）驱动宇树 GO-M8010-6 关节电机。支持 7 种控制模式，另带一个补 I 积分控制器（消除位置静差）。收发全部封装，使用者只需：**初始化 → 选模式 → 循环收发**。

---

## 一、基本结构

| 文件 | 作用 |
|------|------|
| `GO8010_driver.h/.c` | 驱动库：7 种模式 + 阻塞收发 + 反馈读取 |
| `pid.h/.c` | 补 I 积分控制器（做位置控制才用，可选） |
| `gom_protocol.h/.c`、`crc_ccitt.h/.c` | 协议层 / CRC（官方原文件） |

核心调用就 3 个：

```c
GO8010_Motor_Init(&motor, &huart, DE端口, DE引脚, ID);   /* 初始化电机 */
GO8010_Motor_SetXxx(...);                                /* 选一种控制模式 */
GO8010_Motor_SendRecv(&motor, 超时ms);                   /* 阻塞收发，返回1=有效回包 */
```

**7 种控制模式**：

| 模式 | 函数 | 用途 |
|------|------|------|
| 停止 | `GO8010_Motor_Stop(motor)` | 锁死 |
| 速度 | `GO8010_Motor_SetVelocity(motor, w, k_w)` | 恒速旋转 |
| 位置 | `GO8010_Motor_SetPosition(motor, pos, k_p, k_w)` | 停到指定位置 |
| 阻尼 | `GO8010_Motor_SetDamping(motor, k_w)` | 柔顺（被外力推着走） |
| 力矩 | `GO8010_Motor_SetTorque(motor, t)` | 持续出力 |
| 零力矩 | `GO8010_Motor_SetZeroTorque(motor)` | 完全放松 |
| 力位混合 | `GO8010_Motor_SetHybrid(motor, t, w, pos, k_p, k_w)` | 位置控制推荐 |

**反馈读取**：`GetPos / GetVel / GetTorque / GetTemp / GetError / IsAlive`。/位置、速度、力矩、温度、错误、通讯/

---

## 二、建议操作顺序示例

```c
#include "GO8010_driver.h"

GO8010_Motor_t motor;   /* 每个电机一个实例 */

int main(void)
{
    GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);   /* ①初始化 */
    GO8010_Motor_SetVelocity(&motor, 6.28f, 0.02f);              /* ②选模式 */

    while (1)                                                    /* ③循环收发 */
    {
        GO8010_Motor_SendRecv(&motor, 10);
        HAL_Delay(1);
    }
}
```

5. **调参**：位置/速度不理想就改 `K_P`/`K_W`（范围 0~25.599，典型 K_P=0.2、K_W=0.05），位置静差用 PID 补 I。

---

## 三、注意细节

- **命令必须持续发**：1kHz 循环 `SendRecv`
- **单位是转子**：所有位置/速度都是减速前**转子**的量，输出轴 = 转子 ÷ 6.33（`MOTOR_GEAR_RATIO`）。给输出轴目标要 ×6.33。
- **收发器要支持 4Mbps**
- **半双工**：同一时刻只能一个电机收发，多电机轮流调各自实例。
- **错误码**：0正常 / 1过热 / 2过流 / 3过压 / 4编码器故障 / 5母线欠压 / 6绕组过热。
- **位置控制推荐混合模式 + 补 I**（电机内置 PD，本库补 I）：

```c
float t_ff = PID_Compute(&pos_i, target, GO8010_Motor_GetPos(&motor));
GO8010_Motor_SetHybrid(&motor, t_ff, 0.0f, target, K_P, K_W);
```

---
## 四、调试提示

| 现象 | 排查 |
|------|------|
| 电机不转，`timeout` 一直涨 | 接线/收发器、电机 ID、波特率、电源 |
| 转一阵掉使能 | 命令没持续发 |
| 实际速度低于目标 | 速度环稳态误差，加大 K_W |
| 报错误码 5 | 母线欠压，电源不够/线太细 |
