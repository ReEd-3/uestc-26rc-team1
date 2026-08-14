# GO-M8010-6 电机驱动库 使用手册

STM32（HAL 库）驱动宇树 **GO-M8010-6** 关节电机。RS-485 半双工，**4Mbps**，支持 7 种控制模式，附带一个**补 I 积分控制器**（消除位置静差）。所有收发细节封装在库里，使用者只需三步：**初始化 → 选模式 → 循环收发**。

---

## 1. 项目说明

```
| 项目 | 说明 |
|------|------|
| 串口 | 任意 UART，配成 **4Mbps、8N1**（8 位数据、无校验、1 停止位） |
| 方向脚 DE | 任意 GPIO 输出，**高=发送 / 低=接收**，软件控制，接收发器的 DE/RE |
| 收发器 | 必须支持 4Mbps：SP3485 / MAX3485（MAX485 只有 2.5Mbps，**不行**） |
| 电机 ID | 0~14，需与电机实际配置一致（15 为广播，无回包） |


---

## 2. CubeMX 配置

1. **串口**：选一个 UART，模式 Asynchronous，波特率 **4000000**，8 位、无校验、1 停止位。
2. **方向脚**：选一个 GPIO 引脚，设为 **Output**（推挽），初始电平随便（Init 会设为低=接收态）。
3. 生成代码时，UART 的 RX 中断**可开可不开**（本库用阻塞收发，不依赖中断）。

> 4Mbps 对时钟精度有要求，若收发不稳定，检查 PCLK 分频值是否精确。

---

## 3. 加入工程

把下面 8 个文件拷贝进工程，并加进 Keil 编译、设好头文件路径即可：

```
GO8010_driver.h / .c     驱动库本体（7 种模式 + 收发 + 反馈读取）
pid.h / .c               补 I 积分控制器（可选，做位置控制才用）
gom_protocol.h / .c      协议层（官方源文件）
crc_ccitt.h / .c         CRC 校验（官方）
```

---

## 4. 使用流程（三步）

```c
#include "GO8010_driver.h"

GO8010_Motor_t motor;            /* 每个电机一个实例 */

int main(void)
{
    /* ① 初始化：UART 句柄、DE 脚端口/引脚、电机ID */
    GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);

    /* ② 选一种控制模式（例：速度模式） */
    GO8010_Motor_SetVelocity(&motor, 6.28f, 0.02f);   /* 目标6.28rad/s(转子) + K_W */

    /* ③ 循环：阻塞收发（发送命令 + 收反馈 + 解析），1kHz */
    while (1)
    {
        if (GO8010_Motor_SendRecv(&motor, 10))        /* 10ms超时，返回1=有效回包 */
        {
            float pos = GO8010_Motor_GetPos(&motor);  /* 读位置（转子 rad） */
            float vel = GO8010_Motor_GetVel(&motor);  /* 读速度（转子 rad/s） */
        }
        HAL_Delay(1);                                 /* 1ms = 1kHz */
    }
}
```

> ⚠️ 命令必须**持续发送**（电机有通讯看门狗，断一会就掉使能），1kHz 合适，别只发一次。

---

## 5. 七种控制模式

| 模式 | 函数 | 用途 | 参数 |
|------|------|------|------|
| 停止 | `GO8010_Motor_Stop(motor)` | 电机锁死 | — |
| 速度 | `GO8010_Motor_SetVelocity(motor, w, k_w)` | 恒速旋转 | W 目标速度(rad/s) + K_W |
| 位置 | `GO8010_Motor_SetPosition(motor, pos, k_p, k_w)` | 停到指定位置 | Pos + K_P 刚度 + K_W 阻尼 |
| 阻尼 | `GO8010_Motor_SetDamping(motor, k_w)` | 被外力推着走（柔顺） | K_W |
| 力矩 | `GO8010_Motor_SetTorque(motor, t)` | 持续出力 | T（N·m） |
| 零力矩 | `GO8010_Motor_SetZeroTorque(motor)` | 完全放松 | — |
| 力位混合 | `GO8010_Motor_SetHybrid(motor, t, w, pos, k_p, k_w)` | 最常用，见 §6 | T+W+Pos+K_P+K_W |

**常见参数范围**：`K_P` 0~25.599（典型 0.2，越大越"硬"）；`K_W` 0~25.599（典型 0.05）；`T` -127.99~127.99 N·m。此处需要特别注意的是，给电机发送的命令都是针对减速器之前的电机转子，所以在进行实际控制的过程中，一定要注
意考虑电机的减速比。在 GO-8010-6 的电机中，减速比为 6.33。K和W要*减速比再作为api参数，以下是例子：
```c
/* 速度模式：转子 6.28 rad/s（≈1 圈/秒） */
GO8010_Motor_SetVelocity(&motor, 6.28f*MOTOR_GEAR_RATIO, 0.02f);

/* 位置模式：输出轴停在 0.5 rad（≈28.6°），K_P=0.1, K_W=0.05 */
GO8010_Motor_SetPosition(&motor, 0.5f * MOTOR_GEAR_RATIO, 0.1f, 0.05f);

/* 力矩模式：输出 0.1 N·m */
GO8010_Motor_SetTorque(&motor, 0.1f);

/* 阻尼模式：柔顺，转一下松手自己不动 */
GO8010_Motor_SetDamping(&motor, 0.05f);
```

---

## 6. PID：位置控制 + 补 I（推荐做法）

**背景**：GO-M8010-6 电机**内置了 P（K_P）和 D（K_W）**，只缺积分 I。本库的 PID 只做积分，输出**前馈力矩 T**，与内置 PD 一起构成完整 PID：

```
τ = T(积分) + K_P×(Pos − p) + K_W×(W − ω)
```

**用法（位置环补 I，走混合模式）**：

```c
#include "GO8010_driver.h"
#include "pid.h"

GO8010_Motor_t motor;
PID_t pos_i;

float target_rotor = 1.0f * MOTOR_GEAR_RATIO;  /* 目标位置（转子 rad，输出轴 1.0 rad） */
float K_P = 0.05f;   /* 位置刚度（内置 P） */
float K_W = 0.05f;   /* 速度阻尼（内置 D） */

int main(void)
{
    GO8010_Motor_Init(&motor, &huart9, GPIOF, GPIO_PIN_14, 1);

    /* PID_Init(ki, dt=1ms, 积分限幅, 输出力矩限幅) */
    PID_Init(&pos_i, 0.2f, 0.001f, 50.0f, 5.0f);

    while (1)
    {
        /* 积分输出前馈力矩 T */
        float t_ff = PID_Compute(&pos_i, target_rotor, GO8010_Motor_GetPos(&motor));

        /* 混合模式：T=积分, Pos=目标, K_P/K_W=内置PD */
        GO8010_Motor_SetHybrid(&motor, t_ff, 0.0f, target_rotor, K_P, K_W);
        GO8010_Motor_SendRecv(&motor, 10);
        HAL_Delay(1);
    }
}
```

**PID 三个 API**：

```c
PID_Init(&pos_i, ki, dt, 积分限幅, 力矩限幅);   /* 一次设好 */
PID_Reset(&pos_i);                              /* 换目标/换模式时清积分 */
float t = PID_Compute(&pos_i, target, current); /* 每周期调用，返回前馈力矩 T */
```

**调参顺序**：
1. `ki` 先设 0（纯内置 PD），把 `K_P`/`K_W` 调到位置稳定不震荡。
2. 从小到大加 `ki` 消除静差；`ki` 太大位置会来回晃。
3. 先设小力矩限幅（如 5 N·m）保安全，确认不猛冲再放开。

---

## 7. 单位：转子 vs 输出轴

电机带 **6.33 减速比**，协议里所有位置/速度都是**减速前转子**的量：

```
输出轴值 = 转子值 ÷ 6.33     （MOTOR_GEAR_RATIO）
```

- 要输出轴 1 rad/s → 下发 `1.0 × 6.33 = 6.33 rad/s`
- 读回转子 6.33 rad → 输出轴实际 1.0 rad

---

## 8. 反馈读取

| 函数 | 含义 |
|------|------|
| `GO8010_Motor_GetPos(motor)` | 位置（转子 rad） |
| `GO8010_Motor_GetVel(motor)` | 速度（转子 rad/s） |
| `GO8010_Motor_GetTorque(motor)` | 实际力矩（N·m） |
| `GO8010_Motor_GetTemp(motor)` | 温度（℃） |
| `GO8010_Motor_GetError(motor)` | 错误码：0正常 / 1过热 / 2过流 / 3过压 / 4编码器故障 / 5母线欠压 / 6绕组过热 |
| `GO8010_Motor_IsAlive(motor)` | 最近一次回包 CRC 是否正确（1=通讯正常） |

---

## 9. 常见问题（FAQ）

| 现象 | 排查方向 |
|------|----------|
| 电机不转，`timeout` 一直涨 | ① 接线/收发器；② 电机 ID 对不对；③ 波特率精确度；④ 电源没给 |
| 有回包但位置不动 | 模式参数设错、K_P 太小、目标单位忘乘/除了减速比 |
| 转一阵自动掉使能 | 命令没持续发（必须 1kHz 循环 `SendRecv`） |
| 多个电机 | 半双工同一时刻只能一个电机收发，多个电机轮流调用各自实例 |

---

## 10. 移植检查清单

- [ ] 串口 4Mbps、8N1 配好
- [ ] 收发器支持 4Mbps（SP3485/MAX3485）
- [ ] DE 脚接到收发器 DE/RE，GPIO 输出模式
- [ ] 电机 ID 与 Init 一致
- [ ] 电源够功率
- [ ] 主循环 1kHz 持续 `SendRecv`
