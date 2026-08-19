# 机器人工程

- 这是一个实现数个任务的，通过上位机和单片机进行控制的机器人工程

- 这个工作空间用来存放单片机（STM32H723ZGT6）的代码

- 与上位机的 USART 协议、FreeRTOS 任务架构等**已定事项**见下文；改动时先读本文件保持同步

## 任务

机器人一共要完成三个任务

1. 从启动区移动到指定位置，该位置有一个三个方块（每个边长35cm, 650g左右）竖向堆叠的"塔"， 机器人要在不推倒塔的情况下把最上方的方块上下翻转180°

2. 完成任务1后要经过一个高低不一的四平台（高的距离地面40cm，低的距离地面20cm），有三个相同颜色（颜色已知）和一个不同颜色的方块，识别出需要取的三个颜色相同的方块，经过一个15°，水平1.5m的向上斜坡把方块运送到斜坡上的地面

3. 把运送后的方块堆叠成一个竖向放置的塔

## 机器人机构

1. **主控**

    - 迷你主机

    - STM32H723ZGT6

2. **底盘**

    - 麦克纳姆轮*4

    - DJI M3508\*4 + c620电调\*4(CAN控制，FDCAN1/2/3 已初始化于 CubeMX)

3. **机械臂**

    - 可伸缩升降的机械臂*2

    - M3508电机*4控制伸缩升降

    - GO8010电机（RS485协议）* 2进行机械臂末端关节的旋转

    - 其中一个机械臂有一个吸盘，另外一个机械臂有两个吸盘，都使用电磁阀控制

4. **视觉寻路**

    - 2D激光雷达

    - 双目摄像头（可以检测深度）

## 任务思路

1. **任务1**

主机给主控板发信号（USART 协议已定，见下），主控板收到信号后执行两段移动：
1. 启动后向右前方移动指定相对位移 (x, y)；
2. 再向正前方移动指定距离，到位后停车并上报任务完成。
后续机械臂用吸盘吸附到方块后举起，然后关节电机旋转180°，再放下。
第一、二段移动量目前在 `chassis_task_1.c` 的初始化/Setter 中配置（尚未新增上位机命令）。
精确校准/塔距微调相关代码已暂时注释停用，**代码保留不删除**，后续需要可恢复。

2. **任务2**

之后机器人运动到高低不一的平台边，边运动边识别是否需要吸取，使用总共三个吸盘吸取方块后，一并上坡运动到平台上

3. **任务3**

到平台上后机械臂依次从下向上堆叠放置

## USART 通信协议（已定，上位机按此实现）

- 串口参数：**USART3 @115200，8N1，无流控**。
- 实现位于 `repos/chassis_repo/`：`interact_cmds.h`（命令/帧常量）、`uart_interact.c/h`（解析状态机 + 打包 + ACK）。

### 帧格式

```
0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB
```

- `LEN` = DATA 字节数（不含头/CMD/LEN/SUM/帧尾），范围 0~12。
- `SUM` = **仅对 DATA[0..LEN-1] 逐字节异或**（不含 0x55/CMD/LEN/0xBB）；空数据帧（LEN=0）SUM=0。
  - 计算：`sum = 0; for (b in DATA) sum ^= b;`
- 数据统一**小端**（f32 低字节在前）。
- 因帧里带 LEN，DATA 中出现 0x55/0xBB 无需转义。
- 接收方校验：收到 DATA 后逐字节异或，与 SUM 相等才认帧；否则整帧丢弃、回到等帧头。

### 命令字

下行 0x00~0x7F，上行 0x80~0xFF；`CMD_HEARTBEAT=0xA0` 是例外，按内容路由。

> 注：巡线、塔距精确校准相关命令已在代码中注释保留，当前不参与解析。

| 方向 | CMD | DATA 域 | 含义 |
|------|-----|---------|------|
| 下行 | 0x05 | u8 ctrl (1B) | 任务控制 0复位/1启动/2暂停【回执】|
| 下行 | 0x10 | f32 vx + f32 vy + f32 omega (12B) | 手动速度【不回】|
| 下行 | 0xA0 | u8 seq (1B) | 上位机心跳，MCU 回 ACK【回执】|
| 上行 | 0x81 | 空 | 第一次移动完成 |
| 上行 | 0x82 | 空 | 第二次移动完成 |
| 上行 | 0x83 | 空 | 任务完成 |
| 上行 | 0x84 | f32 x + f32 y + f32 yaw (12B) | 状态回传（可选：需主控周期调 `SendStatus`，默认不自动发）|
| 上行 | 0x85 | u8 ack_cmd + u8 result (2B) | EVT_ACK 应答，result: 0=成功 1=NACK |

### ACK/回执

- 单次命令（0x05/0xA0）处理成功后回 `55 85 02 {ack_cmd}{result} SUM BB`；长度错回 NACK（result=1）。
- 应答由 Comms 任务缓冲发送（非中断直发），处理成功下一拍发出。

### 手动模式（上位机只发 0x10 速度帧）

- 用法：上位机周期发送 `55 10 0C {vx}{vy}{omega 各 4B 小端 f32} {SUM} BB`。
- 首条**有效**速度帧（LEN=12 且 SUM 正确）即进入手动接管：主控停用自动任务1 FSM（`task_paused=1`），此后只按最后一条速度目标做底盘闭环。
- **发送周期必须 < `INTERACT_VELOCITY_TIMEOUT_MS`（默认 200ms）**，建议 10~100ms（如 50Hz=20ms）。超过该阈值没有新的**有效**速度帧 → 主控自动 `Chassis_Stop()` 停车。
- 坏帧（SUM 错 / LEN≠12）不会执行，也不会刷新超时计时，等同超时处理。
- 恢复发有效帧立即恢复运动。
- 手动模式默认**无上行**；如需链路确认可发 0xA0 心跳（收 0x85 ACK），或用可选的 0x84 状态回传。

### 单位约定

- 底盘/里程计：**m / rad**（`rea_x/y` m，yaw rad）。
- 手动速度：**vx/vy 单位 m/s，omega 单位 rad/s**；超过主控限幅（main.c `cfg`：max_vx=1、max_vy=1、max_omega=0.5）会被限幅。
- 巡线、塔距精确校准相关命令已暂时注释停用，**定义/处理代码保留不删除**，不再使用 `line_center/slope`、`tower_distance`、`target_distance`。

### 速度帧示例（给上位机）

发 vx=0.5、vy=0.0、omega=0.0（单位见上）：

```
55 10 0C  <0.5 小端4B>  <0.0 小端4B>  <0.0 小端4B>  <SUM> BB
```

`SUM` = 上述 12 个 DATA 字节的逐字节异或。

## FreeRTOS 任务架构（已定）

- 实现位于 `Core/Src/freertos.c`（CubeMX 生成的模板，改动只写在 USER CODE 区）

- **2 个任务 + 1 个字节队列**：

  | 任务 | 优先级 | 栈(字) | 职责 |
  |------|--------|--------|------|
  | ChassisMainTask | osPriorityRealtime | 256 | 1kHz 控制：`osThreadFlagsWait(0x01)` 等 TIM1 信号 → `Chassis_Task1_Update(&t1)`（内部含 `Chassis_Update`）|
  | Comms_Task | osPriorityLow | 256 | 串口通信：`osMessageQueueGet(CmdQueueTask1,..,10ms)` 阻塞 → `UartInteract_RxByte` 喂解析 → `Poll` 发事件/ACK |

- **节奏机制**：TIM1 = 1kHz（PSC=274, Period=999）。TIM1 中断里 `osThreadFlagsSet(ChassisMainTaskHandle,0x01)`（带 NULL 保护）唤醒控制任务；控制任务用 `osThreadFlagsWait` **阻塞等待**，保证让出 CPU 不饿死 Comms。Comms 靠队列阻塞，天然让出。

- **串口收**：USART3 @115200，中断单字节。`main.c` 初始化装填一次 `HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_byte,1)`，`HAL_UART_RxCpltCallback`（==USART3）里 `osMessageQueuePut` 入队 + 重新装填。`rx_byte` 为 `volatile`。

- **队列**：`CmdQueueTask1`，32 × uint8_t（串口字节队列，FIFO）。

- **串口发**：`uart_tx_hook`（main.c）用 `HAL_UART_Transmit` 阻塞发送，只在 Comms 任务上下文调用。

- **已实现**：
  - 手动接管分支：`StartChassisBaseTask` 里按 `UartInteract_IsPaused()` 分流——`task_paused=1` 时**跳过任务1 FSM、只跑 `Chassis_Update`**，避免手控与自动状态机打架
  - 手动模式超时自动停车：`UartInteract_CheckVelocityTimeout(&it, HAL_GetTick())`（1kHz 任务里调用）。手控时**超过 `INTERACT_VELOCITY_TIMEOUT_MS`(默认200ms) 没收到新的有效速度帧（0x10 且长度正确）→ `Chassis_Stop()`**；自动任务阶段不干预

- **尚待补充**：
  - 互斥量：`Chassis_Task1` 输入字段由 Comms 写、控制读，目前未加锁（vision<100Hz 时风险低，但 double 非原子）；需要时再加
  - 断链看门狗/上位机心跳(0xA0)失效急停：目前只有"速度帧超时自动停"，若需在自动任务期间也监测链路保活需另加
  - 机械臂（M3508 伸缩、GO8010/DrEmpower 关节）、吸盘电磁阀的任务划分：尚未定

## 构建与烧录

- 构建：`cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`，然后 `cmake --build build/Debug`
  - **注意**：顶层 `CMakeLists.txt` 里 `"-lm"` 必须以 `target_link_libraries` 的**最后一个链接项**加入（放 chassis 之后），否则 `chassis.c/macnum.c` 的 `cos/sin` 无法解析
- 烧录（需先 `sudo apt install stlink-tools openocd gdb-multiarch`，并确保 udev 权限）：
  - `arm-none-eabi-objcopy -O binary main.elf main.bin`
  - `st-flash --reset write main.bin 0x08000000`
  - 调试可选用 openocd/st-util + gdb-multiarch

## 代码层面结构（分层）

- 依赖方向单向向下：`main.c/freertos.c`(App) → `chassis_repo`(chassis+chassis_task_1+uart_interact+interact_cmds) → `m3508_repo` + `macnum_repo` → HAL(USD)
- FreeRTOS 直接调用的只有 `freertos.c` 两个任务入口；业务被层层调用（控制链：任务→Chassis_Task1→Chassis→Macnum/M3508→CAN；通信链：任务→UartInteract→Chassis_Task1/Chassis+发送钩子→USART）
- 引入HAL代码时使用`#include "stm32h7xx_hal.h"`
