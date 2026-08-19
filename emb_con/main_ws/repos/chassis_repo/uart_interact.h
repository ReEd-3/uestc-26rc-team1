#ifndef UART_INTERACT_H
#define UART_INTERACT_H

#include <stdint.h>
#include "interact_cmds.h"
#include "chassis.h"
#include "chassis_task_1.h"

/* 底层串口发送钩子：由用户实现（例如 USART1_DMA 发送或阻塞发送） */
typedef void (*Interact_SendFn)(const uint8_t *data, uint16_t len);

/* 上行发送缓冲区大小：一帧最大 17B，留一点余量 */
#define INTERACT_TX_BUF_LEN  32u

/* 手动模式（只发速度帧）超时自动停车：超过该时长没收到新的有效速度帧就 Chassis_Stop()。
 * 取值要大于上位机实际的速度帧发送周期（比如 50Hz -> 20ms，设 200ms 留足余量）。 */
#define INTERACT_VELOCITY_TIMEOUT_MS  200u

typedef struct {
    Chassis       *chassis;   /* 底盘句柄（手动调试速度 / 状态回传用） */
    Chassis_Task1 *t1;        /* 任务1 句柄 */

    /* ---- 接收状态机 ---- */
    uint8_t rx_state;         /* 当前解析状态 */
    uint8_t rx_cmd;           /* 本帧命令字 */
    uint8_t rx_len;           /* 本帧数据长度 */
    uint8_t rx_data[INTERACT_MAX_DATA_LEN]; /* 本帧数据域 */
    uint8_t rx_cnt;           /* 已收数据字节数 */
    uint8_t rx_sum;           /* 边收边累计的校验 */

    /* ---- 发送钩子 ---- */
    Interact_SendFn send;

    /* ---- 任务控制 ---- */
    uint8_t task_paused;      /* 1=手动接管/暂停任务逻辑 */

    /* ---- 手动模式超时自动停车 ---- */
    uint32_t last_velocity_ms;      /* 最后一次收到"有效速度帧"的时刻(HAL_GetTick, ms) */
    uint32_t velocity_timeout_ms;   /* 超时阈值(ms)，超过则 Chassis_Stop() */

    /* ---- 命令应答（ACK/回执）---- */
    uint8_t ack_cmd;          /* 待回执的命令字 */
    uint8_t ack_result;       /* 待回执结果: 0=成功 1=失败 */
    uint8_t ack_requested;    /* 1=有待发送的应答 */
} UartInteract;

/* 初始化：把底盘和任务1句柄绑进去 */
void UartInteract_Init(UartInteract *it,
                       Chassis *ch,
                       Chassis_Task1 *t1,
                       Interact_SendFn send);

/* 串口接收中断里逐字节喂给状态机，收到完整合法帧后自动分发处理 */
void UartInteract_RxByte(UartInteract *it, uint8_t byte);

/* 主循环周期调用（放在你的控制周期里）：
 *  - 弹出任务1待上报事件并打包上行
 *  - 发送挂起的命令应答 EVT_ACK（单次命令/心跳的回执）
 *  - 手动调试模式下的任务暂停标志在这里被读取（供外部使用）
 */
void UartInteract_Poll(UartInteract *it);

/* 挂起一条命令应答，下一拍 Poll 时作为 EVT_ACK 发出。
 * 收到上位机"单次发送"的命令后由 OnFrame 自动调用；也可外部手动调用。 */
void UartInteract_RequestAck(UartInteract *it, uint8_t cmd, uint8_t result);

/* 手动组装并发送一帧（正常不需要直接调用，Poll 会自动上报） */
void UartInteract_SendFrame(UartInteract *it, uint8_t cmd,
                            const uint8_t *data, uint8_t len);

/* 里程计状态回传（可选）：55 84 0C {x,y,yaw} SUM BB */
void UartInteract_SendStatus(UartInteract *it);

/* 读取任务暂停标志：外部主循环据此决定是否调用 Chassis_Task1_Update() */
uint8_t UartInteract_IsPaused(const UartInteract *it);

/* 手动模式超时检查：主控在 1kHz 控制任务里每拍调用。
 * 若处于手动接管(task_paused)且距上次有效速度帧超过 velocity_timeout_ms，
 * 立即 Chassis_Stop()。now_ms 由 HAL_GetTick() 提供。 */
void UartInteract_CheckVelocityTimeout(UartInteract *it, uint32_t now_ms);

#endif
