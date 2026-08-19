#include "uart_interact.h"
#include "stm32h7xx_hal.h"   /* HAL_GetTick()：手动模式超时计时用 */
#include <string.h>

/* ======================== 接收状态机各状态 ======================== */
enum {
    RX_WAIT_HEAD = 0,   /* 等 0x55 */
    RX_WAIT_CMD,        /* 等 CMD */
    RX_WAIT_LEN,        /* 等 LEN */
    RX_WAIT_DATA,       /* 收数据域 */
    RX_WAIT_SUM,        /* 等校验，比对 */
    RX_WAIT_TAIL,       /* 等 0xBB */
};

/* ======================== 帧校验 ========================
 * SUM = XOR(DATA[0..LEN-1])，不含 0xBB。
 * buf 的前 len 个字节就是要参与校验的字节。
 */
uint8_t Interact_Checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t s = 0u;
    uint16_t i;
    for (i = 0u; i < len; i++) {
        s ^= buf[i];
    }
    return s;  // 返回校验结果
}

/* 从数据域按小端读一个 float（两端约定统一为小端） */
static float Interact_GetF32(const uint8_t *b)
{
    uint32_t u = (uint32_t)b[0]
               | ((uint32_t)b[1] << 8)
               | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 24);
    float f;
    memcpy(&f, &u, sizeof(f));  // 把uint32_t转为float
    return f;
}

/* 任务控制：0=复位 1=启动 2=暂停 */
static void Interact_OnTaskControl(UartInteract *it, uint8_t ctrl)
{
    if (it == NULL) {
        return;
    }

    switch (ctrl) {
        case 0: /* 复位 */
            if (it->t1 && it->chassis) {
                Chassis_Stop(it->chassis);
                Chassis_Task1_Init(it->t1, it->chassis);
            }
            it->task_paused = 0u;
            break;

        case 1: /* 启动 */
            it->task_paused = 0u;
            break;

        case 2: /* 暂停 */
            it->task_paused = 1u;
            if (it->chassis) {
                Chassis_Stop(it->chassis);
            }
            break;

        default:
            break;
    }
}

/* 挂起一条命令应答，等下一次 Poll 时作为 EVT_ACK 发出去。
 * 缓冲而不是在接收回调里直接发串口，避免中断上下文里做阻塞发送。 */
void UartInteract_RequestAck(UartInteract *it, uint8_t cmd, uint8_t result)
{
    if (it == NULL) {
        return;
    }

    it->ack_cmd      = cmd;
    it->ack_result   = result;
    it->ack_requested = 1u;
}

/* 收到一帧完整且校验通过的帧：按命令字分发处理 */
static void UartInteract_OnFrame(UartInteract *it)
{
    if (it == NULL) {
        return;
    }

    switch (it->rx_cmd) {
        // case CMD_SET_TARGET_DISTANCE:      /* 设置目标停车距离（单次命令，回执）*/
        //     if (it->rx_len == 4u && it->t1) {
        //         float d = Interact_GetF32(it->rx_data);
        //         Chassis_Task1_SetTargetDistance(it->t1, d);
        //         UartInteract_RequestAck(it, it->rx_cmd, 0u);
        //     } else {
        //         UartInteract_RequestAck(it, it->rx_cmd, 1u);  /* 长度错 -> NACK */
        //     }
        //     break;

        // case CMD_SEND_LINE_DATA:           /* 巡线数据（高频帧流，不回执） */
        //     if (it->rx_len == 9u && it->t1) {
        //         uint8_t valid = it->rx_data[0];
        //         float center = Interact_GetF32(&it->rx_data[1]);
        //         float slope  = Interact_GetF32(&it->rx_data[5]);
        //         if (valid) {
        //             Chassis_Task1_OnLineData(it->t1, center, slope);
        //         }
        //         /* valid==0 时不调用，line_found 保持 0，状态机停在找线 */
        //     }
        //     break;

        // case CMD_SEND_TOWER_DIST:          /* 到塔距离（高频帧流，不回执） */
        //     if (it->rx_len == 5u && it->t1) {
        //         uint8_t valid = it->rx_data[0];
        //         float dist   = Interact_GetF32(&it->rx_data[1]);
        //         if (valid) {
        //             Chassis_Task1_OnTowerDistance(it->t1, dist);
        //         }
        //     }
        //     break;

        // case CMD_SEND_JUNCTION:            /* T路口/右转信号（单次命令，回执） */
        //     if (it->rx_len == 1u) {
        //         if (it->rx_data[0] != 0u && it->t1) {
        //             Chassis_Task1_OnJunctionSignal(it->t1);
        //         }
        //         UartInteract_RequestAck(it, it->rx_cmd, 0u);
        //     } else {
        //         UartInteract_RequestAck(it, it->rx_cmd, 1u);
        //     }
        //     break;

        case CMD_TASK_CONTROL:             /* 任务控制（单次命令，回执） */
            if (it->rx_len == 1u) {
                Interact_OnTaskControl(it, it->rx_data[0]);
                UartInteract_RequestAck(it, it->rx_cmd, 0u);
            } else {
                UartInteract_RequestAck(it, it->rx_cmd, 1u);
            }
            break;

        case CMD_SET_VELOCITY:             /* 手动调试速度（帧流，不回执） */
            if (it->rx_len == 12u && it->chassis) {
                // 数据转回浮点数
                float vx    = Interact_GetF32(&it->rx_data[0]);
                float vy    = Interact_GetF32(&it->rx_data[4]);
                float omega = Interact_GetF32(&it->rx_data[8]);
                Chassis_SetVelocity(it->chassis, vx, vy, omega);
                it->last_velocity_ms = HAL_GetTick();  /* 记录最后一条有效速度帧时刻，喂超时看门狗 */
            } else {
                UartInteract_RequestAck(it, it->rx_cmd, 1u);  /* 长度错 -> NACK */
            }
            /* 手动接管后暂停任务状态机，避免自动逻辑和手控打架（可按需去掉） */
            it->task_paused = 1u;
            break;

        case CMD_HEARTBEAT:                /* 上位机定期心跳：回执一下，链路双向保活 */
            if (it->rx_len == 1u) {
                UartInteract_RequestAck(it, it->rx_cmd, 0u);
            }
            break;

        default:
            break;
    }
}

void UartInteract_Init(UartInteract *it,
                       Chassis *ch,
                       Chassis_Task1 *t1,
                       Interact_SendFn send)
{
    if (it == NULL) {
        return;
    }

    memset(it, 0, sizeof(*it));
    it->chassis = ch;
    it->t1      = t1;
    it->send    = send;
    it->rx_state = RX_WAIT_HEAD;
    it->velocity_timeout_ms = INTERACT_VELOCITY_TIMEOUT_MS;
}

/* 串口中断逐字节喂入；帧内一旦出错，回到等帧头重新同步 */
void UartInteract_RxByte(UartInteract *it, uint8_t byte)
{
    if (it == NULL) {
        return;
    }

    switch (it->rx_state) {
        case RX_WAIT_HEAD:
            if (byte == INTERACT_FRAME_HEAD) {
                it->rx_sum   = 0;      /* 从头帧开始累计校验 */
                it->rx_state = RX_WAIT_CMD;
            }
            break;

        case RX_WAIT_CMD:
            it->rx_cmd   = byte;
            // it->rx_sum  ^= byte;
            it->rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            it->rx_len   = byte;
            // it->rx_sum  ^= byte;

            if (it->rx_len > INTERACT_MAX_DATA_LEN) {
                it->rx_state = RX_WAIT_HEAD;  /* 长度非法，放弃本帧 */
            } else if (it->rx_len == 0u) {
                it->rx_state = RX_WAIT_SUM;   /* 无数据域，直接等校验 */
            } else {
                it->rx_cnt   = 0u;
                it->rx_state = RX_WAIT_DATA;
            }
            break;

        case RX_WAIT_DATA:
            it->rx_data[it->rx_cnt++] = byte;
            it->rx_sum               ^= byte;
            if (it->rx_cnt >= it->rx_len) {
                it->rx_state = RX_WAIT_SUM;
            }
            break;

        case RX_WAIT_SUM:
            if (byte == it->rx_sum) {
                it->rx_state = RX_WAIT_TAIL;
            } else {
                it->rx_state = RX_WAIT_HEAD;  /* 校验失败，重新同步 */
            }
            break;

        case RX_WAIT_TAIL:
            if (byte == INTERACT_FRAME_TAIL) {
                UartInteract_OnFrame(it);     /* 完整合法帧，处理 */
            }
            it->rx_state = RX_WAIT_HEAD;
            break;

        default:
            it->rx_state = RX_WAIT_HEAD;
            break;
    }
}

/* 组装并发送一帧 */
void UartInteract_SendFrame(UartInteract *it, uint8_t cmd,
                            const uint8_t *data, uint8_t len)
{
    uint8_t buf[INTERACT_FRAME_MAX_LEN];
    uint8_t n = 0u;
    uint8_t i;

    if (it == NULL || it->send == NULL) {
        return;
    }
    if (len > INTERACT_MAX_DATA_LEN) {
        return;
    }

    buf[n++] = INTERACT_FRAME_HEAD;
    buf[n++] = cmd;
    buf[n++] = len;
    for (i = 0u; i < len; i++) {
        buf[n++] = data[i];
    }
    uint8_t sum = Interact_Checksum(&buf[3], len);  /* 只对 DATA 逐字节异或 */
    buf[n++] = sum;
    buf[n++] = INTERACT_FRAME_TAIL;

    it->send(buf, n);
}

/* 主循环周期调用：把任务1待上报事件转成上行帧发出去
 * （心跳/定频状态回传根据你的周期节奏在外部调用 SendStatus） */
void UartInteract_Poll(UartInteract *it)
{
    Chassis_Task1_HostEvent ev;

    if (it == NULL || it->t1 == NULL || it->send == NULL) {
        return;
    }

    ev = Chassis_Task1_PopHostEvent(it->t1);
    switch (ev) {
        // case CHASSIS_TASK1_HOST_EVENT_TURN_DONE_1:   /* 旧巡线方案，注释保留 */
        //     UartInteract_SendFrame(it, EVT_TURN_DONE_1, NULL, 0u);
        //     break;

        // case CHASSIS_TASK1_HOST_EVENT_TURN_DONE_2:   /* 旧巡线方案，注释保留 */
        //     UartInteract_SendFrame(it, EVT_TURN_DONE_2, NULL, 0u);
        //     break;

        case CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1:
            UartInteract_SendFrame(it, EVT_MOVE_DONE_1, NULL, 0u);
            break;

        case CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2:
            UartInteract_SendFrame(it, EVT_MOVE_DONE_2, NULL, 0u);
            break;

        case CHASSIS_TASK1_HOST_EVENT_TASK_DONE:
            UartInteract_SendFrame(it, EVT_TASK_DONE, NULL, 0u);
            break;

        default:
            break;
    }

    /* 单次命令/心跳的应答（EVT_ACK），事件之后发出 */
    if (it->ack_requested) {
        uint8_t d[2] = { it->ack_cmd, it->ack_result };
        UartInteract_SendFrame(it, EVT_ACK, d, 2u);
        it->ack_requested = 0u;
    }
}

/* 里程计状态回传（可选）：55 84 0C {x,y,yaw} SUM BB */
void UartInteract_SendStatus(UartInteract *it)
{
    uint8_t buf[12];
    float x, y, yaw;

    if (it == NULL || it->send == NULL || it->chassis == NULL) {
        return;
    }

    x   = (float)it->chassis->mn.rea_x;
    y   = (float)it->chassis->mn.rea_y;
    yaw = (float)it->chassis->mn.yaw;

    memcpy(&buf[0], &x,   4u);
    memcpy(&buf[4], &y,   4u);
    memcpy(&buf[8], &yaw, 4u);

    UartInteract_SendFrame(it, CMD_REPORT_STATUS, buf, 12u);
}

// uint8_t UartInteract_IsPaused(const UartInteract *it)
// {
//     if (it == NULL) {
//         return 0u;
//     }
//     return it->task_paused;
// }

/* 手动模式超时自动停车：只对手动接管(task_paused=1)生效，自动任务不受影响。
 * 用无符号减法算时间差，天然处理 HAL_GetTick 回绕。 */
void UartInteract_CheckVelocityTimeout(UartInteract *it, uint32_t now_ms)
{
    uint32_t elapsed;

    if (it == NULL || it->chassis == NULL) {
        return;
    }
    if (!it->task_paused) {
        return;              /* 自动任务阶段不干预 */
    }

    elapsed = now_ms - it->last_velocity_ms;
    if (elapsed > it->velocity_timeout_ms) {
        Chassis_Stop(it->chassis);   /* 超时没收到新的有效速度帧，立即停车 */
    }
}
