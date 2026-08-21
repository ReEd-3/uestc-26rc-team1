#ifndef UART_INTERACT_H
#define UART_INTERACT_H

#include <stdint.h>

// ============================== 帧格式 ==============================
//   55 | CMD | LEN | DATA | SUM | BB
//   - LEN: DATA 的字节数（不含头/命令/长度/校验/尾）
//   - SUM: DATA 逐字节异或（不含 0xBB）
//   因为有 LEN，数据里出现 0x55/0xBB 也不用转义。
// ====================================================================

#define INTERACT_FRAME_HEAD    0x55u   // 帧头
#define INTERACT_FRAME_TAIL    0xBBu   // 帧尾
#define INTERACT_MAX_DATA_LEN  12u     // 最大数据域（vx,vy,omega / 状态回传 12B）
#define INTERACT_FRAME_MAX_LEN (INTERACT_MAX_DATA_LEN + 5u) // 数据+头+命令+长度+校验+尾

// ========================== 下行：上位机 -> 底盘 =====================
// 命令字用 0x00~0x7F；带【回执】的是单次命令，处理好后 MCU 会回 EVT_ACK
// ====================================================================
// #define CMD_SET_TARGET_DISTANCE  0x01u // 设置目标停车距离 (f32, 4B)【回执】（精确校准，注释保留）
// #define CMD_SEND_LINE_DATA       0x02u // 巡线数据 (valid+center+slope, 9B)【高频不回】（旧巡线方案，注释保留）
// #define CMD_SEND_TOWER_DIST      0x03u // 到塔距离 (valid+dist, 5B)【高频不回】（精确校准，注释保留）
// #define CMD_SEND_JUNCTION        0x04u // T路口/右转信号 (u8 flag, 1B)【回执】（旧巡线方案，注释保留）
#define CMD_TASK_CONTROL         0x05u // 任务控制 (u8: 0复位 1启动 2暂停)【回执】
#define CMD_SET_VELOCITY         0x10u // 手动调速度 (vx+vy+omega, 12B)【不回】
#define CMD_HEARTBEAT            0xA0u // 上位机定期心跳 (u8 seq, 1B)，MCU 收到后回执

// ========================== 上行：底盘 -> 上位机 =====================
// 命令字用 0x80~0xFF
// ====================================================================
// #define EVT_TURN_DONE_1          0x81u // 第一次右转完成 (空数据)（旧巡线方案，注释保留）
// #define EVT_TURN_DONE_2          0x82u // 第二次右转完成 (空数据)（旧巡线方案，注释保留）
#define EVT_MOVE_DONE_1          0x81u // 第一次移动完成 (空数据)
#define EVT_MOVE_DONE_2          0x82u // 第二次移动完成 (空数据)
#define EVT_TASK_DONE            0x83u // 任务完成       (空数据)
#define CMD_REPORT_STATUS        0x84u // 状态回传 (x+y+yaw, 12B)，可选
#define EVT_ACK                  0x85u // 命令应答: 回执收到单次命令/心跳
                                       //   (u8 ack_cmd + u8 result, 2B) result:0成功 1失败

// 帧校验：对DATA异或
uint8_t Interact_Checksum(const uint8_t *buf, uint16_t len);

/* 底层串口发送钩子：由用户实现（例如 USART1_DMA 发送或阻塞发送） */
typedef void (*Interact_SendFn)(const uint8_t *data, uint16_t len);

/* 解析完成后的通用帧，协议层不关心具体业务含义 */
typedef struct {
    uint8_t cmd;                              /* 命令字 */
    uint8_t len;                              /* 数据长度 */
    uint8_t data[INTERACT_MAX_DATA_LEN];      /* 数据域 */
} Interact_Frame;

/* 完整合法帧回调：ctx 由 Init 时传入，frame 只在回调期间有效 */
typedef void (*Interact_FrameHandler)(void *ctx, const Interact_Frame *frame);

/* 纯协议层句柄：只负责帧解析/发送，不包含任何 Chassis/Task 业务字段 */
typedef struct {
    /* ---- 上层回调 ---- */
    Interact_FrameHandler on_frame;   /* 收到完整合法帧时调用 */
    void                *ctx;         /* 透传给回调的上下文 */
    Interact_SendFn      send;        /* 底层发送钩子 */

    /* ---- 接收状态机 ---- */
    uint8_t rx_state;                 /* 当前解析状态 */
    uint8_t rx_cmd;                   /* 本帧命令字 */
    uint8_t rx_len;                   /* 本帧数据长度 */
    uint8_t rx_data[INTERACT_MAX_DATA_LEN]; /* 本帧数据域 */
    uint8_t rx_cnt;                   /* 已收数据字节数 */
    uint8_t rx_sum;                   /* 边收边累计的校验 */
} UartInteract;

/* 初始化：绑定帧回调、上下文和发送钩子 */
void UartInteract_Init(UartInteract *it,
                       Interact_FrameHandler on_frame,
                       void *ctx,
                       Interact_SendFn send);

/* 串口接收中断/任务里逐字节喂给状态机，收到完整合法帧后回调 on_frame */
void UartInteract_RxByte(UartInteract *it, uint8_t byte);

/* 组装并发送一帧：55 CMD LEN DATA SUM BB */
void UartInteract_SendFrame(UartInteract *it, uint8_t cmd,
                            const uint8_t *data, uint8_t len);

#endif
