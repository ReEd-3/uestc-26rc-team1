#ifndef INTERACT_CMDS_H
#define INTERACT_CMDS_H

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
#define CMD_SET_TARGET_DISTANCE  0x01u // 设置目标停车距离 (f32, 4B)【回执】
#define CMD_SEND_LINE_DATA       0x02u // 巡线数据 (valid+center+slope, 9B)【高频不回】
#define CMD_SEND_TOWER_DIST      0x03u // 到塔距离 (valid+dist, 5B)【高频不回】
#define CMD_SEND_JUNCTION        0x04u // T路口/右转信号 (u8 flag, 1B)【回执】
#define CMD_TASK_CONTROL         0x05u // 任务控制 (u8: 0复位 1启动 2暂停)【回执】
#define CMD_SET_VELOCITY         0x10u // 手动调速度 (vx+vy+omega, 12B)【不回】
#define CMD_HEARTBEAT            0xA0u // 上位机定期心跳 (u8 seq, 1B)，MCU 收到后回执

// ========================== 上行：底盘 -> 上位机 =====================
// 命令字用 0x80~0xFF
// ====================================================================
#define EVT_TURN_DONE_1          0x81u // 第一次右转完成 (空数据)
#define EVT_TURN_DONE_2          0x82u // 第二次右转完成 (空数据)
#define EVT_TASK_DONE            0x83u // 任务完成       (空数据)
#define CMD_REPORT_STATUS        0x84u // 状态回传 (x+y+yaw, 12B)，可选
#define EVT_ACK                  0x85u // 命令应答: 回执收到单次命令/心跳
                                       //   (u8 ack_cmd + u8 result, 2B) result:0成功 1失败

// 帧校验：对 0x55、CMD、LEN、DATA 逐字节异或
uint8_t Interact_Checksum(const uint8_t *buf, uint16_t len);

#endif
