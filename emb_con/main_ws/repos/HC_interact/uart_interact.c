#include "uart_interact.h"
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
    return s;
}

/* 初始化：只绑定协议回调、上下文和发送钩子 */
void UartInteract_Init(UartInteract *it,
                       Interact_FrameHandler on_frame,
                       void *ctx,
                       Interact_SendFn send)
{
    if (it == NULL) {
        return;
    }

    memset(it, 0, sizeof(*it));
    it->on_frame = on_frame;
    it->ctx      = ctx;
    it->send     = send;
    it->rx_state = RX_WAIT_HEAD;
}

/* 串口逐字节喂入；帧内一旦出错，回到等帧头重新同步 */
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
            it->rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            it->rx_len   = byte;

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
                /* 组装通用帧并交给上层回调 */
                Interact_Frame frame;
                frame.cmd = it->rx_cmd;
                frame.len = it->rx_len;
                if (it->rx_len > 0u) {
                    memcpy(frame.data, it->rx_data, it->rx_len);
                }
                if (it->on_frame != NULL) {
                    it->on_frame(it->ctx, &frame);
                }
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
