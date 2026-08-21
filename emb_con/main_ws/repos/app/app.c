#include "app.h"
#include <string.h>

void Error_Handler(void);

extern App_Context global_app;

static UART_HandleTypeDef *s_interact_uart;

static void App_UartSend(const uint8_t *data, uint16_t len)
{
    if (s_interact_uart != NULL) {
        HAL_UART_Transmit(s_interact_uart, (uint8_t *)data, len, 100u);
    }
}

static float App_GetF32(const uint8_t *b)
{
    uint32_t u = (uint32_t)b[0]
               | ((uint32_t)b[1] << 8)
               | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 24);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static void App_OnFrame(void *ctx, const Interact_Frame *frame)
{
    App_Context *app = (App_Context *)ctx;

    if (app == NULL || frame == NULL) {
        return;
    }

    switch (frame->cmd) {
        case CMD_TASK_CONTROL: {
            uint8_t result = 1u;
            if (frame->len == 1u) {
                uint8_t ctrl = frame->data[0];
                switch (ctrl) {
                    case 0u:
                        Chassis_Stop(&app->chassis);
                        Chassis_Task1_Init(&app->task1, &app->chassis);
                        app->task_paused = 0u;
                        result = 0u;
                        break;
                    case 1u:
                        app->task_paused = 0u;
                        result = 0u;
                        break;
                    case 2u:
                        app->task_paused = 1u;
                        Chassis_Stop(&app->chassis);
                        result = 0u;
                        break;
                    default:
                        break;
                }
            }
            uint8_t ack[2] = { frame->cmd, result };
            UartInteract_SendFrame(&app->interact, EVT_ACK, ack, 2u);
            break;
        }

        case CMD_SET_VELOCITY:
            if (frame->len == 12u) {
                float vx    = App_GetF32(&frame->data[0]);
                float vy    = App_GetF32(&frame->data[4]);
                float omega = App_GetF32(&frame->data[8]);
                Chassis_SetVelocity(&app->chassis, vx, vy, omega);
                app->last_velocity_ms = HAL_GetTick();
            } else {
                uint8_t ack[2] = { frame->cmd, 1u };
                UartInteract_SendFrame(&app->interact, EVT_ACK, ack, 2u);
            }
            app->task_paused = 1u;
            break;

        case CMD_HEARTBEAT:
            if (frame->len == 1u) {
                uint8_t ack[2] = { frame->cmd, 0u };
                UartInteract_SendFrame(&app->interact, EVT_ACK, ack, 2u);
            }
            break;

        default:
            break;
    }
}

void Global_Init(App_Context *glb_app, const App_Config *cfg)
{
    if (glb_app == NULL || cfg == NULL) {
        return;
    }

    s_interact_uart = cfg->interact_uart;

    /* 启动定时器 */
    // HAL_TIM_Base_Start_IT(cfg->update1_tim);

    /* 启动串口交互 */
    HAL_UART_Receive_IT(cfg->interact_uart, (uint8_t *)&glb_app->rx_byte, 1u);

    /* 启动 CAN 总线 */
    if (M3508_CAN_Init(&glb_app->m3508, 0b00001111, cfg->chassis_m3508_hfdcan) != HAL_OK) {
        Error_Handler();
    }
    glb_app->m3508.motors[0].rotation = 1;  // 左前轮
    glb_app->m3508.motors[1].rotation = -1; // 右前轮
    glb_app->m3508.motors[2].rotation = 1;  // 左后轮
    glb_app->m3508.motors[3].rotation = -1; // 右后轮
    HAL_FDCAN_Start(cfg->chassis_m3508_hfdcan);
    /* 使能 FDCAN 接收中断（RxBuffer 新消息） */
    HAL_FDCAN_ActivateNotification(cfg->chassis_m3508_hfdcan,
                                   FDCAN_IT_RX_BUFFER_NEW_MESSAGE,
                                   0);

    /* 初始化码盘 */
    EncoderOdo_Init(&glb_app->encoder, cfg->odo_hfdcan);
    HAL_FDCAN_Start(cfg->odo_hfdcan);
    HAL_FDCAN_ActivateNotification(cfg->odo_hfdcan,
                                   FDCAN_IT_RX_BUFFER_NEW_MESSAGE,
                                   0);
    EncoderOdo_SetBeginCnt(&glb_app->encoder);

    /* 电机速度环 PID */
    M3508_SpeedPID_Init(&glb_app->m3508,
                        cfg->m3508_kp,
                        cfg->m3508_ki,
                        cfg->m3508_kd,
                        cfg->chassis_dt);

    /* 初始化滤波器 */
    Int16_IIRFilter_Init(&glb_app->m3508.motors[0].speed_pid.iir_filter, cfg->m3508_iir_alpha);
    Int16_IIRFilter_Init(&glb_app->m3508.motors[1].speed_pid.iir_filter, cfg->m3508_iir_alpha);
    Int16_IIRFilter_Init(&glb_app->m3508.motors[2].speed_pid.iir_filter, cfg->m3508_iir_alpha);
    Int16_IIRFilter_Init(&glb_app->m3508.motors[3].speed_pid.iir_filter, cfg->m3508_iir_alpha);

    /* 初始化底盘 */
    Chassis_Init(&glb_app->chassis, &glb_app->m3508, &glb_app->encoder, cfg);

    /* 把电机安装方向同步给麦轮里程计 */
    glb_app->chassis.mn.rotation[0] = glb_app->m3508.motors[0].rotation;
    glb_app->chassis.mn.rotation[1] = glb_app->m3508.motors[1].rotation;
    glb_app->chassis.mn.rotation[2] = glb_app->m3508.motors[2].rotation;
    glb_app->chassis.mn.rotation[3] = glb_app->m3508.motors[3].rotation;

    /* 初始化任务1 */
    Chassis_Task1_Init(&glb_app->task1, &glb_app->chassis);

    /* 初始化通信协议 */
    UartInteract_Init(&glb_app->interact, App_OnFrame, glb_app, App_UartSend);

    /* 应用层初始状态 */
    glb_app->task_paused = 0u;
    glb_app->last_velocity_ms = HAL_GetTick();
    glb_app->arm_flag = 0u;
    glb_app->velocity_timeout_ms = cfg->velocity_timeout_ms;
}

uint8_t App_IsPaused(void)
{
    return global_app.task_paused;
}

void App_CheckVelocityTimeout(uint32_t now_ms)
{
    if (!global_app.task_paused) {
        return;
    }

    uint32_t elapsed = now_ms - global_app.last_velocity_ms;
    if (elapsed > global_app.velocity_timeout_ms) {
        Chassis_Stop(&global_app.chassis);
    }
}

void App_PollEvents(void)
{
    Chassis_Task1_HostEvent ev = Chassis_Task1_PopHostEvent(&global_app.task1);

    switch (ev) {
        case CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_1:
            UartInteract_SendFrame(&global_app.interact, EVT_MOVE_DONE_1, NULL, 0u);
            break;
        case CHASSIS_TASK1_HOST_EVENT_MOVE_DONE_2:
            UartInteract_SendFrame(&global_app.interact, EVT_MOVE_DONE_2, NULL, 0u);
            break;
        case CHASSIS_TASK1_HOST_EVENT_TASK_DONE:
            UartInteract_SendFrame(&global_app.interact, EVT_TASK_DONE, NULL, 0u);
            break;
        default:
            break;
    }
}
