#include "robot_task.h"
#include "chassis.h"
#include "gray_sensor.h"
#include "vision.h"
#include "m3508_driver.h"
#include "suction.h"
#include "odometry.h"
#include "math.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define TEAM_COLOR  'R'

// 任务一区目标坐标（需根据场地标定，单位：米，弧度）
#define ZONE1_TARGET_X       1.0f
#define ZONE1_TARGET_Y       0.5f
#define ZONE1_TARGET_YAW     0.0f

// 移动至白线区参数
#define MOVE_TO_LINE_DISTANCE   1.2f
#define MOVE_TO_LINE_DIRECTION  0.0f

// 从进入白线区到二区的累计距离阈值（米），需根据场地标定
#define DISTANCE_TO_ZONE2       2.0f

extern Chassis_t chassis;
extern M3508_CAN_All m3508_can_1;
extern VisionInfo_t vision;

static RobotState_t robot_state = STATE_INIT;
static uint32_t state_enter_time;
static uint8_t  grabbed_count = 0;   // 已吸取的KFS数量（0~2）

// 用于记录巡线起点坐标，计算到二区的距离
static float line_start_x, line_start_y;

static bool Flip_KFS(void);
static bool Grab_KFS1(void);
static bool Grab_KFS2(void);
static bool Grab_KFS3(void);
static bool Place_KFS(void);
static void EnableSlopeFeedforward(void);
static void DisableSlopeFeedforward(void);

static bool Radar_GetPosition(float *x, float *y, float *yaw) {
    return false;   // 若雷达不可用可返回false
}

void RobotTask_Init(void) {
    robot_state = STATE_INIT;
    grabbed_count = 0;
    Odometry_Init(0.0f, 0.0f, 0.0f);
}

void RobotTask_Run(void) {
    Odometry_Update();

    float radar_x, radar_y, radar_yaw;
    if (Radar_GetPosition(&radar_x, &radar_y, &radar_yaw)) {
        Odometry_FuseRadar(radar_x, radar_y, radar_yaw);
    }

    float current_x, current_y, current_yaw;
    Odometry_GetPosition(&current_x, &current_y, &current_yaw);

    switch (robot_state) {
        case STATE_INIT:
            robot_state = STATE_MOVE_TO_ZONE1;
            state_enter_time = HAL_GetTick();
            break;

        case STATE_MOVE_TO_ZONE1:
        {
            bool arrived = Chassis_MoveToPoint(ZONE1_TARGET_X, ZONE1_TARGET_Y, ZONE1_TARGET_YAW,
                                               current_x, current_y, current_yaw);
            if (arrived) {
                robot_state = STATE_ZONE1_APPROACH_TOWER;
                state_enter_time = HAL_GetTick();
                Chassis_Stop();
            } else if (HAL_GetTick() - state_enter_time > 15000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_ZONE1_APPROACH_TOWER:
            Chassis_Stop();
            robot_state = STATE_ZONE1_FLIP_KFS;
            state_enter_time = HAL_GetTick();
            break;

        case STATE_ZONE1_FLIP_KFS:
        {
            Chassis_Stop();
            if (Flip_KFS()) {
                robot_state = STATE_MOVE_TO_LINE;
                state_enter_time = HAL_GetTick();
            } else if (HAL_GetTick() - state_enter_time > 5000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_MOVE_TO_LINE:
        {
            float target_x = current_x + MOVE_TO_LINE_DISTANCE * cosf(MOVE_TO_LINE_DIRECTION);
            float target_y = current_y + MOVE_TO_LINE_DISTANCE * sinf(MOVE_TO_LINE_DIRECTION);
            float target_yaw = current_yaw;

            bool arrived = Chassis_MoveToPoint(target_x, target_y, target_yaw,
                                               current_x, current_y, current_yaw);
            if (arrived) {
                // 到达白线区，记录巡线起点，准备前往二区
                line_start_x = current_x;
                line_start_y = current_y;
                robot_state = STATE_LINE_FOLLOW_TO_ZONE2;
                state_enter_time = HAL_GetTick();
                Chassis_Stop();
            } else if (HAL_GetTick() - state_enter_time > 10000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_LINE_FOLLOW_TO_ZONE2:
        {
            float line_err = Gray_GetLineError();
            Chassis_LineFollow(line_err);

            // 计算从巡线起点到当前位置的直线距离
            float dx = current_x - line_start_x;
            float dy = current_y - line_start_y;
            float dist_travelled = sqrtf(dx*dx + dy*dy);

            // 判断是否到达二区：累计距离超过预设值
            if (dist_travelled >= DISTANCE_TO_ZONE2) {
                robot_state = STATE_ZONE2_FIND_KFS;
                state_enter_time = HAL_GetTick();
                Chassis_Stop();
            }
            else if (HAL_GetTick() - state_enter_time > 15000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_ZONE2_FIND_KFS:
        {
            Chassis_Stop();
            if (vision.updated && vision.color == TEAM_COLOR) {
                vision.updated = false;
                robot_state = STATE_ZONE2_GRAB_KFS;
                state_enter_time = HAL_GetTick();
            } else if (HAL_GetTick() - state_enter_time > 5000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_ZONE2_GRAB_KFS:
        {
            Chassis_Stop();
            bool grab_ok = false;
            switch (grabbed_count) {
                case 0: grab_ok = Grab_KFS1(); break;   // 第一个KFS，吸盘1
                case 1: grab_ok = Grab_KFS2(); break;   // 第二个KFS，吸盘2+翻转90
                case 2: grab_ok = Grab_KFS3(); break;   // 第三个KFS，吸盘3
                default: break;
            }

            if (grab_ok) {
                grabbed_count++;
                robot_state = STATE_CARRY_TO_ZONE3;
                state_enter_time = HAL_GetTick();
            } else if (HAL_GetTick() - state_enter_time > 5000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_CARRY_TO_ZONE3:
        {
            EnableSlopeFeedforward();
            float line_err = Gray_GetLineError();
            Chassis_LineFollow(line_err);

            // 视觉组提供上坡结束标志
            if (vision.updated && vision.slope_finished) {
                vision.updated = false;
                DisableSlopeFeedforward();
                robot_state = STATE_ZONE3_PLACE_KFS;
                state_enter_time = HAL_GetTick();
                Chassis_Stop();
            }
            else if (HAL_GetTick() - state_enter_time > 20000) {
                DisableSlopeFeedforward();
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_ZONE3_PLACE_KFS:
        {
            Chassis_Stop();
            if (Place_KFS()) {
                if (grabbed_count >= 3) {
                    robot_state = STATE_DONE;
                } else {
                    robot_state = STATE_LINE_FOLLOW_TO_ZONE2;
                }
                state_enter_time = HAL_GetTick();
            } else if (HAL_GetTick() - state_enter_time > 5000) {
                robot_state = STATE_ERROR;
            }
            break;
        }

        case STATE_DONE:
            Chassis_Stop();
            break;

        case STATE_ERROR:
            Chassis_Stop();
            break;
    }
}

/* ==================== 动作执行函数 ==================== */

static bool Flip_KFS(void) {
    static uint8_t step = 0;
    static uint32_t step_time = 0;
    const uint32_t suction_delay = 500;
    const uint32_t rotate_delay = 1500;   // GO8010旋转时间
    const uint32_t release_delay = 300;

    switch (step) {
        case 0:
            Arm1_MoveToFlip();
            Suction1_On();
            step = 1;
            step_time = HAL_GetTick();
            break;
        case 1:
            if (HAL_GetTick() - step_time > suction_delay) {
                if (Suction_IsHolding(1)) {
                    // 旋转机构执行180°翻转
                    step = 2;
                    step_time = HAL_GetTick();
                } else {
                    Suction1_Off();
                    step = 0;
                    return false;
                }
            }
            break;
        case 2:
            if (HAL_GetTick() - step_time > rotate_delay) {
                Suction1_Off();
                step = 3;
                step_time = HAL_GetTick();
            }
            break;
        case 3:
            if (HAL_GetTick() - step_time > release_delay) {
                step = 0;
                return true;
            }
            break;
    }
    return false;
}

static bool Grab_KFS1(void) {
    static uint8_t step = 0;
    static uint32_t step_time = 0;
    const uint32_t suction_delay = 500;

    switch (step) {
        case 0:
            Arm1_MoveToGrabKFS1();
            Suction1_On();
            step = 1;
            step_time = HAL_GetTick();
            break;
        case 1:
            if (HAL_GetTick() - step_time > suction_delay) {
                if (Suction_IsHolding(1)) {
                    step = 0;
                    return true;
                } else {
                    Suction1_Off();
                    step = 0;
                    return false;
                }
            }
            break;
    }
    return false;
}

static bool Grab_KFS2(void) {
    static uint8_t step = 0;
    static uint32_t step_time = 0;
    const uint32_t suction_delay = 500;
    const uint32_t flip90_delay = 1000;

    switch (step) {
        case 0:
            Arm2_MoveToGrabKFS2();
            Suction2_On();
            step = 1;
            step_time = HAL_GetTick();
            break;
        case 1:
            if (HAL_GetTick() - step_time > suction_delay) {
                if (Suction_IsHolding(2)) {
                    Arm2_Flip90();
                    step = 2;
                    step_time = HAL_GetTick();
                } else {
                    Suction2_Off();
                    step = 0;
                    return false;
                }
            }
            break;
        case 2:
            if (HAL_GetTick() - step_time > flip90_delay) {
                step = 0;
                return true;
            }
            break;
    }
    return false;
}

static bool Grab_KFS3(void) {
    static uint8_t step = 0;
    static uint32_t step_time = 0;
    const uint32_t suction_delay = 500;

    switch (step) {
        case 0:
            Arm2_MoveToGrabKFS3();
            Suction3_On();
            step = 1;
            step_time = HAL_GetTick();
            break;
        case 1:
            if (HAL_GetTick() - step_time > suction_delay) {
                if (Suction_IsHolding(3)) {
                    step = 0;
                    return true;
                } else {
                    Suction3_Off();
                    step = 0;
                    return false;
                }
            }
            break;
    }
    return false;
}

static bool Place_KFS(void) {
    static uint8_t step = 0;
    static uint32_t step_time = 0;
    const uint32_t adjust_delay = 500;
    const uint32_t release_delay = 300;

    switch (step) {
        case 0:
            Chassis_Stop();
            step = 1;
            step_time = HAL_GetTick();
            break;
        case 1:
            if (HAL_GetTick() - step_time > adjust_delay) {
                if (grabbed_count == 1) Suction1_Off();
                else if (grabbed_count == 2) Suction2_Off();
                else if (grabbed_count == 3) Suction3_Off();
                step = 2;
                step_time = HAL_GetTick();
            }
            break;
        case 2:
            if (HAL_GetTick() - step_time > release_delay) {
                step = 0;
                return true;
            }
            break;
    }
    return false;
}

/* ==================== 电流前馈 ==================== */

static void EnableSlopeFeedforward(void) {
    int16_t ff = 500;
    for (int i = 0; i < 8; i++) {
        if (m3508_can_1.motors[i].status == M3508_ON) {
            M3508_SetCurrentFeedforward(&m3508_can_1.motors[i], ff);
        }
    }
}

static void DisableSlopeFeedforward(void) {
    for (int i = 0; i < 8; i++) {
        if (m3508_can_1.motors[i].status == M3508_ON) {
            M3508_SetCurrentFeedforward(&m3508_can_1.motors[i], 0);
        }
    }
}
