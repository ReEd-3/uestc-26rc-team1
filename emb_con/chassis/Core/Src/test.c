#include "test.h"
#include "chassis.h"
#include "suction.h"
#include "gray_sensor.h"
#include "odometry.h"
#include "m3508_driver.h"

/* ========== 底盘测试 ========== */
static uint32_t chassis_test_start_time;
static uint8_t  chassis_test_step = 0;

void Chassis_Test_Init(void) {
    chassis_test_start_time = HAL_GetTick();
    chassis_test_step = 0;
    Chassis_Stop();
}

void Chassis_Test_Run(void) {
    // 简单的动作序列：前进1秒 -> 停止 -> 旋转90° -> 停止 -> 巡线5秒
    switch (chassis_test_step) {
        case 0:  // 前进
            Chassis_SetBodyVelocity(0.2f, 0, 0);
            if (HAL_GetTick() - chassis_test_start_time > 1000) {
                chassis_test_step = 1;
                chassis_test_start_time = HAL_GetTick();
            }
            break;
        case 1:  // 停止
            Chassis_Stop();
            if (HAL_GetTick() - chassis_test_start_time > 500) {
                chassis_test_step = 2;
                chassis_test_start_time = HAL_GetTick();
            }
            break;
        case 2:  // 原地旋转90°（假设旋转角速度0.5 rad/s，需要π/2/0.5=3.14秒）
            Chassis_SetBodyVelocity(0, 0, 0.5f);
            if (HAL_GetTick() - chassis_test_start_time > 3140) {
                chassis_test_step = 3;
                chassis_test_start_time = HAL_GetTick();
            }
            break;
        case 3:  // 停止
            Chassis_Stop();
            if (HAL_GetTick() - chassis_test_start_time > 500) {
                chassis_test_step = 4;
                chassis_test_start_time = HAL_GetTick();
            }
            break;
        case 4:  // 巡线测试（需有白线）
        {
            float line_err = Gray_GetLineError();
            Chassis_LineFollow(line_err);
            if (HAL_GetTick() - chassis_test_start_time > 5000) {
                chassis_test_step = 5;
                chassis_test_start_time = HAL_GetTick();
            }
            break;
        }
        case 5:  // 测试结束，停止
            Chassis_Stop();
            break;
    }
}

/* ========== 机械臂/吸盘测试 ========== */
static uint32_t arm_test_start_time;
static uint8_t  arm_test_step = 0;

void Arm_Suction_Test_Init(void) {
    arm_test_start_time = HAL_GetTick();
    arm_test_step = 0;
    Suction1_Off();
    Suction2_Off();
    Suction3_Off();
}

void Arm_Suction_Test_Run(void) {
    switch (arm_test_step) {
        case 0:  // 测试吸盘1吸气和释放
            Suction1_On();
            if (HAL_GetTick() - arm_test_start_time > 1000) {
                Suction1_Off();
                arm_test_step = 1;
                arm_test_start_time = HAL_GetTick();
            }
            break;
        case 1:  // 测试吸盘2
            Suction2_On();
            if (HAL_GetTick() - arm_test_start_time > 1000) {
                Suction2_Off();
                arm_test_step = 2;
                arm_test_start_time = HAL_GetTick();
            }
            break;
        case 2:  // 测试吸盘3
            Suction3_On();
            if (HAL_GetTick() - arm_test_start_time > 1000) {
                Suction3_Off();
                arm_test_step = 3;
                arm_test_start_time = HAL_GetTick();
            }
            break;
        case 3:  // 测试机械臂动作
            Arm1_MoveToFlip();
            if (HAL_GetTick() - arm_test_start_time > 2000) {
                arm_test_step = 4;
                arm_test_start_time = HAL_GetTick();
            }
            break;
        case 4:  // 结束
            // 所有吸盘关闭
            Suction1_Off();
            Suction2_Off();
            Suction3_Off();
            break;
    }
}
