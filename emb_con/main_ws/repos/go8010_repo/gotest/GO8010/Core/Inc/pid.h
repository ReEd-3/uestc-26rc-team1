/**
 * @file    pid.h
 * @brief   GO8010 积分控制器（补 I，可移植）
 *
 * GO8010 电机内置了 P（K_P）和 D（K_W），只缺 I（积分）。
 * 本模块只做积分，输出前馈力矩 T，配合内置 PD 构成完整 PID：
 *     τ = T(本模块积分) + K_P×(Pos−p) + K_W×(W−ω)
 *
 * 用法（位置环 + 补 I）：
 *   PID_t pos_i;
 *   PID_Init(&pos_i, 0.5f, 0.001f, 50.0f, 10.0f);   // ki/dt/积分限幅/力矩限幅
 *   while (1) {
 *       float t = PID_Compute(&pos_i, target, GO8010_Motor_GetPos(&motor));
 *       GO8010_Motor_SetHybrid(&motor, t, 0, target, K_P, K_W);
 *       GO8010_Motor_SendRecv(&motor, 10);
 *   }
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float ki;              /* 积分增益 */
    float dt;              /* 采样周期（秒），1ms=0.001f */
    float integral;        /* 积分累加值 ∫e dt（内部用） */
    float integral_limit;  /* 积分限幅（>0 生效，<=0 不限幅），防积分饱和 */
    float output_limit;    /* 输出力矩 T 限幅（>0 生效，<=0 不限幅），防力矩超限 */
} PID_t;

/**
 * @brief 初始化积分控制器（一次设好增益、周期、限幅）
 * @param pid            PID 对象
 * @param ki             积分增益（从小到大调，消除静差）
 * @param dt             采样周期（秒），如 1ms 传 0.001f
 * @param integral_limit 积分限幅（<=0 不限幅）
 * @param output_limit   输出力矩 T 限幅（<=0 不限幅；GO8010 力矩范围 ±127.99）
 */
void PID_Init(PID_t *pid, float ki, float dt, float integral_limit, float output_limit);

/** 复位积分（模式切换/无扰切换时调用，清掉累积的积分） */
void PID_Reset(PID_t *pid);

/**
 * @brief 计算积分输出（前馈力矩 T）
 * @param pid     PID 对象
 * @param target  目标值（转子 rad 或 rad/s）
 * @param current 当前反馈值
 * @return 前馈力矩 T（N·m），已按 output_limit 限幅
 */
float PID_Compute(PID_t *pid, float target, float current);

#endif
