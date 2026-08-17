#ifndef SUCTION_H
#define SUCTION_H

#include "stdint.h"
#include "stdbool.h"

/* 三个吸盘的电磁阀控制引脚（需根据实际硬件修改） */
#define SUCTION1_GPIO_Port  GPIOB
#define SUCTION1_Pin        GPIO_PIN_0   // 吸盘1（用于一区翻转和第一个KFS）

#define SUCTION2_GPIO_Port  GPIOB
#define SUCTION2_Pin        GPIO_PIN_1   // 吸盘2（侧面吸取第二个KFS）

#define SUCTION3_GPIO_Port  GPIOB
#define SUCTION3_Pin        GPIO_PIN_2   // 吸盘3（上面吸取第三个KFS）

/* 机械臂控制函数（用户需实现） */
void Arm1_MoveToFlip(void);    // 机械臂1动作：移动到翻转位置并旋转180°（GO8010）
void Arm1_MoveToGrabKFS1(void); // 机械臂1动作：移动到吸取第一个KFS的位置
void Arm2_MoveToGrabKFS2(void); // 机械臂2动作：移动到侧面吸取第二个KFS的位置
void Arm2_Flip90(void);         // 机械臂2动作：吸取第二个KFS后翻转90°使其朝上
void Arm2_MoveToGrabKFS3(void); // 机械臂2动作：移动到上面吸取第三个KFS的位置

void Suction_Init(void);
void Suction1_On(void);
void Suction1_Off(void);
void Suction2_On(void);
void Suction2_Off(void);
void Suction3_On(void);
void Suction3_Off(void);
bool Suction_IsHolding(uint8_t suction_id);

#endif
