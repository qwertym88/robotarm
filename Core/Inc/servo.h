#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "serial.h"
#include "pump.h"
#include "tim.h"

// 关节n旋转x度
#define JOINT1_TURN2(x) __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 500 + (x + 90) / 180.0 * 2000)
#define JOINT2_TURN2(x) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500 + (x) / 180.0 * 2000)
#define JOINT3_TURN2(x) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500 + (x) / 180.0 * 2000)

// 关节n输出pwm
#define JOINT1_PWM(x) __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, x)
#define JOINT2_PWM(x) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, x)
#define JOINT3_PWM(x) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, x)

// 坐标
typedef struct
{
    double x, y, z;
} point_t;

// 默认角度制
typedef struct
{
    double j1, j2, j3;
} angle_t;

uint8_t angleCheck(angle_t agl);
uint8_t pointCheck(point_t pos);

point_t angleToPoint(angle_t agl);
angle_t pointToAngle(point_t pos);
angle_t a2a(angle_t agl, uint8_t type);
uint8_t move(point_t p1, point_t p2, double V);
uint8_t movea(angle_t agl);
uint8_t movea_s(angle_t beg, angle_t end);
uint8_t curve(point_t ori, point_t dst, point_t op, point_t dp, double T);

#endif
