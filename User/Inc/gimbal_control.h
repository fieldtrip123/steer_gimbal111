//
// Created by Administrator on 2024/12/4.
//

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "mycan.h"
#include "dr16.h"
#include "pid.h"
#define PI       3.14159265358979323846f
#define RAD2DEG  57.295779513f
#define DEG2ENC  22.752777777f
#define  M2006_REDUCTION_RATIO  (9.0f)
void control_yaw_6020(int16_t theta0);
void control_shoot_3508(int16_t v1,int16_t v2);
void control_pitch_6020(int16_t theta0);
void control_2006(int16_t v1);
void shoot();
void stop_shoot();
void all_stop();
typedef struct{
int16_t   UART_yaw;
int16_t   UART_pitch;
int16_t   UART_flag1;
int16_t   UART_flag2;

}boardA_info;







#endif
