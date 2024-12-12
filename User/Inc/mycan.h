//
// Created by Administrator on 2024/12/4.
//

#ifndef  __MYCAN__H
#define  __MYCAN__H




#define  FEEDBACK_ID_M3508_BASE 0x205
#include "can.h"
#include "main.h"

typedef struct {

    uint16_t can_id;         //id
    int16_t  set_current;    //设定电流
    uint16_t rotor_angle;    //机械角度
    int16_t  rotor_speed;    //转速
    int16_t  torque_current; //扭矩电流
    uint8_t  temp;           //温度
    int    circle_count;
    int	target_count;
    int16_t  add_target;
    int16_t  add_encode;
    int16_t  last_encode;
    int16_t  last_target;
    uint16_t UART_chassis;
}    motor_t;

void my_can1_init(void);
void my_can2_init(void);
void set_m6020_v(int16_t v1,int16_t v2);
void set_m3508_v(int16_t v1,int16_t v2);
void set_m2006_v(int16_t v1);
void led(void);





#endif
