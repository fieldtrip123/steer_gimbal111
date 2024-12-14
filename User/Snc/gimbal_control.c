//
// Created by Administrator on 2024/12/4.
//

#include "gimbal_control.h"
#include "mycan.h"


#define encode0  0
#define encode1  0
float    target_yaw_enc;
extern motor_t  shoot_m3508_info[2];
extern motor_t  shoot_m2006_info;
extern motor_t  yaw_6020_info;
extern motor_t  pitch_6020_info;
extern pid_struct_t shoot_m3508_pid[2];
extern pid_struct_t shoot_m2006_pid;
float target_2006_speed;
float target_3508_speed;
extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_angle_pid;
extern pid_struct_t gimbal_pitch_speed_pid;
extern pid_struct_t gimbal_pitch_angle_pid;
extern  boardA_info boardA_info1;
uint16_t now_yaw_enc;
uint16_t now_pitch_enc;
void control_yaw_6020(int16_t theta0)
{
      /*if   ((yaw_6020_info.last_target-theta0*DEG2ENC)>4096)
    {
        yaw_6020_info.target_count++;
    }
    else if ((yaw_6020_info.last_target-theta0*DEG2ENC)<-4096)
    {
        yaw_6020_info.target_count--;
    }
    yaw_6020_info.last_target = theta0;
    yaw_6020_info.add_target=(yaw_6020_info.target_count)*8192+theta0*DEG2ENC+encode0;
    target_yaw_enc=(int)yaw_6020_info.add_target;

    float angle;
   /* yaw_6020_info.last_target=encode0+theta0*DEG2ENC;*/

    /*now_yaw_enc=yaw_6020_info.rotor_angle  ;*/
   /* angle =  pid_calc(&gimbal_yaw_angle_pid, target_yaw_enc, yaw_6020_info.add_encode);//角度环
    pid_calc(&gimbal_yaw_speed_pid,angle, yaw_6020_info.rotor_speed);
    set_m6020_v(gimbal_yaw_speed_pid.output,gimbal_pitch_speed_pid.output);*/
    float angle;
    yaw_6020_info.add_target=encode0+theta0*DEG2ENC;
    now_yaw_enc=yaw_6020_info.rotor_angle;
    angle =  pid_calc(&gimbal_yaw_angle_pid, yaw_6020_info.add_target, now_yaw_enc);//角度环
    pid_calc(&gimbal_yaw_speed_pid,angle, yaw_6020_info.rotor_speed);
    set_m6020_v(gimbal_yaw_speed_pid.output,gimbal_pitch_speed_pid.output);
}

void control_pitch_6020(int16_t theta0)
{
    float angle;
    pitch_6020_info.add_target=encode1+theta0*DEG2ENC;
    now_pitch_enc=pitch_6020_info.rotor_angle;
    angle =  pid_calc(&gimbal_pitch_angle_pid, pitch_6020_info.add_target, now_pitch_enc);//角度环
    pid_calc(&gimbal_pitch_speed_pid,angle, pitch_6020_info.rotor_speed);
    set_m6020_v(gimbal_yaw_speed_pid.output,gimbal_pitch_speed_pid.output);
}

void control_shoot_3508(int16_t v1,int16_t v2)
{
    int target_3508_speed[2] ;
    target_3508_speed[0] = v1 * M2006_REDUCTION_RATIO;
    target_3508_speed[1] = v2 * M2006_REDUCTION_RATIO;

    shoot_m3508_info[0].set_current=pid_calc(&shoot_m3508_pid[0], target_3508_speed[0], shoot_m3508_info[0].rotor_speed);
    shoot_m3508_info[1].set_current=pid_calc(&shoot_m3508_pid[1], target_3508_speed[1], shoot_m3508_info[1].rotor_speed);
    set_m3508_v(shoot_m3508_info[0].set_current,shoot_m3508_info[1].set_current);
   
}
void control_2006(int16_t v1)
{
    target_2006_speed=v1*M2006_REDUCTION_RATIO;
    shoot_m2006_info.set_current=pid_calc(&shoot_m2006_pid, target_2006_speed, shoot_m2006_info.rotor_speed);
    set_m2006_v(shoot_m2006_info.set_current);
}

void shoot()
{
    if(boardA_info1.UART_flag1==3&&boardA_info1.UART_flag2==3)
    {
        control_shoot_3508(200,200);
        control_2006(30);
    }
    else
    {
        stop_shoot();

    }
}
void stop_shoot()
{
    set_m2006_v(0);
    set_m3508_v(0, 0);

}
void all_stop()
{
    set_m2006_v(0);
    set_m3508_v(0, 0);
    set_m2006_v(0);
}
