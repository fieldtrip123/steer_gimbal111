
//
// Created by Administrator on 2024/12/4.
//
#include "pid.h"
pid_struct_t shoot_m3508_pid[2];
pid_struct_t shoot_m2006_pid;
pid_struct_t gimbal_yaw_speed_pid;
pid_struct_t gimbal_yaw_angle_pid;
pid_struct_t gimbal_pitch_speed_pid;
pid_struct_t gimbal_pitch_angle_pid;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
}

/**
  * @brief  pid calculation
  * @param  pid struct
    @param  reference value
    @param  feedback value
  * @retval calculation result
  */
float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
    return pid->output;
}


void all_pid_init()
{
    pid_init(&gimbal_yaw_speed_pid, 10, 0, 0,30000, 30000);//P=85,I=0,D=0
    pid_init(&gimbal_yaw_angle_pid, 1, 0, 0, -1000, 1000);//P=1,I=0,D=0
    pid_init(&gimbal_pitch_speed_pid, 40, 0, 0,16000, 16000);//P=85,I=0,D=0
    pid_init(&gimbal_pitch_angle_pid, 1, 0, 0, -100, 100);//P=1,I=0,D=0
    pid_init(&shoot_m2006_pid, 1, 0, 0, -100, 100);//P=1,I=0,D=0
    for (int i =0;i<2;i++)
    {
        pid_init(&shoot_m3508_pid[i], 15, 0, 0, 30000, 30000);
    }
}

