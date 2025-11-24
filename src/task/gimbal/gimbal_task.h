//
// Created by Gleam on 25-8-22.
//

#ifndef CTRBOARD_H7_ALL_GIMBAL_TASK_H
#define CTRBOARD_H7_ALL_GIMBAL_TASK_H
/**
  * @brief     自瞄相对角度传参
  */
typedef enum
{
    RELATIVE_ANGLE_TRANS = 0,             //云台正在回中
    RELATIVE_ANGLE_OK = 1,            //云台回中完毕
} auto_relative_angle_status_e;

void gimbal_control_task();
void gimbal_task_init(void);
#endif //CTRBOARD_H7_ALL_GIMBAL_TASK_H
