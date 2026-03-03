//
// Created by Gleam on 25-8-22.
//

#ifndef CTRBOARD_H7_ALL_GIMBAL_TASK_H
#define CTRBOARD_H7_ALL_GIMBAL_TASK_H

#define GIMBAL_WX_MAX        270.0f
#define GIMBAL_TURN_RATIO  0.1f  //单位应该为°,有关调节遥控器转向敏感度的系数,自行在安全范围内调节大小

void gimbal_control_task();
void gimbal_task_init(void);
#endif //CTRBOARD_H7_ALL_GIMBAL_TASK_H
