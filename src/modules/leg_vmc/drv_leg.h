/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#ifndef _DRV_LEG_H
#define _DRV_LEG_H

#include "rm_module.h"



enum leg_state_e
{
	LEG_NORMAL	= 0,
	LEG_ERROR	= 1
};

typedef struct leg_obj
{
	/*单位m*/
	float l1,l4;  // l4=l1,与关节电机连接一侧杆长
	float l2,l3; // l3=l2,与轮毂电机连接一侧杆长
    float lBD;
	float motor_distance; //电机间距

	float phi1,phi4;
	float phi2,phi3;


	/*极限值*/
	float phi1_min; // PI/2
	float phi4_max; // PI/2
	/*杆状态*/
	int8_t leg_state;
	
	/*倒立摆长度*/
	float l0;
    float l0_average;
    float last_l0;

    /*用于计算LQR反馈矩阵K*/
    float l0_pow3;  //摆长三次方,下面同理
    float l0_pow2;

	/*倒立摆角度*/
	float phi0;
	/*倒立摆坐标*/
	float XC,YC;
	/*第二象限节点坐标*/
	float XB,YB;
	/*第二象限节点坐标*/
	float XD,YD;

    /*倒立摆期望长度*/
	float length_ref;
    /*倒立摆长度速度*/
    float d_l0;
    float d_l0_lpf;//一阶低通滤波值
    /*上一次倒立摆长度速度*/
    float last_dl0;
    /*倒立摆长度加速度*/
    float dd_l0;

    /*倒立摆角度速度*/
    float d_phi0;
    float last_phi0;

    /*theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float theta;
    /*d_theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float d_theta;
    float d_theta_lpf;//一阶低通滤波值
    float dd_theta;
    float last_d_theta;
    float last_theta;

    /*摆杆力矩*/
    float Tp;

    /* 受到地面的支持力（用于离地检测） */
    float support_force;
    /* 接触地面状态量 */
    uint8_t touch_ground;
    /* 一段时间未接触地面，进入离地状态（离地飞行） */
    uint8_t fly_flag;

    /* vmc计算l0,d_lo,dd_l0,theta,d_theta */
    void (*vmc_calc)(struct leg_obj *leg,struct ins_msg *ins,float dt);
    /* 求得腿部运动速度 [dl0; dphi0] */

    /* FT = [PendulumForce PendulumTorque] */
	void (*vmc_cal_T)(struct leg_obj *leg,float *Tmotor);

    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_L)(struct leg_obj *leg, float phi1_raw, float phi2_raw);
    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_R)(struct leg_obj *leg, float phi1_raw, float phi2_raw);

	int8_t (*input_leg_angle)(struct leg_obj *leg, float phi4, float phi1);
}leg_obj_t;

typedef struct
{
	/*单位mm*/
	float l1;  // l4=l1
	float l2; // l3=l2
	float motor_distance; //电机间距
    /* 后续可能需要加入更多参数 */
}leg_config_t;

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t * leg_register(leg_config_t *config);

#endif //_DRV_LEG_H
