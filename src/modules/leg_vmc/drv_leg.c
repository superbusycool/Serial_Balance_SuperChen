/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#include "drv_leg.h"
#include "rm_algorithm.h"



#define LEFT 0
#define RIGHT 1
static uint8_t idx = 0; // register idx,是该文件的全局轮腿索引,在注册时使用
static leg_obj_t leg_obj[2];

uint8_t One_Older_LPF(float input, float* lpf_output ,float Klpf)//一阶低通滤波函数
{
    if(Klpf >= 1.0f)
        return 1;
    float X0 = input;
    *lpf_output= Klpf  * (*lpf_output) + (1 - Klpf) * X0;
    return 0;
}

static void vmc_calc(leg_obj_t *leg,struct ins_msg *ins,float dt)//右侧腿部计算vmc时使用
{
    leg->l4 = leg->l1;
    leg->l3 = leg->l2;
    /* xdb = xd-xb; ydb = yd-yb; */
    leg->XD= leg->motor_distance + leg->l4 * arm_cos_f32(leg->phi4);
    leg->XB = leg->l1 * arm_cos_f32(leg->phi1);
    leg->YD = leg->l4 * arm_sin_f32(leg->phi4);
    leg->YB = leg->l1 * arm_sin_f32(leg->phi1);

    leg->lBD = sqrtf((leg->XD - leg->XB)*(leg->XD - leg->XB) + (leg->YD - leg->YB)*(leg->YD - leg->YB));

    float A0 = 2 * leg->l2 * (leg->XD - leg->XB);
    float B0 = 2 * leg->l2 * (leg->YD - leg->YB);
    float C0 = leg->l2*leg->l2 + leg->lBD*leg->lBD - leg->l3*leg->l3;
    leg->phi2 = 2.0f * atan2f(B0 + sqrtf(A0*A0 + B0*B0 - C0*C0), (A0+C0));

    /*计算C坐标*/
    leg->XC = leg->XB + leg->l2 * arm_cos_f32(leg->phi2);
    leg->YC = leg->YB + leg->l2 * arm_sin_f32(leg->phi2);

    /*计算phi3*/
    leg->phi3 = atan2f((leg->YB - leg->YD) + leg->l2 * arm_sin_f32(leg->phi2), (leg->XB - leg->XD) + leg->l2 * arm_cos_f32(leg->phi2));

    /*输出摆长摆角,即c点的极坐标*/
    leg->phi0 = atan2f(leg->YC,(leg->XC - leg->motor_distance/2.0f));
    leg->l0 = sqrtf((leg->XC - leg->motor_distance/2.0f)*(leg->XC - leg->motor_distance/2.0f) + leg->YC*leg->YC);

    leg->d_phi0 = (leg->phi0 - leg->last_phi0)/dt;


    leg->theta = PI/2.0f - leg->phi0 - (-ins->pitch* DEGREE_2_RAD);
    leg->d_theta = (leg->theta - leg->last_theta) / dt;

    leg->d_theta = (leg->theta - leg->last_theta) / dt ;

    One_Older_LPF(leg->d_theta,&leg->d_theta_lpf,0.5f);//一阶低通滤波

    leg->last_phi0 = leg->phi0;
    leg->last_theta = leg->theta;

    /*l0的一阶和二阶导数,对应腿长变化的速度和加速度*/
    leg->d_l0 = (leg->l0 - leg->last_l0)/dt;

    One_Older_LPF(leg->d_l0,&leg->d_l0_lpf,0.5f);

    leg->last_l0 = leg->l0;
    leg->last_dl0 = leg->d_l0;
    leg->last_d_theta = leg->d_theta;
    leg->last_theta = leg->theta;


}

static void vmc_calc_inv(struct leg_obj *leg,float phi0_refer,float l0_refer)//vmc的逆解
{
    if(phi0_refer > -PI/2 && phi0_refer < PI/2){
        leg->XC_inv = leg->motor_distance / 2.0f + sqrtf((l0_refer * l0_refer) / (1 + tanf(phi0_refer) * tanf(phi0_refer)));
    }
    else{
        leg->XC_inv = leg->motor_distance / 2.0f - sqrtf((l0_refer * l0_refer) / (1 + tanf(phi0_refer) * tanf(phi0_refer)));
    }
    leg->YC_inv = tanf(phi0_refer) * (leg->XC_inv - leg->motor_distance / 2.0f);

    leg->a = leg->XC_inv * leg->XC_inv + leg->YC_inv * leg->YC_inv + leg->l1 * leg->l1 - leg->l2 * leg->l2 + 2 * leg->l1 * leg->XC_inv;
    leg->b = -4 * leg->l1 * leg->YC_inv;
    leg->c = leg->XC_inv * leg->XC_inv + leg->YC_inv * leg->YC_inv + leg->l1 * leg->l1 - leg->l2 * leg->l2 - 2 * leg->l1 * leg->XC_inv;

    leg->phi1_z1 = 2 * (atanf((-leg->b + sqrtf(leg->b * leg->b - 4 * leg->a * leg->c) ) / (2 * leg->a)));
    leg->phi1_z4 = 2 * (atanf((-leg->b - sqrtf(leg->b * leg->b - 4 * leg->a * leg->c) ) / (2 * leg->a)));

    leg->a = (leg->XC_inv - leg->motor_distance) * (leg->XC_inv - leg->motor_distance) + leg->YC_inv * leg->YC_inv + leg->l1 * leg->l1 - leg->l2 * leg->l2 + 2 * leg->l1 * (leg->XC_inv - leg->motor_distance);
    leg->b = -4 * leg->l1 * leg->YC_inv;
    leg->c = (leg->XC_inv - leg->motor_distance) * (leg->XC_inv - leg->motor_distance) + leg->YC_inv * leg->YC_inv + leg->l1 * leg->l1 - leg->l2 * leg->l2 - 2 * leg->l1 * (leg->XC_inv - leg->motor_distance);

    leg->phi4_z1 = 2 * (atanf((-leg->b + sqrtf(leg->b * leg->b - 4 * leg->a * leg->c) ) / (2 * leg->a)));
    leg->phi4_z4 = 2 * (atanf((-leg->b - sqrtf(leg->b * leg->b - 4 * leg->a * leg->c) ) / (2 * leg->a)));

    if((phi0_refer > 0.0f && phi0_refer <= PI/2) || (phi0_refer > -PI/2 && phi0_refer <= 0.0f)){
        leg->phi1_inv = leg->phi1_z1;
        leg->phi4_inv = fmodf(leg->phi4_z4 + PI2,PI2);
    }
    if((phi0_refer > PI/2 && phi0_refer <= PI) || (phi0_refer > -PI && phi0_refer <= -PI/2)){
        leg->phi1_inv = fmodf(leg->phi1_z1 + PI2,PI2);
        leg->phi4_inv = leg->phi4_z4;
    }


}
/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_inv:phi1对应的电机angle_abs;
 * @param phi4_inv:phi4对应的电机angle_abs
 * */
/*这两个函数在电机位置更换时重新写!!!*/
static void phi1_phi4_calc_left(struct leg_obj *leg, float phi1_inv, float phi4_inv){

    leg->phi1 = fmodf(PI2 - (phi1_inv - DM_ZERO_OFFSET_LF * DEGREE_2_RAD),PI2) ;
    leg->phi4 = fmodf(PI2 - (phi4_inv + DM_ZERO_OFFSET_LB * DEGREE_2_RAD),PI2) ;

}

/*
 * @brief 通过vmc_inv解算phi1_inv和phi2_inv对应的原始电机输出轴角度值(rad)
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_inv:phi1对应的电机对应的原始电机输出轴角度值(rad);
 * @param phi4_inv:phi4对应的电机对应的原始电机输出轴角度值(rad)
 * */
/*这两个函数在电机位置更换时重新写!!!*/
static void phi1_phi4_calc_left_inv(struct leg_obj *leg, float phi1_inv, float phi4_inv){

    leg->motor_phi1_inv_position_refer = fmodf(PI2 - phi1_inv + DM_ZERO_OFFSET_LF * DEGREE_2_RAD - PI2 ,PI2) ;
    leg->motor_phi4_inv_position_refer = fmodf(PI2 - phi4_inv - DM_ZERO_OFFSET_LB * DEGREE_2_RAD - PI2 ,PI2) ;

}

/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_raw:phi1对应的电机angle_abs;
 * @param phi4_raw:phi4对应的电机angle_abs
 * */
static void phi1_phi4_calc_right(struct leg_obj *leg, float phi1_raw, float phi4_raw){

    leg->phi1 = fmodf(phi1_raw + DM_ZERO_OFFSET_RF * DEGREE_2_RAD,PI2);
    leg->phi4 = fmodf(phi4_raw - DM_ZERO_OFFSET_RB * DEGREE_2_RAD,PI2);


}

/*
 * @brief 通过vmc_inv解算phi1_inv和phi2_inv对应的原始电机输出轴角度值(rad)
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_inv:phi1对应的电机对应的原始电机输出轴角度值(rad);
 * @param phi4_inv:phi4对应的电机对应的原始电机输出轴角度值(rad)
 * */
static void phi1_phi4_calc_right_inv(struct leg_obj *leg, float phi1_inv, float phi4_inv){

    leg->motor_phi1_inv_position_refer = fmodf(phi1_inv - DM_ZERO_OFFSET_RF * DEGREE_2_RAD -  PI2,PI2);
    leg->motor_phi4_inv_position_refer = fmodf(phi4_inv + DM_ZERO_OFFSET_RB * DEGREE_2_RAD - PI2 ,PI2);

}

/*
 * @brief 计算气弹簧在不同长度情况下,通过虚功原理得到的竖直方向上的支持
 * */
static void F_Spring_to_F_Vertical(struct leg_obj *leg){

    leg->theta_3 = acosf((leg->l1 * leg->l1 + leg->l2 * leg->l2 - leg->l0 * leg->l0 ) / (2 * leg->l1 * leg->l2));
    leg->l_s = sqrtf(leg->l6 * leg->l6 + leg->l_s2 * leg->l_s2 - 2 * leg->l6 * leg->l_s2 * arm_cos_f32(leg->theta_3 - leg->alpha_s));
    leg->F_Vertical = (leg->F_Spring * leg->l0 * leg->l6 * leg->l_s2 * arm_sin_f32(leg->theta_3 - leg->alpha_s)) / (leg->l_s * leg->l1 * leg->l2 * arm_sin_f32(leg->theta_3));

}

static int8_t input_leg_angle(leg_obj_t *leg, float phi4, float phi1)
{
    leg->phi4 = phi4;
    leg->phi1 = phi1;

    if(phi4>leg->phi4_max)
    {leg->leg_state = LEG_ERROR;}
    else if(phi1<leg->phi1_min)
    {leg->leg_state = LEG_ERROR;}
    else
    {leg->leg_state = LEG_NORMAL;}

    return (uint8_t)leg->leg_state;
}

/*Leg motors*/
/*FT = [F Tp]   Torque = [Motor3Torque(backmotor)  Motor2Torque(frontmotor)] */
static void vmc_calculation_Torque(leg_obj_t *leg,float *Tmotor)
{
    /*计算VMC*/
    volatile float q00,q01,q10,q11;
    /*中间变量*/
    volatile float sin32 = arm_sin_f32(leg->phi3 - leg->phi2);
    volatile float sin12 = arm_sin_f32(leg->phi1 - leg->phi2);
    volatile float sin34 = arm_sin_f32(leg->phi3 - leg->phi4);


    q00 = leg->l1 * arm_sin_f32(leg->phi0 - leg->phi3) * sin12 / sin32;
    q01 = leg->l1 * arm_cos_f32(leg->phi0 - leg->phi3) * sin12 / (leg->l0 *sin32);
    q10 = leg->l4 * arm_sin_f32(leg->phi0 - leg->phi2) * sin34 / sin32;
    q11 = leg->l4 * arm_cos_f32(leg->phi0 - leg->phi2) * sin34 / (leg->l0 *sin32);

    /*矩阵乘法,T1,T2,F,Tp*/
    Tmotor[0] = q00*leg->support_force + q01*leg->Tp;
    Tmotor[1] = q10*leg->support_force + q11*leg->Tp;
}

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t * leg_register(leg_config_t *config/* , void *control */)
{
    leg_obj[idx].l1 = config->l1;
    leg_obj[idx].l2 = config->l2;
    leg_obj[idx].motor_distance = config->motor_distance;
    leg_obj[idx].F_Spring = config->F_Spring;
    leg_obj[idx].l_s2 = config->l_s2;
    leg_obj[idx].l6 = config->l6;
    leg_obj[idx].alpha_s = config->alpha_s;
    leg_obj[idx].phi4_max = PI/2;
    leg_obj[idx].phi1_min = PI/2;
    leg_obj[idx].leg_state = LEG_ERROR;
    leg_obj[idx].phi0 = PI/2;
    leg_obj[idx].vmc_calc = vmc_calc;
    leg_obj[idx].vmc_calc_inv = vmc_calc_inv;
    leg_obj[idx].phi1_phi4_calc_left_inv = phi1_phi4_calc_left_inv;
    leg_obj[idx].phi1_phi4_calc_right_inv = phi1_phi4_calc_right_inv;
    leg_obj[idx].vmc_cal_T = vmc_calculation_Torque;
    leg_obj[idx].phi_calc_L = phi1_phi4_calc_left ;
    leg_obj[idx].phi_calc_R = phi1_phi4_calc_right ;
    leg_obj[idx].F_Spring_to_F_Vertical = F_Spring_to_F_Vertical;
    leg_obj[idx].input_leg_angle = input_leg_angle;

    return &leg_obj[idx++];
}

