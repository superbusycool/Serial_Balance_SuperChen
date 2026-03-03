#ifndef ROBOT_H
#define ROBOT_H

#include "dji_motor.h"
#include "stdbool.h"
#include "chassis_task.h"



/**
 * @brief 机器人初始化,请在开启rtos之前调用
 * 
 */
void robot_init();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 * 
 */
void robot_task();

/* ------------------------------- ipc uMCN 相关 ------------------------------ */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度,°/s
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;  /*yaw,pitch,roll都为°*/
    float pitch;
    float yaw;
    float yaw_total_angle;
};

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    BACK_STEP = 0,             //云台正在回中
    BACK_IS_OK = 1,            //云台回中完毕
} gimbal_back_e;

/**
 * @brief 云台真实反馈状态数据,由gimbal发布
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;  // 云台归中情况

    float yaw_angle;    //云台初始 yaw 轴角度 （由yaw轴dm4310电机得）
    float pitch_angle;    //云台初始 pitch 轴角度 （由imu得）
    float roll_angle;    //云台初始 roll 轴角度 （由imu得）
};

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_RELAX = 0,        //云台断电
    GIMBAL_INIT = 1,         //云台初始化
    GIMBAL_GYRO = 2,         //云台跟随imu闭环
    GIMBAL_AUTO = 3          //云台自瞄
} gimbal_mode_e;

struct gimbal_cmd_msg
{ // 云台期望角度控制
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;  // 当前云台控制模式
    gimbal_mode_e last_mode;  // 上一次云台控制模式
};




//// TODO：后续优化启用，目前时间紧急，使用extern
//struct referee_msg
//{
//    robot_status_t robot_status;
//    ext_power_heat_data_t power_heat_data_t;
//};

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief cmd发布的底盘控制数据,由chassis订阅
 */
struct chassis_cmd_msg
{
    float vx;                  // 前进方向速度
    float vx_set;              //前进速度斜坡过程值

    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    float vw_set;              //转向速度斜坡过程值
    // TODO: 轮腿前期调试使用
    float leg_length;          // 腿长
    float leg_angle;           // 腿角度
    float offset_angle;        // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
    leg_level_e leg_level;     // 腿长等级
    leg_change_e leg_leng_change;      //腿长变化

};

/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
struct chassis_fdb_msg
{
    leg_back_state_e leg_state;  // 腿部归中初始化情况
    chassis_stand_state_e stand_state;  // 机器人站立状态
    /*  底盘任务使用到的电机句柄,仅能对其 measure 成员当作传感器数据读取，禁止改写 */
    dji_motor_measure_t M3508_l;  //左轮毂电机
    dji_motor_measure_t M3508_r;  //右轮毂电机

    bool touch_ground;         // 是否触地
};

/**
 * @brief 发射器模式
 */
typedef enum
{
    /*发射模式*/
    SHOOT_STOP=0        ,     //射击关闭
    SHOOT_ONE=1         ,     //单发模式
    SHOOT_THREE=2       ,     //三连发模式
    SHOOT_COUNTINUE=3   ,     //自动射击
    SHOOT_REVERSE=4     ,     //堵弹反转
    SHOOT_AUTO=5        ,     //自动发射模式
} shoot_mode_e;

/**
 * @brief 扳机模式
 */
typedef enum
{

    /*扳机状态*/
    TRIGGER_ON=1      ,     //扳机开火状态
    TRIGGER_OFF=0     ,     //扳机闭火状态
    TRIGGER_ING=2     ,     //扳机持续状态

} trigger_mode_e;

/**
 * @brief cmd发布的云台控制数据,由shoot订阅
 */
struct shoot_cmd_msg
{ // 发射器
    shoot_mode_e ctrl_mode;  // 当前发射器控制模式
    shoot_mode_e last_mode;  // 上一次发射器控制模式
    trigger_mode_e trigger_status;
    int16_t shoot_freq;      // 发射弹频
    // TODO: 添加发射弹速控制
    int16_t shoot_speed;     // 发射弹速
    uint8_t cover_open;      // 弹仓盖开关
    uint8_t mirror_enable;     // 倍镜使能开关
    bool friction_status;
};

/**
  * @brief   发射器状态回馈
  */
//TODO:具体回馈设置待讨论
typedef enum
{
    SHOOT_OK=1,   //发射正常
    SHOOT_ERR=0,  //发射异常
    SHOOT_WAITING=2, //发射异常
} shoot_back_e;

/**
 * @brief 发射机真实反馈状态数据,由shoot发布
 */
struct shoot_fdb_msg
{
    shoot_back_e trigger_status;  // shoot状态反馈
    int16_t trigger_motor_current; //拨弹电机电流，传给cmd控制反转
    int shoot_cnt;
};

/* ------------------------------ trans解析自瞄数据 ------------------------------ */
/**
 * @brief 上位机自瞄数据,由trans发布
 */
struct trans_fdb_msg
{
    float yaw;
    float pitch;
    float roll;
    uint8_t heartbeat;
};

#endif