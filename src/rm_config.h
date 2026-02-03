 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

#define CPU_FREQUENCY 168     /* CPU主频(mHZ) */


#define BSP_CHASSIS_LEG_MODE

#include "stm32h723xx.h" // 使用的芯片
#include "cmsis_os.h" // 使用的 OS 头文件
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif


/* 底盘和云台分别对应的 can 总线 */
#define CAN_GIMBAL     hfdcan2
#define CAN_WHEEL_MOTOR hfdcan2
#define CAN_ID_WHEEL_MOTOR 2
#define CAN_ID_GIMBAL_MOTOR 2
#define CAN_ID_CHASSIS_MOTOR 1
#define CAN_ID_FIRE_MOTOR 3


/* ---------------------------------- 遥控器相关 --------------------------------- */
#define RC_MAX_VALUE      784.0f  /* 遥控器通道最大值 */

/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 0.4f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f

/* ---------------------------------- 底盘相关 ---------------------------------- */
/*chassis_can_allocation*/
#define CHASSIS_JOINT_LEFT_FRONT     4
#define CHASSIS_JOINT_RIGHT_FRONT    3
#define CHASSIS_JOINT_LEFT_BACK      1
#define CHASSIS_JOINT_RIGHT_BACK     2
#define CHASSIS_JOINT_AND_YAW_RX_ID  0X10

#define CHASSIS_WHEEL_LEFT_ID    0X201
#define CHASSIS_WHEEL_RIGHT_ID   0X202


/*腿部角度phi1和phi2计算时涉及*/
#define DM_ZERO_OFFSET_LF  62.17f   /*对应phi1*/
#define DM_ZERO_OFFSET_LB  35.22f   /*对应phi2*/
#define DM_ZERO_OFFSET_RF  62.17f
#define DM_ZERO_OFFSET_RB  35.22f

#define LEG_SAFE_AREA      20.0f  /*能接受的腿部theta角运动范围扩展角度*/

/*查3508资料和xroll减速箱淘宝详情得到*/
#define M3508_TOR_TO_CUR  2220  //扭矩电流系数
#define M3508_TOR_MAX  3.69  //堵转扭矩
#define M3508_READUCTION_RATIO_R   15.7647        //15.77/1.12 = 14.08  //268/17 貌似淘宝店给的参数不对  //右轮3508改装xroll减速箱减速比
#define M3508_READUCTION_RATIO_L   15.7647        //26.34/2.1 = 12.543   //268/17   //右轮3508改装xroll减速箱减速比

/*底盘机体相关*/
#define WHEEL_RADIUS  0.058f      //轮子半径/m
#define m_b  6.120f
#define g  9.81f
#define Rl 0.214f    //轮间距

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 2000
#define MAX_CHASSIS_VY_SPEED 20 // 对应平步底盘，该值为roll轴倾斜
/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 360

/* ---------------------------------- 云台相关 ---------------------------------- */

 /* [0]为yaw，[1]为pitch */
#define YAW 0
#define PITCH 1
 /*gimbal_can allocation*/
#define YAW_MOTOR_ID     6
#define PITCH_MOTOR_ID   0x206

#define CENTER_ECD_YAW   0        //云台yaw轴编码器归中值
#define CENTER_ECD_PITCH 0        //云台pitch轴编码器归中值

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 100
#define INIT_TIMEOUT 1000  // 单位: ms 初始化归中超时时间

/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        31.5f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -32.9f

/* ------------------------------------------------------- 发射相关 --------------------------------------------------- */
// TODO: 实际值待整定
#define RIGHT_FRICTION_MOTOR_ID     0x203
#define LEFT_FRICTION_MOTOR_ID      0x204
#define TRIGGER_MOTOR_ID            0x205

#define FRICTION_SPEED_ONE           6000
#define FRICTION_SPEED_CONTINUE      6000

#define TRIGGER_MOTOR_51_TO_ANGLE 51.47f
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             18
#define RIGHT_KI_V             0.0f
#define RIGHT_KD_V             0.001f
#define RIGHT_INTEGRAL_V       0
#define RIGHT_MAX_V            16384

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           18
#define LEFT_KI_V           0.0f
#define LEFT_KD_V           0.001f
#define LEFT_INTEGRAL_V     0
#define LEFT_MAX_V          16384

// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M3508电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           10
#define TRIGGER_KI_V           5
#define TRIGGER_KD_V           0.01f
#define TRIGGER_INTEGRAL_V     1500
#define TRIGGER_MAX_V          16384
/* 角度环 */
#define TRIGGER_KP_A           5
#define TRIGGER_KI_A           0.01
#define TRIGGER_KD_A           0.003
#define TRIGGER_INTEGRAL_A     5.0f
#define TRIGGER_MAX_A          10000


/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             5000
#define YAW_KI_V_IMU             200
#define YAW_KD_V_IMU             10
#define YAW_INTEGRAL_V_IMU       1000
#define YAW_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_KP_A_IMU             0.35f
#define YAW_KI_A_IMU             0
#define YAW_KD_A_IMU             0.001f
#define YAW_INTEGRAL_A_IMU       5
#define YAW_MAX_A_IMU            25
/* auto速度环 */
#define YAW_KP_V_AUTO            0
#define YAW_KI_V_AUTO            0
#define YAW_KD_V_AUTO            0
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           0
/* auto角度环 */
#define YAW_KP_A_AUTO            0
#define YAW_KI_A_AUTO            0
#define YAW_KD_A_AUTO            0
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           0

/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU           4250
#define PITCH_KI_V_IMU           1000
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
/* imu角度环 */
#define PITCH_KP_A_IMU           0.5f
#define PITCH_KI_A_IMU           0.0f
#define PITCH_KD_A_IMU           0.005f
#define PITCH_INTEGRAL_A_IMU     0.2f
#define PITCH_MAX_A_IMU          20
/* auto速度环 */
#define PITCH_KP_V_AUTO          0
#define PITCH_KI_V_AUTO          0
#define PITCH_KD_V_AUTO          0
#define PITCH_INTEGRAL_V_AUTO    0
#define PITCH_MAX_V_AUTO         0
/* auto角度环 */
#define PITCH_KP_A_AUTO          0
#define PITCH_KI_A_AUTO          0
#define PITCH_KD_A_AUTO          0
#define PITCH_INTEGRAL_A_AUTO    0
#define PITCH_MAX_A_AUTO         0


/***********chassis部分关于length/theta/yaw/roll的pid参数**********************/

///*位置*/
#define l_length_Kp 280.0
#define l_length_Ki 60.00
#define l_length_Kd 0.00001
#define l_length_InteVal 150
#define l_length_MaxVal 300


/*位置*/
#define r_length_Kp 280.0
#define r_length_Ki 60.00
#define r_length_Kd 0.00001
#define r_length_InteVal 150
#define r_length_MaxVal 300


/*theta相关*/
#define theta_Kp 10
#define theta_Ki 0
#define theta_Kd 0.00001
#define theta_InteVal 0
#define theta_MaxVal 50

/*yaw相关,转向采用pd控制*/
#define yaw_Kp 5.0
#define yaw_Ki 0
#define yaw_Kd 0.00001
#define yaw_InteVal 0
#define yaw_MaxVal 10.0


/*roll相关*/
#define roll_Kp 10.0
#define roll_Ki 0.4
#define roll_Kd 0.000001
#define roll_InteVal 0
#define roll_MaxVal 25

/*****************************function_open******************************************/
/*要使用时打开宏定义!!!*/
/*
 * @brief设置髋关节damiao电机的零点
 * @warning零点设置注意位置,若是位置不对会导致phi的角度都不对会疯车
 * @warning 不小心开启并烧录的话需要重新校准零点,不然一定疯车
 * */
//#define DM_8009_SET_ZERO_POSITION

/*使用imu校准(不用每次都校准,一段时间校准即可,温度在40摄氏度左右再进行校准)*/
//#define BSP_BMI088_CALI
/*使用位于云台的达妙imu*/
#define  BSP_USING_DM_IMU
/*使用damiao板imu加热*/
#define  BSP_USING_IMU_HEAT

/*使用3508*/
#define BSP_USING_DJI_MOTOR

/*使用dm电机,后续可细分电机种类*/
#define BSP_USING_DM_MOTOR

/*使用lk电机*/
//#define  BSP_USING_LK_MOTOR

/*使用ht电机*/
//#define BSP_USING_HT_MOTOR

/*3508输入置零*/
#define M3508_SET_ZERO
/*8009输入置零*/
//#define DM8009P_SET_ZERO

/* -------------------------------------------------------------------------- */
/*                            remote_controler                                  */
/* -------------------------------------------------------------------------- */
#define BSP_USING_RC_DBUS
//#define BSP_USING_RC_DBUS_KEYBOARD


#endif /* _RM_CONFIG_H */
