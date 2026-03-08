/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2025-07-25      SuperChen       second version
* 2025-11-01      SuperChen       third version (结合上交建模lqr)
* 2026-01-15      SuperChen       fourth version (尝试港大的仿真)
* 2026-01-20      SuperChen       fifth version (重回哈工程开源)
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"
#include "DM_motor.h"


/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_cmd;
// 发布
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */

#define LEFT    0
#define RIGHT   1
#define FRONT   0
#define BACK    1


#define LEN_LEN_LOW     0.13f // 单位：m
#define LEN_LEN_MID     0.25f // 单位：m
#define LEN_LEN_HIG     0.30f // 单位：m
#define FORCE_Length_LIMIT 200.0f //
#define FORCE_LIMIT 250.0f // 支持力限幅


static float leg_lenthchange_flag = 0;//倒地自起中腿长切换标志位
static float leg_phi0_refer_L;/*目标phi0位置,用于vmc_inv*/
static float leg_l0_refer_L;/*用于vmc_inv*/
static float leg_phi0_refer_R;/*目标phi0位置,用于vmc_inv*/
static float leg_l0_refer_R;/*用于vmc_inv*/

static float dm_p_set[2][2];
static float dm_v_set[2][2];

#define DM_MIT_KP 0.06f
#define DM_MIT_KD 3.5f
#define DM_V_SET  0.0f

static float phi0_refer_change_flag;/*倒地自起时的phi0切换标志位*/

#define Theta_Compensation  -0.08f

#define CHASSIS_VX_MAX        673.0f
#define CHASSIS_WX_MAX        270.0f

#define CHASSIS_V_SET         4.0f
#define YAW_TURN_RATIO  0.05f  //单位应该为°,有关调节遥控器转向敏感度的系数,自行在安全范围内调节大小
static float Vx_Delta;
#define VX_DELTA_MAX 5.0f

#define yaw_turn_region 50.0f

/*髋关节电机 DMJ8009P-2EC 实例*/
static dm_motor_object_t *dm_motor[4];

/* 驱动电机 3508 实例 */
static dji_motor_object_t *m3508_motor[2];

/*腿部支持力计算相关参数*/
static float F_bl_gravity ; //重力补偿前馈
static float F_bl_intertial ; //侧向惯性力矩补偿前馈
static float F_roll;
static float F_l_L;
static float F_l_R;

/*跳跃相关*/
#define JUMP_TORQUE_PRESS    30.0f   // 跳跃时下压扭矩，正负需对应各电机确定,待重新调试
#define JUMP_TORQUE_SHRINK   22.0f   // 跳跃时回缩扭矩，正负需对应各电机确定,待重新调试
#define OFF_GROUND 18.0f    //检测是否离地,判断离地的阈值,后续可以尝试Knn检测

#define LEN_JUDGE_REGION  0.02  //辅助判断腿长是否变化完成
#define ROLL_JUDGE_REGION  1.0  //辅助判断腿长是否变化完成,考虑到过单边桥腿长不一致
static int Leg_Left_len_JudgeOK;
static int Leg_right_len_JudgeOK;
static int Body_roll_JudgeOK;
static int Leg_len_JudgeOK;
static float FN_Average;       //左右腿平均支持力

static uint8_t Off_Ground_Cnt = 0 ;
static uint8_t Touch_Ground_Cnt = 0;
static uint8_t  Off_Ground_Flag = 0;
static uint8_t Wheel_Shut_Flag = 0;
static uint8_t  Touch_Ground_Flag;

#define Len_pid_output_LIMIT 80.0f

static pid_obj_t *theta_pid;  // 双腿角度协调控制
static pid_obj_t *yaw_pid;    // 航向角控制， 输出为vx的补偿，与vx期望累积
static pid_obj_t *roll_pid;   // 横滚角控制
static pid_obj_t *L_length_pid;  //腿长控制
static pid_obj_t *R_length_pid;  //腿长控制

static float yaw_target;
#define YAW_DELTA_MX 1.396263f //80.0* 0.01745329252 f


#define Location 0      //腿长位置环
#define Speed    1      //腿长速度环
static void leg_calc();

#define WHEEL_RADIUS  0.058f      //轮子半径/m

static leg_obj_t * leg[2] = {NULL};
static float ft_l[2], ft_r[2];  // FT = [PendulumForce PendulumTorque]
static float vmc_out_l[2];  // vmc计算得出的扭矩值 左边腿
static float vmc_out_r[2];  // vmc计算得出的扭矩值 右边腿
static float jump_out_l[2]; // 跳跃附加扭矩值 左边腿
static float jump_out_r[2]; // 跳跃附加扭矩值 右边腿
static float jump_start_time;  // 跳跃开始时间
static float jump_dt_time;     // 起跳后的间隔时间
static float is_jumping;       // 跳跃标志位
static float jump_flag;        // 切入跳跃模式只跳一次

static void dm_motor_init();
static int chassis_motor_init(void);
static void m3508_motor_init();

/*判断腿长是否切换了*/
static void LegIsOk();
/*离地检测*/
static void Ground_Detect();
/*清除为伸开腿时,腿长pid的积分输出,防止腿部站起来时突然伸长*/
void Process_Clear();
/*打滑检测时或者腿的一边卡住时,两边的速度相差大于一定值时,相处转向pid的作用*/
static void Chassis_Vx_Detect();
/*
 * @brief 检查底盘姿态,认为溃散后失能
 * */
static void Security_Checking();
/*
 * @brief 倒地自起
 * */
static void Chassis_Recovery();
/**
 * @brief 底盘跳跃处理
 */
static void jumping_control(void);
/*计算腿部支持力*/
static void Leg_FN_Calculation(float ROLL_TARGET,float L_TARGET);
/*电机失能*/
static void motor_relax();
/*电机使能*/
static void motor_enable();
/*设置电机零点*/
static void leg_init_get_zero();
/*
 * @brief 斜坡函数
 * @param end目标值
 * @param begin初始值
 * @param measure 量测值
 * @param set 斜坡过程值
 * @param step_length 步长
 * @param safe_region 合理范围
 * */

void slope_phi0_following_begin_end(const float *target,const float *measure,float *set,float step_length,const float safe_region,uint8_t *phi0_change_flag);


/* --------------------------------- LQR控制相关 -------------------------------- */
static float a11[4];
static float a12[4];
static float a13[4];
static float a14[4];
static float a15[4];
static float a16[4];
static float a21[4];
static float a22[4];
static float a23[4];
static float a24[4];
static float a25[4];
static float a26[4];


static float MatLQR_K[2][6] = {0};//LQR运算中的反馈系数,在lqr_update参数更新函数中更新


/* [T Tp(髋)] */
static float LQROutBuf[2][2]={0};

static float LQRXerrorBuf[2][6]={0};
static float LQRXObsBuf[2][6]={0};
static float LQRXRefBuf[2][6]={0}; /*LQRXObsBuf[0][2] - LQRXRefBuf[0][2]*/


/*
 * Matrix_LQRObs
 * [θ
 * θ_dot
 * x
 * x_dot
 * φ
 * φ_dot]
 */
static arm_matrix_instance_f32 MatLQRObs_L  = {6, 1, LQRXObsBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRObs_R  = {6, 1, LQRXObsBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRRef_L  = {6, 1, LQRXRefBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRRef_R  = {6, 1, LQRXRefBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRNegK   = {2, 6, (float*)MatLQR_K};/*TODO 后续需要测试是否可行*/
static arm_matrix_instance_f32 MatLQRErrX_L = {6, 1, LQRXerrorBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRErrX_R = {6, 1, LQRXerrorBuf[RIGHT]};
/*
 * U
 * [T  驱动轮输出力矩
 * Tp]  髋关节输出力矩
 * */
static arm_matrix_instance_f32 MatLQROutU_L = {2, 1, LQROutBuf[LEFT]};
static arm_matrix_instance_f32 MatLQROutU_R = {2, 1, LQROutBuf[RIGHT]};


/*Calculate X. Output is u (T,Tp)`*/
/*TODO 加入对于theta和phi的死区,否则难以稳定下来*/
static void LQR_Cal(){


    //Calculate error
    /*cmsis dsp中的矩阵减法运算,error = ref(目标值) - obs(观测值) */
    arm_mat_sub_f32(&MatLQRRef_R,&MatLQRObs_R,&MatLQRErrX_R);

    //Calculate output value
    /*矩阵乘法,[2*6]*[6*1] */
    /*TODO 不同腿长下K矩阵会随之改变*/
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_R, &MatLQROutU_R);
    //Calculate error
    /*cmsis dsp中的矩阵减法运算,error = ref(目标值) - obs(观测值) */
    arm_mat_sub_f32(&MatLQRRef_L,&MatLQRObs_L, &MatLQRErrX_L);

    arm_mat_sub_f32(&MatLQRRef_R,&MatLQRObs_R,&MatLQRErrX_R);

    //Calculate output value
    /*矩阵乘法,[2*6]*[6*1] */
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_L, &MatLQROutU_L);


}

/* ------------------------------- 打滑检测卡尔曼滤波部分 ------------------------------ */
//TODO:后续考虑移入观测线程，建立整车观测器
static KalmanFilter_t chassis_kf_l;
static KalmanFilter_t chassis_kf_r;
static float chassis_vx_filter;
static float chassis_vx_l;
static float chassis_vx_r;
/**
*   |    vx     |
*   |    ax     |
*/
static void chassis_kf_init(void)
{
    static float P_Init[4] =
            {
                    54.5802, 0.0054,
                    0.0054, 16.7869,
            };
    static float F_Init[4] =
            {
                    1, 0.003,
                    0, 1,

            };
    static float Q_Init[4] =
            {
                    50, 0,
                    0, 15,
            };

    // 设置最小方差
    static float state_min_variance[2] = {0.005, 0.1};/*TODO 初次修改怎么优化参数*/

    //  电机编码器测得位移，电机编码器测得速度，加速度计解算得出速度
    static uint8_t measurement_reference[2] = {1,2};
    static float measurement_degree[2] = {1,1};
    // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）

//    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
    static float mat_R_diagonal_elements[3] = {5,2}; // 方差即为影响置信度，方差越大置信度越低  /* 打滑检测：800000 1000 */1
    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）

    // 开启自动调整
    chassis_kf_l.UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf_l, 2, 0, 2);

    // 开启自动调整
    chassis_kf_r.UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf_r, 2, 0, 2);

    // 设置矩阵值
    memcpy(chassis_kf_l.P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf_l.F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf_l.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf_l.MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf_l.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf_l.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf_l.StateMinVariance, state_min_variance, sizeof(state_min_variance));

    // 设置矩阵值
    memcpy(chassis_kf_r.P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf_r.F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf_r.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf_r.MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf_r.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf_r.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf_r.StateMinVariance, state_min_variance, sizeof(state_min_variance));


}
static float  speed_rads_ground_l;
static float  wheel_to_ground_l,speed_rads_ground_r,wheel_to_ground_r;
static void chassis_kf_update(void)
{
    static float _kf_dt, _kf_start;
    if(_kf_start != 0) // 避免第一次计算错误
        _kf_dt = (dwt_get_time_ms() - _kf_start) / 1000.0f;
    else
        _kf_dt = 0.003f;
    _kf_start = dwt_get_time_ms();


//    speed_rads_ground_l = -(m3508_motor[LEFT]->measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD) + ins.gyro[0] * DEGREE_2_RAD - leg[LEFT]->d_theta_lpf ;
    speed_rads_ground_l = -(m3508_motor[LEFT]->measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD) - ins.gyro[0] * DEGREE_2_RAD - leg[LEFT]->d_phi0 ;/*山海机甲打滑部分*/
    wheel_to_ground_l = speed_rads_ground_l * WHEEL_RADIUS + leg[LEFT]->d_theta_lpf*leg[LEFT]->l0* arm_cos_f32(leg[LEFT]->theta)+ leg[LEFT]->d_l0*arm_sin_f32(leg[LEFT]->theta) ;//TODO机体速度推出轮子速度

//    speed_rads_ground_r = (m3508_motor[RIGHT]->measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD) - ins.gyro[0] * DEGREE_2_RAD - leg[LEFT]->d_theta_lpf ;
    speed_rads_ground_r = (m3508_motor[RIGHT]->measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD) - ins.gyro[0] * DEGREE_2_RAD - leg[LEFT]->d_phi0 ;
    wheel_to_ground_r = speed_rads_ground_r * WHEEL_RADIUS + leg[RIGHT]->d_theta_lpf*leg[RIGHT]->l0* arm_cos_f32(leg[RIGHT]->theta)+ leg[RIGHT]->d_l0*arm_sin_f32(leg[RIGHT]->theta);

    chassis_kf_l.MeasuredVector[0] = wheel_to_ground_l ;
    chassis_kf_l.MeasuredVector[1] = ins.motion_accel_b[1];//机体加速度作为整体的加速度

    Kalman_Filter_Update(&chassis_kf_l);

    chassis_kf_r.MeasuredVector[0] = wheel_to_ground_r ;
    chassis_kf_r.MeasuredVector[1] = ins.motion_accel_b[1];//机体加速度作为整体的加速度

    Kalman_Filter_Update(&chassis_kf_r);
}

/* --------------------------------- 底盘线程入口 --------------------------------- */

static void update_LQR_obs() {

    static float dt, start;
    if(start != 0) // 避免第一次计算错误
        dt = (dwt_get_time_ms() - start) / 1000.0f;
    else
        dt = 0.003f;
    start = dwt_get_time_ms();

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY){/*起立时的lqr*/
        /*记录下Q和R
    Q=diag([10 100 1 1 1 1]);
    R=diag([3.75 2.75]);
    K矩阵 =
      [-13.065135,  -5.334417,  -0.497871,  -1.279899,   1.786252,   0.387743;
         2.952166,   1.583785,  -0.159907,  -0.343241,   5.888001,   1.090465]
        */
        // 起立时的LQR参数赋值
        a11[0] = 27.7513f, a11[1] = -21.5353f, a11[2] = -0.2051f, a11[3] = -12.8570f;
        a12[0] = 4.5651f,  a12[1] = -4.4156f,  a12[2] = 0.1265f,   a12[3] = -5.3075f;
        a13[0] = 0.1653f, a13[1] = -0.1009f,   a13[2] = -0.0138f,  a13[3] = -0.4957f;
        a14[0] = 0.1914f, a14[1] = -0.0941f,   a14[2] = -0.0389f,   a14[3] = -1.2753f;
        a15[0] = 10.4646f, a15[1] = -8.2251f, a15[2] = 0.0687f,  a15[3] = 1.8512f;
        a16[0] = 1.7105f,  a16[1] = -1.3423f,  a16[2] = 0.0114f,   a16[3] = 0.3983f;
        a21[0] = -12.2100f, a21[1] = 9.2224f,  a21[2] = 0.0572f,   a21[3] = 2.8664f;
        a22[0] = 8.7097f,  a22[1] = -6.7896f,  a22[2] = 0.0228f,   a22[3] = 1.6407f;
        a23[0] = -0.8515f,  a23[1] = 0.6138f,  a23[2] = 0.0184f,   a23[3] = -0.1670f;
        a24[0] = -1.6668f,  a24[1] = 1.1766f,  a24[2] = 0.0485f,   a24[3] = -0.3582f;
        a25[0] = -3.4674f, a25[1] = 2.8060f,   a25[2] = -0.0655f,  a25[3] = 5.8700f;
        a26[0] = -0.4707f, a26[1] = 0.3734f,   a26[2] = -0.0056f,   a26[3] = 1.0878f;

    }else{
        /*记录下Q和R
    Q=diag([40 80 1 1 800 150]);
    R=diag([3.75 1.05]);
  K矩阵 =
  [-13.421800,  -4.865737,  -0.509292,  -1.356400,   5.037319,   1.769644;
     6.364314,   2.328431,   0.160778,   0.445030,  29.446837,  11.692943]
        */
        // 正常模式的LQR参数赋值
        a11[0] = 23.7036f, a11[1] = -18.5204f, a11[2] = -0.1227f, a11[3] = -13.2480f;
        a12[0] = 4.7774f,  a12[1] = -4.3550f,  a12[2] = 0.0143f,   a12[3] = -4.8284f;
        a13[0] = 0.1078f, a13[1] = -0.0585f,   a13[2] = -0.0108f,  a13[3] = -0.5077f;
        a14[0] = -0.4008f, a14[1] = 0.3067f,   a14[2] = -0.0056f,   a14[3] = -1.3585f;
        a15[0] = 40.3692f, a15[1] = -30.0334f, a15[2] = -0.1539f,  a15[3] = 5.3127f;
        a16[0] = 14.6321f,  a16[1] = -10.7964f,  a16[2] = -0.0686f,   a16[3] = 1.8698f;
        a21[0] = 29.6636f, a21[1] = -22.7546f,  a21[2] = 0.4857f,   a21[3] = 6.5136f;
        a22[0] = 19.9342f,  a22[1] = -14.8277f,  a22[2] = -0.0393f,   a22[3] = 2.4607f;
        a23[0] = 1.6529f,  a23[1] = -1.2227f,  a23[2] = 0.0012f,   a23[3] = 0.1712f;
        a24[0] = 4.6329f,  a24[1] = -3.3407f,  a24[2] = -0.0486f,   a24[3] = 0.4787f;
        a25[0] = -17.7685f, a25[1] = 10.6110f,   a25[2] = 1.4275f,  a25[3] = 29.2157f;
        a26[0] = -5.8616f, a26[1] = 3.3595f,   a26[2] = 0.5358f,   a26[3] = 11.6116f;
    }

    LQRXObsBuf[LEFT][0] = leg[LEFT]->theta + Theta_Compensation ;

    LQRXObsBuf[LEFT][1] = leg[LEFT]->d_theta_lpf ;

    chassis_vx_l = chassis_kf_l.FilteredValue[0];/*观测用*/
    chassis_vx_r = chassis_kf_r.FilteredValue[0];
    chassis_vx_filter = 0.5f * (chassis_kf_l.FilteredValue[0] + chassis_kf_r.FilteredValue[0]);

    LQRXObsBuf[LEFT][2] = 0;
    LQRXObsBuf[LEFT][3] = chassis_vx_filter;


    LQRXObsBuf[LEFT][4] = -ins.pitch * DEGREE_2_RAD ;
    LQRXObsBuf[LEFT][5] = -ins.gyro[1] * DEGREE_2_RAD;


    LQRXObsBuf[RIGHT][0] = leg[RIGHT]->theta + Theta_Compensation ;

    LQRXObsBuf[RIGHT][1] = leg[RIGHT]->d_theta_lpf ;

    LQRXObsBuf[RIGHT][2] = 0;
    LQRXObsBuf[RIGHT][3] = chassis_vx_filter;


    LQRXObsBuf[RIGHT][4] = -ins.pitch * DEGREE_2_RAD ;
    LQRXObsBuf[RIGHT][5] = -ins.gyro[1] * DEGREE_2_RAD;



    LQRXRefBuf[RIGHT][2] = 0;
    LQRXRefBuf[LEFT][2] = 0;

    LQRXRefBuf[LEFT][3]  = (chassis_cmd.vx_set / RC_DBUS_MAX_VALUE) * CHASSIS_V_SET  ;  // 单位为米,对速度积分得到
    LQRXRefBuf[RIGHT][3] = (chassis_cmd.vx_set / RC_DBUS_MAX_VALUE) * CHASSIS_V_SET  ;  // 单位为米


    leg[LEFT]->l0_average = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0); /*TODO 在左右腿长不一致的情况下的k是否合理有待讨论*/
    leg[RIGHT]->l0_average = leg[LEFT]->l0_average;
    leg[LEFT]->l0_pow3 = leg[LEFT]->l0_average * leg[LEFT]->l0_average * leg[LEFT]->l0_average;
    leg[LEFT]->l0_pow2 = leg[LEFT]->l0_average * leg[LEFT]->l0_average ;
    if(chassis_cmd.leg_leng_change == LENGTH_STAY && Off_Ground_Flag == 1 ){//TODO: 应该改成离地检测满足时,起跳时,将除了K21和K22以外的K置零,防止空中腿部姿态溃散

        Wheel_Shut_Flag = 1;

        LQRXRefBuf[RIGHT][2] = 0;
        LQRXRefBuf[LEFT][2] = 0;

        LQRXRefBuf[LEFT][3]  = 0;
        LQRXRefBuf[RIGHT][3] = 0;

        yaw_target = -ins.yaw_total_angle* DEGREE_2_RAD;

        MatLQR_K[0][0] = 0;
        MatLQR_K[0][1] = 0;
        MatLQR_K[0][2] = 0;
        MatLQR_K[0][3] = 0;
        MatLQR_K[0][4] = 0;
        MatLQR_K[0][5] = 0;
        MatLQR_K[1][0] = a21[0] * leg[LEFT]->l0_pow3 + a21[1] * leg[LEFT]->l0_pow2 + a21[2] * leg[LEFT]->l0_average + a21[3];
        MatLQR_K[1][1] = a22[0] * leg[LEFT]->l0_pow3 + a22[1] * leg[LEFT]->l0_pow2 + a22[2] * leg[LEFT]->l0_average + a22[3];
        MatLQR_K[1][2] = 0;
        MatLQR_K[1][3] = 0;
        MatLQR_K[1][4] = 0;
        MatLQR_K[1][5] = 0;

    }else{/*TODO 在腿长不切换是每次都进行运算浪费资源 */
        /*更新LQR反馈矩阵K*/
        Wheel_Shut_Flag = 0;

        MatLQR_K[0][0] = a11[0] * leg[LEFT]->l0_pow3 + a11[1] * leg[LEFT]->l0_pow2 + a11[2] * leg[LEFT]->l0_average + a11[3];
        MatLQR_K[0][1] = a12[0] * leg[LEFT]->l0_pow3 + a12[1] * leg[LEFT]->l0_pow2 + a12[2] * leg[LEFT]->l0_average + a12[3];
        MatLQR_K[0][2] = a13[0] * leg[LEFT]->l0_pow3 + a13[1] * leg[LEFT]->l0_pow2 + a13[2] * leg[LEFT]->l0_average + a13[3];
        MatLQR_K[0][3] = a14[0] * leg[LEFT]->l0_pow3 + a14[1] * leg[LEFT]->l0_pow2 + a14[2] * leg[LEFT]->l0_average + a14[3];
        MatLQR_K[0][4] = a15[0] * leg[LEFT]->l0_pow3 + a15[1] * leg[LEFT]->l0_pow2 + a15[2] * leg[LEFT]->l0_average + a15[3];
        MatLQR_K[0][5] = a16[0] * leg[LEFT]->l0_pow3 + a16[1] * leg[LEFT]->l0_pow2 + a16[2] * leg[LEFT]->l0_average + a16[3];
        MatLQR_K[1][0] = a21[0] * leg[LEFT]->l0_pow3 + a21[1] * leg[LEFT]->l0_pow2 + a21[2] * leg[LEFT]->l0_average + a21[3];
        MatLQR_K[1][1] = a22[0] * leg[LEFT]->l0_pow3 + a22[1] * leg[LEFT]->l0_pow2 + a22[2] * leg[LEFT]->l0_average + a22[3];
        MatLQR_K[1][2] = a23[0] * leg[LEFT]->l0_pow3 + a23[1] * leg[LEFT]->l0_pow2 + a23[2] * leg[LEFT]->l0_average + a23[3];
        MatLQR_K[1][3] = a24[0] * leg[LEFT]->l0_pow3 + a24[1] * leg[LEFT]->l0_pow2 + a24[2] * leg[LEFT]->l0_average + a24[3];
        MatLQR_K[1][4] = a25[0] * leg[LEFT]->l0_pow3 + a25[1] * leg[LEFT]->l0_pow2 + a25[2] * leg[LEFT]->l0_average + a25[3];
        MatLQR_K[1][5] = a26[0] * leg[LEFT]->l0_pow3 + a26[1] * leg[LEFT]->l0_pow2 + a26[2] * leg[LEFT]->l0_average + a26[3];

    }
}

void chassis_task_init(void)
{
    chassis_sub_init();
    chassis_motor_init();
    chassis_kf_init();


}


/**
 * @brief 底盘控制任务,在RTOS中应该设定为200hz运行
 */
void chassis_control_task(void)
{
    /* 更新该线程所有的订阅者 */
    chassis_sub_pull();

    // 切换腿长姿态
    switch (chassis_cmd.leg_level)
    {
        case LEG_LOW:
            /* 更改腿长 */
            leg[LEFT]->length_ref = LEN_LEN_LOW;
            leg[RIGHT]->length_ref = LEN_LEN_LOW;

            break;
        case LEG_MID:
            leg[LEFT]->length_ref = LEN_LEN_MID;
            leg[RIGHT]->length_ref = LEN_LEN_MID;

            break;
        case LEG_HIG:
            leg[LEFT]->length_ref = LEN_LEN_HIG;
            leg[RIGHT]->length_ref = LEN_LEN_HIG;

            break;
        default:
            leg[LEFT]->length_ref = LEN_LEN_LOW;
            leg[RIGHT]->length_ref = LEN_LEN_LOW;

            break;
    }

    switch (chassis_cmd.ctrl_mode)
    {
        case CHASSIS_RELAX:
            Process_Clear();
            motor_relax();
            chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY;

            break;
        case CHASSIS_INIT:
            Process_Clear();
            motor_enable();
#ifdef DM_8009_SET_ZERO_POSITION
            leg_init_get_zero();
#endif
            break;
        case CHASSIS_RECOVERY:/*不稳定的状态,介于init和relex状态的过渡状态*/
            yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;/*消除转向pid的影响*/
            motor_enable();
            Chassis_Recovery();

            //TODO: 处于该模式下，应该屏蔽遥控器等控制

            break;

        case CHASSIS_OPEN_LOOP:/*底盘开环控制,init成功,或是revovery成功,此状态应为稳定站立时*/
            motor_enable();
            Chassis_Vx_Detect();
            Security_Checking();
//            Ground_Detect();   /*稳定站立后再开启离地检测,后续安装气簧也会影响*/
            break;

        case CHASSIS_FOLLOW_GIMBAL:
            motor_enable();
            Security_Checking();

            break;
        case CHASSIS_SPIN:
            motor_enable();
            Security_Checking();

            break;
        case CHASSIS_JUMP:
            motor_enable();
            Security_Checking();

            break;
        case CHASSIS_AUTO:
            motor_enable();
            Security_Checking();
            break;
        default:
            motor_relax();
            break;
    }
#ifdef DM_8009_SET_ZERO_POSITION
    if(fabsf(ins.pitch) > 60.0f)
        chassis_fdb_data.stand_state = CAHSSIS_IS_DANGER;
#endif

    chassis_kf_update();
    leg_calc(); // 保证稳定的运算频率，不受模式影响

    chassis_fdb_data.M3508_l = m3508_motor[LEFT]->measure;
    chassis_fdb_data.M3508_r = m3508_motor[RIGHT]->measure;
    /* 更新发布该线程的msg */
    chassis_pub_push();

    vTaskDelay(1);
}

/**
 * @brief 底盘初始化（注册底盘电机及其控制器初始化等）
 */
static int chassis_motor_init(void)
{
    dm_motor_init();
    m3508_motor_init();

    leg_config_t leg_config =
            {
                    /*单位m*/
                    0.21f,  // l4=l1
                    0.250f, // l3=l2
                    0.0f,   //电机间距
                    300,    //气弹簧压力300N
                    0.0503f,/*气弹簧在l2上安装孔位到膝关节的距离*/
                    0.20209f,/*气弹簧在l1处端点到膝关节的距离*/
                    0.214218f,/*气弹簧在l1的安装孔和膝关节连线与l1的夹角,12.28°,转为rad*/
            };
    leg[LEFT] = leg_register(&leg_config);
    leg[RIGHT] = leg_register(&leg_config);

    /*左侧腿长pid*/
    pid_config_t L_length_pid_config = INIT_PID_CONFIG(l_length_Kp, l_length_Ki, l_length_Kd, l_length_InteVal, l_length_MaxVal,
                                                       (PID_Integral_Limit | PID_DerivativeFilter | PID_OutputFilter));

    L_length_pid = pid_register(&L_length_pid_config);

    /*右侧腿长pid*/
    pid_config_t R_length_pid_config = INIT_PID_CONFIG(r_length_Kp, r_length_Ki, r_length_Kd, r_length_InteVal, r_length_MaxVal,
                                                       (PID_Integral_Limit | PID_OutputFilter));

    R_length_pid = pid_register(&R_length_pid_config);


    /* 两腿协调 PD 控制 */
    pid_config_t theta_pid_config = INIT_PID_CONFIG(theta_Kp, theta_Ki, theta_Kd, theta_InteVal, theta_MaxVal, PID_Integral_Limit | PID_OutputFilter);
    theta_pid = pid_register(&theta_pid_config);

    /* 航向角 PD 控制 */
    pid_config_t yaw_pid_config = INIT_PID_CONFIG(yaw_Kp,yaw_Ki, yaw_Kd, yaw_InteVal, yaw_MaxVal, PID_Integral_Limit | PID_OutputFilter);
    yaw_pid = pid_register(&yaw_pid_config);


    /* 横滚角 PD 控制 控制机体的水平*/
    pid_config_t roll_pid_config = INIT_PID_CONFIG( roll_Kp, roll_Ki, roll_Kd, roll_InteVal,roll_MaxVal, PID_Integral_Limit|PID_OutputFilter);
    roll_pid = pid_register(&roll_pid_config);

    return 0;
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#define chassis_dt 0.005
#ifdef BSP_CHASSIS_LEG_MODE

/**
 * @brief 轮腿底盘运动解算   _正方向
 *    |_____电池架_____|  |
 *      |_4     3_|
 *      |_1     2_|
 *     |___________|
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */

static void leg_calc()
{
    // 左腿解算
    /*Warning: 若是电机没有按照规定安装,需要调整phi1和phi4的计算,这会很大程度影响到后续的vmc解算*/
    leg[LEFT]->phi_calc_L(leg[LEFT],dm_motor[3]->measure.angle_abs,dm_motor[0]->measure.angle_abs);
    leg[LEFT]->vmc_calc(leg[LEFT],&ins,chassis_dt);
    // 右腿解算
    leg[RIGHT]->phi_calc_R(leg[RIGHT],dm_motor[2]->measure.angle_abs,dm_motor[1]->measure.angle_abs);
    leg[RIGHT]->vmc_calc(leg[RIGHT],&ins,chassis_dt);

    update_LQR_obs();
    LQR_Cal();

    /* 双腿角度协调控制 */
    pid_calculate(theta_pid, leg[LEFT]->theta - leg[RIGHT]->theta, 0);
    /* 航向角控制 */
    pid_calculate(yaw_pid, -ins.yaw_total_angle* DEGREE_2_RAD , yaw_target);

    /* 横滚角控制 */
    pid_calculate(roll_pid, ins.roll, 0);

    Leg_FN_Calculation(0,0.5f*(leg[LEFT]->length_ref + leg[RIGHT]->length_ref));


    leg[LEFT]->Tp = -LQROutBuf[LEFT][1] - theta_pid->Output ;
    leg[RIGHT]->Tp = -LQROutBuf[RIGHT][1] + theta_pid->Output ;

    /* 离地检测，计算两腿地面支持力 */
//    TODO：目前离地检测还存在问题 ins.acc 存在问题，可能需要卡尔曼滤波
    FN_Average = 0.5f * (leg[LEFT]->support_force + leg[RIGHT]->support_force);

    leg[LEFT]->vmc_cal_T(leg[LEFT], vmc_out_l);
    leg[RIGHT]->vmc_cal_T(leg[RIGHT], vmc_out_r);

}

static void Leg_FN_Calculation(float ROLL_TARGET,float L_TARGET){/*交23年平步开源中对于支持力的计算*/

    F_roll = pid_calculate(roll_pid, ins.roll * DEGREE_2_RAD, ROLL_TARGET);  //TODO 注意方向,沿正方形顺时针为正

    F_l_L = pid_calculate(L_length_pid, leg[LEFT]->l0, L_TARGET);

    F_l_R = pid_calculate(R_length_pid, leg[RIGHT]->l0, L_TARGET);

    F_bl_gravity = 0.5 * m_b * g;
    F_bl_intertial = 0.5 * m_b * (leg[LEFT]->l0_average / (2.0f*Rl)) * (-ins.gyro[2] * DEGREE_2_RAD) * chassis_vx_filter;

    leg[LEFT]->F_Spring_to_F_Vertical(leg[LEFT]);
    leg[RIGHT]->F_Spring_to_F_Vertical(leg[RIGHT]);

    leg[LEFT]->support_force = -F_roll + F_l_L + F_bl_gravity - F_bl_intertial - leg[LEFT]->F_Vertical;
    LIMIT_MIN_MAX(leg[LEFT]->support_force, -FORCE_LIMIT, FORCE_LIMIT);
    leg[RIGHT]->support_force = F_roll + F_l_R + F_bl_gravity + F_bl_intertial - leg[RIGHT]->F_Vertical;
    LIMIT_MIN_MAX(leg[RIGHT]->support_force, -FORCE_LIMIT, FORCE_LIMIT);
}

/*****************************************电机接口*************************************************************************/
/**
 * @brief 轮腿底盘运动解算   _正方向
 *    |_______________|  |
 *      |_4     3_|
 *      |_1     2_|
 *     |___________|
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */


/* ----------------------------------------- 电机控制相关 ---------------------------------------------------------- */
static void leg_init_get_zero()
{
    // 首先各个电机给定一个适当的力矩，并持续，确保撞到限位
    chassis_fdb_data.leg_state = LEG_BACK_STEP;
    osDelay(2000);
    // 撞到限位后，控制电机在此处设置零点
    for (uint8_t i = 0; i < 4; i++)
    {
        dm_motor[i]->set_mode(dm_motor[i], DM_CMD_ZERO_POSITION);
    }
    // 电机零点设置完成，正常零点为减去各偏移量
    chassis_fdb_data.leg_state = LEG_BACK_IS_OK;
    // 该函数仅在每次重新上电执行
}
static void motor_enable()
{
    dm_motor_enable_all();  // 所有电机进入 motor 模式
    for (uint8_t i = 0; i < 2; i++)
    {
        dji_motor_enable(m3508_motor[i]);
    }
}

static void motor_relax()
{
    dm_motor_disable_all();
    for (uint8_t i = 0; i < 2; i++)
    {
        dji_motor_relax(m3508_motor[i]);
    }
}

static float control_dt[4];
static float control_start[4];
static float dm_send_t[4];
float dm_obs[4];
#define DM_RATIO 1.0f
#define DM_OUTPUT_LIMIT  15.0f


/*目前是以护栏较窄的一侧为正方向,从正方向向后看去,rigdm_front:id 2 ; rigdm_back:id 3;left_front:id 1;left_back:id 4*/
/* 1 号电机
 * 输入值为正,逆时针
 * */
static dm_motor_para_t dm_control_1(dm_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static dm_motor_para_t set;

    dm_send_t[0] = -vmc_out_l[1] * DM_RATIO;

    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[0] = dm_send_t[0] ;


    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[0] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK)){/*倒地自起中,腿部未在规定起立位置就采用位置控制*/
        dm_send_t[0] = 0;/*T置零*/

        LIMIT_MIN_MAX(dm_p_set[LEFT][BACK], DM_P_MIN, DM_P_MAX);
        set.p = dm_p_set[LEFT][BACK];

        set.kp = DM_MIT_KP;/*TODO 尝试参数待定*/
        LIMIT_MIN_MAX(set.kp, DM_KP_MIN, DM_KP_MAX);

        LIMIT_MIN_MAX(dm_v_set[LEFT][BACK], DM_V_MIN, DM_V_MAX);
        set.v = dm_v_set[LEFT][BACK];

        set.kd = DM_MIT_KD;
        LIMIT_MIN_MAX(set.kd, DM_KD_MIN, DM_KD_MAX);

    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[0] = 0;
        set.kp = 0;
        set.kd = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.t = dm_send_t[0]; // 正负没问题
    }
    return set;
}
/* 2 号电机
* 输入值为正,逆时针
 * */
/*应该顺时针转*/
static dm_motor_para_t dm_control_2(dm_motor_measure_t measure)
{
    control_dt[1] = dwt_get_time_us() - control_start[1];
    control_start[1] = dwt_get_time_us();
    static dm_motor_para_t set;


    dm_send_t[1] = vmc_out_r[1] * DM_RATIO;

    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[1] = dm_send_t[1] ;


    if((chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT)){
        dm_send_t[1] = 0;
        set.kp = 0;
        set.kd = 0;
    }

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK)){/*倒地自起中,腿部未在规定起立位置就采用位置控制*/
        dm_send_t[1] = 0;/*T置零*/

        LIMIT_MIN_MAX(dm_p_set[RIGHT][BACK], DM_P_MIN, DM_P_MAX);
        set.p = dm_p_set[RIGHT][BACK];

        set.kp = DM_MIT_KP;/*TODO 尝试参数待定*/
        LIMIT_MIN_MAX(set.kp, DM_KP_MIN, DM_KP_MAX);

        LIMIT_MIN_MAX(dm_v_set[RIGHT][BACK], DM_V_MIN, DM_V_MAX);
        set.v = dm_v_set[RIGHT][BACK];

        set.kd = DM_MIT_KD;
        LIMIT_MIN_MAX(set.kd, DM_KD_MIN, DM_KD_MAX);

    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[1] = 0;
        set.kp = 0;
        set.kd = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.t = dm_send_t[1]; // 正负没问题
    }
    return set;
}
/* 3 号电机
 * 输入值为正,逆时针
 * */
/*应该逆时针转*/
static dm_motor_para_t dm_control_3(dm_motor_measure_t measure)
{
    control_dt[2] = dwt_get_time_us() - control_start[2];
    control_start[2] = dwt_get_time_us();
    static dm_motor_para_t set;

    dm_send_t[2] = vmc_out_r[0] * DM_RATIO ;

    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[2] = dm_send_t[2] ;


    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[2] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK)){/*倒地自起中,腿部未在规定起立位置就采用位置控制*/
        dm_send_t[2] = 0;/*T置零*/

        LIMIT_MIN_MAX(dm_p_set[RIGHT][FRONT], DM_P_MIN, DM_P_MAX);
        set.p = dm_p_set[RIGHT][FRONT];

        set.kp = DM_MIT_KP;/*TODO 尝试参数待定*/
        LIMIT_MIN_MAX(set.kp, DM_KP_MIN, DM_KP_MAX);

        LIMIT_MIN_MAX(dm_v_set[RIGHT][FRONT], DM_V_MIN, DM_V_MAX);
        set.v = dm_v_set[RIGHT][FRONT];

        set.kd = DM_MIT_KD;
        LIMIT_MIN_MAX(set.kd, DM_KD_MIN, DM_KD_MAX);

    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[2] = 0;
        set.kp = 0;
        set.kd = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.t = dm_send_t[2]; // 正负没问题
    }
    return set;
}
/* 4 号电机
 * 输入为正,方向逆时针
 * */
static dm_motor_para_t dm_control_4(dm_motor_measure_t measure)
{
    control_dt[3] = dwt_get_time_us() - control_start[3];
    control_start[3] = dwt_get_time_us();
    static dm_motor_para_t set;


    dm_send_t[3] = -vmc_out_l[0] * DM_RATIO;

    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[3] = dm_send_t[3];

    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[3] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK)){/*倒地自起中,腿部未在规定起立位置就采用位置控制*/
        dm_send_t[3] = 0;/*T置零*/

        LIMIT_MIN_MAX(dm_p_set[LEFT][FRONT], DM_P_MIN, DM_P_MAX);
        set.p = dm_p_set[LEFT][FRONT];

        set.kp = DM_MIT_KP;/*TODO 尝试参数待定*/
        LIMIT_MIN_MAX(set.kp, DM_KP_MIN, DM_KP_MAX);

        LIMIT_MIN_MAX(dm_v_set[LEFT][FRONT], DM_V_MIN, DM_V_MAX);
        set.v = dm_v_set[LEFT][FRONT];

        set.kd = DM_MIT_KD;
        LIMIT_MIN_MAX(set.kd, DM_KD_MIN, DM_KD_MAX);

    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
    dm_send_t[3] = 0;
    set.kp = 0;
    set.kd = 0;

#endif

    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.t = dm_send_t[3]; // 正负没问题
    }
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *dm_control[4] =
        {
                dm_control_1,
                dm_control_2,
                dm_control_3,
                dm_control_4,
        };

static void dm_motor_init()
{
    /*电机id等可参考平步实物进行修改*/
    motor_config_t dm_motor_config1 = {   //left_back
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_LEFT_BACK,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[0] = dm_motor_register(&dm_motor_config1, dm_control[0]);

    motor_config_t dm_motor_config4 = {     //left_front
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_LEFT_FRONT,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[3] = dm_motor_register(&dm_motor_config4, dm_control[3]);

    motor_config_t dm_motor_config2 = {     //right_back
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_RIGHT_BACK,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[1] = dm_motor_register(&dm_motor_config2, dm_control[1]);

    motor_config_t dm_motor_config3 = {     //roght_front
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_RIGHT_FRONT,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[2] = dm_motor_register(&dm_motor_config3, dm_control[2]);


}


static int16_t set_l,set_r;/*观测用*/
/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为负
 * */
static int16_t M3508_control_l(dji_motor_measure_t  measure){
    static int16_t set;
    LIMIT_MIN_MAX(LQROutBuf[LEFT][0],-M3508_TOR_MAX,M3508_TOR_MAX);

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && chassis_fdb_data.stand_state == CHASSIS_LEG_BACK_IS_OK){/*倒地自起没好时还是关闭轮子*/
        set = (int16_t)(-(LQROutBuf[LEFT][0])* M3508_TOR_TO_CUR);
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK){
        set = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP){ //平衡时,才启动转向
        if(Wheel_Shut_Flag == 0){
            set = (int16_t)(-(LQROutBuf[LEFT][0] - yaw_pid->Output )* M3508_TOR_TO_CUR);
        }else{
            set = (int16_t)(-(LQROutBuf[LEFT][0])* M3508_TOR_TO_CUR);
        }
    }

    set_l = set;

#ifdef M3508_SET_ZERO
    set = 0;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
    }
    return set;
}
/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为正
 * */
static int16_t M3508_control_r(dji_motor_measure_t measure){
    static int16_t set;
    LIMIT_MIN_MAX(LQROutBuf[RIGHT][0],-M3508_TOR_MAX,M3508_TOR_MAX);

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && chassis_fdb_data.stand_state == CHASSIS_LEG_BACK_IS_OK){/*倒地自起没好时还是关闭轮子*/
        set = (int16_t)((LQROutBuf[RIGHT][0]) * M3508_TOR_TO_CUR);
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK){
        set = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP){ //平衡时,才启动转向
        if(Wheel_Shut_Flag == 0){
            set = (int16_t)((LQROutBuf[RIGHT][0] + yaw_pid->Output) * M3508_TOR_TO_CUR);
        }else{
            set = (int16_t)((LQROutBuf[RIGHT][0]) * M3508_TOR_TO_CUR);
        }
    }

    set_r = set;

#ifdef M3508_SET_ZERO
    set = 0;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
    }

    return set;
}

static void *M3508_control[2] =
        {
                M3508_control_l,
                M3508_control_r,
        };

static void m3508_motor_init()
{
    motor_config_t motor_config1 = {
            .motor_type = M3508,
            .can_id = CAN_ID_WHEEL_MOTOR,
            .rx_id = CHASSIS_WHEEL_LEFT_ID,
    };
    m3508_motor[LEFT] = dji_motor_register(&motor_config1, M3508_control[LEFT]);

    motor_config_t motor_config_2 = {
            .motor_type = M3508,
            .can_id = CAN_ID_WHEEL_MOTOR,
            .rx_id = CHASSIS_WHEEL_RIGHT_ID,
    };
    m3508_motor[RIGHT] = dji_motor_register(&motor_config_2, M3508_control[RIGHT]);
}


/********************************************************************************************************************/
/**
 * @brief 底盘跳跃处理
 * @note 进入跳跃模式时记录当前时间，并将jump输出力矩加上跳跃下压力矩值，持续0.2s,开始跳跃0，15s后加上跳跃回缩力矩值，持续0.4s,落地后跳跃结束
 */
// TODO: 结合离地检测进行优化（进阶玩法：空中轮子充当动量轮调整姿态
// TODO: 跳跃力矩和持续时间还需继续优化
// TODO: 跳跃前需要增加机体pitch轴范围判断，或起跳前通过改写k矩阵，使得机体pitch轴更快收敛，充分准备起跳，减少空中发撒
// TODO：目前通过多状态量判断跳跃是否结束，以及切入跳跃模式仅跳跃一次，需要进一步优化，可以结合cmd线程
static void jumping_control(void)
{
    float current_time = dwt_get_time_ms();
    float dt;
    if(chassis_cmd.last_mode != CHASSIS_JUMP)
    {
        jump_flag = 1;
    }

    if(jump_flag)
    {

        if(!is_jumping)
        {
            is_jumping = 1;
            jump_start_time = current_time;
        }
        dt = current_time - jump_start_time;

        jump_out_l[FRONT] = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<200);
        jump_out_l[BACK]  =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<200);
        jump_out_r[FRONT] =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<200);
        jump_out_r[BACK]  = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<200);

        if (dt > 200)
        {
            is_jumping = 0;  // 跳跃结束
            jump_flag = 0;  // 跳跃结束
        }
    }
}
#endif /* BSP_CHASSIS_LEG_MODE */


/*
 * @brief 判断是否离地
 * @param:Off_Ground_Flag离地标志位
 * @param:Touch_Ground_Flag触地标志位
 * */
static void Ground_Detect(){

    if(FN_Average < OFF_GROUND){
        Off_Ground_Cnt++;
        if(Off_Ground_Cnt > 4){
            Off_Ground_Flag = 1;
            Touch_Ground_Flag = 0;
            Off_Ground_Cnt = 0;

        }
        Touch_Ground_Cnt = 0;
    }else{
        Touch_Ground_Cnt ++;
        if(Touch_Ground_Cnt > 4){
            Touch_Ground_Flag = 1;
            Touch_Ground_Cnt = 0;
            Off_Ground_Flag =0;
        }
        Off_Ground_Cnt = 0;
    }
}

static void Leg_Is_Ok(){

    if((leg[LEFT]->l0 > (leg[LEFT]->length_ref - LEN_JUDGE_REGION)) && (leg[LEFT]->l0 < (leg[LEFT]->length_ref + LEN_JUDGE_REGION))){
        Leg_Left_len_JudgeOK = 1;
    }else{
        Leg_Left_len_JudgeOK = 0;
    }

    if((leg[RIGHT]->l0 > (leg[RIGHT]->length_ref - LEN_JUDGE_REGION)) && (leg[RIGHT]->l0 < (leg[RIGHT]->length_ref + LEN_JUDGE_REGION))){
        Leg_right_len_JudgeOK = 1;
    }else{
        Leg_right_len_JudgeOK = 0;
    }

    if((ins.roll > (0.0f -ROLL_JUDGE_REGION)) && (ins.roll < (0.0f +ROLL_JUDGE_REGION))){
        Body_roll_JudgeOK = 1;

    }else{
        Body_roll_JudgeOK = 0;
    }
    if((Leg_Left_len_JudgeOK == 1 || Leg_right_len_JudgeOK == 1) && Body_roll_JudgeOK == 1){
        chassis_cmd.leg_leng_change = LENGTH_STAY;
    }

}

/*
 * @brief 在非平衡状态下,把length_pid相关的积分清空,防止长时间累加开腿时失控
 */
void Process_Clear(){
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    pid_clear(roll_pid);
    pid_clear(theta_pid);
    pid_clear(yaw_pid);

    LQRXRefBuf[LEFT][3] = 0;
    LQRXRefBuf[RIGHT][3] = 0;

    LQRXObsBuf[LEFT][3] = 0;
    LQRXObsBuf[RIGHT][3] = 0;

    yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;

    phi0_refer_change_flag = 0;

}


static void Chassis_Vx_Detect(){

    Vx_Delta = fabsf(chassis_kf_l.FilteredValue[0] - chassis_kf_r.FilteredValue[0]);//一边卡住时
    if(Vx_Delta > VX_DELTA_MAX){
        yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
    }
    else{
        //更新航向角期望
        yaw_target += ( - chassis_cmd.vw_relative_set / RC_DBUS_MAX_VALUE) * YAW_TURN_RATIO * DEGREE_2_RAD;

    }
}

/*
 * @brief 检查底盘姿态,认为溃散后失能
 * */
static void Security_Checking(){

    if((fabsf(ins.pitch) > 60.0f) || (leg[LEFT]->phi0 <= LEG_SAFE_AREA * DEGREE_2_RAD || leg[LEFT]->phi0 >= PI - LEG_SAFE_AREA * DEGREE_2_RAD || leg[LEFT]->phi0 <= 0 )
       || (leg[RIGHT]->phi0 <= LEG_SAFE_AREA * DEGREE_2_RAD || leg[RIGHT]->phi0 >= PI - LEG_SAFE_AREA * DEGREE_2_RAD || leg[RIGHT]->phi0 <= 0 ))
        chassis_fdb_data.stand_state = CHASSIS_IS_DANGER;

}

/*
 * @brief 倒地自起
 * */
#define PHI0_DIRECTION1 0.0F
#define PHI0_DIRECTION2 (PI - 10.0f * DEGREE_2_RAD)

static void Chassis_Recovery() {

    dm_v_set[LEFT][FRONT] = DM_V_SET;
    dm_v_set[LEFT][BACK] = DM_V_SET;
    dm_v_set[RIGHT][FRONT] = DM_V_SET;
    dm_v_set[RIGHT][BACK] = DM_V_SET;
    leg_l0_refer_L = 0.33f;
    leg_l0_refer_R = 0.33f;

    //可以通过机体的ins.pitch将机体分成四个区域每个区域90°,通过pitch角度调整腿部摆动方向,即调整phi0_refer
    if(((ins.pitch <= 0.0f && ins.pitch >= -90.0f)||(ins.pitch > 0.0f && ins.pitch <= 90.0f))&&(phi0_refer_change_flag == 0)){/*将ins.pitch分成四块区域,每块区域90°*/
        leg_phi0_refer_L = PHI0_DIRECTION1;
        leg_phi0_refer_R = PHI0_DIRECTION1;
        phi0_refer_change_flag = 1;/*确保在腿部转动过程中不会被其他区域的状态打断*/
    }

    if((ins.pitch >= -180.0f && ins.pitch <= -90.0f)||(ins.pitch <= 180.0f && ins.pitch > 90.0f)&&(phi0_refer_change_flag == 0)) {/*将ins.pitch分成四块区域,每块区域90°*/
        leg_phi0_refer_L = PHI0_DIRECTION2;
        leg_phi0_refer_R = PHI0_DIRECTION2;
        phi0_refer_change_flag = 1;

    }
    if(phi0_refer_change_flag == 1){
        if(leg_phi0_refer_L == PHI0_DIRECTION2 && leg_phi0_refer_R == PHI0_DIRECTION2){/*判断腿部是否到达目标位置1*/
            if((fabsf(ins.pitch) < 10.0f) && (fabsf(fabsf(leg[LEFT]->phi0) - 3.14f) < 0.64f) && (fabsf(fabsf(leg[RIGHT]->phi0) - 3.14f) < 0.64f)){
                phi0_refer_change_flag = 0;
            }
        }
        if(leg_phi0_refer_L == PHI0_DIRECTION1 && leg_phi0_refer_R == PHI0_DIRECTION1){/*判断腿部是否到达目标位置2*/
            if((fabsf(ins.pitch) < 5.0f) && ((fabsf(leg[LEFT]->phi0) < 0.3f) && (fabsf(leg[RIGHT]->phi0) < 0.3f))){
                phi0_refer_change_flag = 0;
                chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
            }
        }
    }
    if(chassis_fdb_data.stand_state == CHASSIS_IS_RECOVERY_READY){
        if((fabsf(leg[LEFT]->l0 - LEN_LEN_LOW) < 0.04f) && (fabsf(leg[RIGHT]->l0 - LEN_LEN_LOW) < 0.04f)){
            chassis_fdb_data.stand_state = CHASSIS_LEG_BACK_IS_OK;
        }
    }

    if ((fabsf(ins.pitch) < 2.0f) && (fabsf(leg[LEFT]->theta) < 0.15f) &&
        (fabsf(leg[RIGHT]->theta) < 0.15f)) {/*判断是否站立稳定是通过phi角大小*/
        chassis_fdb_data.stand_state = CHASSIS_IS_STAND;/*完成起立可以正常控制,若是起立动作幅度过大可以考虑提高站立条件*/
        phi0_refer_change_flag = 0;
    }

    leg[LEFT]->vmc_calc_inv(leg[LEFT],leg_phi0_refer_L,leg_l0_refer_L);
    leg[RIGHT]->vmc_calc_inv(leg[RIGHT],leg_phi0_refer_R,leg_l0_refer_R);

    leg[LEFT]->phi1_phi4_calc_left_inv(leg[LEFT],leg[LEFT]->phi1_inv,leg[LEFT]->phi4_inv);
    leg[RIGHT]->phi1_phi4_calc_right_inv(leg[RIGHT],leg[RIGHT]->phi1_inv,leg[RIGHT]->phi4_inv);


    dm_p_set[LEFT][BACK] = leg[LEFT]->motor_phi4_inv_position_refer;
    dm_p_set[LEFT][FRONT] = leg[LEFT]->motor_phi1_inv_position_refer;
    dm_p_set[RIGHT][BACK] = leg[RIGHT]->motor_phi4_inv_position_refer;
    dm_p_set[RIGHT][FRONT] = leg[RIGHT]->motor_phi1_inv_position_refer;
}

/*
 * @brief 斜坡函数
 * @param target目标值
 * @param measure 量测值
 * @param set 斜坡过程值
 * @param step_length 步长
 * @param safe_region 合理范围
 * */
void slope_phi0_following_begin_end(const float *target,const float *measure,float *set,float step_length,const float safe_region,uint8_t *phi0_change_flag){

    if(*target > *measure)
    {
        if(*phi0_change_flag == 0 ) {
            *set = *measure + step_length;
            *phi0_change_flag = 1;
        }
        if((*set - *measure) < safe_region){
            *set = *set + step_length;
        }
        if((*target - *set) < safe_region){
            *set = *target;
            *phi0_change_flag = 0;
        }
    }
    else if(*target < *measure)
    {
        if(*phi0_change_flag == 0 ) {
            *set = *measure - step_length;
            *phi0_change_flag = 1;
        }
        if((*measure - *set) < safe_region){
            *set = *set - step_length;
        }
        if((*set - *target) < safe_region){
            *set = *target;
            *phi0_change_flag = 0;
        }

    }
}

/*********************************************subcription and publication***************************************************************************/

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd);
    }
}