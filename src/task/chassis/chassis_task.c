/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2025-07-25      SuperChen       second version
* 2025-11-01      SuperChen       third version (结合上交建模lqr)
* 2026-01-15      SuperChen       fourth version (尝试港大的仿真)
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
#define LEN_LEN_MID     0.20f // 单位：m
#define LEN_LEN_HIG     0.28f // 单位：m
#define FORCE_Length_LIMIT 200.0f //
#define FORCE_LIMIT 200.0f // 支持力限幅

#define Theta_Compensation  -0.125f //-0.07

#define VX_MAX        673.0f
#define WX_MAX        270.0f
#define V_SET         2.0f
#define YAW_TURN_RATIO  0.4f  //有关调节遥控器转向敏感度的系数,自行在安全范围内调节大小
static float Vx_Delta;
#define VX_DELTA_MAX 0.05f
static float yaw_turn_region_max ;//限制转向范围,防止转向时抽风
static float yaw_turn_region_min ;
#define yaw_turn_region 50.0f

/*髋关节电机 DMJ8009P-2EC 实例*/
static dm_motor_object_t *dm_motor[4];

/* 驱动电机 LK9025 实例 */
static dji_motor_object_t *m3508_motor[2];

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
#define YAW_DELTA_MX 80.0f


#define Location 0      //腿长位置环
#define Speed    1      //腿长速度环
static void leg_calc();


static wbr_leg_obj_t * leg[2] = {NULL};
static float ft_l[2], ft_r[2];  // FT = [PendulumForce PendulumTorque]
static float WBR_T_L[2];  // wbr计算得出的扭矩值 左边腿,即关节电机扭矩控制值
static float WBR_T_R[2];  // wbr计算得出的扭矩值 右边腿
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

/* --------------------------------- LQR控制相关 -------------------------------- */

/*记录下Q和R
L = 0.2m
float K[4][10] = {
    {  -0.218365f,   -0.886697f,   -4.366343f,   -0.681761f,    7.851981f,    0.460853f,  -11.688398f,   -0.716310f,   41.927717f,    1.992520f},  // T_r_to_b
    {  -0.218365f,   -0.886697f,    4.366343f,    0.681761f,  -11.688398f,   -0.716310f,    7.851981f,    0.460853f,   41.927717f,    1.992520f},  // T_l_to_b
    {   0.405560f,    1.660786f,   -3.353969f,   -0.428159f,    2.728525f,    0.211072f,    8.620103f,    0.600530f,    5.751655f,    0.480080f},  // T_wr_to_r
    {   0.405560f,    1.660786f,    3.353969f,    0.428159f,    8.620103f,    0.600530f,    2.728525f,    0.211072f,    5.751655f,    0.480080f}   // T_wl_to_l
};

% Q矩阵: 状态权重
%      状态: [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]
%              位置    速度  偏航  偏航速  左腿角   左腿速   右腿角   右腿速    俯仰角   俯仰速
lqr_Q = diag([1,      10,  100,   1  ,  200,     1 ,        200,      1,       4000,    1]);

% R矩阵: 控制输入权重
% 控制: [T_{r→b}, T_{l→b}, T_{wr→r}, T_{wl→l}]
%        右髋扭矩   左髋扭矩   右轮扭矩   左轮扭矩
lqr_R = diag([1,       1,        2.75,        2.75]);

    'a00 + a10*x + a01*y + a20*x^2 + a11*x*y + a02*y^2'
*/

static float K_coef[40][6] = {
        {    -0.73737f,      3.05967f,    -0.241781f,     -1.82056f,     -4.22821f,      2.33138f},  // K[0][0]
        {    -0.73737f,    -0.241781f,      3.05967f,      2.33138f,     -4.22821f,     -1.82056f},  // K[0][1]
        {    0.202708f,    -0.248723f,      1.55762f,     0.263085f,    -0.664854f,     -1.73986f},  // K[0][2]
        {    0.202708f,      1.55762f,    -0.248723f,     -1.73986f,    -0.664854f,     0.263085f},  // K[0][3]
        {    -3.73239f,      15.5394f,     -0.60202f,     -8.84552f,        -23.2f,      11.7164f},  // K[0][4]
        {    -3.73239f,     -0.60202f,      15.5394f,      11.7164f,        -23.2f,     -8.84552f},  // K[0][5]
        {     1.09386f,     -1.58924f,      7.21367f,     0.533274f,     -0.63337f,     -9.45177f},  // K[0][6]
        {     1.09386f,      7.21367f,     -1.58924f,     -9.45177f,     -0.63337f,     0.533274f},  // K[0][7]
        {    -3.03014f,     -7.06972f,     -3.06951f,      11.9458f,     -13.1216f,      10.8667f},  // K[0][8]
        {     3.03014f,      3.06951f,      7.06972f,     -10.8667f,      13.1216f,     -11.9458f},  // K[0][9]
        {    -3.91538f,       -1.568f,      5.45772f,     0.704705f,      7.86216f,      -10.155f},  // K[1][0]
        {     3.91538f,     -5.45772f,        1.568f,       10.155f,     -7.86216f,    -0.704705f},  // K[1][1]
        {   -0.420261f,    -0.917012f,    -0.678215f,      1.46123f,     -2.14686f,      1.74738f},  // K[1][2]
        {    0.420261f,     0.678215f,     0.917012f,     -1.74738f,      2.14686f,     -1.46123f},  // K[1][3]
        {   -0.510137f,    -0.336413f,     0.775887f,      0.21584f,      1.05994f,     -1.23357f},  // K[1][4]
        {    0.510137f,    -0.775887f,     0.336413f,      1.23357f,     -1.05994f,     -0.21584f},  // K[1][5]
        {     3.66028f,      19.8788f,      25.0155f,     -17.8548f,      26.4867f,     -50.6838f},  // K[1][6]
        {    -25.7367f,      55.9712f,     -16.4797f,     -57.4689f,     -28.8167f,      20.8026f},  // K[1][7]
        {     4.28318f,      25.7321f,     -24.6871f,     -18.1716f,     -25.2738f,      30.1839f},  // K[1][8]
        {      12.549f,      35.5487f,     -8.18873f,     -53.8857f,      22.6452f,      2.46959f},  // K[1][9]
        {    0.249431f,     0.231897f,      1.04175f,      1.02326f,    0.0480767f,     -1.69203f},  // K[2][0]
        {     -1.0644f,      2.58963f,    -0.653863f,     -3.88313f,    -0.483392f,     0.668367f},  // K[2][1]
        {    0.210319f,      1.01455f,    -0.887321f,    -0.223898f,    -0.607355f,     0.957851f},  // K[2][2]
        {    0.516413f,      1.27039f,    -0.343494f,     -1.00927f,     0.963344f,    0.0908871f},  // K[2][3]
        {    -25.7367f,     -16.4797f,      55.9712f,      20.8026f,     -28.8167f,     -57.4689f},  // K[2][4]
        {     3.66028f,      25.0155f,      19.8788f,     -50.6838f,      26.4867f,     -17.8548f},  // K[2][5]
        {      12.549f,     -8.18873f,      35.5487f,      2.46959f,      22.6452f,     -53.8857f},  // K[2][6]
        {     4.28318f,     -24.6871f,      25.7321f,      30.1839f,     -25.2738f,     -18.1716f},  // K[2][7]
        {     -1.0644f,    -0.653863f,      2.58963f,     0.668367f,    -0.483392f,     -3.88313f},  // K[2][8]
        {    0.249431f,      1.04175f,     0.231897f,     -1.69203f,    0.0480767f,      1.02326f},  // K[2][9]
        {    0.516413f,    -0.343494f,      1.27039f,    0.0908871f,     0.963344f,     -1.00927f},  // K[3][0]
        {    0.210319f,    -0.887321f,      1.01455f,     0.957851f,    -0.607355f,    -0.223898f},  // K[3][1]
        {     30.1141f,     -47.6694f,      87.8999f,      38.0828f,      14.0628f,     -115.955f},  // K[3][2]
        {     30.1141f,      87.8999f,     -47.6694f,     -115.955f,      14.0628f,      38.0828f},  // K[3][3]
        {     14.3487f,     -14.6601f,      -40.803f,       11.736f,      31.2987f,       27.813f},  // K[3][4]
        {     14.3487f,      -40.803f,     -14.6601f,       27.813f,      31.2987f,       11.736f},  // K[3][5]
        {     1.39376f,     -2.68527f,      5.80599f,      2.38371f,     -1.20063f,     -6.18575f},  // K[3][6]
        {     1.39376f,      5.80599f,     -2.68527f,     -6.18575f,     -1.20063f,      2.38371f},  // K[3][7]
        {     1.09968f,     -1.75109f,     -2.27919f,      1.38558f,      2.86209f,     0.977652f},  // K[3][8]
        {     1.09968f,     -2.27919f,     -1.75109f,     0.977652f,      2.86209f,      1.38558f}   // K[3][9]
};


/* [T_lwl T_lwr(轮子输出扭矩) T_bll T_blr(髋关节输出扭矩)] */
static float LQROutBuf[4]={0};

/* X= [s s_dot φ φ_dot θ_ll θ_ll_dot θ_lr θ_lr_dot θ_b θ_b_dot] 参量命名跟上交开源一致*/
static float LQRXerrorBuf[1][10]={0};
static float LQRXObsBuf[1][10]={0};

/*LQRXRefBuf[1][10] - LQRObsBuf[1][10]*/
static float LQRXRefBuf[1][10]={0};

/*反馈系数K由matlab拟合出的参数结合当前左右腿腿长拟合得出 K为4*10矩阵 */
static float MatLQR_K[4][10] = {0};//LQR运算中的反馈系数,在lqr_update参数更新函数中更新


/*
 * Matrix_LQRObs
 * [s
 * s_dot
 * φ
 * φ_dot
 * θ_ll
 * θ_ll_dot
 * θ_lr
 * θ_lr_dot
 * θ_b
 * θ_b_dot
 * ]
 */
static arm_matrix_instance_f32 MatLQRObs    =  {10, 1, LQRXObsBuf[0]};
static arm_matrix_instance_f32 MatLQRRef    =  {10, 1, LQRXRefBuf[0]};
static arm_matrix_instance_f32 MatLQRNegK   =  {4, 10, (float*)MatLQR_K};/*TODO 后续需要测试是否可行*/
static arm_matrix_instance_f32 MatLQRErrX    = {10, 1, LQRXerrorBuf[0]};

/*
 * U
 * [T_lwl  驱动轮输出力矩
 * T_lwr
 * T_bll  髋关节输出力矩
 * T_blr
 * ]
 * */
static arm_matrix_instance_f32 MatLQROutU = {4, 1, LQROutBuf};



/*Calculate X. Output is u (T,Tp)`*/
/*TODO 加入对于theta和phi的死区,否则难以稳定下来*/
static void LQR_Cal(){

    //Calculate error
    /*cmsis dsp中的矩阵减法运算,error = -(ref(目标值) - obs(观测值)) TODO 正负对不对待定,港大开源的仿真算出来的k好像要加符号,这里相当于加了*/
    arm_mat_sub_f32(&MatLQRObs,&MatLQRRef,&MatLQRErrX);

    //Calculate output value
    /*矩阵乘法,[4*10]*[10*1] */
    /*TODO 不同腿长下K矩阵会随之改变*/
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX, &MatLQROutU);

}


/* ------------------------------- 打滑检测卡尔曼滤波部分 ------------------------------ */
//TODO:后续考虑移入观测线程，建立整车观测器
static KalmanFilter_t chassis_kf_l;
static KalmanFilter_t chassis_kf_r;
static float chassis_vx_filter;

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
static float Wecd_L,Wecd_R; //轮子对地的角速度
static void chassis_kf_update(void)
{
    static float _kf_dt, _kf_start;
    if(_kf_start != 0) // 避免第一次计算错误
        _kf_dt = (dwt_get_time_ms() - _kf_start) / 1000.0f;
    else
        _kf_dt = 0.003f;
    _kf_start = dwt_get_time_ms();

    Wecd_L = -(m3508_motor[LEFT]->measure.speed_aps / M3508_READUCTION_RATIO_L * DEGREE_2_RAD);
    speed_rads_ground_l = Wecd_L +(-ins.gyro[0] * DEGREE_2_RAD)/*φ_dot_bc*/ + leg[LEFT]->wbr_d_theta/*Web*/ ;
    wheel_to_ground_l = speed_rads_ground_l * WHEEL_RADIUS + leg[LEFT]->wbr_d_theta*leg[LEFT]->L* arm_cos_f32(leg[LEFT]->wbr_theta)+ leg[LEFT]->d_L*arm_sin_f32(leg[LEFT]->wbr_theta) ;//TODO机体速度推出轮子速度

    Wecd_R = (m3508_motor[RIGHT]->measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD);
    speed_rads_ground_r = Wecd_R +(-ins.gyro[0] * DEGREE_2_RAD)/*φ_dot_bc*/ + leg[RIGHT]->wbr_d_theta/*Web*/ ;
    wheel_to_ground_r = speed_rads_ground_r * WHEEL_RADIUS + leg[RIGHT]->wbr_d_theta*leg[RIGHT]->L* arm_cos_f32(leg[RIGHT]->wbr_theta)+ leg[RIGHT]->d_L*arm_sin_f32(leg[RIGHT]->wbr_theta);

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

    chassis_vx_filter = 0.5f * (chassis_kf_l.FilteredValue[0] + chassis_kf_r.FilteredValue[0]);
    /*  更新观测矩阵 [0]:s;[1]:s_dot;[2]:φ;[3]:φ_dot;[4]:θ_ll;[5]:θ_ll_dot;[6]:θ_lr;[7]:θ_lr_dot;[8]:θ_b;[9]:θ_b_dot */

    LQRXObsBuf[0][0] = 0;
//    LQRXObsBuf[0][1] = -chassis_vx_filter ;
    LQRXObsBuf[0][1] = 0 ;
//    LQRXObsBuf[0][2] = -ins.yaw_total_angle * DEGREE_2_RAD;    //沿着机体正方向逆时针设为正,单位°,TODO: 不知道目前单位是否正确,需要后续测试,或应该把所有角度全部用弧度表示
//    LQRXObsBuf[0][3] = -ins.gyro[2] * DEGREE_2_RAD;  //偏航角角速度,沿着机体正方向逆时针设为正
    LQRXObsBuf[0][2] = 0;    //沿着机体正方向逆时针设为正,单位°,TODO: 不知道目前单位是否正确,需要后续测试,或应该把所有角度全部用弧度表示
    LQRXObsBuf[0][3] = 0;  //偏航角角速度,沿着机体正方向逆时针设为正
    LQRXObsBuf[0][4] = leg[LEFT]->wbr_theta + Theta_Compensation;
//    LIMIT_MIN_MAX(LQRXObsBuf[0][4],-1.4,1.4);
    LQRXObsBuf[0][5] = leg[LEFT]->wbr_d_theta;
    LQRXObsBuf[0][6] = leg[RIGHT]->wbr_theta + Theta_Compensation;//Theta_Compensation质心补偿
//    LIMIT_MIN_MAX(LQRXObsBuf[0][6],-1.4,1.4);
    LQRXObsBuf[0][7] = leg[RIGHT]->wbr_d_theta;
    LQRXObsBuf[0][8] = ins.pitch * DEGREE_2_RAD;  //机体与水平方向倾角正负待标定
    LQRXObsBuf[0][9] = -ins.gyro[0] * DEGREE_2_RAD ;


    LQRXRefBuf[0][0] = 0; //目前采用速度控制,后续对速度误差积分作为位移s项
//    LQRXRefBuf[0][1] = (chassis_cmd.vx_set / VX_MAX) * V_SET;  //  m/s
    LQRXRefBuf[0][1] = 0;  //  m/s
//    LQRXRefBuf[0][2] = yaw_target;  //转向控制
    LQRXRefBuf[0][2] = 0;  //转向控制
    LQRXRefBuf[0][3] = 0;
    LQRXRefBuf[0][4] = 0;
    LQRXRefBuf[0][5] = 0;
    LQRXRefBuf[0][6] = 0;
    LQRXRefBuf[0][7] = 0;
    LQRXRefBuf[0][8] = 0;
    LQRXRefBuf[0][9] = 0;

    if(chassis_cmd.leg_leng_change == LENGTH_STAY && Off_Ground_Flag == 1 ){//TODO: 应该改成离地检测满足时,起跳时,将除了K21和K22以外的K置零,防止空中腿部姿态溃散
        Wheel_Shut_Flag = 1;
        yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;

        LQRXRefBuf[0][0] = 0; //目前采用速度控制,后续对速度误差积分作为位移s项
        LQRXRefBuf[0][1] = 0;  //  m/s
        LQRXRefBuf[0][2] = yaw_target;  //转向控制,取消转向控制

/* K_4*10 X:[[0]:s;[1]:s_dot;[2]:φ;[3]:φ_dot;  [4]:θ_ll;[5]:θ_ll_dot;[6]:θ_lr;[7]:θ_lr_dot;  [8]:θ_b;[9]:θ_b_dot],U:[T_lwl T_lwr(轮子输出扭矩) T_bll T_blr(髋关节输出扭矩)]检测到离地时只考虑维持腿部姿态的量更新,其余量置零,轮子关闭*/
// 拟合系数 K_coef[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
// 根据腿长计算K矩阵
        for (int n = 0; n < 40; n++) {
            int col = n / 4;   // 每4个元素对应一列（0~9）
            int row = n % 4;   // 每列里的行号（0~3）
            if((row == 2 && (col >= 4 && col <= 7)) || (row == 4 && (col >= 4 && col <= 7))){
                MatLQR_K[row][col] = K_coef[n][0]
                                     + K_coef[n][1] * leg[LEFT]->L
                                     + K_coef[n][2] * leg[RIGHT]->L
                                     + K_coef[n][3] * leg[LEFT]->L * leg[LEFT]->L
                                     + K_coef[n][4] * leg[LEFT]->L * leg[RIGHT]->L
                                     + K_coef[n][5] * leg[RIGHT]->L * leg[RIGHT]->L;
            }
            else{
                MatLQR_K[row][col] = 0;
            }
        }


    }else{/*TODO 在腿长不切换是每次都进行运算浪费资源 */
        /*更新LQR反馈矩阵K*/
        Wheel_Shut_Flag = 0;

// 拟合系数 K_coef[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
// 根据腿长计算K矩阵
        for (int n = 0; n < 40; n++) {
            int col = n / 4;   // 每4个元素对应一列（0~9）
            int row = n % 4;   // 每列里的行号（0~3）
            MatLQR_K[row][col] = K_coef[n][0]
                          + K_coef[n][1] * leg[LEFT]->L
                          + K_coef[n][2] * leg[RIGHT]->L
                          + K_coef[n][3] * leg[LEFT]->L * leg[LEFT]->L
                          + K_coef[n][4] * leg[LEFT]->L * leg[RIGHT]->L
                          + K_coef[n][5] * leg[RIGHT]->L * leg[RIGHT]->L;
        }
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
            leg[LEFT]->wbr_L_ref = LEN_LEN_LOW;
            leg[RIGHT]->wbr_L_ref = LEN_LEN_LOW;

            break;
        case LEG_MID:
            leg[LEFT]->wbr_L_ref = LEN_LEN_MID;
            leg[RIGHT]->wbr_L_ref = LEN_LEN_MID;

            break;
        case LEG_HIG:
            leg[LEFT]->wbr_L_ref = LEN_LEN_HIG;
            leg[RIGHT]->wbr_L_ref = LEN_LEN_HIG;

            break;
        default:
            leg[LEFT]->wbr_L_ref = LEN_LEN_LOW;
            leg[RIGHT]->wbr_L_ref = LEN_LEN_LOW;

            break;
    }

    switch (chassis_cmd.ctrl_mode)
    {
        case CHASSIS_RELAX:
            Process_Clear();
            motor_relax();
            chassis_fdb_data.stand_state = CAHSSIS_IS_FALL;
            LQRXRefBuf[LEFT][2] = 0;

            break;
        case CHASSIS_INIT:
            Process_Clear();
            motor_enable();
#ifdef DM_8009_SET_ZERO_POSITION
            leg_init_get_zero();
#endif
            break;
        case CHASSIS_RECOVERY:/*不稳定的状态,介于init和relex状态的过渡状态*/
            leg[LEFT]->wbr_L_ref = LEN_LEN_LOW;
            leg[RIGHT]->wbr_L_ref = LEN_LEN_LOW;
            yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
            motor_enable();
//            if(usr_abs(ins.pitch) < 2.0f )/*判断是否站立稳定是通过phi角大小*/
//                chassis_fdb_data.stand_state = CAHSSIS_IS_STAND;
            if((usr_abs(ins.pitch) < 2.0f) && (usr_abs(leg[LEFT]->wbr_d_theta) < 0.2f) && (usr_abs(leg[RIGHT]->wbr_d_theta) < 0.2f))/*判断是否站立稳定是通过phi角大小*/
                chassis_fdb_data.stand_state = CAHSSIS_IS_STAND;

            //TODO: 处于该模式下，应该屏蔽遥控器等控制

            break;

        case CHASSIS_OPEN_LOOP:/*底盘开环控制,init成功,或是revovery成功,此状态应为稳定站立时*/
            motor_enable();
            if(usr_abs(ins.pitch) > 50.0f )
                chassis_fdb_data.stand_state = CAHSSIS_IS_FALL;
            Chassis_Vx_Detect();
//            Ground_Detect();
            yaw_turn_region_max = -ins.yaw_total_angle * DEGREE_2_RAD + yaw_turn_region * DEGREE_2_RAD;
            yaw_turn_region_min = -ins.yaw_total_angle * DEGREE_2_RAD - yaw_turn_region * DEGREE_2_RAD;

            break;

        case CHASSIS_FOLLOW_GIMBAL:
            motor_enable();

            break;
        case CHASSIS_SPIN:
            motor_enable();

            break;
        case CHASSIS_JUMP:
            motor_enable();


            break;
        case CHASSIS_STOP:
            motor_relax();
            break;
        case CHASSIS_FLY:
            motor_enable();
            break;
        case CHASSIS_AUTO:
            motor_enable();
            break;
        default:

            break;
    }
#ifdef DM_8009_SET_ZERO_POSITION
    if(usr_abs(ins.pitch) > 60.0f)
        chassis_fdb_data.stand_state = CAHSSIS_IS_DANGER;
#else
    if((usr_abs(ins.pitch) > 60.0f) || (leg[LEFT]->wbr_theta <= -PI/2 - LEG_SAFE_AREA * DEGREE_2_RAD || leg[LEFT]->wbr_theta >= PI/2  + LEG_SAFE_AREA * DEGREE_2_RAD )
                                       || (leg[RIGHT]->wbr_theta <= -PI/2  - LEG_SAFE_AREA * DEGREE_2_RAD || leg[RIGHT]->wbr_theta >= PI/2  + LEG_SAFE_AREA * DEGREE_2_RAD ))
        chassis_fdb_data.stand_state = CAHSSIS_IS_DANGER;
#endif

    chassis_kf_update();
    leg_calc(); // 保证稳定的运算频率，不受模式影响

    chassis_fdb_data.M3508_l = m3508_motor[LEFT]->measure;
    chassis_fdb_data.M3508_r = m3508_motor[RIGHT]->measure;
    /* 更新发布该线程的msg */
    chassis_pub_push();
}

/**
 * @brief 底盘初始化（注册底盘电机及其控制器初始化等）
 */
static int chassis_motor_init(void)
{
    dm_motor_init();
    m3508_motor_init();

    wbr_leg_config_t leg_config =
            {
                    /*单位m*/
                    0.21f,  // l4=l1
                    0.21f,
                    0.250f, // l3=l2
                    0.250f,
                    0.0f,   //电机间距
                    0.0f
                    /* TODO: 改为宏定义 */
            };
    leg[LEFT] = wbr_leg_register(&leg_config);
    leg[RIGHT] = wbr_leg_register(&leg_config);

    /*左侧腿长pid*/
    pid_config_t L_length_pid_config = INIT_PID_CONFIG(l_length_Kp, l_length_Ki, l_length_Kd, l_length_InteVal, l_length_MaxVal,
                                                     (PID_Integral_Limit | PID_DerivativeFilter | PID_OutputFilter));

    L_length_pid = pid_register(&L_length_pid_config);

    /*右侧腿长pid*/
    pid_config_t R_length_pid_config = INIT_PID_CONFIG(r_length_Kp, r_length_Ki, r_length_Kd, r_length_InteVal, r_length_MaxVal,
                                                                (PID_Integral_Limit | PID_DerivativeFilter | PID_OutputFilter));

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
    leg[LEFT]->wbr_calc(leg[LEFT],&ins,chassis_dt);
    // 右腿解算
    leg[RIGHT]->phi_calc_R(leg[RIGHT],dm_motor[2]->measure.angle_abs,dm_motor[1]->measure.angle_abs);
    leg[RIGHT]->wbr_calc(leg[RIGHT],&ins,chassis_dt);

    update_LQR_obs();
    LQR_Cal();

    /* 离地检测，计算两腿地面支持力 */
    Leg_FN_Calculation(0,0.5*(leg[LEFT]->wbr_L_ref + leg[RIGHT]->wbr_L_ref));

    leg[LEFT]->WBR_Tbl =  LQROutBuf[1];
    leg[RIGHT]->WBR_Tbl = LQROutBuf[0];

    leg[LEFT]->wbr_cal_T(leg[LEFT],WBR_T_L);
    leg[RIGHT]->wbr_cal_T(leg[RIGHT],WBR_T_R);
}


static void Leg_FN_Calculation(float ROLL_TARGET,float L_TARGET){

    leg[LEFT]->L_average = 0.5f * (leg[LEFT]->L + leg[RIGHT]->L);
    leg[RIGHT]->L_average = leg[LEFT]->L_average;

    F_roll = pid_calculate(roll_pid, ins.roll * DEGREE_2_RAD, ROLL_TARGET);  //TODO 注意方向,沿正方形顺时针为正

    F_l_L = pid_calculate(L_length_pid, leg[LEFT]->L_average, L_TARGET);

    F_l_R = pid_calculate(L_length_pid, leg[RIGHT]->L_average, L_TARGET);

    F_bl_gravity = 0.5 * m_b * g;
    F_bl_intertial = 0.5 * m_b * (leg[LEFT]->L_average / (2.0f*Rl)) * LQRXObsBuf[0][3] * LQRXObsBuf[0][1];


    leg[LEFT]->wbr_Fbl = F_roll + F_l_L + F_bl_gravity - F_bl_intertial;
    LIMIT_MIN_MAX(leg[LEFT]->wbr_Fbl, -FORCE_LIMIT, FORCE_LIMIT);
    leg[RIGHT]->wbr_Fbl = -F_roll + F_l_R + F_bl_gravity + F_bl_intertial;
    LIMIT_MIN_MAX(leg[RIGHT]->wbr_Fbl, -FORCE_LIMIT, FORCE_LIMIT);
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
        dm_motor[i]->set_mode(dm_motor[i], CMD_ZERO_POSITION);
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
#define DM_OUTPUT_LIMIT  10.0f


/*目前是以护栏较窄的一侧为正方向,从正方向向后看去,rigdm_front:id 2 ; rigdm_back:id 3;left_front:id 1;left_back:id 4*/
/* 1 号电机
 * 输入值为正,逆时针
 * */
static dm_motor_para_t dm_control_1(dm_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static dm_motor_para_t set;

    dm_send_t[0] = WBR_T_L[1] * DM_RATIO;
//    dm_send_t[0] = 0;
    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[0] = dm_send_t[0] ;

//    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_RECOVERY){
//        dm_send_t[0] = 0;
//    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX ){
        dm_send_t[0] = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[0] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
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


    dm_send_t[1] = - WBR_T_R[1] * DM_RATIO;
//    dm_send_t[1] = 0;
    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[1] = dm_send_t[1] ;


//    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_RECOVERY){
//        dm_send_t[1] = 0;
//    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX ){
        dm_send_t[1] = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[1] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
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

    dm_send_t[2] = WBR_T_R[0] * DM_RATIO ;
//    dm_send_t[2] = 0;
    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[2] = dm_send_t[2] ;

//    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_RECOVERY){
//        dm_send_t[2] = 0;
//    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX ){
        dm_send_t[2] = 0;
    }

#ifdef DM8009P_SET_ZERO
        dm_send_t[2] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
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

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位

    dm_send_t[3] = - WBR_T_L[0] * DM_RATIO;
//    dm_send_t[3] = 0;
    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[3] = dm_send_t[3];

//    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_RECOVERY){
//        dm_send_t[3] = 0;
//    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX ){
        dm_send_t[3] = 0;
    }

#ifdef DM8009P_SET_ZERO
    dm_send_t[3] = 0;


#endif

    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
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
static int16_t M3508_control_l(lk_motor_measure_t measure){
    static int16_t set;
    LIMIT_MIN_MAX(LQROutBuf[3],-M3508_TOR_MAX,M3508_TOR_MAX);

    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP){ //平衡时,才启动转向
            if(Wheel_Shut_Flag == 0){
                set = (int16_t)(-LQROutBuf[3] * M3508_TOR_TO_CUR) ;
            }else{
                set = 0.0f;
            }

        }else{
            set = (int16_t)(-LQROutBuf[3] * M3508_TOR_TO_CUR) ;
        }

    }
    set_l = set;

#ifdef M3508_SET_ZERO
    set = 0;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
    }
//    set = -800;
    return set;
}
/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为正
 * */
static int16_t M3508_control_r(lk_motor_measure_t measure){
    static int16_t set;
    LIMIT_MIN_MAX(LQROutBuf[2],-M3508_TOR_MAX,M3508_TOR_MAX);
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP){ //平衡时,才启动转向
            if(Wheel_Shut_Flag == 0){
                set = (int16_t)(LQROutBuf[2] * M3508_TOR_TO_CUR);
            }else{
                set = 0.0f;
            }

        }else{
            set = (int16_t)(LQROutBuf[2] * M3508_TOR_TO_CUR) ;
        }

    }
    set_r = set;

#ifdef M3508_SET_ZERO
    set = 0;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
    }
//    set = 800;
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

    if((leg[LEFT]->L > (leg[LEFT]->wbr_L_ref - LEN_JUDGE_REGION)) && (leg[LEFT]->L < (leg[LEFT]->wbr_L_ref + LEN_JUDGE_REGION))){
        Leg_Left_len_JudgeOK = 1;
    }else{
        Leg_Left_len_JudgeOK = 0;
    }

    if((leg[RIGHT]->L > (leg[RIGHT]->wbr_L_ref - LEN_JUDGE_REGION)) && (leg[RIGHT]->L < (leg[RIGHT]->wbr_L_ref + LEN_JUDGE_REGION))){
        Leg_right_len_JudgeOK = 1;
    }else{
        Leg_right_len_JudgeOK = 0;
    }

    if((ins.roll > (0.0f -ROLL_JUDGE_REGION)) && (ins.roll < (0.0f +ROLL_JUDGE_REGION))){
        Body_roll_JudgeOK = 1;

    }else{
        Body_roll_JudgeOK = 0;
    }
//    if((Leg_Left_len_JudgeOK == 1 || Leg_right_len_JudgeOK == 1) && Body_roll_JudgeOK == 1){
//        chassis_cmd.leg_leng_change = LENGTH_STAY;
//    }
    if(Leg_Left_len_JudgeOK == 1 || Leg_right_len_JudgeOK == 1){
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

    LQRXRefBuf[0][1] = 0; //不稳定状态时应避免速度的影响
    LQRXObsBuf[0][1] = 0;

    yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
}

static void Chassis_Vx_Detect(){


//    Vx_Delta = usr_abs(chassis_kf_l.FilteredValue[0] - chassis_kf_r.FilteredValue[0]);//一边卡住时
//    if(Vx_Delta > VX_DELTA_MAX){
//        yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
//    }
//    else{
//            //更新航向角期望
//            yaw_target += ( - chassis_cmd.vw_set / WX_MAX) * YAW_TURN_RATIO * DEGREE_2_RAD;
////            LIMIT_MIN_MAX(yaw_target,yaw_turn_region_min,yaw_turn_region_max); //限制与目标偏航角的误差,防止失控
//
//        }
//    Vx_Delta = 0;
    yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
    yaw_target += ( - chassis_cmd.vw_set / WX_MAX) * YAW_TURN_RATIO * DEGREE_2_RAD;
            LIMIT_MIN_MAX(yaw_target,yaw_turn_region_min,yaw_turn_region_max); //限制与目标偏航角的误差,防止失控
//    yaw_target = -ins.yaw_total_angle * DEGREE_2_RAD;
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