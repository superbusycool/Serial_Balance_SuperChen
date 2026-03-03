
/*
* Change Logs:
* Date            Author          Notes
* 25-8-22         Gleam            1.0
* 25-8-22         SuperChen        2.0
*/
#include <stdlib.h>
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#define GIM_PITCH_MOTOR_NUM 1
#define GIM_YAW_MOTOR_NUM 1
#define GIM_MOTOR_NUM 2
/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_fdb_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;
MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;
MCN_DECLARE(transmission_fdb);
static McnNode_t trans_fdb_node;
static struct trans_fdb_msg trans_fdb;

// 发布
MCN_DECLARE(gimbal_fdb);
struct gimbal_fdb_msg gimbal_fdb_data;

static void gimbal_pub_push(void);
static void gimbal_sub_init(void);
static void gimbal_sub_pull(void);

static dm_motor_para_t dm_yaw_control(dm_motor_measure_t measure);//dm电机控制
static int16_t GM6020_Control(dji_motor_measure_t measure);//6020电机控制

/* ------------------------------------------------- 电机控制相关 ------------------------------------------------------ */


static float dm_yaw_send_t[GIM_YAW_MOTOR_NUM];
static float dm_yaw_obs[1];
static float yaw_recovery_position;//在倒地自起前gimbal得先转到正前方或正后方,防止腿部摆动时撞到云台
static float yaw_recovery_velocity;
#define DM_GIMBAL_OUTPUT_LIMIT 7.0f
#define DM_YAW_RATIO 1.0f
#define DM_YAW_MIT_KP 0.06f
#define DM_YAW_MIT_KD 3.5f

static float  K_pitch[2] = {2.236068, 1.000596};
static float K_yaw[2] = {2.236068, 0.328709};


static void LQR_CALC();

static float LQROutBuf_Yaw[1]={0};
/* phi phi_dot*/
static float LQRXerrorBuf_Yaw[2][1]={0};
static float LQRXObsBuf_Yaw[2][1]={0};
static float LQRXRefBuf_Yaw[2][1]={0}; /*LQRXObsBuf[0][2] - LQRXRefBuf[0][2]*/
static void yaw_lqr_calc();/*lqr运算*/

static float LQROutBuf_Pitch[1]={0};
/* phi phi_dot*/
static float LQRXerrorBuf_Pitch[2][1]={0};
static float LQRXObsBuf_Pitch[2][1]={0};
static float LQRXRefBuf_Pitch[2][1]={0}; /*LQRXObsBuf[0][2] - LQRXRefBuf[0][2]*/
static void Pitch_lqr_calc();/*lqr运算*/

motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
        {
                .motor_type = DM4310,
                .can_id = CAN_ID_CHASSIS_MOTOR,
                .tx_id = YAW_MOTOR_ID,
                .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
        },

        {
                .motor_type = GM6020,   //英雄pitch轴改用丝杆结构，换用3508电机
                .can_id = CAN_ID_GIMBAL_MOTOR,
                .rx_id = PITCH_MOTOR_ID,   //电机ID待定
        }
};


/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/* gyro三轴：[0]为X，[1]为Y，[2]为Z */
#define X 0
#define Y 1
#define Z 2

static ramp_obj_t *yaw_ramp;//yaw 轴云台控制斜坡
static ramp_obj_t *pit_ramp;//pitch 轴云台控制斜坡

static dji_motor_object_t *gim_motor_pitch[GIM_PITCH_MOTOR_NUM];  // 底盘电机实例
static dm_motor_object_t *gim_motor_yaw[GIM_YAW_MOTOR_NUM];  // 底盘电机实例
static float gim_motor_ref[GIM_MOTOR_NUM]; // 电机控制期望值

static void gimbal_motor_init();

/* ------------------------------------------------ 云台线程入口 ----------------------------------------------------- */
static float gim_dt;

void gimbal_task_init(void){
    gimbal_sub_init();
    gimbal_motor_init();
    yaw_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);
    pit_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);

}

static float gim_start;
static uint32_t init_start_time; // 云台初始化归中开始时间，避免长时间因为静态误差，卡在归中模式
static uint32_t init_dt; // 云台初始化归中进行时长
/* USER CODE END Header_ChassisTask_Entry */
void gimbal_control()
{
    gim_start = dwt_get_time_ms();

    for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
    {
        dji_motor_enable(gim_motor_pitch[i]);
    }
    for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
    {
        dm_motor_enable(gim_motor_yaw[i]);
    }

    switch (gim_cmd.ctrl_mode)
    {
        case GIMBAL_RELAX:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
            {
                dm_motor_relax(gim_motor_yaw[i]);
            }
            gimbal_fdb_data.back_mode = BACK_STEP;
            yaw_ramp->reset(yaw_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);
            pit_ramp->reset(pit_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);

            break;
        case GIMBAL_INIT:
            // TODO：加入斜坡算法，可以控制归中时间
            // TODO: 将编码器值转化为角度值
            // TODO: 优化归中逻辑，yaw轴选取最近的方向
            if(gim_cmd.last_mode != GIMBAL_INIT)
                init_start_time = dwt_get_time_ms();
            else
                init_dt = dwt_get_time_ms() - init_start_time;

            gim_motor_ref[YAW] = CENTER_YAW_ANGLE * ( 1 - yaw_ramp->calc(yaw_ramp));
            gim_motor_ref[PITCH] = CENTER_PITCH_ANGLE * ( 1 - pit_ramp->calc(pit_ramp));/*根据云台的imu信息*/

            //pitch轴改用丝杆结构，只能根据imu数据控制归中
            if(abs(gim_ins.pitch) <= (0.02f)
               && (abs(gim_motor_yaw[0]->measure.yaw_angle - CENTER_YAW_ANGLE) <= 0.05f)
               // 若长时间陷于归中模式，可以适当放宽归中条件
               || ((abs(gim_ins.pitch) <= (0.1f))
                   && (abs(gim_motor_yaw[0]->measure.yaw_angle - CENTER_YAW_ANGLE) <= 0.01f)
                   && (init_dt > INIT_TIMEOUT)))
            {
                gimbal_fdb_data.back_mode = BACK_IS_OK;
                gimbal_fdb_data.pitch_angle = gim_ins.pitch;
                gimbal_fdb_data.yaw_angle = gim_motor_yaw[0]->measure.yaw_angle;
                gimbal_fdb_data.yaw_delta = gim_ins.yaw_total_angle - ins.yaw_total_angle/*底盘yaw_totoal_angle*/;
                gim_motor_ref[YAW] = gim_ins.yaw_total_angle ;/*TODO 注意弧度还是角度*/
            }
            break;
        case GIMBAL_GYRO:

            gim_motor_ref[YAW] += ( - gim_cmd.vw_set / GIMBAL_WX_MAX) * GIMBAL_TURN_RATIO * DEGREE_2_RAD;
            gim_motor_ref[PITCH] = gim_cmd.pitch;
            // 底盘相对于云台归中值的角度，取负
            gimbal_fdb_data.pitch_angle=gim_ins.pitch;

            break;

            // TODO: add auto mode
        case GIMBAL_AUTO:
            /*gim_motor_ref[YAW] = gim_cmd.yaw_auto;*/
            gim_motor_ref[YAW] = trans_fdb.yaw_target;
            gim_motor_ref[PITCH] = trans_fdb.pitch_target;
            // 底盘相对于云台归中值的角度，取负
            break;

        default:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
            {
                dm_motor_relax(gim_motor_yaw[i]);
            }
            break;
    }

    LQR_CALC();/*lqr运算 PITCH&YAW均采用lqr控制,可根据实际控制效果加入前馈或pid进行补偿*/
    /* 用于调试监测线程调度使用 */
    gim_dt = dwt_get_time_ms() - gim_start;
    vTaskDelay(1);
}

void gimbal_control_task(){
    /* 更新该线程所有的订阅者 */
    gimbal_sub_pull();
    gimbal_control();
    /* 更新发布该线程的msg */
    gimbal_pub_push();

}

/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/**
 * @brief 注册云台电机及其控制器初始化
 */
static void gimbal_motor_init()
{
/* ----------------------------------- yaw ---------------------------------- */

    gim_motor_yaw[0] = dm_motor_register(&gimbal_motor_config[YAW], dm_yaw_control);

/* ---------------------------------- pitch --------------------------------- */
    gim_motor_pitch[0] = dji_motor_register(&gimbal_motor_config[PITCH], GM6020_Control);
}


/*******************gimbal的yaw轴用lqr控制****************************************/

/*
 * @brief lqr运算
 * */
#define YAW_ROUND_ANGLE PI2  /*云台连接处的转一圈对应4310的角度总变化,大于2pi*/
static void yaw_lqr_calc(){
    /*目标值*/
    LQRXRefBuf_Yaw[0][0] = gim_motor_ref[YAW];
    LQRXRefBuf_Yaw[1][0] = 0.0f;
    /*观测值*/
    LQRXObsBuf_Yaw[0][0] = gim_ins.yaw_total_angle;/*yaw轴观测角度*/
    LQRXObsBuf_Yaw[1][0] = gim_ins.gyro[Z];/*yaw_dot偏航角速度 TODO 需要检验是否正确*/

    if(gim_cmd.ctrl_mode == GIMBAL_INIT){/*使用mit的位置速度控制*/
        yaw_recovery_position = gim_motor_ref[YAW];
        yaw_recovery_velocity = 0.0f;/*匀速转动*/
    }

    LQRXerrorBuf_Yaw[0][0] = LQRXRefBuf_Yaw[0][0] - LQRXObsBuf_Yaw[0][0];
    LQRXerrorBuf_Yaw[1][0] = LQRXRefBuf_Yaw[1][0] - LQRXObsBuf_Yaw[1][0];/*X_refer - X_obs*/

    LQROutBuf_Yaw[0] = K_yaw[0] * LQRXerrorBuf_Yaw[0][0] + K_yaw[1] * LQRXerrorBuf_Yaw[1][0];
}

/*
 * @brief lqr运算
 * */
static void pitch_lqr_calc(){
    /*目标值*/
    LQRXRefBuf_Pitch[0][0] = gim_motor_ref[PITCH];
    LQRXRefBuf_Pitch[1][0] = 0.0f;
    /*观测值*/
    LQRXObsBuf_Pitch[0][0] = gim_ins.pitch;/*pitch轴观测角度*/
    LQRXObsBuf_Pitch[1][0] = gim_ins.gyro[Y];/*pitch_dot角速度 TODO 需要检验是否正确*/

    LQRXerrorBuf_Pitch[0][0] = LQRXRefBuf_Pitch[0][0] - LQRXObsBuf_Pitch[0][0];
    LQRXerrorBuf_Pitch[1][0] = LQRXRefBuf_Pitch[1][0] - LQRXObsBuf_Pitch[1][0];/*X_refer - X_obs*/

    LQROutBuf_Pitch[0] = K_pitch[0] * LQRXerrorBuf_Pitch[0][0] + K_pitch[1] * LQRXerrorBuf_Pitch[1][0];
}

/*
 * @brief lqr运算
 * */
static void LQR_CALC(){
    yaw_lqr_calc();
    pitch_lqr_calc();
}

/*
 * @brief 云台yaw轴电机
 * */
static dm_motor_para_t dm_yaw_control(dm_motor_measure_t measure)
{
    static dm_motor_para_t set;
    dm_yaw_send_t[0] = LQROutBuf_Yaw[0] * DM_YAW_RATIO;

    LIMIT_MIN_MAX(dm_yaw_send_t[0], -DM_GIMBAL_OUTPUT_LIMIT, DM_GIMBAL_OUTPUT_LIMIT);
    dm_yaw_obs[0] = dm_yaw_send_t[0] ;


    if(gim_cmd.ctrl_mode == GIMBAL_RELAX || gim_cmd.ctrl_mode == GIMBAL_INIT){
        dm_yaw_send_t[0] = 0;
    }
    if(chassis_fdb_cmd.ctrl_mode == CHASSIS_RECOVERY && gim_cmd.ctrl_mode == GIMBAL_INIT ){/*倒地自起中,腿部未在规定起立位置就采用位置控制*/
        dm_yaw_send_t[0] = 0;/*T置零*/

        LIMIT_MIN_MAX(yaw_recovery_position, DM_P_MIN, DM_P_MAX);
        set.p = yaw_recovery_position;

        set.kp = DM_YAW_MIT_KP;/*TODO 尝试参数待定*/
        LIMIT_MIN_MAX(set.kp, DM_KP_MIN, DM_KP_MAX);

        LIMIT_MIN_MAX(yaw_recovery_velocity, DM_V_MIN, DM_V_MAX);
        set.v = yaw_recovery_velocity;

        set.kd = DM_YAW_MIT_KD;
        LIMIT_MIN_MAX(set.kd, DM_KD_MIN, DM_KD_MAX);

    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#if defined(GIMBAL_RELEX) || defined(DM4310_SET_ZERO)
        dm_yaw_send_t[0] = 0;
        set.kp = 0;
        set.kd = 0;
#endif

    LIMIT_MIN_MAX(dm_yaw_send_t[0], -DM_GIMBAL_OUTPUT_LIMIT, DM_GIMBAL_OUTPUT_LIMIT);
    {
        set.t = dm_yaw_send_t[0]; // 正负没问题
    }
    return set;
}

static int16_t GM6020_Control(dji_motor_measure_t measure){
    static int16_t set;
    LIMIT_MIN_MAX(LQROutBuf_Pitch[0],-GM6020_TOR_MAX,GM6020_TOR_MAX);

    set = (int16_t)((LQROutBuf_Pitch[0])* GM6020_TOR_TO_CUR);

#ifdef GIMBAL_RELEX
    set = 0;
#endif
    if(gim_cmd.ctrl_mode == GIMBAL_RELAX){
        set = 0;
    }
    return set;
}

static void leg_init_get_zero()
{
    // 撞到限位后，控制电机在此处设置零点
    for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
    {
        gim_motor_yaw[i]->set_mode(gim_motor_yaw[i], DM_CMD_ZERO_POSITION);
    }

}
/******************************************************消息订阅*************************************************************************/
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void gimbal_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(gimbal_fdb), &gimbal_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void gimbal_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void gimbal_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_fdb_cmd);
    }

    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }

    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }

    if (mcn_poll(trans_fdb_node))
    {
        mcn_copy(MCN_HUB(transmission_fdb), trans_fdb_node, &trans_fdb);
    }
}

