#include "cmd_task.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "robot.h"



/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;
MCN_DECLARE(gimbal_cmd);
static struct gimbal_cmd_msg gimbal_cmd_data;
MCN_DECLARE(shoot_cmd);
static struct shoot_cmd_msg shoot_cmd_data;

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/* --------------------------------------------------- 遥控器相关 ---------------------------------------------------- */
#define RC_UP 1u
#define RC_MI 3u
#define RC_DN 2u
#ifdef BSP_USING_RC_DBUS
static Remote_Info_Typedef *remote_ctrl_now;
static Remote_Info_Typedef remote_ctrl_last;
#else
extern sbus_data_t sbus_data_fdb;
#endif

/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *km_vw_ramp = NULL;//y周控制斜坡

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd(void);
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式

/* -------------------------------- cmd 线程主体 -------------------------------- */
void cmd_task_init(void)
{
    km_vx_ramp = ramp_register(0, 200); //2500000
    km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    km_vw_ramp = ramp_register(0, 200);
    remote_ctrl_now = &remote_ctrl;
    BSP_USART_Init();
    cmd_sub_init();

}

void cmd_control_task(void)
{
    cmd_sub_pull();

    remote_to_cmd();

    cmd_pub_push();
}
static float jump_flag=0;
static float flag=0;

static float jump_count=0;
/**
 * @brief 将遥控器数据转换为控制指令
 */
static void remote_to_cmd(void)
{
    /*检测数据接收是否正常*/
    Remote_Message_Moniter(remote_ctrl_now);
    PC_Handle_kb();
    // TODO: 目前状态机转换较为简单，有很多优化和改进空间
    //遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    chassis_cmd_data.vx = remote_ctrl_last.rc.ch[1] * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
    slope_following(&chassis_cmd_data.vx,&chassis_cmd_data.vx_set,50.0f);
//    chassis_cmd_data.vy = remote_ctrl_last.rc.ch[0] * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED; // roll 控制，暂时将ch1通道换绑为vw
    chassis_cmd_data.vw = remote_ctrl_last.rc.ch[0] * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED;  // TODO: 暂时换绑为vw
    slope_following(&chassis_cmd_data.vw,&chassis_cmd_data.vw_set,1.0f);
   // TODO: 轮腿前期调试
   // chassis_cmd_data.offset_angle = gim_fdb.yaw_relative_angle;
   // gimbal_cmd_data.yaw += remote_ctrl_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;
   // gimbal_cmd_data.pitch += remote_ctrl_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;
   // /* 限制云台角度 */
   // VAL_LIMIT(gimbal_cmd_data.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

   // 右拨杆s[0]]为上时，底盘和云台均REALX；为中时，云台为GYRO，地盘为OPEN；为下时，云台为AUTO。
   // 左拨杆s[1]为上时，腿长为LOW；为中时，腿长为MID；为下时，腿长为HIG。（当前暂不考虑遥控器对发射机构的控制）
   switch (remote_ctrl_last.rc.s[0])
   {
       case RC_UP:
           chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
           break;

       case RC_MI:

           if((chassis_fdb.stand_state != CHASSIS_IS_DANGER))
           {
                if(chassis_cmd_data.last_mode == CHASSIS_INIT || chassis_cmd_data.last_mode == CHASSIS_RELAX)
                {
                    chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;//完成倒地自起
                }
                else if(chassis_cmd_data.last_mode == CHASSIS_RECOVERY || chassis_cmd_data.last_mode == CHASSIS_AUTO)
                {
                    if(chassis_fdb.stand_state == CHASSIS_IS_STAND)
                    {
                        chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;

                    }
                    else if(chassis_fdb.stand_state == CHASSIS_IS_DANGER)
                    {
                        chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
                    }
                    else{
                        chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
                    }
                }

           }
           else{
               chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
           }

           break;

       case RC_DN:

           if(chassis_cmd_data.last_mode == CHASSIS_OPEN_LOOP){
               chassis_cmd_data.ctrl_mode = CHASSIS_AUTO;
           }

            break;
    }

    switch (remote_ctrl_last.rc.s[1])
    {
        case RC_UP:
            chassis_cmd_data.leg_level = LEG_LOW;
            break;
        case RC_MI:
            chassis_cmd_data.leg_level = LEG_MID;

            break;
        case RC_DN:
            chassis_cmd_data.leg_level = LEG_HIG;
            break;
        default:
            chassis_cmd_data.leg_level = LEG_LOW;
            break;
    }

//    if(chassis_cmd_data.last_mode==CHASSIS_JUMP){
//        flag=1;
//    }

   /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
   /*switch(remote_ctrl_now->sw3)
   {
   case RC_UP:
       gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
       chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
       break;
   case RC_MI:
       if(gimbal_cmd_data.last_mode == GIMBAL_RELAX)
       {*//* 判断上次状态是否为RELAX，是则先归中 *//*
           gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
       }
       else
       {
           if(gim_fdb.back_mode == BACK_IS_OK)
           {
               gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;
           }
       }
       break;
   case RC_DN:
       if(gimbal_cmd_data.last_mode == GIMBAL_RELAX)
       {*//* 判断上次状态是否为RELAX，是则先归中 *//*
           gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
       }
       else
       {
           if(gim_fdb.back_mode == BACK_IS_OK)
           {*//* 判断归中是否完成 *//*
               gimbal_cmd_data.ctrl_mode = GIMBAL_AUTO;
           }
       }
       break;
   }*/
    /* 保存上一次数据 */
    gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;
    chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    shoot_cmd_data.last_mode=shoot_cmd_data.ctrl_mode;

    /* 保存上一次数据 */
    // gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;
    if(remote_ctrl_last.rc.s[0] != remote_ctrl_last.rc.s[0]){
        chassis_cmd_data.leg_leng_change = LENGTH_CHANGE;
    }

    chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    remote_ctrl_last = *remote_ctrl_now;

}

/* -------------------------------------------- 将遥控器数据转换为控制指令 ----------------------------------------------- */
#ifdef BSP_USING_RC_DBUS_KEYBOARD
static void remote_to_cmd_pc_DT7(void)
{
    /* 保存上一次数据 */
    gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;
    chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    shoot_cmd_data.last_mode=shoot_cmd_data.ctrl_mode;
    *remote_ctrl_now = dbus_data_fdb;  // 复制到临时变量

    gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;

    //目前此版本代码这里有问题，可能是因为键鼠数据缺失导致的野指针，暂时注释掉了用他的地方
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,remote_ctrl_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,remote_ctrl_now->mouse.y);

    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,remote_ctrl_now->mouse.y)*0.05f;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd_data.vx =  (float)remote_ctrl_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd_data.vy =  (float)remote_ctrl_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    //chassis_cmd_data.vw =  (float)remote_ctrl_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + remote_ctrl_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;

    chassis_cmd_data.offset_angle = gim_fdb.yaw_relative_angle;

    /*云台命令*/
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_GYRO)
    {
        gimbal_cmd_data.yaw +=   (float)remote_ctrl_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW ;//+ fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gimbal_cmd_data.pitch += (float)remote_ctrl_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;//- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gimbal_cmd_data.yaw;
        gyro_pitch_inherit =gimbal_cmd_data.pitch;
        mouse_accumulate_x=0;
        mouse_accumulate_y=0;
    }
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO)
    {
        if (auto_relative_angle_status==RELATIVE_ANGLE_TRANS)
        {
            trans_fdb.yaw=0;
            trans_fdb.pitch=0;
        }
        //mouse_accumulate_x+=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW; /*鼠标x轴的自瞄补偿，测试结果建议注释掉暂不使用*/
        //mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT; //建议打开
        if(trans_fdb.roll != 2)
        {
            gimbal_cmd_data.yaw = trans_fdb.yaw+gyro_yaw_inherit + mouse_accumulate_x/* + 150 * remote_ctrl_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW*/;//上位机自瞄
            gimbal_cmd_data.pitch = trans_fdb.pitch+gyro_pitch_inherit + mouse_accumulate_y/* +100 * remote_ctrl_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT */;//上位机自瞄
        }

    }

    /*!限制云台pitch轴角度 */
    VAL_LIMIT(gimbal_cmd_data.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_INIT||gimbal_cmd_data.ctrl_mode==GIMBAL_RELAX)
    {
        gimbal_cmd_data.pitch=0;
        gimbal_cmd_data.yaw=0;
    }
    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/


    /*TODO:手动模式和自瞄模式状态机*/
    /*!如果鼠标未按下右键*/
    if (remote_ctrl_now->mouse.r==0)
    {
        if (gimbal_cmd_data.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
            chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;

        }
        else
        {
            if (gim_fdb.back_mode == BACK_IS_OK)  /*!是否归中完成*/
            {
                gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                memset(r_buffer_point,0,sizeof (*r_buffer_point));
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;
                gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    if (remote_ctrl_now->mouse.r==1||remote_ctrl_now->sw2==RC_DN) /*!如果鼠标按下右键或者遥控器选择自瞄模式*/
    {
        if (gimbal_cmd_data.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
            chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == 1)
//            if (gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gimbal_cmd_data.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;
                gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    /* -------------初始化ui按键B-----------------*/
//    if(km.b_sta == KEY_PRESS_DOWN){
//
//        ui_cmd.ui_init = 1;//B键被按下满足ui初始化条件
//
//    }else{
//
//        ui_cmd.ui_init = 0;
//
//    }


    /*TODO:小陀螺*/
    //开小陀螺
    if(km.e_sta==KEY_PRESS_ONCE)
    {
        key_e_status=1;
    }
    if ( key_e_status==1||remote_ctrl_now->sw1==RC_DN)
    {
        if (gim_fdb.back_mode==BACK_IS_OK)
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_SPIN;
        }
        else
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;
            gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;
        }
    }
    //关小陀螺
    if(remote_ctrl_now->kb.bit.Q == 1 )
    {
        key_e_status=0;
    }
    if ( key_e_status==0)
    {
        chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }

    if (chassis_cmd_data.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd_data.vw=3;// * msg_cmd->robot_status.chassis_power_limit/55;/*!小陀螺转速，随着功率限制提升加快转速*/
        if(chassis_fdb.vw_ch < chassis_cmd_data.vw*0.85f) //当小陀螺被堵住时，自动退出小陀螺模式
        {
            spin_cnt++;
            if(spin_cnt>2000)
            {
                chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                spin_cnt=0;
            }
        }
        else
        {
            spin_cnt =0;
        }

    }

    /*TODO:--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*!-----------------------------------------开关摩擦轮--------------------------------------------*/
    /*-----------------------------------------开关摩擦轮--------------------------------------------*/
    if(km.f_sta==KEY_PRESS_ONCE)
    {
        key_f_status=1;
    }
    if(remote_ctrl_now->sw1==RC_MI)
    {
        rc_f_status = 1;
    }
    if ( key_f_status==1||rc_f_status == 1)
    {
        shoot_cmd_data.friction_status=1;
    }
    else
    {
        shoot_cmd_data.friction_status = 0;
    }

    //关摩擦轮
    if(remote_ctrl_now->kb.bit.G == 1)
    {
        key_f_status=0;
    }
    if(remote_ctrl_now->sw1 != RC_MI)
    {
        if(rc_f_status == 1)
        {
            rc_f_status = 0;
        }
    }




    /*!------------------------------------------------------------扳机连发模式---------------------------------------------------------*/
    //自瞄模式下，开启自动扳机
    // if(gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO)
    // {    //单发模式
    //     if((remote_ctrl_now->mouse.l==1||remote_ctrl_now->wheel >= 300)&&gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO
    //          && shoot_cmd_data.friction_status==1)
    //     {
    //         if(trans_fdb.roll == 1)
    //         {
    //             shoot_cmd_data.ctrl_mode = SHOOT_ONE;
    //             shoot_cmd_data.trigger_status = TRIGGER_ON;
    //         }
    //         else
    //         {
    //             if(shoot_fdb.trigger_status == SHOOT_OK)
    //             {
    //                 shoot_cmd_data.ctrl_mode = SHOOT_ONE;
    //                 shoot_cmd_data.trigger_status = TRIGGER_OFF;
    //             }
    //
    //         }
    //     }//连发模式
    //     else if((remote_ctrl_now->mouse.l==1||remote_ctrl_now->wheel <= -300)&&gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO
    //          && shoot_cmd_data.friction_status==1 )
    //     {
    //         if(trans_fdb.roll == 1)
    //         {
    //             shoot_cmd_data.ctrl_mode = SHOOT_COUNTINUE;
    //             shoot_cmd_data.shoot_freq = 2000;
    //         }
    //         else
    //         {
    //             shoot_cmd_data.ctrl_mode = SHOOT_STOP;
    //             shoot_cmd_data.shoot_freq = 0;
    //         }
    //     }
    // }
    // else

    //连发模式
    if((km.v_sta != KEY_RELEASE && km.v_sta != KEY_WAIT_EFFECTIVE)
       ||(remote_ctrl_now->wheel <= -300)
        /*&&(referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat < (referee_fdb.robot_status.shooter_barrel_heat_limit-10))*/)
    {
        if(shoot_cmd_data.friction_status==1)
        {
            shoot_cmd_data.ctrl_mode=SHOOT_COUNTINUE;
            shoot_cmd_data.shoot_freq=2000;
        }
    }
        //开启摩擦轮默认进入单发模式,首先判断鼠标左键键是否按下或者拨轮是否向下，标记开火标志位
    else if(km.lk_sta == KEY_PRESS_ONCE || remote_ctrl_now->wheel>=400 &&(shoot_cmd_data.friction_status==1))
    {
        trigger_flag=1;
        shoot_cmd_data.ctrl_mode=SHOOT_ONE;
        shoot_cmd_data.trigger_status=TRIGGER_OFF;
    }
        //当V键松开时或拨轮恢复到0时，根据标志位判断状态
    else if(km.lk_sta == KEY_RELEASE || remote_ctrl_now->wheel ==0 && (shoot_cmd_data.friction_status==1))
    {
        //当shoot线程反馈信息显示开火完成后，清零开火标志位（记得在shoot线程shoot_stop状态将反馈信息设置为SHOOT_WAITNG）
        if(shoot_fdb.trigger_status == SHOOT_OK)
        {
            trigger_flag=0;
        }
        //如果开火标志位等于0，不开火
        if(trigger_flag ==0)
        {
            shoot_cmd_data.ctrl_mode=SHOOT_ONE;
            shoot_cmd_data.trigger_status=TRIGGER_OFF;
        }
            //如果开火标志位等于1，表示进入单发模式，开火
        else if(trigger_flag == 1)
        {
            shoot_cmd_data.ctrl_mode=SHOOT_ONE;
            shoot_cmd_data.trigger_status=TRIGGER_ON;
        }
    }
    else
    {
        shoot_cmd_data.ctrl_mode=SHOOT_STOP;
        shoot_cmd_data.shoot_freq=0;
    }


    /*-------------------------------------------------------------堵弹反转检测------------------------------------------------------------*/
    if (shoot_fdb.trigger_motor_current>=16300)/*M3508电机的堵转电流是2500*/
    {
        reverse_cnt++;
        if (reverse_cnt<300)
            reverse_cnt++;
        else
        {
            reverse_cnt=0;
            shoot_cmd_data.ctrl_mode=SHOOT_REVERSE;
        }

    }

    // /*-----------------------------------------------------------舵机开盖关盖--------------------------------------------------------------*/
    // if(remote_ctrl_now->kb.bit.R==1||remote_ctrl_now->wheel<=-200)
    // {
    //     shoot_cmd_data.cover_open=1;
    // }
    // else
    // {
    //     shoot_cmd_data.cover_open=0;
    // }
    /*-----------------------------------------------------------倍镜舵机控制--------------------------------------------------------------*/
    //在未开启摩擦轮时，向上拨轮选择是否旋转倍镜
    if(remote_ctrl_now->kb.bit.R==1||remote_ctrl_now->wheel <= -600 && shoot_cmd_data.friction_status==0)
    {
        mirror_servo_flag = 1;//旋转倍镜标志位
    }
        //当拨轮恢复到0时，根据旋转倍镜标志位判断是否旋转倍镜
    else if(remote_ctrl_now->wheel == 0 && mirror_servo_flag==1)
    {
        mirror_servo_flag = 0;
        if(shoot_cmd_data.mirror_enable==1)
            shoot_cmd_data.mirror_enable=0;
        else
            shoot_cmd_data.mirror_enable=1;
    }

    /*--------------------------------------------------手动模式下清空自瞄传过来的角度buffer------------------------------------------------------*/
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_GYRO)
    {
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }
    /*----------------------------------------------------------------使能判断---------------------------------------------------------------*/

    //关闭云台接口，便于调试
    if(remote_ctrl_now->wheel >= 600 && remote_ctrl_now->sw2==RC_MI)
    {
        cnt_flag++;
        if(cnt_flag >=800)
        {
            cnt_flag =0;
            if(deploy_flag == 0)
                deploy_flag = 1;
            else if(deploy_flag == 1)
                deploy_flag =0;
        }
    }
    else
    {
        cnt_flag =0;
    }
    if(deploy_flag == 1)
    {
        chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
    }


    //TODO:使能判断放最后，防止抽风
    if (remote_ctrl_now->sw2==RC_UP)
    {
        gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd_data.ctrl_mode=SHOOT_STOP;
        shoot_cmd_data.friction_status = 0; //在失能状态下，摩擦轮也不能开启，从而也进一步保证拨弹盘不能被开启
        /*放开状态下，gim不接收值*/
        gimbal_cmd_data.pitch=0;
        gimbal_cmd_data.yaw=0;
        gyro_yaw_inherit=0;
        gyro_pitch_inherit=0;
        /*案件状态标志位重置*/
        key_e_status=-1;
        key_f_status=-1;
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }

}
#endif

/*********************************************subcription and publication***************************************************************************/
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_cmd_data);
    mcn_publish(MCN_HUB(shoot_cmd), &shoot_cmd_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node))
    {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_fdb);
    }
}

