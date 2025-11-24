#include <stdio.h>
#include "cmsis_os.h"
#include "rm_module.h"
#include "usbd_cdc_if.h"
#include "trans_task.h"
#include "gimbal_task.h"


#define HEART_BEAT 500 //ms

/*------------------------------传输数据相关 --------------------------------- */

// 接收数据回调函数
static uint8_t frame_buffer[sizeof(RpyTypeDef)];
static uint32_t frame_index = 0;
static enum {
    WAIT_FOR_HEADER,
    RECEIVING_DATA
} receive_state = WAIT_FOR_HEADER;

uint8_t buf[31] = {0};
RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
RpyTypeDef rpy_rx_data; //接收解析结构体
static uint32_t heart_dt;
TeamColor  team_color;
/* ---------------------------------usb虚拟串口数据相关 --------------------------------- */

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
// 发布
MCN_DECLARE(transmission_fdb);
static struct trans_fdb_msg trans_fdb_data;

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gimbal_cmd;
MCN_DECLARE(gimbal_fdb);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;

static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);

/*------------------------------自瞄相对角传参反馈--------------------------------------*/
extern auto_relative_angle_status_e auto_relative_angle_status;

uint8_t *r_buffer_point; //用于清除环形缓冲区buffer的指针

/* --------------------------------- 通讯线程入口 --------------------------------- */
static float trans_dt;
static float openfire;
static float trans_start;
static float heart_start;
int8_t a;

void trans_task_init(){
    trans_sub_init();
}

void trans_control(){

    trans_start = dwt_get_time_ms();
/*--------------------------------------------------具体需要发送的数据--------------------------------- */
    if((dwt_get_time_ms()-heart_dt)>=HEART_BEAT)
    {
        // if(flag == 1)
        // {
        //     flag = 0;
        //     trans_fdb_data.pitch = pitch_auto_0;
        //     trans_fdb_data.yaw = yaw_auto_0;
        // }
        // else
        // {
        //     flag = 1;
        //     trans_fdb_data.pitch = pitch_auto_1;
        //     trans_fdb_data.yaw = yaw_auto_1;
        // }
        //
        heart_dt=dwt_get_time_ms();
    }
//        judge_color();
    Send_to_pc(rpy_tx_data);
    vTaskDelay(100);

/*--------------------------------------------------具体需要发送的数据---------------------------------*/
    /* 用于调试监测线程调度使用 */
    trans_dt = dwt_get_time_ms() - trans_start;
    if (trans_dt > 1)
        LOGINFO("Transmission Task is being DELAY! dt = [%f]\r\n", &trans_dt);
    vTaskDelay(1);

}

void trans_control_task(){
    /*订阅数据更新*/
    trans_sub_pull();
    trans_control();
    /* 发布数据更新 */
    trans_pub_push();
}

void Send_to_pc(RpyTypeDef data_r)
{
    /*填充数据*/
    pack_Rpy(&data_r, (gimbal_fdb.yaw_offset_angle - ins.yaw), ins.pitch-gimbal_fdb.pit_offset_angle, openfire,team_color);
    Check_Rpy(&data_r);

    CDC_Transmit_HS((uint8_t*)data_r.DATA,  sizeof(data_r.DATA));

    if (gimbal_cmd.ctrl_mode==GIMBAL_AUTO&&auto_relative_angle_status==RELATIVE_ANGLE_TRANS)
    {
        auto_relative_angle_status=RELATIVE_ANGLE_OK;
    }
}

//void judge_color()
//{
//    referee_data.robot_status.robot_id = 103 ;   //以后在此处进行机器人id的赋值，即确定机器人颜色和种类
//    if(referee_data.robot_status.robot_id < 10)
//        team_color = RED;
//    else
//        team_color = BLUE;
//}


void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch,float openfire, int team_color)   //此处roll值作为开火标志位
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = openfire *1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;
    rpy_data = team_color *1000;
    rpy_tx_buffer[13] = *gimbal_rpy;
    rpy_tx_buffer[14] = *gimbal_rpy >> 8;
    rpy_tx_buffer[15] = *gimbal_rpy >> 16;
    rpy_tx_buffer[16] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer,17);

    frame->LEN = FRAME_RPY_LEN;
}

void Check_Rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}



static void usb_input(uint8_t* Buf, uint32_t *Len)
{
    a++;

    for(uint32_t i = 0; i < *Len; i++) {
        uint8_t current_byte = Buf[i];

        switch(receive_state) {
            case WAIT_FOR_HEADER:
                if(current_byte == 0xFF) {
                    frame_index = 0;
                    frame_buffer[frame_index++] = current_byte;
                    receive_state = RECEIVING_DATA;
                }
                break;

            case RECEIVING_DATA:
                frame_buffer[frame_index++] = current_byte;

                // 检查是否收到完整帧
                if(frame_index >= sizeof(RpyTypeDef))
                {
                    // 处理完整帧
                    memcpy(&rpy_rx_data, frame_buffer, sizeof(rpy_rx_data));

                    switch (rpy_rx_data.ID) {
                        case GIMBAL: {
                            if (rpy_rx_data.DATA[0]) { // 相对角度控制
                                trans_fdb_data.yaw = -(*(int32_t *)&rpy_rx_data.DATA[1] / 1000.0);
                                trans_fdb_data.pitch = (*(int32_t *)&rpy_rx_data.DATA[5] / 1000.0);
                                trans_fdb_data.roll = (*(int32_t *)&rpy_rx_data.DATA[9] / 1000.0);
                            } else { // 绝对角度控制
                                trans_fdb_data.yaw = -(*(int32_t *)&rpy_rx_data.DATA[1] / 1000.0);
                                trans_fdb_data.pitch = (*(int32_t *)&rpy_rx_data.DATA[5] / 1000.0);
                                trans_fdb_data.roll = (*(int32_t *)&rpy_rx_data.DATA[9] / 1000.0);
                            }
                        } break;

                        case HEARTBEAT: {
                            trans_fdb_data.heartbeat = (*(uint8_t *)&rpy_rx_data.DATA[0]);
                            heart_dt = dwt_get_time_ms();
                        } break;
                    }

                    memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
                    memset(frame_buffer, 0, sizeof(frame_buffer));
                    receive_state = WAIT_FOR_HEADER;
                }
                break;
        }
    }
}
// 非静态函数，供其他文件调用
void process_usb_data(uint8_t* Buf, uint32_t *Len)
{
    usb_input(Buf, Len);
}

// 提供获取 trans_fdb_data 数据的函数
struct trans_fdb_msg* get_trans_fdb(void)
{
    return &trans_fdb_data;
}

/******************************************************消息订阅*************************************************************************/
void trans_pub_push(){
    // data_content my_data = ;
    mcn_publish(MCN_HUB(transmission_fdb), &trans_fdb_data);
}

void trans_sub_init(){
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
}

void trans_sub_pull(){

    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chass_cmd);
    }
    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gimbal_cmd);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb), gimbal_fdb_node, &gimbal_fdb);
    }

}