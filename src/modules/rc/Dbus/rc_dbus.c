// Tip: 遥控器接收模块
#include "stm32h7xx_hal.h"
#include "rc_dbus.h"
#include "rm_config.h"
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart5;

/* 数据有效性检查 */
#define VALID_CHANNEL(val) (abs(val) <= RC_MAX_VALUE)


#define SBUS_RX_BUF_NUM 36
#define RC_FRAME_LENGTH 18 /*DT7遥控器一次发送的数据量为18字节*/
static uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];
uint32_t DataLength = 36;

rc_dbus_obj_t rc_dbus_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST

rc_dbus_obj_t dbus_data_fdb;


/*
 * @brief dma双缓冲区配置
 * @param UART_HandleTypeDef *huart：接收哪个串口数据的结构体指针
 * @param uint32_t *DstAddress：第一个缓冲区的地址
 * @param uint32_t *SecondMemAddress ：第二个缓冲区的地址
 * @param uint32_t DataLength：接收数据的长度
 * */
static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{   /*串口接收与UART接收事件类型*/
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    /*设定串口接收数据的长度*/
    huart->RxXferSize    = DataLength;
    /*使能串口DMA模式*/
    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
    /*使能串口的空闲中断*/
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);/*在配置DMA的传输起点和终点的地址前需要先关闭DMA数据传输，以免发生传输意外*/
    /*将DMA 数据流 x 外设地址寄存器 (DMA_SxPAR) 等于USART 接收数据寄存器 (USART_RDR)即可*/
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
    /*将这M0AR寄存器和M1AR寄存器配置成代码中的变量地址即可*/
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
    /*设置DMA数据传输量*/
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
    /*使能DMA*/
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/*和static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)实现同一效果*/
//static void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){
//
//    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
//
//    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
//
//    huart->RxXferSize    = DataLength;
//
//    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
//
//    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
//
//    HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->RDR,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
//}

void dbus_init()
{
    USART_DMAEx_MultiBuffer_Init(&huart5,SBUS_MultiRx_Buf[0], SBUS_MultiRx_Buf[1],36);
    memset(&rc_dbus_obj[NOW], 0, sizeof(rc_dbus_obj_t));
    memset(&rc_dbus_obj[LAST], 0, sizeof(rc_dbus_obj_t));
    memset(&dbus_data_fdb, 0, sizeof(rc_dbus_obj_t));

}
/**
 * @brief 遥控器dbus数据解析
 *
 * @param rc_dbus_obj 指向dbus_rc实例的指针
 */
int dbus_rc_decode(uint8_t *buff)
{
    /* 下面是正常遥控器数据的处理 */
    rc_dbus_obj[NOW].ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc_dbus_obj[NOW].ch1 -= 1024;
    rc_dbus_obj[NOW].ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc_dbus_obj[NOW].ch2 -= 1024;
    rc_dbus_obj[NOW].ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc_dbus_obj[NOW].ch3 -= 1024;
    rc_dbus_obj[NOW].ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc_dbus_obj[NOW].ch4 -= 1024;

    /* 防止遥控器零点有偏差 */
    if(rc_dbus_obj[NOW].ch1 <= 5 && rc_dbus_obj[NOW].ch1 >= -5)
        rc_dbus_obj[NOW].ch1 = 0;
    if(rc_dbus_obj[NOW].ch2 <= 5 && rc_dbus_obj[NOW].ch2 >= -5)
        rc_dbus_obj[NOW].ch2 = 0;
    if(rc_dbus_obj[NOW].ch3 <= 5 && rc_dbus_obj[NOW].ch3 >= -5)
        rc_dbus_obj[NOW].ch3 = 0;
    if(rc_dbus_obj[NOW].ch4 <= 5 && rc_dbus_obj[NOW].ch4 >= -5)
        rc_dbus_obj[NOW].ch4 = 0;

    /* 拨杆值获取 */
    rc_dbus_obj[NOW].sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc_dbus_obj[NOW].sw2 = (buff[5] >> 4) & 0x0003;

    /* 遥控器异常值处理，函数直接返回 */
    if ((abs(rc_dbus_obj[NOW].ch1) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch2) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch3) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch4) > RC_DBUS_MAX_VALUE))
    {
        memset(&rc_dbus_obj[NOW], 0, sizeof(rc_dbus_obj_t));
        return -1;
    }

    /* 鼠标移动速度获取 */
    rc_dbus_obj[NOW].mouse.x = buff[6] | (buff[7] << 8);
    rc_dbus_obj[NOW].mouse.y = buff[8] | (buff[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc_dbus_obj[NOW].mouse.l = buff[12];
    rc_dbus_obj[NOW].mouse.r = buff[13];

    /* 键盘按键键值获取 */
    rc_dbus_obj[NOW].kb.key_code = buff[14] | buff[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    rc_dbus_obj[NOW].wheel = buff[16] | buff[17] << 8;
    rc_dbus_obj[NOW].wheel -= 1024;

    rc_dbus_obj[LAST] = rc_dbus_obj[NOW];
}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

        if(Size == RC_FRAME_LENGTH)
        {
            dbus_rc_decode(SBUS_MultiRx_Buf[0]);
        }

    }else{
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

        if(Size == RC_FRAME_LENGTH)
        {
            dbus_rc_decode(SBUS_MultiRx_Buf[1]);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &huart5){

        USER_USART5_RxHandler(huart,Size);

    }
}
