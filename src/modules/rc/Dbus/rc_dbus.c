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

/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
#define SBUS_RX_BUF_NUM_L	18u
static uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
        .online_cnt = 0xFAU,
        .rc_lost = true,
};

rc_dbus_obj_t rc_dbus_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

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

/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void){

    USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

}

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxXferSize    = DataLength * 2;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    /* Configure the source memory Buffer address  */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

    /* Configure the destination memory Buffer address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

    /* Configure the length of data to be transferred from source to destination */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    /* Enable double memory buffer */
    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);


}

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    /* reset the online count */
    remote_ctrl->online_cnt = 0xFAU;

    /* reset the lost flag */
    remote_ctrl->rc_lost = false;
}

/**
  * @brief  USER USART5 Reception Event Callback.(SBUS remote_ctrl)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){

        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

        /* Juge whether size is equal to the length of the received data */
        if(Size == SBUS_RX_BUF_NUM)
        {

            /* Memory 0 data update to remote_ctrl*/
            SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);

        }

    }
        /* Current memory buffer used is Memory 1 */
    else{
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

        if(Size == SBUS_RX_BUF_NUM)
        {
            /* Memory 1 to data update to remote_ctrl*/
            SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
        }

    }

}


/**
  * @brief  USER USART10 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  USER USART3 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  USER USART2 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &huart5){

        USER_USART5_RxHandler(huart,Size);

    }

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    /* Enalbe IDLE interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* Enable the DMA transfer for the receiver request */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_dbus_obj_t *dbus_rc_init(void)
{
    BSP_USART_Init();
    // 遥控器离线检测定时器相关
    return rc_dbus_obj;
}