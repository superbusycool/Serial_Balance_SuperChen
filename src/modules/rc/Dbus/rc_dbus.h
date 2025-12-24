#ifndef _RC_DBUS_H
#define _RC_DBUS_H

#include <stdbool.h>
#include "cmsis_os.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
    /**
     * @brief structure that contains the information for the lever/Switch.
     */
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;

    /**
     * @brief structure that contains the information for the mouse.
     */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;


    /**
     * @brief structure that contains the information for the keyboard.
     */
    union
    {
        uint16_t v;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } set;
    } key;

    bool rc_lost;   /*!< lost flag */
    uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);


void BSP_USART_Init();




#endif /* _RC_DBUS_H */
