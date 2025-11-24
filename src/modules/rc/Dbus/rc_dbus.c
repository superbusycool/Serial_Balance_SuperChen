// Tip: 遥控器接收模块
#include "rc_dbus.h"
#include "rm_config.h"
#include <stm32h723xx.h>
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"


/* 数据有效性检查 */
#define VALID_CHANNEL(val) (abs(val) <= RC_MAX_VALUE)

// 声明互斥锁句柄
static SemaphoreHandle_t  sbus_mutex;


rc_dbus_obj_t rc_dbus_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST

rc_dbus_obj_t dbus_data_fdb;

void dbus_data_init()
{
    memset(&rc_dbus_obj[NOW], 0, sizeof(rc_dbus_obj_t));
    memset(&rc_dbus_obj[LAST], 0, sizeof(rc_dbus_obj_t));
    memset(&dbus_data_fdb, 0, sizeof(rc_dbus_obj_t));
    sbus_mutex = xSemaphoreCreateMutex();  // 初始化互斥锁
}
void dbus_data_unpack(uint8_t *data, uint16_t len){
/* 下面是正常遥控器数据的处理 */
    rc_dbus_obj[NOW].ch1 = (data[0] | data[1] << 8) & 0x07FF;
    rc_dbus_obj[NOW].ch1 -= 1024;
    rc_dbus_obj[NOW].ch2 = (data[1] >> 3 | data[2] << 5) & 0x07FF;
    rc_dbus_obj[NOW].ch2 -= 1024;
    rc_dbus_obj[NOW].ch3 = (data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
    rc_dbus_obj[NOW].ch3 -= 1024;
    rc_dbus_obj[NOW].ch4 = (data[4] >> 1 | data[5] << 7) & 0x07FF;
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
    rc_dbus_obj[NOW].sw1 = ((data[5] >> 4) & 0x000C) >> 2;
    rc_dbus_obj[NOW].sw2 = (data[5] >> 4) & 0x0003;


    /* 鼠标移动速度获取 */
    rc_dbus_obj[NOW].mouse.x = data[6] | (data[7] << 8);
    rc_dbus_obj[NOW].mouse.y = data[8] | (data[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc_dbus_obj[NOW].mouse.l = data[12];
    rc_dbus_obj[NOW].mouse.r = data[13];

    /* 键盘按键键值获取 */
    rc_dbus_obj[NOW].kb.key_code = data[14] | data[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    rc_dbus_obj[NOW].wheel = data[16] | data[17] << 8;
    rc_dbus_obj[NOW].wheel -= 1024;

    rc_dbus_obj[LAST] = rc_dbus_obj[NOW];

    /* 数据有效性检查 */
    if (VALID_CHANNEL(rc_dbus_obj[NOW].ch1) &&
        VALID_CHANNEL(rc_dbus_obj[NOW].ch2) &&
        VALID_CHANNEL(rc_dbus_obj[NOW].ch3) &&
        VALID_CHANNEL(rc_dbus_obj[NOW].ch4))
    {
        xSemaphoreTake(sbus_mutex, portMAX_DELAY);
        memcpy(&dbus_data_fdb, &rc_dbus_obj[NOW], sizeof(rc_dbus_obj_t));
        xSemaphoreGive(sbus_mutex);
    }
}