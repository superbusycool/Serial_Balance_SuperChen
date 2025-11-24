#include "motor_task.h"
#include "rm_module.h"
#include "DM_motor.h"

static osMutexId semMotorHandle; // 触发CAN消息发送的信号量

void motor_task_init(void)
{
    osMutexDef(motor_Sem);
    semMotorHandle = osMutexCreate(osMutex(motor_Sem));  // 初始化信号量


}

static float motor_dt;

void motor_control_task(void)
{
    static float motor_start;
    LOGINFO("Motor Task Start\r\n");

    motor_start = dwt_get_time_ms();

#ifdef BSP_USING_DJI_MOTOR
    dji_motor_control();
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_LK_MOTOR
    lk_motor_control();
#endif /* BSP_USING_LK_MOTOR */
#ifdef BSP_USING_HT_MOTOR
    ht_controll_all_poll();
#endif /* BSP_USING_HT_MOTOR */

    /* 用于调试监测线程调度使用 */
    motor_dt = dwt_get_time_ms() - motor_start;
    if (motor_dt > 3){
        LOGERROR("Motor Task is being DELAY! dt = [%f]\n", &motor_dt);
    }
    osDelay(1);
}

static float can_tim_dt, can_tim_start;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    uint8_t data[8]={0};
    if (htim->Instance == htim4.Instance)
    {
        can_tim_dt = dwt_get_time_us() - can_tim_start;
        can_tim_start = dwt_get_time_us();
#ifdef BSP_USING_DM_MOTOR
        dm_controll_all_poll();
#endif /* BSP_USING_DM_MOTOR */

    }
    if (htim->Instance == TIM23)
    {
        HAL_IncTick();
    }

}


