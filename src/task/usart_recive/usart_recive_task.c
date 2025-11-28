//
// Created by 刘嘉俊 on 25-4-9.
//


#include "rm_module.h"
#include "usart.h"
#include <string.h>


// 福斯遥控器
static volatile uint8_t usart5_rx_buffer_index;  // 当前使用的接收缓冲区
static volatile uint16_t usart5_rx_size;
static uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART5;

// 裁判系统串口 10
static volatile uint8_t referee_rx_buffer_index;  // 当前使用的接收缓冲区
static volatile uint16_t referee_rx_size;
static uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
extern SemaphoreHandle_t xSemaphoreUART10;


void USART5_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收完毕后重启
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART10_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //使能DMA串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // 关闭DMA的传输过半中断，仅保留完成中断
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}



void process_uart5_data(void) {
    uint8_t finishedBuffer;

    finishedBuffer = usart5_rx_buffer_index ^ 1;
    /* SBUS协议解析 */
//        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);
    dbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);

    memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);

}

void process_uart10_data(void) {
    uint8_t finishedBuffer;


    finishedBuffer = referee_rx_buffer_index ^ 1;
    /* 裁判系统数据解析 */
    referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);

    memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if (Size > SBUS_RX_BUF_SIZE)
        {
            return;
        }

        usart5_rx_size = Size;
        usart5_rx_buffer_index = usart5_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE);
        process_uart5_data();  // 福斯遥控器

    }

    if (huart->Instance == USART10)
    {
        // 判断接收的数据大小是否限制，如果超过，则不处理
        if (Size > REFEREE_RX_BUF_SIZE)
        {
            return;
        }

        referee_rx_size = Size;
        referee_rx_buffer_index = referee_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
        process_uart10_data(); // 裁判系统（电管）


    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // 接收发生错误后重启
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // 清除接收缓存
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // 接收发生错误后重启
        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// 清除双缓存
    }

}