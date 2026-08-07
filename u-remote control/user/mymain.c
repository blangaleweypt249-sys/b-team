#include "mymain.h"
#include "Remote.h"
#include "usart.h"

#define DEBUG_UART_TIMEOUT_MS  100U  // 串口调试发送超时
#define VOFA_CHANNEL_NUM        6U    // JustFloat通道数量

typedef struct
{
    float channel[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} vofa_frame_t;

/**
 * @brief 初始化用户应用
 * @param None
 * @retval None
 */
void MyMain_Init(void)
{
    Remote_Init();
}

/**
 * @brief 通过USART1输出VOFA+ JustFloat数据
 * @param None
 * @retval None
 */
void MyMain_Debug(void)
{
    vofa_frame_t vofa_frame;

    vofa_frame.channel[0] = (float)remote_data.left_shoulder;
    vofa_frame.channel[1] = (float)remote_data.right_shoulder;
    vofa_frame.channel[2] = (float)remote_data.left_x;
    vofa_frame.channel[3] = (float)remote_data.left_y;
    vofa_frame.channel[4] = (float)remote_data.right_x;
    vofa_frame.channel[5] = (float)remote_data.right_y;
    vofa_frame.tail[0] = 0x00;
    vofa_frame.tail[1] = 0x00;
    vofa_frame.tail[2] = 0x80;
    vofa_frame.tail[3] = 0x7F;

    HAL_UART_Transmit(&huart1, (uint8_t *)&vofa_frame, sizeof(vofa_frame),
                      DEBUG_UART_TIMEOUT_MS);
}
