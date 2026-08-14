#include "mymain.h"
#include "RemoteInput.h"
#include "usart.h"

#define DEBUG_UART_TIMEOUT_MS 30U
#define VOFA_CHANNEL_COUNT 17U

typedef struct
{
    float channel[VOFA_CHANNEL_COUNT];
    uint8_t tail[4];
} vofa_justfloat_frame_t;

/**
 * @brief 初始化遥控器输入模块
 * @param None
 * @retval None
 */
void MyMain_Init(void)
{
    Remote_Init();
    Remote_LoRaInit();
}

/**
 * @brief 通过板载 CH340/USART1 输出 VOFA+ JustFloat 曲线数据
 * @param None
 * @retval None
 */
void MyMain_Debug(void)
{
    remote_data_t snapshot;
    vofa_justfloat_frame_t frame;
    uint8_t i;

    Remote_GetSnapshot(&snapshot);
    frame.channel[0] = ((Remote_LoRaReady() != 0U) &&
                        (remote_tx_frames != 0U)) ? 1.0f : 0.0f;
    for (i = 0U; i < REMOTE_KEY_COUNT; i++)
    {
        frame.channel[1U + i] = (float)snapshot.key_state[i];
    }
    frame.channel[13] = (float)remote_axis_adc[2]; /* Left X */
    frame.channel[14] = (float)remote_axis_adc[3]; /* Left Y */
    frame.channel[15] = (float)remote_axis_adc[0]; /* Right X */
    frame.channel[16] = (float)remote_axis_adc[1]; /* Right Y */
    frame.tail[0] = 0x00U;
    frame.tail[1] = 0x00U;
    frame.tail[2] = 0x80U;
    frame.tail[3] = 0x7FU;

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&frame,
                            (uint16_t)sizeof(frame), DEBUG_UART_TIMEOUT_MS);
}
