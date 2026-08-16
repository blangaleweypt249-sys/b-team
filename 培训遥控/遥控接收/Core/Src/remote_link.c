#include "remote_link.h"
#include "spi.h"
#include "usart.h"

volatile uint32_t remote_link_uart_errors;
volatile uint32_t remote_link_rx_bytes;
volatile uint32_t remote_link_rx_callbacks;
volatile uint32_t remote_link_rx_arm_errors;
volatile uint32_t remote_link_forward_errors;
volatile uint32_t remote_link_forward_overflows;
volatile uint32_t remote_link_forwarded_bytes;
volatile uint32_t remote_link_control_frames;
volatile uint32_t remote_link_switch_events;
volatile uint32_t remote_link_aux_control_frames;
volatile uint32_t remote_link_aux_crc_errors;
volatile uint32_t remote_link_aux_spi_errors;

static uint8_t remote_link_rx_byte;
static uint8_t remote_link_forward_buffer[128U];
static volatile uint16_t remote_link_forward_head;
static volatile uint16_t remote_link_forward_tail;
static uint8_t remote_link_rx_frame[10U];
static uint8_t remote_link_rx_index;
static uint8_t remote_link_previous_switches;
static uint8_t remote_link_have_switch_state;
static uint8_t remote_link_aux_frame[8U];
static uint8_t remote_link_previous_aux_outputs;

#define REMOTE_LINK_FORWARD_BUFFER_SIZE 128U
#define REMOTE_LINK_FORWARD_CHUNK_SIZE 32U
#define REMOTE_LINK_FRAME_SIZE 10U
#define REMOTE_LINK_HEADER_0 0xA5U
#define REMOTE_LINK_HEADER_1 0x5AU
#define REMOTE_LINK_SECOND_SWITCHES_INDEX 9U
#define REMOTE_LINK_PE4_SWITCH_MASK (1U << 0U)
#define REMOTE_LINK_PE3_SWITCH_MASK (1U << 1U)
#define REMOTE_LINK_PE1_SWITCH_MASK (1U << 2U)
#define REMOTE_LINK_PD5_SWITCH_MASK (1U << 5U)
#define REMOTE_LINK_OUTPUT_SWITCH_MASK \
    (REMOTE_LINK_PE4_SWITCH_MASK | REMOTE_LINK_PE3_SWITCH_MASK | \
     REMOTE_LINK_PE1_SWITCH_MASK | REMOTE_LINK_PD5_SWITCH_MASK)
#define REMOTE_LINK_AUX_FRAME_SIZE 8U
#define REMOTE_LINK_AUX_TARGET_RECEIVER 0x01U
#define REMOTE_LINK_AUX_TYPE_CONTROL 0x02U
#define REMOTE_LINK_AUX_PAYLOAD_SIZE 1U
#define REMOTE_LINK_AUX_OUTPUT_MASK 0x0FU
#define REMOTE_LINK_AUX_ARM_MASK (1U << 0U)
#define REMOTE_LINK_AUX_PUSH_MASK (1U << 1U)
#define REMOTE_LINK_AUX_GRIPPER_MASK (1U << 2U)
#define REMOTE_LINK_AUX_ESTOP_MASK (1U << 3U)

static uint8_t RemoteLink_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x80U) != 0U) ?
                  (uint8_t)((crc << 1U) ^ 0x07U) :
                  (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static void RemoteLink_ToggleSwitchOutputs(uint8_t changed_switches)
{
    if ((changed_switches & REMOTE_LINK_PE4_SWITCH_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_3_PB3_GPIO_Port, U1_3_PB3_Pin);
        remote_link_switch_events++;
    }
    if ((changed_switches & REMOTE_LINK_PE3_SWITCH_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_5_PB5_GPIO_Port, U1_5_PB5_Pin);
        remote_link_switch_events++;
    }
    if ((changed_switches & REMOTE_LINK_PE1_SWITCH_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_7_PB7_GPIO_Port, U1_7_PB7_Pin);
        remote_link_switch_events++;
    }
    if ((changed_switches & REMOTE_LINK_PD5_SWITCH_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(GPIOB, U1_10_PB8_Pin | U1_9_PB9_Pin);
        remote_link_switch_events++;
    }
}

static void RemoteLink_ToggleAuxOutputs(uint8_t changed_outputs)
{
    if ((changed_outputs & REMOTE_LINK_AUX_ARM_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_3_PB3_GPIO_Port, U1_3_PB3_Pin);
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_PUSH_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_5_PB5_GPIO_Port, U1_5_PB5_Pin);
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_GRIPPER_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(U1_7_PB7_GPIO_Port, U1_7_PB7_Pin);
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_ESTOP_MASK) != 0U)
    {
        HAL_GPIO_TogglePin(GPIOB, U1_10_PB8_Pin | U1_9_PB9_Pin);
        remote_link_switch_events++;
    }
}

static void RemoteLink_HandleAuxFrame(const uint8_t *frame)
{
    uint8_t output_bits;
    uint8_t changed_outputs;

    if ((frame[0] != REMOTE_LINK_HEADER_0) ||
        (frame[1] != REMOTE_LINK_HEADER_1) ||
        (frame[2] != REMOTE_LINK_AUX_TARGET_RECEIVER) ||
        (frame[3] != REMOTE_LINK_AUX_TYPE_CONTROL) ||
        (frame[4] != REMOTE_LINK_AUX_PAYLOAD_SIZE) ||
        ((frame[6] & (uint8_t)~REMOTE_LINK_AUX_OUTPUT_MASK) != 0U) ||
        (RemoteLink_Crc8(&frame[2], REMOTE_LINK_AUX_FRAME_SIZE - 3U) !=
         frame[REMOTE_LINK_AUX_FRAME_SIZE - 1U]))
    {
        remote_link_aux_crc_errors++;
        return;
    }

    output_bits = frame[6] & REMOTE_LINK_AUX_OUTPUT_MASK;
    changed_outputs = output_bits ^ remote_link_previous_aux_outputs;
    remote_link_previous_aux_outputs = output_bits;
    RemoteLink_ToggleAuxOutputs(changed_outputs);
    remote_link_aux_control_frames++;
}

static void RemoteLink_HandleFrame(const uint8_t *frame)
{
    uint8_t switch_bits;
    uint8_t changed_switches;

    remote_link_control_frames++;
    switch_bits = (uint8_t)(frame[REMOTE_LINK_SECOND_SWITCHES_INDEX] &
                            REMOTE_LINK_OUTPUT_SWITCH_MASK);
    if (remote_link_have_switch_state == 0U)
    {
        /* The first frame records switch positions; it is not a user toggle. */
        remote_link_previous_switches = switch_bits;
        remote_link_have_switch_state = 1U;
        return;
    }

    changed_switches = (uint8_t)(switch_bits ^ remote_link_previous_switches);
    remote_link_previous_switches = switch_bits;
    RemoteLink_ToggleSwitchOutputs(changed_switches);
}

static void RemoteLink_ParseByte(uint8_t byte)
{
    if (remote_link_rx_index == 0U)
    {
        if (byte == REMOTE_LINK_HEADER_0)
        {
            remote_link_rx_frame[0] = byte;
            remote_link_rx_index = 1U;
        }
        return;
    }

    if (remote_link_rx_index == 1U)
    {
        if (byte == REMOTE_LINK_HEADER_1)
        {
            remote_link_rx_frame[1] = byte;
            remote_link_rx_index = 2U;
        }
        else if (byte != REMOTE_LINK_HEADER_0)
        {
            remote_link_rx_index = 0U;
        }
        return;
    }

    remote_link_rx_frame[remote_link_rx_index++] = byte;
    if (remote_link_rx_index < REMOTE_LINK_FRAME_SIZE)
    {
        return;
    }

    remote_link_rx_index = 0U;
    RemoteLink_HandleFrame(remote_link_rx_frame);
}

static void RemoteLink_QueueByte(uint8_t byte)
{
    uint16_t head = remote_link_forward_head;
    uint16_t next = head + 1U;

    if (next >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
    {
        next = 0U;
    }
    if (next == remote_link_forward_tail)
    {
        remote_link_forward_overflows++;
        return;
    }

    remote_link_forward_buffer[head] = byte;
    remote_link_forward_head = next;
}

static void RemoteLink_ArmReceive(void)
{
    if (huart3.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }
    if (HAL_UART_Receive_IT(&huart3, &remote_link_rx_byte, 1U) != HAL_OK)
    {
        remote_link_rx_arm_errors++;
    }
}

static void RemoteLink_ArmAuxReceive(void)
{
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
    {
        return;
    }
    if (HAL_SPI_Receive_IT(&hspi1,
                          remote_link_aux_frame,
                          REMOTE_LINK_AUX_FRAME_SIZE) != HAL_OK)
    {
        remote_link_aux_spi_errors++;
    }
}

void RemoteLink_Init(void)
{
    remote_link_uart_errors = 0U;
    remote_link_rx_bytes = 0U;
    remote_link_rx_callbacks = 0U;
    remote_link_rx_arm_errors = 0U;
    remote_link_forward_errors = 0U;
    remote_link_forward_overflows = 0U;
    remote_link_forwarded_bytes = 0U;
    remote_link_control_frames = 0U;
    remote_link_switch_events = 0U;
    remote_link_aux_control_frames = 0U;
    remote_link_aux_crc_errors = 0U;
    remote_link_aux_spi_errors = 0U;
    remote_link_forward_head = 0U;
    remote_link_forward_tail = 0U;
    remote_link_rx_index = 0U;
    remote_link_previous_switches = 0U;
    remote_link_have_switch_state = 0U;
    remote_link_previous_aux_outputs = 0U;
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    RemoteLink_ArmReceive();
    RemoteLink_ArmAuxReceive();
}

void RemoteLink_ForwardRawData(void)
{
    uint8_t data[REMOTE_LINK_FORWARD_CHUNK_SIZE];
    uint8_t length = 0U;
    uint16_t head = remote_link_forward_head;
    uint16_t tail = remote_link_forward_tail;

    while ((tail != head) && (length < REMOTE_LINK_FORWARD_CHUNK_SIZE))
    {
        data[length++] = remote_link_forward_buffer[tail];
        tail++;
        if (tail >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
        {
            tail = 0U;
        }
    }
    if (length == 0U)
    {
        return;
    }

    if (HAL_UART_Transmit(&huart2, data, length, 10U) == HAL_OK)
    {
        remote_link_forward_tail = tail;
        remote_link_forwarded_bytes += length;
    }
    else
    {
        remote_link_forward_errors++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_rx_callbacks++;
        remote_link_rx_bytes++;
        RemoteLink_ParseByte(remote_link_rx_byte);
        RemoteLink_QueueByte(remote_link_rx_byte);
        RemoteLink_ArmReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_uart_errors++;
        remote_link_rx_index = 0U;
        RemoteLink_ArmReceive();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        RemoteLink_HandleAuxFrame(remote_link_aux_frame);
        RemoteLink_ArmAuxReceive();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        remote_link_aux_spi_errors++;
        RemoteLink_ArmAuxReceive();
    }
}
