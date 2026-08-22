#include "remote_link.h"
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
volatile uint32_t remote_link_aux_uart_errors;

static uint8_t remote_link_forward_buffer[512U];
static volatile uint16_t remote_link_forward_head;
static volatile uint16_t remote_link_forward_tail;
static uint8_t remote_link_tx_buffer[64U];
static volatile uint8_t remote_link_tx_busy;
static volatile uint8_t remote_link_tx_length;
static volatile uint16_t remote_link_tx_tail;
static uint8_t remote_link_rx_dma_buffer[256U];
static uint16_t remote_link_rx_dma_position;
static uint8_t remote_link_rx_frame[10U];
static uint8_t remote_link_rx_index;
static uint8_t remote_link_previous_switches;
static uint8_t remote_link_have_switch_state;
static uint8_t remote_link_aux_rx_byte;
static uint8_t remote_link_previous_aux_outputs;

#define REMOTE_LINK_FORWARD_BUFFER_SIZE 512U
#define REMOTE_LINK_FORWARD_CHUNK_SIZE 64U
#define REMOTE_LINK_RX_DMA_BUFFER_SIZE 256U
#define REMOTE_LINK_FRAME_SIZE 10U
#define REMOTE_LINK_HEADER_0 0xA5U
#define REMOTE_LINK_HEADER_1 0x5AU
#define REMOTE_LINK_SECOND_SWITCHES_INDEX 9U
#define REMOTE_LINK_PE4_SWITCH_MASK (1U << 0U)
#define REMOTE_LINK_PE3_SWITCH_MASK (1U << 1U)
#define REMOTE_LINK_PE1_SWITCH_MASK (1U << 2U)
#define REMOTE_LINK_PD5_SWITCH_MASK (1U << 5U)
#define REMOTE_LINK_DIRECT_SWITCH_MASK \
    (REMOTE_LINK_PE4_SWITCH_MASK | REMOTE_LINK_PE3_SWITCH_MASK | \
     REMOTE_LINK_PE1_SWITCH_MASK)
#define REMOTE_LINK_OUTPUT_SWITCH_MASK \
    (REMOTE_LINK_DIRECT_SWITCH_MASK | REMOTE_LINK_PD5_SWITCH_MASK)
#define REMOTE_LINK_AUX_FRAME_SIZE 8U
#define REMOTE_LINK_AUX_TARGET_RECEIVER 0x01U
#define REMOTE_LINK_AUX_TYPE_CONTROL 0x02U
#define REMOTE_LINK_AUX_PAYLOAD_SIZE 1U
#define REMOTE_LINK_AUX_OUTPUT_MASK 0x0FU
#define REMOTE_LINK_AUX_UPDATE_SHIFT 4U
#define REMOTE_LINK_AUX_STREAM_BUFFER_SIZE 32U
#define REMOTE_LINK_AUX_PE4_MASK (1U << 0U)
#define REMOTE_LINK_AUX_PUSH_MASK (1U << 1U)
#define REMOTE_LINK_AUX_GRIPPER_MASK (1U << 2U)
#define REMOTE_LINK_AUX_ESTOP_MASK (1U << 3U)

static uint8_t remote_link_aux_stream[REMOTE_LINK_AUX_STREAM_BUFFER_SIZE];
static uint8_t remote_link_aux_stream_size;

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
static void RemoteLink_UpdateManualSwitchOutputs(uint8_t switch_bits,
                                                 uint8_t changed_switches)
{
    if ((changed_switches & REMOTE_LINK_PE4_SWITCH_MASK) != 0U)
    {
        /* UART3 manual PE4: 1->0 opens (high), 0->1 closes (low). */
        HAL_GPIO_WritePin(U1_3_PB3_GPIO_Port, U1_3_PB3_Pin,
                          ((switch_bits & REMOTE_LINK_PE4_SWITCH_MASK) == 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
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

static void RemoteLink_ApplyAuxOutputs(uint8_t output_bits,
                                       uint8_t update_mask,
                                       uint8_t changed_outputs)
{
    if ((update_mask & REMOTE_LINK_AUX_PE4_MASK) != 0U)
    {
        HAL_GPIO_WritePin(U1_3_PB3_GPIO_Port, U1_3_PB3_Pin,
                          ((output_bits & REMOTE_LINK_AUX_PE4_MASK) == 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if ((update_mask & REMOTE_LINK_AUX_PUSH_MASK) != 0U)
    {
        HAL_GPIO_WritePin(U1_5_PB5_GPIO_Port, U1_5_PB5_Pin,
                          ((output_bits & REMOTE_LINK_AUX_PUSH_MASK) != 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if ((update_mask & REMOTE_LINK_AUX_GRIPPER_MASK) != 0U)
    {
        HAL_GPIO_WritePin(U1_7_PB7_GPIO_Port, U1_7_PB7_Pin,
                          ((output_bits & REMOTE_LINK_AUX_GRIPPER_MASK) != 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if ((update_mask & REMOTE_LINK_AUX_ESTOP_MASK) != 0U)
    {
        HAL_GPIO_WritePin(GPIOB, U1_10_PB8_Pin | U1_9_PB9_Pin,
                          ((output_bits & REMOTE_LINK_AUX_ESTOP_MASK) != 0U) ?
                          GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    /* Keep the existing counter as a command-state change diagnostic only. */
    if ((changed_outputs & REMOTE_LINK_AUX_PE4_MASK) != 0U)
    {
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_PUSH_MASK) != 0U)
    {
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_GRIPPER_MASK) != 0U)
    {
        remote_link_switch_events++;
    }
    if ((changed_outputs & REMOTE_LINK_AUX_ESTOP_MASK) != 0U)
    {
        remote_link_switch_events++;
    }
}

static uint8_t RemoteLink_AuxFrameIsValid(const uint8_t *frame)
{
    if ((frame[0] != REMOTE_LINK_HEADER_0) ||
        (frame[1] != REMOTE_LINK_HEADER_1) ||
        (frame[2] != REMOTE_LINK_AUX_TARGET_RECEIVER) ||
        (frame[3] != REMOTE_LINK_AUX_TYPE_CONTROL) ||
        (frame[4] != REMOTE_LINK_AUX_PAYLOAD_SIZE) ||
        (RemoteLink_Crc8(&frame[2], REMOTE_LINK_AUX_FRAME_SIZE - 3U) !=
         frame[REMOTE_LINK_AUX_FRAME_SIZE - 1U]))
    {
        return 0U;
    }
    return 1U;
}

static void RemoteLink_HandleAuxFrame(const uint8_t *frame)
{
    uint8_t output_bits = frame[6] & REMOTE_LINK_AUX_OUTPUT_MASK;
    uint8_t update_mask =
        (frame[6] >> REMOTE_LINK_AUX_UPDATE_SHIFT) &
        REMOTE_LINK_AUX_OUTPUT_MASK;
    uint8_t changed_outputs =
        (output_bits ^ remote_link_previous_aux_outputs) &
        REMOTE_LINK_AUX_OUTPUT_MASK;

    if (update_mask == 0U)
    {
        /* Legacy frame behavior: PB3 was explicit, other outputs edge-based. */
        update_mask = changed_outputs | REMOTE_LINK_AUX_PE4_MASK;
    }
    changed_outputs &= update_mask;
    remote_link_previous_aux_outputs =
        (remote_link_previous_aux_outputs & (uint8_t)~update_mask) |
        (output_bits & update_mask);
    RemoteLink_ApplyAuxOutputs(output_bits, update_mask, changed_outputs);
    remote_link_aux_control_frames++;
}

static void RemoteLink_AuxStreamDiscard(uint8_t count)
{
    uint8_t index;

    if (count >= remote_link_aux_stream_size)
    {
        remote_link_aux_stream_size = 0U;
        return;
    }

    remote_link_aux_stream_size =
        (uint8_t)(remote_link_aux_stream_size - count);
    for (index = 0U; index < remote_link_aux_stream_size; index++)
    {
        remote_link_aux_stream[index] = remote_link_aux_stream[index + count];
    }
}

static void RemoteLink_ProcessAuxStream(void)
{
    uint8_t header_index;

    for (;;)
    {
        if (remote_link_aux_stream_size < 2U)
        {
            return;
        }

        header_index = 0U;
        while (((uint16_t)header_index + 1U) < remote_link_aux_stream_size)
        {
            if ((remote_link_aux_stream[header_index] == REMOTE_LINK_HEADER_0) &&
                (remote_link_aux_stream[header_index + 1U] ==
                 REMOTE_LINK_HEADER_1))
            {
                break;
            }
            header_index++;
        }

        if (((uint16_t)header_index + 1U) >= remote_link_aux_stream_size)
        {
            if (remote_link_aux_stream[remote_link_aux_stream_size - 1U] ==
                REMOTE_LINK_HEADER_0)
            {
                remote_link_aux_stream[0] = REMOTE_LINK_HEADER_0;
                remote_link_aux_stream_size = 1U;
            }
            else
            {
                remote_link_aux_stream_size = 0U;
            }
            return;
        }

        if (header_index > 0U)
        {
            RemoteLink_AuxStreamDiscard(header_index);
        }
        if (remote_link_aux_stream_size < REMOTE_LINK_AUX_FRAME_SIZE)
        {
            return;
        }

        if (RemoteLink_AuxFrameIsValid(remote_link_aux_stream) != 0U)
        {
            RemoteLink_HandleAuxFrame(remote_link_aux_stream);
            RemoteLink_AuxStreamDiscard(REMOTE_LINK_AUX_FRAME_SIZE);
        }
        else
        {
            remote_link_aux_crc_errors++;
            RemoteLink_AuxStreamDiscard(1U);
        }
    }
}

static void RemoteLink_QueueAuxByte(uint8_t byte)
{
    if (remote_link_aux_stream_size >= REMOTE_LINK_AUX_STREAM_BUFFER_SIZE)
    {
        RemoteLink_AuxStreamDiscard(1U);
    }
    remote_link_aux_stream[remote_link_aux_stream_size++] = byte;
    RemoteLink_ProcessAuxStream();
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
        /* UART3 first frame only establishes the manual switch baseline. */
        remote_link_previous_switches = switch_bits;
        remote_link_have_switch_state = 1U;
        return;
    }

    changed_switches = (uint8_t)(switch_bits ^ remote_link_previous_switches);
    remote_link_previous_switches = switch_bits;
    RemoteLink_UpdateManualSwitchOutputs(switch_bits, changed_switches);
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
    __DMB();
    remote_link_forward_head = next;
}

static void RemoteLink_ProcessReceivedByte(uint8_t byte)
{
    remote_link_rx_bytes++;
    RemoteLink_ParseByte(byte);
    RemoteLink_QueueByte(byte);
}

static void RemoteLink_ProcessDmaRange(uint16_t start, uint16_t end)
{
    __DMB();
    while (start < end)
    {
        RemoteLink_ProcessReceivedByte(remote_link_rx_dma_buffer[start]);
        start++;
    }
}

static void RemoteLink_ProcessDmaData(uint16_t position)
{
    if ((position == 0U) || (position > REMOTE_LINK_RX_DMA_BUFFER_SIZE))
    {
        return;
    }

    if (position == REMOTE_LINK_RX_DMA_BUFFER_SIZE)
    {
        RemoteLink_ProcessDmaRange(remote_link_rx_dma_position,
                                   REMOTE_LINK_RX_DMA_BUFFER_SIZE);
        remote_link_rx_dma_position = 0U;
        return;
    }

    if (position < remote_link_rx_dma_position)
    {
        RemoteLink_ProcessDmaRange(remote_link_rx_dma_position,
                                   REMOTE_LINK_RX_DMA_BUFFER_SIZE);
        remote_link_rx_dma_position = 0U;
    }

    RemoteLink_ProcessDmaRange(remote_link_rx_dma_position, position);
    remote_link_rx_dma_position = position;
}

static void RemoteLink_ArmReceive(void)
{
    if (huart3.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }

    remote_link_rx_dma_position = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                                     remote_link_rx_dma_buffer,
                                     REMOTE_LINK_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        remote_link_rx_arm_errors++;
    }
}

static void RemoteLink_ArmAuxReceive(void)
{
    if (huart2.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }
    if (HAL_UART_Receive_IT(&huart2, &remote_link_aux_rx_byte, 1U) != HAL_OK)
    {
        remote_link_aux_uart_errors++;
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
    remote_link_aux_uart_errors = 0U;
    remote_link_forward_head = 0U;
    remote_link_forward_tail = 0U;
    remote_link_tx_busy = 0U;
    remote_link_tx_length = 0U;
    remote_link_tx_tail = 0U;
    remote_link_rx_dma_position = 0U;
    remote_link_rx_index = 0U;
    remote_link_previous_switches = 0U;
    remote_link_have_switch_state = 0U;
    /* Match MX_GPIO_Init: PB3 closed, PB5/PB7 off, PB8/PB9 released. */
    remote_link_previous_aux_outputs = REMOTE_LINK_AUX_PE4_MASK;
    remote_link_aux_stream_size = 0U;
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    RemoteLink_ArmReceive();
    RemoteLink_ArmAuxReceive();
}

void RemoteLink_ForwardRawData(void)
{
    uint8_t length = 0U;
    uint16_t head;
    uint16_t tail;

    if (remote_link_tx_busy != 0U)
    {
        return;
    }

    __DMB();
    head = remote_link_forward_head;
    tail = remote_link_forward_tail;

    while ((tail != head) && (length < REMOTE_LINK_FORWARD_CHUNK_SIZE))
    {
        remote_link_tx_buffer[length++] = remote_link_forward_buffer[tail];
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

    remote_link_tx_tail = tail;
    remote_link_tx_length = length;
    remote_link_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart2, remote_link_tx_buffer, length) != HAL_OK)
    {
        remote_link_tx_busy = 0U;
        remote_link_forward_errors++;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART3)
    {
        remote_link_rx_callbacks++;
        RemoteLink_ProcessDmaData(size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance == USART2) && (remote_link_tx_busy != 0U))
    {
        remote_link_forward_tail = remote_link_tx_tail;
        remote_link_forwarded_bytes += remote_link_tx_length;
        __DMB();
        remote_link_tx_busy = 0U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RemoteLink_QueueAuxByte(remote_link_aux_rx_byte);
        RemoteLink_ArmAuxReceive();
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
    else if (huart->Instance == USART2)
    {
        remote_link_forward_errors++;
        remote_link_tx_busy = 0U;
        remote_link_aux_uart_errors++;
        remote_link_aux_stream_size = 0U;
        RemoteLink_ArmAuxReceive();
    }
}
