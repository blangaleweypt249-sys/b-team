#include "remote_link.h"
#include "usart.h"

volatile remote_link_data_t remote_link_data;
volatile uint32_t remote_link_valid_frames;
volatile uint32_t remote_link_crc_errors;
volatile uint32_t remote_link_lost_frames;
volatile uint32_t remote_link_uart_errors;
volatile uint32_t remote_link_rx_bytes;
volatile uint32_t remote_link_rx_callbacks;
volatile uint32_t remote_link_rx_arm_errors;
volatile uint32_t remote_link_lora_config_attempts;
volatile remote_link_lora_config_status_t remote_link_lora_config_status;
volatile uint8_t remote_link_lora_config_readback[REMOTE_LINK_LORA_CONFIG_SIZE];

static uint8_t remote_link_rx_byte;
static uint8_t remote_link_rx_frame[REMOTE_LINK_FRAME_SIZE];
static uint8_t remote_link_rx_index;
static uint8_t remote_link_have_frame;
static uint32_t remote_link_last_rx_tick;
static uint8_t remote_link_lora_ready;
static volatile uint8_t remote_link_led_pulse_active;
static volatile uint32_t remote_link_led_pulse_started_at;
static uint8_t remote_link_uart2_frame[REMOTE_LINK_FRAME_SIZE];
static volatile uint8_t remote_link_uart2_frame_pending;

#define E32_CONFIG_BAUDRATE 9600U
#define E32_NORMAL_BAUDRATE 115200U
#define E32_AUX_TIMEOUT_MS 3000U
#define E32_UART_TIMEOUT_MS 300U
#define E32_CONFIG_RETRY_COUNT 3U
#define E32_MODE_SETTLE_MS 10U
#define E32_COMMAND_SETTLE_MS 20U
#define REMOTE_LINK_LED_PULSE_MS 100U
#define REMOTE_LINK_PE4_SWITCH_INDEX 0U
#define REMOTE_LINK_PE3_SWITCH_INDEX 1U
#define REMOTE_LINK_PE1_SWITCH_INDEX 2U
#define REMOTE_LINK_PD5_SWITCH_INDEX 5U

/* Address 6, 115200 8N1, 9.6 kbps air rate,
 * transparent mode, channel 6. */
static const uint8_t remote_link_lora_expected_config[REMOTE_LINK_LORA_CONFIG_SIZE] = {
    0xC0U, 0x00U, 0x06U, 0x3CU, 0x06U, 0x44U
};

static uint8_t RemoteLink_LoRaWaitAuxHigh(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do
    {
        if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(2U);
            if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
            {
                return 1U;
            }
        }
    } while ((uint32_t)(HAL_GetTick() - start) < timeout_ms);

    return 0U;
}

static HAL_StatusTypeDef RemoteLink_LoRaSetBaudrate(uint32_t baudrate)
{
    if (HAL_UART_DeInit(&huart3) != HAL_OK)
    {
        return HAL_ERROR;
    }
    huart3.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart3);
}

static void RemoteLink_LoRaFlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        (void)huart3.Instance->DR;
    }
}

static void RemoteLink_LoRaSaveReadback(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        remote_link_lora_config_readback[i] = config[i];
    }
}

static uint8_t RemoteLink_LoRaConfigMatches(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        if (config[i] != remote_link_lora_expected_config[i])
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t RemoteLink_LoRaReadConfig(uint8_t *config)
{
    uint8_t command[3] = {0xC1U, 0xC1U, 0xC1U};

    RemoteLink_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart3, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_UART_Receive(&huart3, config, REMOTE_LINK_LORA_CONFIG_SIZE,
                         E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    RemoteLink_LoRaSaveReadback(config);
    return 1U;
}

static uint8_t RemoteLink_LoRaWriteConfig(void)
{
    uint8_t command[REMOTE_LINK_LORA_CONFIG_SIZE];
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        command[i] = remote_link_lora_expected_config[i];
    }
    RemoteLink_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart3, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    HAL_Delay(E32_COMMAND_SETTLE_MS);
    return RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS);
}

static uint8_t RemoteLink_LoRaEnterNormalMode(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(E32_MODE_SETTLE_MS);

    if (RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }
    return (RemoteLink_LoRaSetBaudrate(E32_NORMAL_BAUDRATE) == HAL_OK) ? 1U : 0U;
}

static void RemoteLink_LoRaConfigure(void)
{
    uint8_t config[REMOTE_LINK_LORA_CONFIG_SIZE];
    uint8_t attempt;
    uint8_t configured = 0U;

    remote_link_lora_ready = 0U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIGURING;
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);

    if (RemoteLink_LoRaSetBaudrate(E32_CONFIG_BAUDRATE) != HAL_OK)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_UART_ERROR;
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }
    HAL_Delay(E32_MODE_SETTLE_MS);
    if (RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_AUX_TIMEOUT;
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }

    for (attempt = 0U; attempt < E32_CONFIG_RETRY_COUNT; attempt++)
    {
        remote_link_lora_config_attempts++;
        if (RemoteLink_LoRaReadConfig(config) == 0U)
        {
            remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_READ_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if (RemoteLink_LoRaConfigMatches(config) != 0U)
        {
            configured = 1U;
            break;
        }
        if (RemoteLink_LoRaWriteConfig() == 0U)
        {
            remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_WRITE_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if ((RemoteLink_LoRaReadConfig(config) != 0U) &&
            (RemoteLink_LoRaConfigMatches(config) != 0U))
        {
            configured = 1U;
            break;
        }
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_VERIFY_ERROR;
        HAL_Delay(50U);
    }

    if (configured == 0U)
    {
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }
    if (RemoteLink_LoRaEnterNormalMode() == 0U)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_NORMAL_MODE_TIMEOUT;
        return;
    }

    remote_link_lora_ready = 1U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_READY;
}

static uint8_t RemoteLink_Crc8(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0U;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x80U) != 0U) ?
                  (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static void RemoteLink_QueueRawFrame(const uint8_t *frame)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_FRAME_SIZE; i++)
    {
        remote_link_uart2_frame[i] = frame[i];
    }
    remote_link_uart2_frame_pending = 1U;
}

static void RemoteLink_UpdateToggleOutputs(uint8_t switch_mask)
{
    if (remote_link_have_frame == 0U)
    {
        return;
    }

    if (((switch_mask >> REMOTE_LINK_PE4_SWITCH_INDEX) & 1U) !=
        remote_link_data.switch_state[REMOTE_LINK_PE4_SWITCH_INDEX])
    {
        HAL_GPIO_TogglePin(U4_12_PB14_GPIO_Port, U4_12_PB14_Pin);
    }
    if (((switch_mask >> REMOTE_LINK_PE3_SWITCH_INDEX) & 1U) !=
        remote_link_data.switch_state[REMOTE_LINK_PE3_SWITCH_INDEX])
    {
        HAL_GPIO_TogglePin(U4_10_PA8_GPIO_Port, U4_10_PA8_Pin);
    }
    if (((switch_mask >> REMOTE_LINK_PE1_SWITCH_INDEX) & 1U) !=
        remote_link_data.switch_state[REMOTE_LINK_PE1_SWITCH_INDEX])
    {
        HAL_GPIO_TogglePin(U4_8_PA10_GPIO_Port, U4_8_PA10_Pin);
    }
    if (((switch_mask >> REMOTE_LINK_PD5_SWITCH_INDEX) & 1U) !=
        remote_link_data.switch_state[REMOTE_LINK_PD5_SWITCH_INDEX])
    {
        HAL_GPIO_TogglePin(U4_6_PA12_GPIO_Port, U4_6_PA12_Pin);
    }
}

static uint8_t RemoteLink_HasKeyOrSwitchEvent(uint8_t switch_mask,
                                              uint16_t key_mask)
{
    uint8_t i;

    if (remote_link_have_frame == 0U)
    {
        return 0U;
    }
    for (i = 0U; i < REMOTE_LINK_SWITCH_COUNT; i++)
    {
        if (((switch_mask >> i) & 1U) != remote_link_data.switch_state[i])
        {
            return 1U;
        }
    }
    for (i = 0U; i < REMOTE_LINK_KEY_COUNT; i++)
    {
        if ((((key_mask >> i) & 1U) != 0U) &&
            (remote_link_data.key_state[i] == 0U))
        {
            return 1U;
        }
    }
    return 0U;
}

static void RemoteLink_CommitFrame(const uint8_t *frame)
{
    uint8_t switch_mask = frame[12];
    uint16_t key_mask = (uint16_t)frame[13] |
                        ((uint16_t)(frame[14] & 0x0FU) << 8U);
    uint8_t sequence = frame[3];
    uint8_t key_or_switch_event =
        RemoteLink_HasKeyOrSwitchEvent(switch_mask, key_mask);
    uint8_t i;

    if (remote_link_have_frame != 0U)
    {
        uint8_t gap = (uint8_t)(sequence - remote_link_data.sequence);
        if (gap > 1U)
        {
            remote_link_lost_frames += (uint32_t)(gap - 1U);
        }
    }

    remote_link_data.left_shoulder = (uint16_t)frame[4] |
                                     ((uint16_t)frame[5] << 8U);
    remote_link_data.right_shoulder = (uint16_t)frame[6] |
                                      ((uint16_t)frame[7] << 8U);
    remote_link_data.left_x = frame[8];
    remote_link_data.left_y = frame[9];
    remote_link_data.right_x = frame[10];
    remote_link_data.right_y = frame[11];
    RemoteLink_UpdateToggleOutputs(switch_mask);
    for (i = 0U; i < REMOTE_LINK_SWITCH_COUNT; i++)
    {
        remote_link_data.switch_state[i] = (uint8_t)((switch_mask >> i) & 1U);
    }
    for (i = 0U; i < REMOTE_LINK_KEY_COUNT; i++)
    {
        remote_link_data.key_state[i] = (uint8_t)((key_mask >> i) & 1U);
    }
    remote_link_data.adc_error = ((frame[14] & 0x80U) != 0U) ? 1U : 0U;
    remote_link_data.sequence = sequence;
    remote_link_last_rx_tick = HAL_GetTick();
    remote_link_have_frame = 1U;
    remote_link_valid_frames++;
    if (key_or_switch_event != 0U)
    {
        remote_link_led_pulse_started_at = remote_link_last_rx_tick;
        remote_link_led_pulse_active = 1U;
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    }
}

static void RemoteLink_ParseByte(uint8_t byte)
{
    if (remote_link_rx_index == 0U)
    {
        if (byte == 0xA5U)
        {
            remote_link_rx_frame[0] = byte;
            remote_link_rx_index = 1U;
        }
        return;
    }

    if (remote_link_rx_index == 1U)
    {
        if (byte == 0x5AU)
        {
            remote_link_rx_frame[1] = byte;
            remote_link_rx_index = 2U;
        }
        else if (byte != 0xA5U)
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
    RemoteLink_QueueRawFrame(remote_link_rx_frame);
    if ((remote_link_rx_frame[2] != 0x01U) ||
        (RemoteLink_Crc8(remote_link_rx_frame, REMOTE_LINK_FRAME_SIZE - 1U) !=
         remote_link_rx_frame[REMOTE_LINK_FRAME_SIZE - 1U]))
    {
        remote_link_crc_errors++;
        return;
    }
    RemoteLink_CommitFrame(remote_link_rx_frame);
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

void RemoteLink_Init(void)
{
    remote_link_rx_index = 0U;
    remote_link_have_frame = 0U;
    remote_link_valid_frames = 0U;
    remote_link_crc_errors = 0U;
    remote_link_lost_frames = 0U;
    remote_link_uart_errors = 0U;
    remote_link_rx_bytes = 0U;
    remote_link_rx_callbacks = 0U;
    remote_link_rx_arm_errors = 0U;
    remote_link_lora_config_attempts = 0U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_NOT_STARTED;
    remote_link_led_pulse_active = 0U;
    remote_link_uart2_frame_pending = 0U;
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    RemoteLink_LoRaConfigure();
    if (RemoteLink_LoRaReady() != 0U)
    {
        RemoteLink_ArmReceive();
    }
}

uint8_t RemoteLink_LoRaReady(void)
{
    return remote_link_lora_ready;
}

uint8_t RemoteLink_IsConnected(void)
{
    return ((RemoteLink_LoRaReady() != 0U) &&
            (remote_link_have_frame != 0U) &&
            ((HAL_GetTick() - remote_link_last_rx_tick) <= REMOTE_LINK_TIMEOUT_MS)) ?
           1U : 0U;
}

uint8_t RemoteLink_GetSnapshot(remote_link_data_t *snapshot)
{
    uint32_t primask;
    uint8_t connected;

    if (snapshot == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = remote_link_data;
    connected = RemoteLink_IsConnected();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return connected;
}

void RemoteLink_Process(void)
{
    if ((remote_link_led_pulse_active != 0U) &&
        ((uint32_t)(HAL_GetTick() - remote_link_led_pulse_started_at) >=
         REMOTE_LINK_LED_PULSE_MS))
    {
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
        remote_link_led_pulse_active = 0U;
    }
}

void RemoteLink_ForwardRawFrame(void)
{
    uint8_t frame[REMOTE_LINK_FRAME_SIZE];
    uint8_t pending;
    uint8_t i;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = remote_link_uart2_frame_pending;
    if (pending != 0U)
    {
        for (i = 0U; i < REMOTE_LINK_FRAME_SIZE; i++)
        {
            frame[i] = remote_link_uart2_frame[i];
        }
        remote_link_uart2_frame_pending = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (pending != 0U)
    {
        (void)HAL_UART_Transmit(&huart2, frame, REMOTE_LINK_FRAME_SIZE, 10U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_rx_callbacks++;
        if (RemoteLink_LoRaReady() != 0U)
        {
            remote_link_rx_bytes++;
            RemoteLink_ParseByte(remote_link_rx_byte);
        }
        RemoteLink_ArmReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_uart_errors++;
        remote_link_rx_index = 0U;
        if (RemoteLink_LoRaReady() != 0U)
        {
            RemoteLink_ArmReceive();
        }
    }
}
