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
volatile uint32_t remote_link_forward_errors;
volatile uint32_t remote_link_lora_config_attempts;
volatile remote_link_lora_config_status_t remote_link_lora_config_status;
volatile uint8_t remote_link_lora_config_readback[REMOTE_LINK_LORA_CONFIG_SIZE];

static uint8_t remote_link_rx_byte;
static uint8_t remote_link_rx_frame[REMOTE_LINK_MAX_FRAME_SIZE];
static uint8_t remote_link_rx_index;
static uint8_t remote_link_rx_expected_length;
static uint8_t remote_link_have_frame;
static uint32_t remote_link_last_rx_tick;
static uint8_t remote_link_lora_ready;
static volatile uint8_t remote_link_led_pulse_active;
static volatile uint32_t remote_link_led_pulse_started_at;
static uint8_t remote_link_uart2_frame[REMOTE_LINK_MAX_FRAME_SIZE];
static uint8_t remote_link_uart2_frame_length;
static volatile uint8_t remote_link_uart2_frame_pending;

#define E32_CONFIG_BAUDRATE 9600U
#define E32_NORMAL_BAUDRATE 115200U
#define E32_AUX_TIMEOUT_MS 3000U
#define E32_UART_TIMEOUT_MS 300U
#define E32_CONFIG_RETRY_COUNT 3U
#define E32_MODE_SETTLE_MS 10U
#define E32_COMMAND_SETTLE_MS 20U
#define REMOTE_LINK_LED_PULSE_MS 100U
#define REMOTE_LINK_HEADER_0 0xA5U
#define REMOTE_LINK_HEADER_1 0x5AU
#define REMOTE_LINK_ID_LOCAL 0x01U
#define REMOTE_LINK_ID_SECONDARY 0x02U
#define REMOTE_LINK_TYPE_CONTROL 0x01U
#define REMOTE_LINK_LOCAL_PAYLOAD_SIZE 6U
#define REMOTE_LINK_SECOND_PAYLOAD_SIZE 2U
#define REMOTE_LINK_PE0_SWITCH_INDEX 3U
#define REMOTE_LINK_SHOULDER_ACTIVE 1000U

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

static void RemoteLink_QueueRawFrame(const uint8_t *frame, uint8_t length)
{
    uint8_t i;

    if (length > REMOTE_LINK_MAX_FRAME_SIZE)
    {
        return;
    }
    for (i = 0U; i < length; i++)
    {
        remote_link_uart2_frame[i] = frame[i];
    }
    remote_link_uart2_frame_length = length;
    remote_link_uart2_frame_pending = 1U;
}

static uint8_t RemoteLink_HasLocalEvent(uint8_t local_buttons,
                                        uint8_t pe0_switch)
{
    uint8_t i;

    if (remote_link_have_frame == 0U)
    {
        return 0U;
    }
    for (i = 0U; i < 6U; i++)
    {
        if ((((local_buttons >> i) & 1U) != 0U) &&
            (remote_link_data.key_state[i] == 0U))
        {
            return 1U;
        }
    }
    if ((((local_buttons >> 6U) & 1U) != 0U) &&
        (remote_link_data.left_shoulder == 0U))
    {
        return 1U;
    }
    if ((((local_buttons >> 7U) & 1U) != 0U) &&
        (remote_link_data.right_shoulder == 0U))
    {
        return 1U;
    }
    if (pe0_switch !=
        remote_link_data.switch_state[REMOTE_LINK_PE0_SWITCH_INDEX])
    {
        return 1U;
    }
    return 0U;
}

static void RemoteLink_CommitLocalFrame(const uint8_t *frame)
{
    uint8_t local_buttons = frame[10];
    uint8_t pe0_switch = frame[11] & 1U;
    uint8_t sequence = frame[5];
    uint8_t local_event =
        RemoteLink_HasLocalEvent(local_buttons, pe0_switch);
    uint8_t i;

    if (remote_link_have_frame != 0U)
    {
        uint8_t gap = (uint8_t)(sequence - remote_link_data.sequence);
        if (gap > 1U)
        {
            remote_link_lost_frames += (uint32_t)(gap - 1U);
        }
    }

    remote_link_data.left_x = frame[6];
    remote_link_data.left_y = frame[7];
    remote_link_data.right_x = frame[8];
    remote_link_data.right_y = frame[9];
    remote_link_data.left_shoulder =
        ((local_buttons & 0x40U) != 0U) ? REMOTE_LINK_SHOULDER_ACTIVE : 0U;
    remote_link_data.right_shoulder =
        ((local_buttons & 0x80U) != 0U) ? REMOTE_LINK_SHOULDER_ACTIVE : 0U;
    for (i = 0U; i < REMOTE_LINK_SWITCH_COUNT; i++)
    {
        remote_link_data.switch_state[i] = 0U;
    }
    remote_link_data.switch_state[REMOTE_LINK_PE0_SWITCH_INDEX] = pe0_switch;
    for (i = 0U; i < REMOTE_LINK_KEY_COUNT; i++)
    {
        remote_link_data.key_state[i] =
            (i < 6U) ? (uint8_t)((local_buttons >> i) & 1U) : 0U;
    }
    remote_link_data.adc_error = 0U;
    remote_link_data.sequence = sequence;
    remote_link_last_rx_tick = HAL_GetTick();
    remote_link_have_frame = 1U;
    remote_link_valid_frames++;
    if (local_event != 0U)
    {
        remote_link_led_pulse_started_at = remote_link_last_rx_tick;
        remote_link_led_pulse_active = 1U;
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    }
}

static void RemoteLink_HandleFrame(const uint8_t *frame, uint8_t length)
{
    uint8_t payload_size = frame[4];

    if ((length != (uint8_t)(REMOTE_LINK_FRAME_OVERHEAD + payload_size)) ||
        (frame[3] != REMOTE_LINK_TYPE_CONTROL) ||
        (RemoteLink_Crc8(&frame[2], (uint32_t)(length - 3U)) !=
         frame[length - 1U]))
    {
        remote_link_crc_errors++;
        return;
    }

    if ((frame[2] == REMOTE_LINK_ID_LOCAL) &&
        (payload_size == REMOTE_LINK_LOCAL_PAYLOAD_SIZE))
    {
        RemoteLink_CommitLocalFrame(frame);
    }
    else if ((frame[2] == REMOTE_LINK_ID_SECONDARY) &&
             (payload_size == REMOTE_LINK_SECOND_PAYLOAD_SIZE))
    {
        RemoteLink_QueueRawFrame(frame, length);
    }
    else
    {
        remote_link_crc_errors++;
    }
}

static void RemoteLink_ParseByte(uint8_t byte)
{
    uint8_t frame_length;

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
    if (remote_link_rx_index == 5U)
    {
        remote_link_rx_expected_length =
            (uint8_t)(REMOTE_LINK_FRAME_OVERHEAD + remote_link_rx_frame[4]);
        if (remote_link_rx_expected_length > REMOTE_LINK_MAX_FRAME_SIZE)
        {
            remote_link_crc_errors++;
            remote_link_rx_index = 0U;
            remote_link_rx_expected_length = 0U;
            return;
        }
    }
    if ((remote_link_rx_expected_length == 0U) ||
        (remote_link_rx_index < remote_link_rx_expected_length))
    {
        return;
    }

    frame_length = remote_link_rx_expected_length;
    remote_link_rx_index = 0U;
    remote_link_rx_expected_length = 0U;
    RemoteLink_HandleFrame(remote_link_rx_frame, frame_length);
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
    remote_link_rx_expected_length = 0U;
    remote_link_have_frame = 0U;
    remote_link_valid_frames = 0U;
    remote_link_crc_errors = 0U;
    remote_link_lost_frames = 0U;
    remote_link_uart_errors = 0U;
    remote_link_rx_bytes = 0U;
    remote_link_rx_callbacks = 0U;
    remote_link_rx_arm_errors = 0U;
    remote_link_forward_errors = 0U;
    remote_link_lora_config_attempts = 0U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_NOT_STARTED;
    remote_link_led_pulse_active = 0U;
    remote_link_uart2_frame_length = 0U;
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
    uint8_t frame[REMOTE_LINK_MAX_FRAME_SIZE];
    uint8_t length = 0U;
    uint8_t pending;
    uint8_t i;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = remote_link_uart2_frame_pending;
    if (pending != 0U)
    {
        length = remote_link_uart2_frame_length;
        for (i = 0U; i < length; i++)
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
        if (HAL_UART_Transmit(&huart2, frame, length, 10U) != HAL_OK)
        {
            remote_link_forward_errors++;
        }
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
        remote_link_rx_expected_length = 0U;
        if (RemoteLink_LoRaReady() != 0U)
        {
            RemoteLink_ArmReceive();
        }
    }
}
