#include "Remote.h"
#include "RemoteInput.h"
#include "usart.h"

volatile remote_data_t remote_data;
volatile uint32_t remote_tx_frames;
volatile uint32_t remote_tx_attempts;
volatile uint32_t remote_tx_busy_skips;
volatile uint32_t remote_tx_errors;
volatile uint32_t remote_tx_config_skips;
volatile uint32_t remote_lora_config_attempts;
volatile remote_lora_config_status_t remote_lora_config_status;
volatile uint8_t remote_lora_config_readback[REMOTE_LORA_CONFIG_SIZE];

static uint8_t remote_tx_sequence;
static uint8_t remote_lora_ready;

#define E32_CONFIG_BAUDRATE 9600U
#define E32_NORMAL_BAUDRATE 115200U
#define E32_AUX_TIMEOUT_MS 3000U
#define E32_UART_TIMEOUT_MS 300U
#define E32_CONFIG_RETRY_COUNT 3U
#define E32_MODE_SETTLE_MS 10U
#define E32_COMMAND_SETTLE_MS 20U

/* Address 6, 115200 8N1, 9.6 kbps air rate,
 * transparent mode, channel 6. */
static const uint8_t remote_lora_expected_config[REMOTE_LORA_CONFIG_SIZE] = {
    0xC0U, 0x00U, 0x06U, 0x3CU, 0x06U, 0x44U
};

static uint8_t Remote_LoRaWaitAuxHigh(uint32_t timeout_ms)
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

static HAL_StatusTypeDef Remote_LoRaSetBaudrate(uint32_t baudrate)
{
    if (HAL_UART_DeInit(&huart6) != HAL_OK)
    {
        return HAL_ERROR;
    }
    huart6.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart6);
}

static void Remote_LoRaFlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        (void)huart6.Instance->DR;
    }
}

static uint8_t Remote_LoRaConfigMatches(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LORA_CONFIG_SIZE; i++)
    {
        if (config[i] != remote_lora_expected_config[i])
        {
            return 0U;
        }
    }
    return 1U;
}

static void Remote_LoRaSaveReadback(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LORA_CONFIG_SIZE; i++)
    {
        remote_lora_config_readback[i] = config[i];
    }
}

static uint8_t Remote_LoRaReadConfig(uint8_t *config)
{
    uint8_t command[3] = {0xC1U, 0xC1U, 0xC1U};

    Remote_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart6, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_UART_Receive(&huart6, config, REMOTE_LORA_CONFIG_SIZE,
                         E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    Remote_LoRaSaveReadback(config);
    return 1U;
}

static uint8_t Remote_LoRaWriteConfig(void)
{
    uint8_t command[REMOTE_LORA_CONFIG_SIZE];
    uint8_t i;

    for (i = 0U; i < REMOTE_LORA_CONFIG_SIZE; i++)
    {
        command[i] = remote_lora_expected_config[i];
    }
    Remote_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart6, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    HAL_Delay(E32_COMMAND_SETTLE_MS);
    return Remote_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS);
}

static uint8_t Remote_LoRaEnterNormalMode(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(E32_MODE_SETTLE_MS);

    if (Remote_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }
    return (Remote_LoRaSetBaudrate(E32_NORMAL_BAUDRATE) == HAL_OK) ? 1U : 0U;
}

static void Remote_LoRaConfigure(void)
{
    uint8_t config[REMOTE_LORA_CONFIG_SIZE];
    uint8_t attempt;
    uint8_t configured = 0U;

    remote_lora_ready = 0U;
    remote_lora_config_status = REMOTE_LORA_CONFIGURING;
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);

    if (Remote_LoRaSetBaudrate(E32_CONFIG_BAUDRATE) != HAL_OK)
    {
        remote_lora_config_status = REMOTE_LORA_CONFIG_UART_ERROR;
        (void)Remote_LoRaEnterNormalMode();
        return;
    }
    HAL_Delay(E32_MODE_SETTLE_MS);
    if (Remote_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        remote_lora_config_status = REMOTE_LORA_CONFIG_AUX_TIMEOUT;
        (void)Remote_LoRaEnterNormalMode();
        return;
    }

    for (attempt = 0U; attempt < E32_CONFIG_RETRY_COUNT; attempt++)
    {
        remote_lora_config_attempts++;
        if (Remote_LoRaReadConfig(config) == 0U)
        {
            remote_lora_config_status = REMOTE_LORA_CONFIG_READ_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if (Remote_LoRaConfigMatches(config) != 0U)
        {
            configured = 1U;
            break;
        }
        if (Remote_LoRaWriteConfig() == 0U)
        {
            remote_lora_config_status = REMOTE_LORA_CONFIG_WRITE_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if ((Remote_LoRaReadConfig(config) != 0U) &&
            (Remote_LoRaConfigMatches(config) != 0U))
        {
            configured = 1U;
            break;
        }
        remote_lora_config_status = REMOTE_LORA_CONFIG_VERIFY_ERROR;
        HAL_Delay(50U);
    }

    if (configured == 0U)
    {
        (void)Remote_LoRaEnterNormalMode();
        return;
    }
    if (Remote_LoRaEnterNormalMode() == 0U)
    {
        remote_lora_config_status = REMOTE_LORA_CONFIG_NORMAL_MODE_TIMEOUT;
        return;
    }

    remote_lora_ready = 1U;
    remote_lora_config_status = REMOTE_LORA_CONFIG_READY;
}

static uint8_t Remote_Crc8(const uint8_t *data, uint32_t length)
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

void Remote_GetSnapshot(remote_data_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = remote_data;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Remote_LoRaInit(void)
{
    remote_tx_sequence = 0U;
    remote_tx_frames = 0U;
    remote_tx_attempts = 0U;
    remote_tx_busy_skips = 0U;
    remote_tx_errors = 0U;
    remote_tx_config_skips = 0U;
    remote_lora_config_attempts = 0U;
    remote_lora_config_status = REMOTE_LORA_CONFIG_NOT_STARTED;
    Remote_LoRaConfigure();
}

uint8_t Remote_LoRaReady(void)
{
    return remote_lora_ready;
}

void Remote_SendLoRa(void)
{
    remote_data_t snapshot;
    uint8_t frame[REMOTE_LINK_FRAME_SIZE];
    uint8_t switch_mask = 0U;
    uint16_t key_mask = 0U;
    uint8_t i;

    remote_tx_attempts++;

    if (Remote_LoRaReady() == 0U)
    {
        remote_tx_config_skips++;
        return;
    }
    if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
    {
        remote_tx_busy_skips++;
        return;
    }

    Remote_GetSnapshot(&snapshot);
    for (i = 0U; i < REMOTE_SWITCH_COUNT; i++)
    {
        switch_mask |= (uint8_t)((snapshot.switch_state[i] & 1U) << i);
    }
    for (i = 0U; i < REMOTE_KEY_COUNT; i++)
    {
        key_mask |= (uint16_t)((uint16_t)(snapshot.key_state[i] & 1U) << i);
    }

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 0x01U;
    frame[3] = remote_tx_sequence;
    frame[4] = (uint8_t)snapshot.left_shoulder;
    frame[5] = (uint8_t)(snapshot.left_shoulder >> 8U);
    frame[6] = (uint8_t)snapshot.right_shoulder;
    frame[7] = (uint8_t)(snapshot.right_shoulder >> 8U);
    frame[8] = snapshot.left_x;
    frame[9] = snapshot.left_y;
    frame[10] = snapshot.right_x;
    frame[11] = snapshot.right_y;
    frame[12] = switch_mask;
    frame[13] = (uint8_t)key_mask;
    frame[14] = (uint8_t)((key_mask >> 8U) & 0x0FU);
    if (remote_adc_state != REMOTE_ADC_OK)
    {
        frame[14] |= 0x80U;
    }
    frame[15] = Remote_Crc8(frame, REMOTE_LINK_FRAME_SIZE - 1U);

    if (HAL_UART_Transmit(&huart6, frame, REMOTE_LINK_FRAME_SIZE, 10U) == HAL_OK)
    {
        remote_tx_sequence++;
        remote_tx_frames++;
    }
    else
    {
        remote_tx_errors++;
    }
}
