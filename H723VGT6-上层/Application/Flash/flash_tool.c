#include "flash_tool.h"

#include <stdio.h>
#include <string.h>

#include "W25Qxx.h"
#include "comm_runtime.h"

#define FLASH_TOOL_LINE_SIZE       832U
#define FLASH_TOOL_RESPONSE_SIZE   900U
#define FLASH_TOOL_MAX_DATA_SIZE   256U
#define FLASH_TOOL_TX_TIMEOUT_MS   1500U
#define FLASH_TOOL_DECIMAL_BASE    10U
#define FLASH_TOOL_HEX_BASE        16U
#define FLASH_TOOL_TOKEN_DELIMITER " \t"

typedef enum
{
    FLASH_TOOL_ERROR_INVALID_CMD = 0x80,
    FLASH_TOOL_ERROR_INVALID_ARG,
    FLASH_TOOL_ERROR_LINE_TOO_LONG,
    FLASH_TOOL_ERROR_UART
} flash_tool_error_t;

static void FlashTool_ProcessLine(void);
static void FlashTool_ProcessHelp(void);
static void FlashTool_ProcessInfo(void);
static void FlashTool_ProcessErase(void);
static void FlashTool_ProcessWrite(void);
static void FlashTool_ProcessRead(void);
static void FlashTool_ProcessExit(void);
static void FlashTool_SendText(const char *text);
static void FlashTool_SendToolError(flash_tool_error_t status);
static void FlashTool_SendFlashError(w25q_status_t status);
static bool FlashTool_LooksLikeCommand(const uint8_t *data, size_t size);
static bool FlashTool_ParseNumber(const char *text,
                                  uint8_t default_base,
                                  uint32_t *value);
static bool FlashTool_ParseHexByte(const char *text, uint8_t *value);
static bool FlashTool_StringEquals(const char *left, const char *right);
static const char *FlashTool_GetDeviceName(uint32_t flash_id);
static const char *FlashTool_GetStatusName(w25q_status_t status);

static char rx_line[FLASH_TOOL_LINE_SIZE];
static char response_text[FLASH_TOOL_RESPONSE_SIZE];
static uint8_t flash_data[FLASH_TOOL_MAX_DATA_SIZE];
static uint16_t rx_index;
static bool line_overflow;
static volatile bool flash_tool_active;
static w25q_status_t flash_init_status = W25Q_ERROR_NOT_INIT;
static w25q_status_t flash_device_id_status = W25Q_ERROR_NOT_INIT;
static uint16_t flash_device_id;

void FlashTool_Init(void)
{
    (void)memset(rx_line, 0, sizeof(rx_line));
    (void)memset(response_text, 0, sizeof(response_text));
    (void)memset(flash_data, 0, sizeof(flash_data));
    rx_index = 0U;
    line_overflow = false;
    /* UART4 is the VOFA Flash console by default. This prevents the upper
       computer's binary state frames from being mixed into text responses. */
    flash_tool_active = true;
    flash_init_status = W25Q_PortInit();
    flash_device_id_status =
        W25Q_ReadDeviceId(W25Q_PortGetDevice(), &flash_device_id);
}

void FlashTool_SendReady(bool comm_ready)
{
    w25q_handle_t *flash = W25Q_PortGetDevice();

    (void)snprintf(response_text,
                   sizeof(response_text),
                   "FLASH READY COMM=%u INIT=%u JEDEC_ID=0x%06lX "
                   "DEVICE_STATUS=%u DEVICE_ID=0x%04X MODEL=%s\r\n",
                   comm_ready ? 1U : 0U,
                   (unsigned int)flash_init_status,
                   (unsigned long)flash->flash_id,
                   (unsigned int)flash_device_id_status,
                   (unsigned int)flash_device_id,
                   FlashTool_GetDeviceName(flash->flash_id));
    FlashTool_SendText(response_text);
}

bool FlashTool_Receive(const uint8_t *data, size_t size)
{
    size_t data_index;

    if ((data == NULL) || (size == 0U))
    {
        return false;
    }
    if (!flash_tool_active && !FlashTool_LooksLikeCommand(data, size))
    {
        return false;
    }

    flash_tool_active = true;
    for (data_index = 0U; data_index < size; data_index++)
    {
        uint8_t value = data[data_index];

        if ((value == '\r') || (value == '\n'))
        {
            if ((rx_index > 0U) || line_overflow)
            {
                if (line_overflow)
                {
                    FlashTool_SendToolError(FLASH_TOOL_ERROR_LINE_TOO_LONG);
                }
                else
                {
                    rx_line[rx_index] = '\0';
                    FlashTool_ProcessLine();
                }
                rx_index = 0U;
                line_overflow = false;
                rx_line[0] = '\0';
                if (!flash_tool_active)
                {
                    return true;
                }
            }
            continue;
        }

        if ((value != '\t') && ((value < 0x20U) || (value > 0x7EU)))
        {
            rx_index = 0U;
            line_overflow = false;
            rx_line[0] = '\0';
            continue;
        }
        if (line_overflow)
        {
            continue;
        }
        if (rx_index < (FLASH_TOOL_LINE_SIZE - 1U))
        {
            rx_line[rx_index++] = (char)value;
        }
        else
        {
            line_overflow = true;
        }
    }

    return true;
}

bool FlashTool_IsActive(void)
{
    return flash_tool_active;
}

static void FlashTool_ProcessLine(void)
{
    char *command = strtok(rx_line, FLASH_TOOL_TOKEN_DELIMITER);

    if (command == NULL)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_CMD);
    }
    else if (FlashTool_StringEquals(command, "FLASH"))
    {
        FlashTool_ProcessHelp();
    }
    else if (FlashTool_StringEquals(command, "HELP"))
    {
        FlashTool_ProcessHelp();
    }
    else if (FlashTool_StringEquals(command, "INFO"))
    {
        FlashTool_ProcessInfo();
    }
    else if (FlashTool_StringEquals(command, "ERASE"))
    {
        FlashTool_ProcessErase();
    }
    else if (FlashTool_StringEquals(command, "WRITE"))
    {
        FlashTool_ProcessWrite();
    }
    else if (FlashTool_StringEquals(command, "READ"))
    {
        FlashTool_ProcessRead();
    }
    else if (FlashTool_StringEquals(command, "EXIT"))
    {
        FlashTool_ProcessExit();
    }
    else
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_CMD);
    }
}

static void FlashTool_ProcessHelp(void)
{
    if (strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER) != NULL)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    FlashTool_SendText(
        "OK HELP COMMANDS=FLASH|INFO|ERASE <addr> [count]|"
        "WRITE <addr> <hex...>|READ <addr> <len>|EXIT\r\n");
}

static void FlashTool_ProcessInfo(void)
{
    w25q_handle_t *flash = W25Q_PortGetDevice();

    if (strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER) != NULL)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    (void)snprintf(response_text,
                   sizeof(response_text),
                   "OK INFO INIT=%u INIT_NAME=%s INITIALIZED=%u "
                   "JEDEC_ID=0x%06lX DEVICE_STATUS=%u DEVICE_ID=0x%04X "
                   "MODEL=%s CAPACITY_KB=%lu SECTORS=%lu "
                   "PAGE=%u SECTOR=%lu\r\n",
                   (unsigned int)flash_init_status,
                   FlashTool_GetStatusName(flash_init_status),
                   flash->is_initialized ? 1U : 0U,
                   (unsigned long)flash->flash_id,
                   (unsigned int)flash_device_id_status,
                   (unsigned int)flash_device_id,
                   FlashTool_GetDeviceName(flash->flash_id),
                   (unsigned long)flash->capacity_kb,
                   (unsigned long)flash->sector_count,
                   (unsigned int)flash->page_size_byte,
                   (unsigned long)W25Q_SECTOR_SIZE_BYTE);
    FlashTool_SendText(response_text);
}

static void FlashTool_ProcessErase(void)
{
    char *address_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    char *count_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    char *extra_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    uint32_t address;
    uint32_t sector_count = 1U;
    w25q_status_t status;

    if ((address_text == NULL) || (extra_text != NULL) ||
        !FlashTool_ParseNumber(address_text, FLASH_TOOL_DECIMAL_BASE, &address) ||
        ((count_text != NULL) &&
         (!FlashTool_ParseNumber(count_text,
                                 FLASH_TOOL_DECIMAL_BASE,
                                 &sector_count) ||
          (sector_count == 0U))))
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    status = W25Q_EraseSectors(W25Q_PortGetDevice(), address, sector_count);
    if (status != W25Q_OK)
    {
        FlashTool_SendFlashError(status);
        return;
    }

    (void)snprintf(response_text,
                   sizeof(response_text),
                   "OK ERASE ADDR=0x%08lX COUNT=%lu\r\n",
                   (unsigned long)address,
                   (unsigned long)sector_count);
    FlashTool_SendText(response_text);
}

static void FlashTool_ProcessWrite(void)
{
    char *address_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    char *data_text;
    uint32_t address;
    uint16_t data_len_byte = 0U;
    w25q_status_t status;

    if ((address_text == NULL) ||
        !FlashTool_ParseNumber(address_text, FLASH_TOOL_DECIMAL_BASE, &address))
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    data_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    while (data_text != NULL)
    {
        if ((data_len_byte >= FLASH_TOOL_MAX_DATA_SIZE) ||
            !FlashTool_ParseHexByte(data_text, &flash_data[data_len_byte]))
        {
            FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
            return;
        }
        data_len_byte++;
        data_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    }
    if (data_len_byte == 0U)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    status = W25Q_WriteData(W25Q_PortGetDevice(),
                            address,
                            flash_data,
                            data_len_byte);
    if (status != W25Q_OK)
    {
        FlashTool_SendFlashError(status);
        return;
    }

    (void)snprintf(response_text,
                   sizeof(response_text),
                   "OK WRITE ADDR=0x%08lX LEN=%u\r\n",
                   (unsigned long)address,
                   (unsigned int)data_len_byte);
    FlashTool_SendText(response_text);
}

static void FlashTool_ProcessRead(void)
{
    char *address_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    char *length_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    char *extra_text = strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER);
    uint32_t address;
    uint32_t read_len_value;
    uint16_t read_len_byte;
    uint16_t data_index;
    uint16_t response_len_byte;
    int write_count;
    w25q_status_t status;

    if ((address_text == NULL) || (length_text == NULL) ||
        (extra_text != NULL) ||
        !FlashTool_ParseNumber(address_text, FLASH_TOOL_DECIMAL_BASE, &address) ||
        !FlashTool_ParseNumber(length_text,
                               FLASH_TOOL_DECIMAL_BASE,
                               &read_len_value) ||
        (read_len_value == 0U) ||
        (read_len_value > FLASH_TOOL_MAX_DATA_SIZE))
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    read_len_byte = (uint16_t)read_len_value;
    status = W25Q_ReadData(W25Q_PortGetDevice(),
                           address,
                           flash_data,
                           read_len_byte);
    if (status != W25Q_OK)
    {
        FlashTool_SendFlashError(status);
        return;
    }

    write_count = snprintf(response_text,
                           sizeof(response_text),
                           "OK READ ADDR=0x%08lX LEN=%u DATA=",
                           (unsigned long)address,
                           (unsigned int)read_len_byte);
    if (write_count < 0)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_UART);
        return;
    }
    response_len_byte = (uint16_t)write_count;

    for (data_index = 0U; data_index < read_len_byte; data_index++)
    {
        write_count = snprintf(&response_text[response_len_byte],
                               sizeof(response_text) - response_len_byte,
                               (data_index + 1U < read_len_byte) ?
                                   "%02X " : "%02X\r\n",
                               (unsigned int)flash_data[data_index]);
        if ((write_count < 0) ||
            ((uint32_t)write_count >=
             (sizeof(response_text) - response_len_byte)))
        {
            FlashTool_SendToolError(FLASH_TOOL_ERROR_LINE_TOO_LONG);
            return;
        }
        response_len_byte += (uint16_t)write_count;
    }
    FlashTool_SendText(response_text);
}

static void FlashTool_ProcessExit(void)
{
    if (strtok(NULL, FLASH_TOOL_TOKEN_DELIMITER) != NULL)
    {
        FlashTool_SendToolError(FLASH_TOOL_ERROR_INVALID_ARG);
        return;
    }

    FlashTool_SendText("OK EXIT\r\n");
    rx_index = 0U;
    line_overflow = false;
    flash_tool_active = false;
}

static void FlashTool_SendText(const char *text)
{
    size_t text_len_byte;

    if (text == NULL)
    {
        return;
    }
    text_len_byte = strlen(text);
    if ((text_len_byte == 0U) || (text_len_byte > UINT16_MAX))
    {
        return;
    }

    (void)CommRuntime_PcTransmitBlocking((const uint8_t *)text,
                                         (uint16_t)text_len_byte,
                                         FLASH_TOOL_TX_TIMEOUT_MS);
}

static void FlashTool_SendToolError(flash_tool_error_t status)
{
    (void)snprintf(response_text,
                   sizeof(response_text),
                   "ERR TOOL=%u HINT=SEND_HELP\r\n",
                   (unsigned int)status);
    FlashTool_SendText(response_text);
}

static void FlashTool_SendFlashError(w25q_status_t status)
{
    (void)snprintf(response_text,
                   sizeof(response_text),
                   "ERR FLASH=%u NAME=%s\r\n",
                   (unsigned int)status,
                   FlashTool_GetStatusName(status));
    FlashTool_SendText(response_text);
}

static bool FlashTool_LooksLikeCommand(const uint8_t *data, size_t size)
{
    static const char *const commands[] =
    {
        "FLASH", "HELP", "INFO", "ERASE", "WRITE", "READ", "EXIT"
    };
    size_t data_index = 0U;
    size_t command_index;

    while ((data_index < size) &&
           ((data[data_index] == ' ') || (data[data_index] == '\t') ||
            (data[data_index] == '\r') || (data[data_index] == '\n')))
    {
        data_index++;
    }
    if (data_index >= size)
    {
        return false;
    }

    for (command_index = 0U;
         command_index < (sizeof(commands) / sizeof(commands[0]));
         command_index++)
    {
        size_t input_index = data_index;
        size_t name_index = 0U;

        while ((input_index < size) &&
               (data[input_index] != ' ') &&
               (data[input_index] != '\t') &&
               (data[input_index] != '\r') &&
               (data[input_index] != '\n') &&
               (commands[command_index][name_index] != '\0'))
        {
            char input_char = (char)data[input_index];
            char name_char = commands[command_index][name_index];

            if ((input_char >= 'a') && (input_char <= 'z'))
            {
                input_char = (char)(input_char - 'a' + 'A');
            }
            if (input_char != name_char)
            {
                break;
            }
            input_index++;
            name_index++;
        }
        if ((input_index == size) ||
            (data[input_index] == ' ') ||
            (data[input_index] == '\t') ||
            (data[input_index] == '\r') ||
            (data[input_index] == '\n'))
        {
            if (name_index > 0U)
            {
                return true;
            }
        }
    }

    return false;
}

static bool FlashTool_ParseNumber(const char *text,
                                  uint8_t default_base,
                                  uint32_t *value)
{
    uint32_t result = 0U;
    uint32_t digit;
    uint8_t base = default_base;
    uint16_t text_index = 0U;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0'))
    {
        return false;
    }
    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X')))
    {
        base = FLASH_TOOL_HEX_BASE;
        text_index = 2U;
        if (text[text_index] == '\0')
        {
            return false;
        }
    }

    while (text[text_index] != '\0')
    {
        char character = text[text_index];

        if ((character >= '0') && (character <= '9'))
        {
            digit = (uint32_t)(character - '0');
        }
        else if ((character >= 'A') && (character <= 'F'))
        {
            digit = (uint32_t)(character - 'A') + 10U;
        }
        else if ((character >= 'a') && (character <= 'f'))
        {
            digit = (uint32_t)(character - 'a') + 10U;
        }
        else
        {
            return false;
        }
        if ((digit >= base) || (result > ((UINT32_MAX - digit) / base)))
        {
            return false;
        }
        result = result * base + digit;
        text_index++;
    }

    *value = result;
    return true;
}

static bool FlashTool_ParseHexByte(const char *text, uint8_t *value)
{
    uint32_t parsed_value;

    if (!FlashTool_ParseNumber(text, FLASH_TOOL_HEX_BASE, &parsed_value) ||
        (parsed_value > UINT8_MAX))
    {
        return false;
    }
    *value = (uint8_t)parsed_value;
    return true;
}

static bool FlashTool_StringEquals(const char *left, const char *right)
{
    char left_char;
    char right_char;

    while ((*left != '\0') && (*right != '\0'))
    {
        left_char = *left;
        right_char = *right;
        if ((left_char >= 'a') && (left_char <= 'z'))
        {
            left_char = (char)(left_char - 'a' + 'A');
        }
        if ((right_char >= 'a') && (right_char <= 'z'))
        {
            right_char = (char)(right_char - 'a' + 'A');
        }
        if (left_char != right_char)
        {
            return false;
        }
        left++;
        right++;
    }

    return (*left == '\0') && (*right == '\0');
}

static const char *FlashTool_GetDeviceName(uint32_t flash_id)
{
    switch (flash_id)
    {
    case W25Q16_ID:
        return "W25Q16";
    case W25Q32_ID:
        return "W25Q32";
    case W25Q64_ID:
        return "W25Q64";
    case W25Q128_ID:
        return "W25Q128";
    default:
        return "UNKNOWN";
    }
}

static const char *FlashTool_GetStatusName(w25q_status_t status)
{
    switch (status)
    {
    case W25Q_OK:
        return "W25Q_OK";
    case W25Q_ERROR_INVALID_ARG:
        return "W25Q_ERROR_INVALID_ARG";
    case W25Q_ERROR_NOT_INIT:
        return "W25Q_ERROR_NOT_INIT";
    case W25Q_ERROR_UNSUPPORTED_DEVICE:
        return "W25Q_ERROR_UNSUPPORTED_DEVICE";
    case W25Q_ERROR_OUT_OF_RANGE:
        return "W25Q_ERROR_OUT_OF_RANGE";
    case W25Q_ERROR_NOT_ALIGNED:
        return "W25Q_ERROR_NOT_ALIGNED";
    case W25Q_ERROR_TIMEOUT:
        return "W25Q_ERROR_TIMEOUT";
    case W25Q_ERROR_SPI:
        return "W25Q_ERROR_SPI";
    case W25Q_ERROR_WRITE_ENABLE:
        return "W25Q_ERROR_WRITE_ENABLE";
    case W25Q_ERROR_LOCK_TIMEOUT:
        return "W25Q_ERROR_LOCK_TIMEOUT";
    default:
        return "W25Q_ERROR_UNKNOWN";
    }
}
