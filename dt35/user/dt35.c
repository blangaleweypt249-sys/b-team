#include "dt35.h"

// ========== INA226 芯片内部寄存器地址定义 ==========
#define INA226_REG_CONFIG      0x00U   // 配置寄存器：改芯片工作模式、采样速度等
#define INA226_REG_BUS         0x02U   // 总线电压寄存器：存放测到的电压原始值（要读的核心数据）
#define INA226_REG_MAN_ID      0xFEU   // 厂商ID寄存器：用来验证芯片是不是正品
#define INA226_REG_DIE_ID      0xFFU   // 芯片ID寄存器：验证是不是INA226

// ========== 寄存器配置数值定义 ==========
#define INA226_RESET           0x8000U // 复位命令：写入配置寄存器后芯片会重启
#define INA226_CONFIG          0x4526U // 默认配置值：设置连续采样、平均次数、转换时间等
#define INA226_MAN_ID          0x5449U // 正确的厂商ID值（TI厂家固定值）
#define INA226_DIE_ID          0x2260U // 正确的芯片ID值（INA226固定值）
#define INA226_DIE_MASK        0xFFF0U // ID掩码：只对比高12位，低4位忽略

// ========== 超时与延时参数 ==========
#define DT35_I2C_TIMEOUT_MS    20U     // I2C通信超时时间，单位毫秒，超过还没应答就报错
#define DT35_STARTUP_MS        20U     // 芯片配置后等待启动稳定的时间

static I2C_HandleTypeDef *dt35_i2c;
// DMA接收缓冲区：INA226寄存器是16位（2字节），DMA把读到的数据直接存在这里
static uint8_t rx_data[2];
static volatile uint8_t rx_busy;           
static volatile uint8_t rx_ready;
static volatile HAL_StatusTypeDef rx_state;

static HAL_StatusTypeDef RegWrite(uint8_t reg, uint16_t value);  // 写INA226寄存器（轮询方式）
static HAL_StatusTypeDef RegRead(uint8_t reg, uint16_t *value);  // 读INA226寄存器（轮询方式）
static float ToDistance(uint16_t voltage_mv);                    // 电压值换算成距离值

HAL_StatusTypeDef DT35_Init(I2C_HandleTypeDef *i2c)
{
    HAL_StatusTypeDef state;
    uint16_t id;

    if (i2c == NULL)
    {
        return HAL_ERROR;
    }

    dt35_i2c = i2c;
    rx_busy = 0U;
    rx_ready = 0U;
    rx_state = HAL_ERROR;

	// 检查I2C总线上有没有这个设备（俗称“扫设备”）
    // 参数：I2C句柄、从机地址（左移1位是HAL库要求，I2C地址格式）、尝试次数、超时时间
    state = HAL_I2C_IsDeviceReady(dt35_i2c, DT35_I2C_ADDR << 1U,
                                  2U, DT35_I2C_TIMEOUT_MS);
    if (state != HAL_OK)
    {
        return state;
    }

    state = RegWrite(INA226_REG_CONFIG, INA226_RESET);
    if (state != HAL_OK)
    {
        return state;
    }
    HAL_Delay(2U);

    state = RegRead(INA226_REG_MAN_ID, &id);
    if ((state != HAL_OK) || (id != INA226_MAN_ID))
    {
        return HAL_ERROR;
    }

    state = RegRead(INA226_REG_DIE_ID, &id);
    if ((state != HAL_OK) || ((id & INA226_DIE_MASK) != INA226_DIE_ID))
    {
        return HAL_ERROR;
    }
    //写入我们的配置值，让芯片开始连续测量电压
    state = RegWrite(INA226_REG_CONFIG, INA226_CONFIG);
    if (state == HAL_OK)
    {
        HAL_Delay(DT35_STARTUP_MS);
    }

    return state;
}

HAL_StatusTypeDef DT35_Read(void)
{
    HAL_StatusTypeDef state;

    if (dt35_i2c == NULL)
    {
        return HAL_ERROR;
    }
    if ((rx_busy != 0U) || (rx_ready != 0U))
    {
        return HAL_BUSY;
    }

    rx_busy = 1U;
    rx_state = HAL_BUSY;
	// 核心：发起DMA方式的寄存器读取
    // 参数：I2C句柄、从机地址、要读的寄存器地址、寄存器地址长度、接收缓冲区、要读的字节数
    state = HAL_I2C_Mem_Read_DMA(dt35_i2c, DT35_I2C_ADDR << 1U,
                                 INA226_REG_BUS, I2C_MEMADD_SIZE_8BIT,
                                 rx_data, 2U);
    if (state != HAL_OK)
    {
        rx_busy = 0U;
        rx_state = state;
    }

    return state;
}

HAL_StatusTypeDef DT35_Get(DT35_Data *data)
{
    uint16_t raw;
    uint16_t voltage_mv;

    if (data == NULL)
    {
        return HAL_ERROR;
    }
    if (rx_ready == 0U)
    {
        return HAL_BUSY;
    }

    rx_ready = 0U;
    if (rx_state != HAL_OK)
    {
        return rx_state;
    }

    raw = ((uint16_t)rx_data[0] << 8U) | rx_data[1];

    /* INA226 bus-voltage LSB is 1.25 mV. */
    voltage_mv = (uint16_t)(((uint32_t)raw * 5U + 2U) / 4U);

    data->raw = raw;
    data->voltage_mv = voltage_mv;
    data->distance_cm = ToDistance(voltage_mv);

    return HAL_OK;
}

static float ToDistance(uint16_t voltage_mv)
{
    float voltage_span;
    float distance_span;

    if (voltage_mv <= DT35_VOLT_NEAR_MV)
    {
        return DT35_DIST_NEAR_CM;
    }
    if (voltage_mv >= DT35_VOLT_FAR_MV)
    {
        return DT35_DIST_FAR_CM;
    }

    voltage_span = (float)(DT35_VOLT_FAR_MV - DT35_VOLT_NEAR_MV);
    distance_span = DT35_DIST_FAR_CM - DT35_DIST_NEAR_CM;
    
	// 线性插值公式：实际距离 = 近点距离 + (当前电压占比 × 距离总范围)
    return DT35_DIST_NEAR_CM +
           (float)(voltage_mv - DT35_VOLT_NEAR_MV) * distance_span /
           voltage_span;
}

// 存储器读取完成回调：DMA读数据成功结束时自动调用
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *i2c)
{
    if ((i2c == dt35_i2c) && (rx_busy != 0U))
    {
        rx_state = HAL_OK;
        rx_busy = 0U;
        rx_ready = 1U;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{
    if ((i2c == dt35_i2c) && (rx_busy != 0U))
    {
        rx_state = HAL_ERROR;
        rx_busy = 0U;
        rx_ready = 1U;
    }
}

static HAL_StatusTypeDef RegWrite(uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;

    return HAL_I2C_Mem_Write(dt35_i2c, DT35_I2C_ADDR << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             DT35_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef RegRead(uint8_t reg, uint16_t *value)
{
    HAL_StatusTypeDef state;
    uint8_t data[2];

    state = HAL_I2C_Mem_Read(dt35_i2c, DT35_I2C_ADDR << 1U, reg,
                            I2C_MEMADD_SIZE_8BIT, data, 2U,
                            DT35_I2C_TIMEOUT_MS);
    if (state == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8U) | data[1];
    }

    return state;
}
