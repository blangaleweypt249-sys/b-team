/**
 * @file W25Qxx.c
 * @brief W25Q16 至 W25Q128 外部 Flash 通用驱动实现
 */

#include "W25Qxx.h"

#include <stddef.h>
#include <stdint.h>

#define W25Q_CMD_WRITE_ENABLE          0x06U /**< W25Qxx 允许后续写入或擦除操作的指令码。 */
#define W25Q_CMD_READ_STATUS_REG1      0x05U /**< W25Qxx 读取状态寄存器一的指令码。 */
#define W25Q_CMD_READ_DATA             0x03U /**< W25Qxx 从指定地址连续读取数据的指令码。 */
#define W25Q_CMD_PAGE_PROGRAM          0x02U /**< W25Qxx 对指定页执行编程写入的指令码。 */
#define W25Q_CMD_SECTOR_ERASE          0x20U /**< W25Qxx 擦除指定扇区的指令码。 */
#define W25Q_CMD_JEDEC_ID              0x9FU /**< W25Qxx 读取 JEDEC 厂商、类型和容量标识的指令码。 */
#define W25Q_CMD_DEVICE_ID             0x90U /**< W25Qxx 读取厂商及器件标识的指令码。 */

#define W25Q_STATUS_BUSY_MASK          0x01U /**< 从状态寄存器一提取芯片忙状态的位掩码。 */
#define W25Q_STATUS_WEL_MASK           0x02U /**< 从状态寄存器一提取写使能锁存状态的位掩码。 */
#define W25Q_DUMMY_BYTE                0xFFU /**< SPI 只读传输时由主机发送的占位字节。 */
#define W25Q_KB_SIZE_BYTE              1024UL /**< 一 KB 对应的字节数，用于 Flash 容量换算。 */
#define W25Q_ADDR_HIGH_SHIFT           16U /**< 把 24 位 Flash 地址的高字节移到发送位置所需的位数。 */
#define W25Q_ADDR_MID_SHIFT            8U /**< 把 24 位 Flash 地址的中字节移到发送位置所需的位数。 */
#define W25Q_ID_MFR_SHIFT              16U /**< 组合器件标识时厂商编号需要左移的位数。 */
#define W25Q_ID_TYPE_SHIFT             8U /**< 组合 JEDEC 标识时存储器类型需要左移的位数。 */
#define W25Q_JEDEC_TRANSFER_SIZE_BYTE  5U /**< 读取 JEDEC 标识时 SPI 命令和返回数据合计传输的字节数。 */
#define W25Q_JEDEC_MFR_INDEX           1U /**< JEDEC 返回数据中厂商标识所在的字节下标。 */
#define W25Q_JEDEC_TYPE_INDEX          2U /**< JEDEC 返回数据中存储器类型所在的字节下标。 */
#define W25Q_JEDEC_CAPACITY_INDEX      3U /**< JEDEC 返回数据中容量编码所在的字节下标。 */
#define W25Q_DEVICE_ID_TRANSFER_SIZE   6U /**< 读取器件标识时 SPI 命令、地址和返回数据合计传输的字节数。 */
#define W25Q_DEVICE_MFR_INDEX          4U /**< 器件标识返回数据中厂商编号所在的字节下标。 */
#define W25Q_DEVICE_CODE_INDEX         5U /**< 器件标识返回数据中型号编号所在的字节下标。 */
#define W25Q_DEVICE_MFR_SHIFT          8U /**< 组合器件标识时厂商编号需要左移的位数。 */

#define W25Q16_CAPACITY_KB             2048UL /**< 16 Mbit W25Qxx 器件的存储容量，单位：KB。 */
#define W25Q32_CAPACITY_KB             4096UL /**< 32 Mbit W25Qxx 器件的存储容量，单位：KB。 */
#define W25Q64_CAPACITY_KB             8192UL /**< 64 Mbit W25Qxx 器件的存储容量，单位：KB。 */
#define W25Q128_CAPACITY_KB            16384UL /**< 128 Mbit W25Qxx 器件的存储容量，单位：KB。 */

/* 功能：声明拉低 Flash 片选信号的内部接口。 */
static void W25Q_CsSelect(void);
/* 功能：声明拉高 Flash 片选信号的内部接口。 */
static void W25Q_CsDeselect(void);
/* 功能：声明 SPI 单字节同步收发接口；返回 true 表示收发成功。 */
static bool W25Q_SpiRwByte(uint8_t tx_data /**< SPI发送字节 */,
                           uint8_t *rx_data /**< 用于写出SPI接收字节的地址 */);
/* 功能：声明 SPI 批量同步传输接口；返回 true 表示传输成功。 */
static bool W25Q_SpiTransfer(const uint8_t *tx_data /**< SPI 连续发送缓冲区首地址 */,
                             uint8_t *rx_data /**< SPI 连续接收缓冲区首地址 */,
                             uint16_t data_len_byte /**< SPI 连续传输的字节数 */);
/* 功能：声明毫秒延时接口。 */
static void W25Q_DelayMs(uint32_t delay_ms /**< 延时时间，单位：毫秒 */);
/* 功能：声明设备句柄有效性检查接口；返回 true 表示句柄及底层回调可用。 */
static bool W25Q_IsHandleValid(const w25q_handle_t *dev /**< 待校验的W25Q Flash设备句柄 */);
/* 功能：声明设备状态检查接口；返回值表示设备是否已完成初始化。 */
static w25q_status_t W25Q_CheckDevice(const w25q_handle_t *dev /**< 待检查初始化状态的W25Q Flash设备句柄 */);
/* 功能：声明访问地址范围检查接口；返回值表示目标区间是否合法。 */
static w25q_status_t W25Q_CheckRange(const w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                     uint32_t start_addr /**< Flash访问起始地址 */,
                                     uint32_t data_len_byte /**< 待检查 Flash 地址范围的字节数 */);
/* 功能：声明设备总线加锁接口；返回值表示加锁结果。 */
static w25q_status_t W25Q_Lock(w25q_handle_t *dev /**< 需要取得总线锁的W25Q Flash设备句柄 */);
/* 功能：声明设备总线解锁接口。 */
static void W25Q_Unlock(w25q_handle_t *dev /**< 需要释放总线锁的W25Q Flash设备句柄 */);
/* 功能：声明经设备端口收发单字节的接口；返回值表示传输结果。 */
static w25q_status_t W25Q_TransferByte(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                       uint8_t tx_data /**< SPI发送字节 */,
                                       uint8_t *rx_data /**< 用于写出SPI接收字节的地址 */);
/* 功能：声明向 Flash 发送 24 位地址的接口；返回值表示发送结果。 */
static w25q_status_t W25Q_SendAddress(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                      uint32_t address /**< 待发送的24位Flash地址 */);
/* 功能：声明读取 Flash 状态寄存器的接口；返回值表示读取结果。 */
static w25q_status_t W25Q_ReadStatus(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                     uint8_t *status_reg /**< 用于写出Flash状态寄存器值的地址 */);
/* 功能：声明读取 JEDEC 标识的接口；返回值表示读取结果。 */
static w25q_status_t W25Q_ReadJedecId(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                      uint32_t *jedec_id /**< 用于写出JEDEC标识的地址 */);
/* 功能：声明在总线已加锁时读取器件标识的接口；返回值表示读取结果。 */
static w25q_status_t W25Q_ReadDeviceIdLocked(w25q_handle_t *dev /**< 已取得总线锁的W25Q Flash设备句柄 */,
                                              uint16_t *device_id /**< 用于写出制造商和器件组合标识的地址 */);
/* 功能：声明等待 Flash 结束忙状态的接口；返回值表示等待结果。 */
static w25q_status_t W25Q_WaitReady(w25q_handle_t *dev /**< 等待结束忙状态的W25Q Flash设备句柄 */);
/* 功能：声明发送写使能命令的接口；返回值表示写使能结果。 */
static w25q_status_t W25Q_WriteEnable(w25q_handle_t *dev /**< 需要发送写使能命令的W25Q Flash设备句柄 */);
/* 功能：声明连续读取 Flash 数据的内部接口；返回值表示读取结果。 */
static w25q_status_t W25Q_ReadBytes(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                    uint32_t read_addr /**< Flash读取起始地址 */,
                                    uint8_t *data /**< Flash读取数据的输出缓冲区 */,
                                    uint32_t data_len_byte /**< Flash连续读取字节数 */);
/* 功能：声明单页编程接口；返回值表示页写入结果。 */
static w25q_status_t W25Q_WritePage(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                    uint32_t write_addr /**< 页编程起始地址 */,
                                    const uint8_t *data /**< 待写入当前Flash页的数据缓冲区 */,
                                    uint32_t data_len_byte /**< 当前Flash页的写入字节数 */);
/* 功能：声明发送扇区擦除命令的接口；返回值表示命令执行结果。 */
static w25q_status_t W25Q_EraseCommand(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                       uint32_t sector_addr /**< 待发送擦除命令的扇区起始地址 */);
/* 功能：声明在总线已加锁时擦除扇区的接口；返回值表示擦除结果。 */
static w25q_status_t W25Q_EraseSectorLocked(w25q_handle_t *dev /**< 已取得总线锁的W25Q Flash设备句柄 */,
                                             uint32_t sector_addr /**< 持锁擦除的扇区起始地址 */);
/* 功能：声明在总线已加锁时连续写入数据的接口；返回值表示写入结果。 */
static w25q_status_t W25Q_ProgramDataLocked(w25q_handle_t *dev /**< 已取得总线锁的W25Q Flash设备句柄 */,
                                             uint32_t write_addr /**< Flash写入起始地址 */,
                                             const uint8_t *data /**< 待连续写入Flash的数据缓冲区 */,
                                             uint32_t data_len_byte /**< Flash连续写入字节数 */);
/* 功能：声明在总线已加锁时更新扇区部分数据的接口；返回值表示更新结果。 */
static w25q_status_t W25Q_UpdateSectorLocked(w25q_handle_t *dev /**< 已取得总线锁的W25Q Flash设备句柄 */,
                                              uint32_t sector_addr /**< 待保留并改写数据的扇区起始地址 */,
                                              uint32_t sector_offset /**< 写入数据在扇区内的偏移 */,
                                              const uint8_t *data /**< 待更新到当前Flash扇区的数据缓冲区 */,
                                              uint32_t data_len_byte /**< 当前Flash扇区的更新字节数 */);

static w25q_handle_t w25q_device =
{
    .cs_select = W25Q_CsSelect,
    .cs_deselect = W25Q_CsDeselect,
    .spi_rw_byte = W25Q_SpiRwByte,
    .spi_transfer = W25Q_SpiTransfer,
    .delay_ms = W25Q_DelayMs,
    .bus_lock = NULL,
    .bus_unlock = NULL
};
static w25q_status_t w25q_port_init_status = W25Q_ERROR_NOT_INIT;

/* 自动擦除写入时用于保存并恢复同一扇区内未修改的数据。 */
static uint8_t w25q_sector_buffer[W25Q_SECTOR_SIZE_BYTE];

/**
 * @brief 绑定头文件中的 MCU 配置并初始化 W25Qxx
 * @retval W25Q_OK 初始化成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_PortInit(void)
{
    w25q_port_init_status = W25Q_Init(&w25q_device);
    return w25q_port_init_status;
}

/**
 * @brief 获取当前工程的 W25Qxx 设备句柄
 * @retval W25Qxx 设备句柄地址
 */
w25q_handle_t *W25Q_PortGetDevice(void)
{
    return &w25q_device;
}

/* 功能：读取 W25Q 板级端口初始化状态；用途：向上层报告 Flash 初始化结果；返回值表示当前初始化状态。 */
w25q_status_t W25Q_PortGetInitStatus(void)
{
    return w25q_port_init_status;
}

/**
 * @brief 使用 0x90 命令读取制造商和器件 ID
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadDeviceId(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                uint16_t *device_id /**< 用于写出制造商和器件组合标识的地址 */)
{
    w25q_status_t status;

    if ((device_id == NULL) || !W25Q_IsHandleValid(dev))
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    status = W25Q_ReadDeviceIdLocked(dev, device_id);
    W25Q_Unlock(dev);
    return status;
}

/**
 * @brief 拉低 W25Qxx 片选引脚
 * @retval 无
 */
static void W25Q_CsSelect(void)
{
    W25Q_MCU_CS_SELECT();
}

/**
 * @brief 拉高 W25Qxx 片选引脚
 * @retval 无
 */
static void W25Q_CsDeselect(void)
{
    W25Q_MCU_CS_DESELECT();
}

/**
 * @brief 使用头文件配置的 SPI 同步收发一个字节
 * @retval true SPI 收发成功
 * @retval false SPI 收发失败
 */
static bool W25Q_SpiRwByte(uint8_t tx_data /**< SPI 传输时发送的字节 */, uint8_t *rx_data /**< 用于写出 SPI 接收字节的缓冲区 */)
{
    return W25Q_MCU_SPI_RW_BYTE(&tx_data, rx_data);
}

/* 功能：通过端口层执行一段 SPI 全双工传输；用途：为 Flash 驱动提供批量收发适配；返回 true 表示底层传输成功。 */
static bool W25Q_SpiTransfer(const uint8_t *tx_data /**< SPI 连续发送缓冲区首地址 */,
                             uint8_t *rx_data /**< SPI 连续接收缓冲区首地址 */,
                             uint16_t data_len_byte /**< SPI 连续传输的字节数 */)
{
    return W25Q_MCU_SPI_TRANSFER(tx_data, rx_data, data_len_byte);
}

/**
 * @brief 根据 W25Q_DELAY_MODE 选择毫秒延时函数
 * @retval 无
 */
static void W25Q_DelayMs(uint32_t delay_ms /**< 需要等待的时间，单位：毫秒 */)
{
#if (W25Q_DELAY_MODE == W25Q_DELAY_USE_HAL)
    W25Q_MCU_HAL_DELAY_MS(delay_ms);
#else
    W25Q_MCU_OS_DELAY_MS(delay_ms);
#endif
}

/**
 * @brief 初始化 W25Qxx 设备并读取 JEDEC ID
 * @retval W25Q_OK 初始化成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_Init(w25q_handle_t *dev /**< 待初始化的W25Q Flash设备句柄 */)
{
    w25q_status_t status;
    uint32_t jedec_id = 0U;

    if (dev == NULL)
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    dev->flash_id = 0U;
    dev->capacity_kb = 0U;
    dev->sector_count = 0U;
    dev->page_size_byte = 0U;
    dev->is_initialized = false;

    if (!W25Q_IsHandleValid(dev))
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    dev->cs_deselect();
    status = W25Q_ReadJedecId(dev, &jedec_id);
    W25Q_Unlock(dev);

    if (status != W25Q_OK)
    {
        return status;
    }

    dev->flash_id = jedec_id;
    switch (dev->flash_id)
    {
    case W25Q16_ID:
        dev->capacity_kb = W25Q16_CAPACITY_KB;
        break;

    case W25Q32_ID:
        dev->capacity_kb = W25Q32_CAPACITY_KB;
        break;

    case W25Q64_ID:
        dev->capacity_kb = W25Q64_CAPACITY_KB;
        break;

    case W25Q128_ID:
        dev->capacity_kb = W25Q128_CAPACITY_KB;
        break;

    default:
        return W25Q_ERROR_UNSUPPORTED_DEVICE;
    }

    dev->sector_count = (dev->capacity_kb * W25Q_KB_SIZE_BYTE) /
                        W25Q_SECTOR_SIZE_BYTE;
    dev->page_size_byte = W25Q_PAGE_SIZE_BYTE;
    dev->is_initialized = true;

    return W25Q_OK;
}

/**
 * @brief 擦除指定的 4 KB 扇区
 * @retval W25Q_OK 擦除成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_EraseSector(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                               uint32_t sector_addr /**< 待擦除扇区的起始地址 */)
{
    w25q_status_t status;

    status = W25Q_CheckDevice(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    if ((sector_addr % W25Q_SECTOR_SIZE_BYTE) != 0U)
    {
        return W25Q_ERROR_NOT_ALIGNED;
    }

    status = W25Q_CheckRange(dev,
                             sector_addr,
                             W25Q_SECTOR_SIZE_BYTE);
    if (status != W25Q_OK)
    {
        return status;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    status = W25Q_EraseSectorLocked(dev, sector_addr);

    W25Q_Unlock(dev);
    return status;
}

/**
 * @brief 从指定扇区首地址开始擦除若干个完整的 4 KB 扇区
 * @retval W25Q_OK 擦除成功，数量为 0 时不执行操作
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_EraseSectors(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                                uint32_t sector_addr /**< 连续擦除区域的首个扇区地址 */,
                                uint32_t sector_count /**< 本次连续擦除的扇区数量 */)
{
    w25q_status_t status;
    uint32_t first_sector_index;
    uint32_t erased_sector_count;

    status = W25Q_CheckDevice(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    if ((sector_addr % W25Q_SECTOR_SIZE_BYTE) != 0U)
    {
        return W25Q_ERROR_NOT_ALIGNED;
    }

    status = W25Q_CheckRange(dev, sector_addr, 0U);
    if ((status != W25Q_OK) || (sector_count == 0U))
    {
        return status;
    }

    first_sector_index = sector_addr / W25Q_SECTOR_SIZE_BYTE;
    if ((first_sector_index >= dev->sector_count) ||
        (sector_count > (dev->sector_count - first_sector_index)))
    {
        return W25Q_ERROR_OUT_OF_RANGE;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    for (erased_sector_count = 0U;
         (erased_sector_count < sector_count) && (status == W25Q_OK);
         erased_sector_count++)
    {
        status = W25Q_EraseSectorLocked(dev, sector_addr);
        sector_addr += W25Q_SECTOR_SIZE_BYTE;
    }

    W25Q_Unlock(dev);
    return status;
}

/**
 * @brief 从 Flash 连续读取数据
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadData(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                            uint32_t read_addr /**< Flash读取起始地址 */,
                            uint8_t *data /**< Flash读取数据的输出缓冲区 */,
                            uint32_t data_len_byte /**< Flash连续读取字节数 */)
{
    w25q_status_t status;

    status = W25Q_CheckDevice(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    if ((data == NULL) && (data_len_byte > 0U))
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    status = W25Q_CheckRange(dev, read_addr, data_len_byte);
    if ((status != W25Q_OK) || (data_len_byte == 0U))
    {
        return status;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    status = W25Q_WaitReady(dev);
    if (status == W25Q_OK)
    {
        status = W25Q_ReadBytes(dev,
                                read_addr,
                                data,
                                data_len_byte);
    }

    W25Q_Unlock(dev);
    return status;
}

/**
 * @brief 向 Flash 写入数据，自动处理页边界和必要的扇区擦除
 * @retval W25Q_OK 写入成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_WriteData(w25q_handle_t *dev /**< W25Q Flash设备句柄 */,
                             uint32_t write_addr /**< Flash写入起始地址 */,
                             const uint8_t *data /**< 待写入Flash的数据缓冲区 */,
                             uint32_t data_len_byte /**< Flash连续写入字节数 */)
{
    w25q_status_t status;
    uint32_t sector_addr;
    uint32_t sector_offset;
    uint32_t sector_write_len_byte;

    status = W25Q_CheckDevice(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    if ((data == NULL) && (data_len_byte > 0U))
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    status = W25Q_CheckRange(dev, write_addr, data_len_byte);
    if ((status != W25Q_OK) || (data_len_byte == 0U))
    {
        return status;
    }

    status = W25Q_Lock(dev);
    if (status != W25Q_OK)
    {
        return status;
    }

    while ((data_len_byte > 0U) && (status == W25Q_OK))
    {
        sector_addr = write_addr -
                      (write_addr % W25Q_SECTOR_SIZE_BYTE);
        sector_offset = write_addr - sector_addr;
        sector_write_len_byte = W25Q_SECTOR_SIZE_BYTE - sector_offset;
        if (sector_write_len_byte > data_len_byte)
        {
            sector_write_len_byte = data_len_byte;
        }

        status = W25Q_UpdateSectorLocked(dev,
                                         sector_addr,
                                         sector_offset,
                                         data,
                                         sector_write_len_byte);
        if (status == W25Q_OK)
        {
            write_addr += sector_write_len_byte;
            data += sector_write_len_byte;
            data_len_byte -= sector_write_len_byte;
        }
    }

    W25Q_Unlock(dev);
    return status;
}

/**
 * @brief 检查设备句柄及底层接口
 * @retval true 句柄有效
 * @retval false 句柄无效
 */
static bool W25Q_IsHandleValid(const w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    bool lock_pair_valid;

    if (dev == NULL)
    {
        return false;
    }

    lock_pair_valid = (((dev->bus_lock == NULL) &&
                        (dev->bus_unlock == NULL)) ||
                       ((dev->bus_lock != NULL) &&
                        (dev->bus_unlock != NULL)));

    return ((dev->cs_select != NULL) &&
            (dev->cs_deselect != NULL) &&
            (dev->spi_rw_byte != NULL) &&
            (dev->spi_transfer != NULL) &&
            (dev->delay_ms != NULL) &&
            lock_pair_valid);
}

/**
 * @brief 检查设备是否可以执行读写操作
 * @retval W25Q_OK 设备可用
 * @retval 其他值 设备句柄无效或尚未初始化
 */
static w25q_status_t W25Q_CheckDevice(const w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    if (!W25Q_IsHandleValid(dev))
    {
        return W25Q_ERROR_INVALID_ARG;
    }

    if (!dev->is_initialized)
    {
        return W25Q_ERROR_NOT_INIT;
    }

    return W25Q_OK;
}

/**
 * @brief 检查访问范围是否位于芯片容量内
 * @retval W25Q_OK 地址范围有效
 * @retval W25Q_ERROR_OUT_OF_RANGE 地址范围越界
 */
static w25q_status_t W25Q_CheckRange(const w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                     uint32_t start_addr /**< 待检查数据区的起始字节地址 */,
                                     uint32_t data_len_byte /**< 待检查 Flash 地址范围的字节数 */)
{
    uint32_t capacity_byte;

    capacity_byte = dev->capacity_kb * W25Q_KB_SIZE_BYTE;
    if (start_addr > capacity_byte)
    {
        return W25Q_ERROR_OUT_OF_RANGE;
    }

    if (data_len_byte > (capacity_byte - start_addr))
    {
        return W25Q_ERROR_OUT_OF_RANGE;
    }

    return W25Q_OK;
}

/**
 * @brief 获取可选的 SPI 总线锁
 * @retval W25Q_OK 获取成功或未配置锁
 * @retval W25Q_ERROR_LOCK_TIMEOUT 获取锁超时
 */
static w25q_status_t W25Q_Lock(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    if ((dev->bus_lock != NULL) &&
        (!dev->bus_lock(W25Q_LOCK_TIMEOUT_MS)))
    {
        return W25Q_ERROR_LOCK_TIMEOUT;
    }

    return W25Q_OK;
}

/**
 * @brief 释放可选的 SPI 总线锁
 * @retval 无
 */
static void W25Q_Unlock(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    if (dev->bus_unlock != NULL)
    {
        dev->bus_unlock();
    }
}

/**
 * @brief 同步收发一个 SPI 字节并转换底层错误
 * @retval W25Q_OK 收发成功
 * @retval W25Q_ERROR_SPI 收发失败
 */
static w25q_status_t W25Q_TransferByte(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                       uint8_t tx_data /**< SPI 传输时发送的字节 */,
                                       uint8_t *rx_data /**< 用于写出 SPI 接收字节的缓冲区 */)
{
    uint8_t discard_data = 0U;
    uint8_t *receive_data = rx_data;

    if (receive_data == NULL)
    {
        receive_data = &discard_data;
    }

    if (!dev->spi_rw_byte(tx_data, receive_data))
    {
        return W25Q_ERROR_SPI;
    }

    return W25Q_OK;
}

/**
 * @brief 发送三字节 Flash 地址
 * @retval W25Q_OK 发送成功
 * @retval W25Q_ERROR_SPI 发送失败
 */
static w25q_status_t W25Q_SendAddress(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                      uint32_t address /**< Flash 操作的起始字节地址 */)
{
    w25q_status_t status;

    status = W25Q_TransferByte(dev,
                               (uint8_t)(address >> W25Q_ADDR_HIGH_SHIFT),
                               NULL);
    if (status == W25Q_OK)
    {
        status = W25Q_TransferByte(dev,
                                   (uint8_t)(address >> W25Q_ADDR_MID_SHIFT),
                                   NULL);
    }
    if (status == W25Q_OK)
    {
        status = W25Q_TransferByte(dev,
                                   (uint8_t)address,
                                   NULL);
    }

    return status;
}

/**
 * @brief 读取状态寄存器 1
 * @retval W25Q_OK 读取成功
 * @retval W25Q_ERROR_SPI 读取失败
 */
static w25q_status_t W25Q_ReadStatus(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                     uint8_t *status_reg /**< 用于写出 W25Q 状态寄存器值 */)
{
    w25q_status_t status;

    dev->cs_select();
    status = W25Q_TransferByte(dev,
                               W25Q_CMD_READ_STATUS_REG1,
                               NULL);
    if (status == W25Q_OK)
    {
        status = W25Q_TransferByte(dev,
                                   W25Q_DUMMY_BYTE,
                                   status_reg);
    }
    dev->cs_deselect();

    return status;
}

/**
 * @brief 读取三字节 JEDEC ID
 * @retval W25Q_OK 读取成功
 * @retval W25Q_ERROR_SPI 读取失败
 */
static w25q_status_t W25Q_ReadJedecId(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                      uint32_t *jedec_id /**< Flash 的 JEDEC 器件标识 */)
{
    w25q_status_t status;
    uint8_t tx_data[W25Q_JEDEC_TRANSFER_SIZE_BYTE] =
    {
        W25Q_CMD_JEDEC_ID,
        W25Q_DUMMY_BYTE,
        W25Q_DUMMY_BYTE,
        W25Q_DUMMY_BYTE,
        W25Q_DUMMY_BYTE
    };
    uint8_t rx_data[W25Q_JEDEC_TRANSFER_SIZE_BYTE] = {0U};

    dev->cs_select();
    status = dev->spi_transfer(tx_data,
                               rx_data,
                               W25Q_JEDEC_TRANSFER_SIZE_BYTE) ?
                 W25Q_OK : W25Q_ERROR_SPI;
    dev->cs_deselect();

    if (status == W25Q_OK)
    {
        *jedec_id =
            ((uint32_t)rx_data[W25Q_JEDEC_MFR_INDEX] << W25Q_ID_MFR_SHIFT) |
            ((uint32_t)rx_data[W25Q_JEDEC_TYPE_INDEX] << W25Q_ID_TYPE_SHIFT) |
            rx_data[W25Q_JEDEC_CAPACITY_INDEX];
    }

    return status;
}

/**
 * @brief 在总线已加锁时使用 0x90 命令读取制造商和器件 ID
 * @retval W25Q_OK 读取成功
 * @retval W25Q_ERROR_SPI SPI 传输失败
 */
static w25q_status_t W25Q_ReadDeviceIdLocked(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                              uint16_t *device_id /**< 用于写出 W25Q 器件标识 */)
{
    w25q_status_t status;
    uint8_t tx_data[W25Q_DEVICE_ID_TRANSFER_SIZE] =
    {
        W25Q_CMD_DEVICE_ID,
        0U,
        0U,
        0U,
        W25Q_DUMMY_BYTE,
        W25Q_DUMMY_BYTE
    };
    uint8_t rx_data[W25Q_DEVICE_ID_TRANSFER_SIZE] = {0U};

    dev->cs_select();
    status = dev->spi_transfer(tx_data,
                               rx_data,
                               W25Q_DEVICE_ID_TRANSFER_SIZE) ?
                 W25Q_OK : W25Q_ERROR_SPI;
    dev->cs_deselect();

    if (status == W25Q_OK)
    {
        *device_id =
            ((uint16_t)rx_data[W25Q_DEVICE_MFR_INDEX] <<
             W25Q_DEVICE_MFR_SHIFT) |
            rx_data[W25Q_DEVICE_CODE_INDEX];
    }

    return status;
}

/**
 * @brief 轮询等待 Flash 空闲，每次轮询后释放片选
 * @retval W25Q_OK Flash 已空闲
 * @retval W25Q_ERROR_TIMEOUT 等待超时
 * @retval W25Q_ERROR_SPI SPI 通信失败
 */
static w25q_status_t W25Q_WaitReady(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    w25q_status_t status;
    uint32_t waited_ms = 0U;
    uint8_t status_reg = 0U;

    while (waited_ms <= W25Q_READY_TIMEOUT_MS)
    {
        status = W25Q_ReadStatus(dev, &status_reg);
        if (status != W25Q_OK)
        {
            return status;
        }

        if ((status_reg & W25Q_STATUS_BUSY_MASK) == 0U)
        {
            return W25Q_OK;
        }

        if (waited_ms == W25Q_READY_TIMEOUT_MS)
        {
            return W25Q_ERROR_TIMEOUT;
        }

        dev->delay_ms(1U);
        waited_ms += 1U;
    }

    return W25Q_ERROR_TIMEOUT;
}

/**
 * @brief 发送写使能并检查 WEL 标志
 * @retval W25Q_OK 写使能成功
 * @retval W25Q_ERROR_WRITE_ENABLE 写使能失败
 * @retval W25Q_ERROR_SPI SPI 通信失败
 */
static w25q_status_t W25Q_WriteEnable(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */)
{
    w25q_status_t status;
    uint8_t status_reg = 0U;

    dev->cs_select();
    status = W25Q_TransferByte(dev, W25Q_CMD_WRITE_ENABLE, NULL);
    dev->cs_deselect();

    if (status == W25Q_OK)
    {
        status = W25Q_ReadStatus(dev, &status_reg);
    }
    if ((status == W25Q_OK) &&
        ((status_reg & W25Q_STATUS_WEL_MASK) == 0U))
    {
        status = W25Q_ERROR_WRITE_ENABLE;
    }

    return status;
}

/**
 * @brief 执行连续读取事务
 * @retval W25Q_OK 读取成功
 * @retval W25Q_ERROR_SPI SPI 通信失败
 */
static w25q_status_t W25Q_ReadBytes(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                    uint32_t read_addr /**< Flash 读取操作的起始字节地址 */,
                                    uint8_t *data /**< Flash读取数据的输出缓冲区 */,
                                    uint32_t data_len_byte /**< Flash连续读取字节数 */)
{
    w25q_status_t status;
    uint32_t data_index;

    dev->cs_select();
    status = W25Q_TransferByte(dev, W25Q_CMD_READ_DATA, NULL);
    if (status == W25Q_OK)
    {
        status = W25Q_SendAddress(dev, read_addr);
    }

    for (data_index = 0U;
         (data_index < data_len_byte) && (status == W25Q_OK);
         data_index++)
    {
        status = W25Q_TransferByte(dev,
                                   W25Q_DUMMY_BYTE,
                                   &data[data_index]);
    }
    dev->cs_deselect();

    return status;
}

/**
 * @brief 在单个页内写入数据
 * @retval W25Q_OK 写入成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
static w25q_status_t W25Q_WritePage(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                    uint32_t write_addr /**< Flash 写入操作的起始字节地址 */,
                                    const uint8_t *data /**< 待写入当前Flash页的数据缓冲区 */,
                                    uint32_t data_len_byte /**< 当前Flash页的写入字节数 */)
{
    w25q_status_t status;
    uint32_t data_index;

    status = W25Q_WaitReady(dev);
    if (status == W25Q_OK)
    {
        status = W25Q_WriteEnable(dev);
    }

    if (status != W25Q_OK)
    {
        return status;
    }

    dev->cs_select();
    status = W25Q_TransferByte(dev, W25Q_CMD_PAGE_PROGRAM, NULL);
    if (status == W25Q_OK)
    {
        status = W25Q_SendAddress(dev, write_addr);
    }

    for (data_index = 0U;
         (data_index < data_len_byte) && (status == W25Q_OK);
         data_index++)
    {
        status = W25Q_TransferByte(dev, data[data_index], NULL);
    }
    dev->cs_deselect();

    if (status == W25Q_OK)
    {
        status = W25Q_WaitReady(dev);
    }

    return status;
}

/**
 * @brief 发送扇区擦除命令
 * @retval W25Q_OK 命令发送成功
 * @retval W25Q_ERROR_SPI SPI 通信失败
 */
static w25q_status_t W25Q_EraseCommand(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                       uint32_t sector_addr /**< 待发送擦除命令的扇区起始地址 */)
{
    w25q_status_t status;

    dev->cs_select();
    status = W25Q_TransferByte(dev, W25Q_CMD_SECTOR_ERASE, NULL);
    if (status == W25Q_OK)
    {
        status = W25Q_SendAddress(dev, sector_addr);
    }
    dev->cs_deselect();

    return status;
}

/**
 * @brief 在已持有总线锁时擦除一个 4 KB 扇区
 * @retval W25Q_OK 擦除成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
static w25q_status_t W25Q_EraseSectorLocked(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                             uint32_t sector_addr /**< 持锁擦除的扇区起始地址 */)
{
    w25q_status_t status;

    status = W25Q_WaitReady(dev);
    if (status == W25Q_OK)
    {
        status = W25Q_WriteEnable(dev);
    }
    if (status == W25Q_OK)
    {
        status = W25Q_EraseCommand(dev, sector_addr);
    }
    if (status == W25Q_OK)
    {
        status = W25Q_WaitReady(dev);
    }

    return status;
}

/**
 * @brief 在已持有总线锁时连续分页编程
 * @retval W25Q_OK 写入成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
static w25q_status_t W25Q_ProgramDataLocked(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                             uint32_t write_addr /**< Flash 写入操作的起始字节地址 */,
                                             const uint8_t *data /**< 待连续写入Flash的数据缓冲区 */,
                                             uint32_t data_len_byte /**< Flash连续写入字节数 */)
{
    w25q_status_t status = W25Q_OK;
    uint32_t page_remain_byte;
    uint32_t write_len_byte;

    while ((data_len_byte > 0U) && (status == W25Q_OK))
    {
        page_remain_byte = W25Q_PAGE_SIZE_BYTE -
                           (write_addr % W25Q_PAGE_SIZE_BYTE);
        write_len_byte = data_len_byte;
        if (write_len_byte > page_remain_byte)
        {
            write_len_byte = page_remain_byte;
        }

        status = W25Q_WritePage(dev,
                                write_addr,
                                data,
                                write_len_byte);
        if (status == W25Q_OK)
        {
            write_addr += write_len_byte;
            data += write_len_byte;
            data_len_byte -= write_len_byte;
        }
    }

    return status;
}

/**
 * @brief 更新单个扇区，并在需要出现 0 到 1 位变化时擦除后恢复整扇区
 * @retval W25Q_OK 更新成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
static w25q_status_t W25Q_UpdateSectorLocked(w25q_handle_t *dev /**< W25Q Flash 设备句柄 */,
                                              uint32_t sector_addr /**< 待保留并改写数据的扇区起始地址 */,
                                              uint32_t sector_offset /**< 写入数据相对扇区起始位置的字节偏移 */,
                                              const uint8_t *data /**< 待更新到当前Flash扇区的数据缓冲区 */,
                                              uint32_t data_len_byte /**< 当前Flash扇区的更新字节数 */)
{
    w25q_status_t status;
    uint32_t data_index;
    uint32_t page_offset;
    uint32_t page_index;
    bool data_changed = false;
    bool erase_required = false;
    bool page_has_data;

    status = W25Q_WaitReady(dev);
    if (status == W25Q_OK)
    {
        status = W25Q_ReadBytes(dev,
                                sector_addr,
                                w25q_sector_buffer,
                                W25Q_SECTOR_SIZE_BYTE);
    }
    if (status != W25Q_OK)
    {
        return status;
    }

    for (data_index = 0U; data_index < data_len_byte; data_index++)
    {
        uint8_t old_data = w25q_sector_buffer[sector_offset + data_index];
        uint8_t new_data = data[data_index];

        if (old_data != new_data)
        {
            data_changed = true;
        }
        if ((old_data & new_data) != new_data)
        {
            erase_required = true;
        }
        w25q_sector_buffer[sector_offset + data_index] = new_data;
    }

    if (!data_changed)
    {
        return W25Q_OK;
    }

    if (!erase_required)
    {
        return W25Q_ProgramDataLocked(dev,
                                       sector_addr + sector_offset,
                                       data,
                                       data_len_byte);
    }

    status = W25Q_EraseSectorLocked(dev, sector_addr);
    for (page_offset = 0U;
         (page_offset < W25Q_SECTOR_SIZE_BYTE) && (status == W25Q_OK);
         page_offset += W25Q_PAGE_SIZE_BYTE)
    {
        page_has_data = false;
        for (page_index = 0U; page_index < W25Q_PAGE_SIZE_BYTE; page_index++)
        {
            if (w25q_sector_buffer[page_offset + page_index] !=
                W25Q_DUMMY_BYTE)
            {
                page_has_data = true;
                break;
            }
        }

        if (page_has_data)
        {
            status = W25Q_WritePage(dev,
                                    sector_addr + page_offset,
                                    &w25q_sector_buffer[page_offset],
                                    W25Q_PAGE_SIZE_BYTE);
        }
    }

    return status;
}
