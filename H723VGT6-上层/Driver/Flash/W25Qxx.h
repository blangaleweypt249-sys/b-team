/**
 * @file W25Qxx.h
 * @brief W25Q16 至 W25Q128 外部 Flash 驱动及硬件移植配置
 * @note 底层 SPI 接口必须为同步接口，不能直接绑定异步 DMA 函数
 */

#ifndef W25QXX_H
#define W25QXX_H

#include <stdbool.h>
#include <stdint.h>

/*
 * ======================== 更换 MCU / 工程时必改区 ========================
 * 1. STM32CubeMX 工程通常仍使用 main.h、spi.h，先核对新工程中的文件名。
 * 2. 把 W25Q_SPI_HANDLE、W25Q_CS_GPIO_PORT、W25Q_CS_GPIO_PIN 改成
 *    新工程由 CubeMX 生成的名称。底层不是 STM32 HAL 时才改 W25Q_MCU_* 宏。
 * 3. 裸机或不希望任务切换时，W25Q_DELAY_MODE 设为 0（HAL_Delay）。
 *    使用 CMSIS-RTOS 的任务延时时，设为 1（osDelay），并修改 RTOS 头文件。
 * 4. 若 RTOS 的 1 Tick 不等于 1 ms，请修改 W25Q_MCU_OS_DELAY_MS 宏，
 *    在此处完成毫秒到 Tick 的换算。
 *
 * 当前 H723VGT6 工程使用 hspi1、FLASH_CS_GPIO_Port、FLASH_CS_Pin 和
 * CMSIS-RTOS2 延时。更换 STM32 型号示例（F103 -> H750、F405 或其他型号）：
 * - 若新工程仍使用这些名称和 RTOS 延时，下面四项无需改。
 * - 若新工程改用 hspi2，且 CubeMX 将片选命名为 FLASH_CS，则改为：
 *   #define W25Q_SPI_HANDLE          hspi2
 *   #define W25Q_CS_GPIO_PORT        FLASH_CS_GPIO_Port
 *   #define W25Q_CS_GPIO_PIN         FLASH_CS_Pin
 * MCU 型号本身不决定使用 HAL_Delay 还是 osDelay；是否运行 RTOS 才决定模式。
 */
#include "main.h"
#include "spi.h"

#define W25Q_DELAY_USE_HAL            0U
#define W25Q_DELAY_USE_OS             1U

/* 只改这里：0U = HAL_Delay，1U = osDelay。 */
#define W25Q_DELAY_MODE               1U

#if (W25Q_DELAY_MODE == W25Q_DELAY_USE_OS)
/* CMSIS-RTOS v1 工程请改为 cmsis_os.h。 */
#include "cmsis_os2.h"
#endif

#define W25Q_SPI_HANDLE               hspi1
#define W25Q_CS_GPIO_PORT             FLASH_CS_GPIO_Port
#define W25Q_CS_GPIO_PIN              FLASH_CS_Pin
#define W25Q_SPI_TIMEOUT_MS           100U

#define W25Q_MCU_CS_SELECT()                                             \
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_RESET)

#define W25Q_MCU_CS_DESELECT()                                           \
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_SET)

#define W25Q_MCU_SPI_RW_BYTE(tx_data_ptr, rx_data_ptr)                    \
    (HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,                            \
                             (tx_data_ptr),                              \
                             (rx_data_ptr),                              \
                             1U,                                        \
                             W25Q_SPI_TIMEOUT_MS) == HAL_OK)

#define W25Q_MCU_SPI_TRANSFER(tx_data_ptr, rx_data_ptr, data_len_byte)    \
    (HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,                            \
                             (tx_data_ptr),                              \
                             (rx_data_ptr),                              \
                             (data_len_byte),                            \
                             W25Q_SPI_TIMEOUT_MS) == HAL_OK)

#define W25Q_MCU_HAL_DELAY_MS(delay_ms)  HAL_Delay(delay_ms)

#if (W25Q_DELAY_MODE == W25Q_DELAY_USE_OS)
#define W25Q_MCU_OS_DELAY_MS(delay_ms)   ((void)osDelay(delay_ms))
#endif
/* ====================== 更换 MCU / 工程时必改区结束 ===================== */

#if ((W25Q_DELAY_MODE != W25Q_DELAY_USE_HAL) && \
     (W25Q_DELAY_MODE != W25Q_DELAY_USE_OS))
#error "W25Q_DELAY_MODE must be 0 (HAL_Delay) or 1 (osDelay)"
#endif

#define W25Q16_ID                    0xEF4015UL
#define W25Q32_ID                    0xEF4016UL
#define W25Q64_ID                    0xEF4017UL
#define W25Q128_ID                   0xEF4018UL

#define W25Q_PAGE_SIZE_BYTE          256U
#define W25Q_SECTOR_SIZE_BYTE        4096UL

#ifndef W25Q_READY_TIMEOUT_MS
#define W25Q_READY_TIMEOUT_MS        1000U
#endif

#ifndef W25Q_LOCK_TIMEOUT_MS
#define W25Q_LOCK_TIMEOUT_MS         1000U
#endif

/**
 * @brief W25Qxx 驱动返回状态
 */
typedef enum
{
    W25Q_OK = 0,
    W25Q_ERROR_INVALID_ARG,
    W25Q_ERROR_NOT_INIT,
    W25Q_ERROR_UNSUPPORTED_DEVICE,
    W25Q_ERROR_OUT_OF_RANGE,
    W25Q_ERROR_NOT_ALIGNED,
    W25Q_ERROR_TIMEOUT,
    W25Q_ERROR_SPI,
    W25Q_ERROR_WRITE_ENABLE,
    W25Q_ERROR_LOCK_TIMEOUT
} w25q_status_t;

/**
 * @brief W25Qxx 设备句柄
 * @note bus_lock 和 bus_unlock 可在单线程环境中同时留空
 */
typedef struct
{
    void (*cs_select)(void);
    void (*cs_deselect)(void);
    bool (*spi_rw_byte)(uint8_t tx_data, uint8_t *rx_data);
    bool (*spi_transfer)(const uint8_t *tx_data,
                         uint8_t *rx_data,
                         uint16_t data_len_byte);
    void (*delay_ms)(uint32_t delay_ms);

    /* 多任务或共享 SPI 总线时，两个锁接口必须同时绑定。 */
    bool (*bus_lock)(uint32_t timeout_ms);
    void (*bus_unlock)(void);

    uint32_t flash_id;
    uint32_t capacity_kb;
    uint32_t sector_count;
    uint16_t page_size_byte;
    bool is_initialized;
} w25q_handle_t;

/**
 * @brief 绑定头文件移植配置并初始化 W25Qxx
 * @retval W25Q_OK 初始化成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_PortInit(void);

/**
 * @brief 获取使用头文件移植配置的 W25Qxx 设备句柄
 * @retval W25Qxx 设备句柄地址
 */
w25q_handle_t *W25Q_PortGetDevice(void);

/**
 * @brief 使用 0x90 命令读取制造商和器件 ID
 * @param dev W25Qxx 设备句柄
 * @param device_id 制造商 ID 与器件 ID 的组合值
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadDeviceId(w25q_handle_t *dev, uint16_t *device_id);

/**
 * @brief 初始化 W25Qxx 设备并读取 JEDEC ID
 * @param dev W25Qxx 设备句柄
 * @retval W25Q_OK 初始化成功
 * @retval W25Q_ERROR_INVALID_ARG 底层接口未完整绑定
 * @retval W25Q_ERROR_UNSUPPORTED_DEVICE 芯片不受支持
 * @retval W25Q_ERROR_SPI SPI 通信失败
 * @retval W25Q_ERROR_LOCK_TIMEOUT 获取总线锁超时
 */
w25q_status_t W25Q_Init(w25q_handle_t *dev);

/**
 * @brief 擦除指定的 4 KB 扇区
 * @param dev W25Qxx 设备句柄
 * @param sector_addr 扇区首地址，必须按 4 KB 对齐
 * @retval W25Q_OK 擦除成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_EraseSector(w25q_handle_t *dev,
                               uint32_t sector_addr);

/**
 * @brief 从指定扇区首地址开始擦除若干个完整的 4 KB 扇区
 * @param dev W25Qxx 设备句柄
 * @param sector_addr 第一个扇区的首地址，必须按 4 KB 对齐
 * @param sector_count 要擦除的扇区数量
 * @retval W25Q_OK 擦除成功，数量为 0 时不执行操作
 * @retval 其他值 具体错误见 w25q_status_t
 * @note 实际擦除长度固定为 sector_count * W25Q_SECTOR_SIZE_BYTE
 */
w25q_status_t W25Q_EraseSectors(w25q_handle_t *dev,
                                uint32_t sector_addr,
                                uint32_t sector_count);

/**
 * @brief 从 Flash 连续读取数据
 * @param dev W25Qxx 设备句柄
 * @param read_addr 起始读取地址
 * @param data 接收缓冲区
 * @param data_len_byte 读取长度(Byte)
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadData(w25q_handle_t *dev,
                            uint32_t read_addr,
                            uint8_t *data,
                            uint32_t data_len_byte);

/**
 * @brief 向 Flash 写入数据，自动处理页边界和必要的扇区擦除
 * @param dev W25Qxx 设备句柄
 * @param write_addr 起始写入地址
 * @param data 待写入数据
 * @param data_len_byte 写入长度(Byte)
 * @retval W25Q_OK 写入成功
 * @retval 其他值 具体错误见 w25q_status_t
 * @note 若需要擦除，驱动会保留同一扇区中写入范围外的原数据
 * @note 自动擦除写入使用一个静态 4 KB 扇区缓冲区
 */
w25q_status_t W25Q_WriteData(w25q_handle_t *dev,
                             uint32_t write_addr,
                             const uint8_t *data,
                             uint32_t data_len_byte);

#endif
