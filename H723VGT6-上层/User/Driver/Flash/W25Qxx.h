/**
 * @file W25Qxx.h
 * @brief W25Q16 至 W25Q128 外部 Flash 驱动及硬件移植配置
 * @note 底层 SPI 接口必须为同步接口，不能直接绑定异步 DMA 函数
 */

#ifndef W25QXX_H
/** 防止 W25Qxx.h 被重复包含。 */
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

/** 选择 HAL 阻塞延时方式时使用的配置值。 */
#define W25Q_DELAY_USE_HAL            0U
/** 选择 RTOS 任务延时方式时使用的配置值。 */
#define W25Q_DELAY_USE_OS             1U

/* 只改这里：0U = HAL_Delay，1U = osDelay。 */
#define W25Q_DELAY_MODE               1U

#if (W25Q_DELAY_MODE == W25Q_DELAY_USE_OS)
/* CMSIS-RTOS v1 工程请改为 cmsis_os.h。 */
#include "cmsis_os2.h"
#endif

/** W25Qxx 芯片连接的 HAL SPI 句柄。 */
#define W25Q_SPI_HANDLE               hspi1
/** W25Qxx 片选信号所在的 GPIO 端口。 */
#define W25Q_CS_GPIO_PORT             FLASH_CS_GPIO_Port
/** W25Qxx 片选信号所在的 GPIO 引脚。 */
#define W25Q_CS_GPIO_PIN              FLASH_CS_Pin
/** 单次 W25Qxx SPI 阻塞传输允许等待的最长时间，单位：毫秒。 */
#define W25Q_SPI_TIMEOUT_MS           100U

/** 拉低片选信号，开始一次 W25Qxx SPI 事务。 */
#define W25Q_MCU_CS_SELECT()                                             \
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_RESET)

/** 释放片选信号，结束一次 W25Qxx SPI 事务。 */
#define W25Q_MCU_CS_DESELECT()                                           \
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_SET)

/**
 * 通过已配置的 SPI 外设同时收发一个字节。
 * @param tx_data_ptr SPI 发送缓冲区首地址。
 * @param rx_data_ptr SPI 接收缓冲区首地址。
 */
#define W25Q_MCU_SPI_RW_BYTE(tx_data_ptr, rx_data_ptr)                    \
    (HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,                            \
                             (tx_data_ptr),                              \
                             (rx_data_ptr),                              \
                             1U,                                        \
                             W25Q_SPI_TIMEOUT_MS) == HAL_OK)

/**
 * 通过已配置的 SPI 外设收发一段连续数据。
 * @param tx_data_ptr SPI 发送缓冲区首地址。
 * @param rx_data_ptr SPI 接收缓冲区首地址。
 * @param data_len_byte 本次传输或处理的数据字节数。
 */
#define W25Q_MCU_SPI_TRANSFER(tx_data_ptr, rx_data_ptr, data_len_byte)    \
    (HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,                            \
                             (tx_data_ptr),                              \
                             (rx_data_ptr),                              \
                             (data_len_byte),                            \
                             W25Q_SPI_TIMEOUT_MS) == HAL_OK)

/**
 * 在未启动调度器时使用 HAL 提供毫秒延时。
 * @param delay_ms 需要等待的时间，单位：毫秒。
 */
#define W25Q_MCU_HAL_DELAY_MS(delay_ms)  HAL_Delay(delay_ms)

#if (W25Q_DELAY_MODE == W25Q_DELAY_USE_OS)
/**
 * 在 RTOS 运行期间挂起当前任务完成毫秒延时。
 * @param delay_ms 需要等待的时间，单位：毫秒。
 */
#define W25Q_MCU_OS_DELAY_MS(delay_ms)   ((void)osDelay(delay_ms))
#endif
/* ====================== 更换 MCU / 工程时必改区结束 ===================== */

#if ((W25Q_DELAY_MODE != W25Q_DELAY_USE_HAL) && \
     (W25Q_DELAY_MODE != W25Q_DELAY_USE_OS))
#error "W25Q_DELAY_MODE must be 0 (HAL_Delay) or 1 (osDelay)"
#endif

/** 16 Mbit W25Qxx 器件对应的 JEDEC 标识。 */
#define W25Q16_ID                    0xEF4015UL
/** 32 Mbit W25Qxx 器件对应的 JEDEC 标识。 */
#define W25Q32_ID                    0xEF4016UL
/** 64 Mbit W25Qxx 器件对应的 JEDEC 标识。 */
#define W25Q64_ID                    0xEF4017UL
/** 128 Mbit W25Qxx 器件对应的 JEDEC 标识。 */
#define W25Q128_ID                   0xEF4018UL

/** W25Qxx 单页可编程的数据字节数。 */
#define W25Q_PAGE_SIZE_BYTE          256U
/** W25Qxx 单个可擦除扇区的字节数。 */
#define W25Q_SECTOR_SIZE_BYTE        4096UL

#ifndef W25Q_READY_TIMEOUT_MS
/** 等待 W25Qxx 完成编程或擦除操作的最长时间，单位：毫秒。 */
#define W25Q_READY_TIMEOUT_MS        1000U
#endif

#ifndef W25Q_LOCK_TIMEOUT_MS
/** 等待取得共享 SPI 总线锁的最长时间，单位：毫秒。 */
#define W25Q_LOCK_TIMEOUT_MS         1000U
#endif

/**
 * @brief W25Qxx 驱动返回状态
 */
typedef enum
{
    W25Q_OK = 0, /**< 操作成功完成。 */
    W25Q_ERROR_INVALID_ARG, /**< 传入的设备、地址或缓冲区参数无效。 */
    W25Q_ERROR_NOT_INIT, /**< Flash 尚未完成初始化。 */
    W25Q_ERROR_UNSUPPORTED_DEVICE, /**< 读取到的器件标识不属于支持的 W25Qxx 型号。 */
    W25Q_ERROR_OUT_OF_RANGE, /**< 访问地址或数据长度超出 Flash 容量。 */
    W25Q_ERROR_NOT_ALIGNED, /**< 擦除地址没有按扇区边界对齐。 */
    W25Q_ERROR_TIMEOUT, /**< 等待 Flash 就绪超过允许时间。 */
    W25Q_ERROR_SPI, /**< 底层 SPI 收发失败。 */
    W25Q_ERROR_WRITE_ENABLE, /**< 发送写使能后状态寄存器未确认写使能锁存。 */
    W25Q_ERROR_LOCK_TIMEOUT /**< 在规定时间内未取得共享 SPI 总线锁。 */
} w25q_status_t;

/**
 * @brief W25Qxx 设备句柄
 * @note bus_lock 和 bus_unlock 可在单线程环境中同时留空
 */
typedef struct
{
    void (*cs_select)(void); /**< 开始 SPI 事务时拉低 Flash 片选信号的回调。 */
    void (*cs_deselect)(void); /**< 结束 SPI 事务时释放 Flash 片选信号的回调。 */
    bool (*spi_rw_byte)(uint8_t tx_data, uint8_t *rx_data); /**< 通过 SPI 同时收发单字节的回调。 */
    bool (*spi_transfer)(const uint8_t *tx_data,
                         uint8_t *rx_data,
                         uint16_t data_len_byte); /**< 通过 SPI 收发连续数据块的回调。 */
    void (*delay_ms)(uint32_t delay_ms); /**< 等待指定毫秒数的延时回调。 */

    /* 多任务或共享 SPI 总线时，两个锁接口必须同时绑定。 */
    bool (*bus_lock)(uint32_t timeout_ms); /**< 在共享 SPI 总线上取得互斥锁的回调。 */
    void (*bus_unlock)(void); /**< 释放共享 SPI 总线互斥锁的回调。 */

    uint32_t flash_id; /**< 初始化时读取到的完整 JEDEC 器件标识。 */
    uint32_t capacity_kb; /**< 当前 Flash 的总容量，单位：KB。 */
    uint32_t sector_count; /**< 当前 Flash 可擦除扇区的总数量。 */
    uint16_t page_size_byte; /**< 当前 Flash 单页可编程的字节数。 */
    bool is_initialized; /**< Flash 是否已经识别并完成初始化。 */
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
 * @brief 获取最近一次 W25Q_PortInit 调用的结果。
 * @retval W25Q_ERROR_NOT_INIT 表示端口尚未初始化，否则返回确切的初始化结果。
 */
/* 功能：读取 W25Q 板级端口初始化状态；用途：向上层报告 Flash 初始化结果；返回值表示当前初始化状态。 */
w25q_status_t W25Q_PortGetInitStatus(void);

/**
 * @brief 使用 0x90 命令读取制造商和器件 ID
 * @param dev W25Qxx 设备句柄
 * @param device_id 制造商 ID 与器件 ID 的组合值
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadDeviceId(w25q_handle_t *dev /* 函数读取或写入的对象地址 */, uint16_t *device_id /* 函数读取或写入的对象地址 */);

/**
 * @brief 初始化 W25Qxx 设备并读取 JEDEC ID
 * @param dev W25Qxx 设备句柄
 * @retval W25Q_OK 初始化成功
 * @retval W25Q_ERROR_INVALID_ARG 底层接口未完整绑定
 * @retval W25Q_ERROR_UNSUPPORTED_DEVICE 芯片不受支持
 * @retval W25Q_ERROR_SPI SPI 通信失败
 * @retval W25Q_ERROR_LOCK_TIMEOUT 获取总线锁超时
 */
w25q_status_t W25Q_Init(w25q_handle_t *dev /* 函数读取或写入的对象地址 */);

/**
 * @brief 擦除指定的 4 KB 扇区
 * @param dev W25Qxx 设备句柄
 * @param sector_addr 扇区首地址，必须按 4 KB 对齐
 * @retval W25Q_OK 擦除成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_EraseSector(w25q_handle_t *dev /* 函数读取或写入的对象地址 */,
                               uint32_t sector_addr /* 待擦除或改写扇区的起始地址 */);

/**
 * @brief 从指定扇区首地址开始擦除若干个完整的 4 KB 扇区
 * @param dev W25Qxx 设备句柄
 * @param sector_addr 第一个扇区的首地址，必须按 4 KB 对齐
 * @param sector_count 要擦除的扇区数量
 * @retval W25Q_OK 擦除成功，数量为 0 时不执行操作
 * @retval 其他值 具体错误见 w25q_status_t
 * @note 实际擦除长度固定为 sector_count * W25Q_SECTOR_SIZE_BYTE
 */
w25q_status_t W25Q_EraseSectors(w25q_handle_t *dev /* 函数读取或写入的对象地址 */,
                                uint32_t sector_addr /* 待擦除或改写扇区的起始地址 */,
                                uint32_t sector_count /* Flash 扇区总数或本次连续操作的扇区数量 */);

/**
 * @brief 从 Flash 连续读取数据
 * @param dev W25Qxx 设备句柄
 * @param read_addr 起始读取地址
 * @param data 接收缓冲区
 * @param data_len_byte 读取长度(Byte)
 * @retval W25Q_OK 读取成功
 * @retval 其他值 具体错误见 w25q_status_t
 */
w25q_status_t W25Q_ReadData(w25q_handle_t *dev /* 函数读取或写入的对象地址 */,
                            uint32_t read_addr /* Flash 读取操作的起始字节地址 */,
                            uint8_t *data /* 待处理数据的首地址 */,
                            uint32_t data_len_byte /* 本次传输或处理的数据字节数 */);

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
w25q_status_t W25Q_WriteData(w25q_handle_t *dev /* 函数读取或写入的对象地址 */,
                             uint32_t write_addr /* Flash 写入操作的起始字节地址 */,
                             const uint8_t *data /* 待处理数据的首地址 */,
                             uint32_t data_len_byte /* 本次传输或处理的数据字节数 */);

#endif
