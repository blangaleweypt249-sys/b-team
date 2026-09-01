/**
 * @file upper_remote_link.h
 * @brief 定义遥控器链路的数据格式、运行状态和访问接口。
 */

#ifndef UPPER_REMOTE_LINK_H
/** 防止 upper_remote_link.h 被重复包含。 */
#define UPPER_REMOTE_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 超过该时间未收到遥控帧后判定遥控链路离线，单位：毫秒。 */
#define UPPER_REMOTE_TIMEOUT_MS       200U
/** 从状态字中筛选对应位的掩码。 */
#define UPPER_REMOTE_PRIMARY_KEY_MASK    0x06U
/** 从状态字中筛选对应位的掩码。 */
#define UPPER_REMOTE_PRIMARY_SWITCH_MASK 0x03U
/** 从状态字中筛选对应位的掩码。 */
#define UPPER_REMOTE_KEY_MASK         0x3FU
/** 从状态字中筛选对应位的掩码。 */
#define UPPER_REMOTE_SWITCH_MASK      0x37U
/** 用于暂存该模块数据的缓冲区容量。 */
#define UPPER_REMOTE_FRAME_BUFFER_SIZE 32U

/** 遥控帧中对应按键或开关的位标志。 */
#define UPPER_REMOTE_PRIMARY_KEY_PC0     (1U << 2U)
/** 遥控帧中对应按键或开关的位标志。 */
#define UPPER_REMOTE_PRIMARY_KEY_PC1     (1U << 1U)
/** 遥控帧中对应按键或开关的位标志。 */
#define UPPER_REMOTE_PRIMARY_SWITCH_PE0 (1U << 0U)
/** 遥控帧中对应按键或开关的位标志。 */
#define UPPER_REMOTE_PRIMARY_SWITCH_PD6 (1U << 1U)
/** 遥控帧中对应按键或开关的位标志。 */
#define UPPER_REMOTE_SWITCH_PD6          (1U << 4U)
/** PE0、PD6 模式开关状态必须连续一致才确认切换的帧数。 */
#define UPPER_REMOTE_MODE_STABLE_FRAMES  3U

/** 保存 遥控链路 运行过程中需要集中管理的数据。 */
typedef struct
{
    uint8_t primary_key_bits; /**< 主遥控 PC0、PC1 按键的当前位图。 */
    uint8_t primary_switch; /**< 主遥控 PE0、PD6 开关的当前位图。 */
    uint8_t key_bits; /**< 副遥控 PD8 至 PD13 按键的当前位图。 */
    uint8_t switch_bits; /**< 副遥控各拨动开关的当前位图。 */
    uint8_t sequence; /**< 用于匹配请求和响应的消息序号。 */
    uint32_t updated_at_ms; /**< 最近一次收到有效反馈的系统毫秒时刻。 */
    bool online; /**< 对应设备当前是否在线。 */
} upper_remote_control_t;

/** 保存 遥控链路 通信和运行诊断数据。 */
typedef struct
{
    uint32_t valid_frame_count; /**< 遥控链路累计接受的有效数据帧数量。 */
    uint32_t ignored_frame_count; /**< 因目标接收端不匹配而忽略的数据帧数量。 */
    uint32_t crc_error_count; /**< 累计 CRC 校验失败的数据帧数量。 */
    uint32_t format_error_count; /**< 累计格式或字段范围不合法的数据帧数量。 */
    uint32_t buffer_overflow_count; /**< 接收缓冲区容量不足导致丢弃数据的次数。 */
    uint32_t lost_frame_count; /**< 根据连续帧序号统计的丢失帧数量。 */
    uint32_t duplicate_frame_count; /**< 累计收到的重复序号数据帧数量。 */
} upper_remote_diagnostics_t;

/** 保存 遥控链路 运行过程中需要集中管理的数据。 */
typedef struct
{
    uint8_t frame_buffer[UPPER_REMOTE_FRAME_BUFFER_SIZE]; /**< 用于暂存尚未完成解析的数据字节。 */
    size_t buffered_size; /**< 用于暂存尚未完成解析的数据字节。 */
    volatile upper_remote_control_t control; /**< 最近一帧有效遥控数据解析得到的控制快照。 */
    volatile uint32_t control_version; /**< 遥控控制快照每次更新时递增的版本号。 */
    upper_remote_diagnostics_t diagnostics; /**< 遥控链路累计的收帧和错误诊断数据。 */
    volatile bool has_valid_control; /**< 链路是否已经解析出至少一帧有效遥控数据。 */
    uint8_t mode_candidate_bits; /**< 正在进行消抖确认的 PE0、PD6 模式位。 */
    uint8_t mode_pe0_stable_frames; /**< PE0 候选状态连续保持不变的帧数。 */
    uint8_t mode_pd6_stable_frames; /**< PD6 候选状态连续保持不变的帧数。 */
    bool mode_switches_initialized; /**< PE0、PD6 模式开关的消抖状态是否已经初始化。 */
    bool has_sequence; /**< 链路是否已经记录过有效帧序号。 */
    uint8_t last_sequence; /**< 最近一帧有效遥控数据的序号。 */
} upper_remote_link_t;

/* 功能：初始化遥控链路对象和诊断状态；用途：建立可接收定长遥控帧的初始状态；无返回值表示对象已复位。 */
void UpperRemoteLink_Init(upper_remote_link_t *link /* 需要操作的通信链路对象 */);
/* 功能：向遥控链路压入新收到的字节流；用途：缓存数据并触发定长帧解析；返回值表示本次接收的有效帧数。 */
void UpperRemoteLink_Push(upper_remote_link_t *link /* 需要操作的通信链路对象 */,
                          const uint8_t *data /* 待处理数据的首地址 */,
                          size_t size /* 待处理数据的字节数 */,
                          uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：读取当前遥控输入并判断是否超时；用途：向应用层提供按键、摇杆和在线状态；返回 true 表示输出参数有效。 */
bool UpperRemoteLink_GetControl(const upper_remote_link_t *link /* 需要操作的通信链路对象 */,
                                uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                upper_remote_control_t *control /* 需要读取或更新的控制状态 */);
/* 功能：读取遥控链路诊断统计；用途：观察收帧、丢帧、重同步和缓存状态；无返回值表示统计已复制到输出。 */
void UpperRemoteLink_GetDiagnostics(const upper_remote_link_t *link /* 需要操作的通信链路对象 */,
                                    upper_remote_diagnostics_t *diagnostics /* 用于写出诊断统计的对象 */);

#endif
