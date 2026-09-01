/**
 * @file j4310.h
 * @brief 定义 J4310 电机协议、状态数据和控制接口。
 */

#ifndef J4310_H
/** 防止 j4310.h 被重复包含。 */
#define J4310_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/** 该模块可同时管理的机械臂 J4310 关节数量。 */
#define J4310_MAX_MOTOR_COUNT  16U

/** 表示 J4310 可选择的工作模式。 */
typedef enum
{
    J4310_MODE_MIT = 0x000U, /**< 使用 MIT 位置、速度和转矩联合控制。 */
    J4310_MODE_POSITION_VELOCITY = 0x100U, /**< 按位置和速度联合控制。 */
    J4310_MODE_VELOCITY = 0x200U /**< 按目标速度闭环控制。 */
} j4310_mode_t;

/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    float position_max_rad; /**< J4310的当前位置，单位：弧度。 */
    float velocity_max_rad_s; /**< J4310的最大允许速度，单位：弧度每秒。 */
    float torque_max_nm; /**< J4310的转矩上限，单位：牛米。 */
} j4310_limits_t;

/** 保存 J4310 最近一次有效反馈及其时间信息。 */
typedef struct
{
    float position_rad; /**< J4310的当前位置，单位：弧度。 */
    float velocity_rad_s; /**< J4310的当前速度，单位：弧度每秒。 */
    float torque_nm; /**< J4310的反馈转矩，单位：牛米。 */
    uint8_t mos_temperature_c; /**< J4310反馈的温度，单位：摄氏度。 */
    uint8_t rotor_temperature_c; /**< J4310反馈的温度，单位：摄氏度。 */
    uint8_t fault; /**< J4310 最近反馈的故障码。 */
    uint32_t updated_at_ms; /**< 最近一次收到有效反馈的系统毫秒时刻。 */
    uint32_t rx_frames; /**< 累计接收并接受的反馈帧数量。 */
} j4310_feedback_t;

/** 保存 J4310 当前运行状态和中间计算数据。 */
typedef struct
{
    bool enabled; /**< 对应控制功能是否启用。 */
    float base_kp; /**< 在线调整前的基准比例增益。 */
    float base_kd; /**< 在线调整前的基准微分增益。 */
    float applied_kp; /**< 本控制周期实际应用的比例增益。 */
    float applied_kd; /**< 本控制周期实际应用的微分增益。 */
} j4310_online_mit_state_t;

/** 表示 J4310 处理一帧数据后的校验结果。 */
typedef enum
{
    J4310_RX_NONE = 0U, /**< 尚未处理任何候选反馈帧。 */
    J4310_RX_ACCEPTED = 1U, /**< 反馈帧格式和节点标识均通过校验。 */
    J4310_RX_REJECTED_FORMAT = 2U, /**< 因帧类型或数据长度不正确而拒绝。 */
    J4310_RX_REJECTED_MASTER_ID = 3U, /**< 因 CAN 标识符与主控标识不一致而拒绝。 */
    J4310_RX_REJECTED_FEEDBACK_ID = 4U /**< 因反馈帧中的电机编号不匹配而拒绝。 */
} j4310_rx_result_t;

/** 保存 J4310 通信和运行诊断数据。 */
typedef struct
{
    uint32_t frames_seen; /**< 累计检查到的候选数据帧数量。 */
    uint32_t accepted_frames; /**< 累计通过全部格式和标识校验的数据帧数量。 */
    uint32_t rejected_format_frames; /**< 因帧格式不合法而拒绝的数据帧数量。 */
    uint32_t rejected_master_id_frames; /**< 因主控标识不匹配而拒绝的数据帧数量。 */
    uint32_t rejected_feedback_id_frames; /**< 因反馈节点编号不匹配而拒绝的数据帧数量。 */
    uint16_t last_can_id; /**< 最近一次处理的数据帧 CAN 标识符。 */
    uint8_t last_dlc; /**< 最近一次处理的数据帧长度。 */
    uint8_t last_data0; /**< 最近一次处理的数据帧首字节。 */
    j4310_rx_result_t last_result; /**< 最近一次数据帧解析结果。 */
} j4310_rx_diagnostics_t;

/* 功能：清空全部 J4310 驱动上下文；用途：在注册电机前复位驱动状态；无返回值表示上下文表已初始化。 */
void J4310_Init(void);
/* 功能：注册一台 J4310 及其模式和物理限制；用途：建立节点运行上下文；返回 true 表示注册成功。 */
bool J4310_AddMotor(uint8_t motor_id /* DJI 电机编号 */,
                    uint16_t master_id /* J4310 反馈帧发送给主控时使用的 CAN 标识符 */,
                    uint8_t feedback_id /* J4310 反馈帧首字节携带的电机编号 */,
                    j4310_mode_t mode /* 需要设置或判断的工作模式 */,
                    const j4310_limits_t *limits /* 函数读取或写入的对象地址 */);
/* 功能：修改已注册 J4310 的控制模式；用途：切换 MIT、位置速度或速度控制；返回 true 表示模式已接受。 */
bool J4310_SetMode(uint8_t motor_id /* DJI 电机编号 */, j4310_mode_t mode /* 需要设置或判断的工作模式 */);
/* 功能：构造 J4310 使能帧；用途：让目标节点进入可控状态；返回 true 表示构帧成功。 */
bool J4310_BuildEnable(uint8_t motor_id /* DJI 电机编号 */, can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 失能帧并复位在线调参；用途：安全停止目标节点；返回 true 表示构帧成功。 */
bool J4310_BuildDisable(uint8_t motor_id /* DJI 电机编号 */, can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 清除故障帧；用途：请求节点退出故障状态；返回 true 表示构帧成功。 */
bool J4310_BuildClearFault(uint8_t motor_id /* DJI 电机编号 */, can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 保存当前位置为零点的帧；用途：执行机械零点标定；返回 true 表示构帧成功。 */
bool J4310_BuildSaveZero(uint8_t motor_id /* DJI 电机编号 */, can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 MIT 五参数控制帧；用途：发送位置、速度、刚度、阻尼和转矩目标；返回 true 表示参数有效并构帧完成。 */
bool J4310_BuildMit(uint8_t motor_id /* DJI 电机编号 */,
                    float position_rad /* 目标或反馈位置，单位：弧度 */,
                    float velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */,
                    float kp /* 比例增益 */,
                    float kd /* 微分增益 */,
                    float torque_nm /* 目标或反馈转矩，单位：牛米 */,
                    can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 位置速度模式帧；用途：发送位置与速度上限目标；返回 true 表示构帧成功。 */
bool J4310_BuildPositionVelocity(uint8_t motor_id /* DJI 电机编号 */,
                                 float position_rad /* 目标或反馈位置，单位：弧度 */,
                                 float velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */,
                                 can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：构造 J4310 纯速度模式帧；用途：发送目标转速；返回 true 表示构帧成功。 */
bool J4310_BuildVelocity(uint8_t motor_id /* DJI 电机编号 */,
                         float velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */,
                         can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);
/* 功能：设置 J4310 软件转矩限制；用途：在构造控制帧时进一步约束输出；返回 true 表示限制合法并已保存。 */
bool J4310_SetTorqueLimit(uint8_t motor_id /* DJI 电机编号 */, float torque_limit_nm /* 允许设置的转矩上限，单位：牛米 */);
/* 功能：解析 J4310 CAN 反馈并更新时间和故障信息；用途：维护闭环反馈快照；返回 true 表示该帧属于已注册节点且解析成功。 */
bool J4310_OnFrame(const can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */, uint32_t tick_ms /* 当前系统毫秒时刻 */);
/* 功能：读取指定 J4310 的最新反馈快照；用途：供控制、诊断和在线调参使用；返回 true 表示已有有效反馈。 */
bool J4310_GetFeedback(uint8_t motor_id /* DJI 电机编号 */, j4310_feedback_t *feedback /* 用于写出或读取最新反馈的对象 */);
/* 功能：读取达妙接收诊断快照；用途：向上位机报告有效帧和具体拒绝原因；返回 true 表示参数有效。 */
bool J4310_GetRxDiagnostics(j4310_rx_diagnostics_t *diagnostics /* 用于写出诊断统计的对象 */);
/* 功能：启用或关闭指定 J4310 的 MIT 在线调参；用途：切换固定和动态 kp、kd；返回 true 表示设置成功。 */
bool J4310_SetOnlineMitEnabled(uint8_t motor_id /* DJI 电机编号 */, bool enabled /* 是否启用对应功能 */);
/* 功能：读取指定 J4310 的 MIT 在线调参状态；用途：诊断当前增益和收敛情况；返回 true 表示状态已写出。 */
bool J4310_GetOnlineMitState(uint8_t motor_id /* DJI 电机编号 */,
                             j4310_online_mit_state_t *state /* 需要检查或上报的当前状态 */);

#endif
