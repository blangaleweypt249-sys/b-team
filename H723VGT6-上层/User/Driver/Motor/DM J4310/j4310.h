/**
 * @file j4310.h
 * @brief 定义 J4310 电机协议、状态数据和控制接口。
 */

#ifndef J4310_H
#define J4310_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define J4310_MAX_MOTOR_COUNT  16U

typedef enum
{
    J4310_MODE_MIT = 0x000U,
    J4310_MODE_POSITION_VELOCITY = 0x100U,
    J4310_MODE_VELOCITY = 0x200U
} j4310_mode_t;

typedef struct
{
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
} j4310_limits_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint8_t fault;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
} j4310_feedback_t;

typedef struct
{
    bool enabled;
    float base_kp;
    float base_kd;
    float applied_kp;
    float applied_kd;
} j4310_online_mit_state_t;

typedef enum
{
    J4310_RX_NONE = 0U,
    J4310_RX_ACCEPTED = 1U,
    J4310_RX_REJECTED_FORMAT = 2U,
    J4310_RX_REJECTED_MASTER_ID = 3U,
    J4310_RX_REJECTED_FEEDBACK_ID = 4U
} j4310_rx_result_t;

typedef struct
{
    uint32_t frames_seen;
    uint32_t accepted_frames;
    uint32_t rejected_format_frames;
    uint32_t rejected_master_id_frames;
    uint32_t rejected_feedback_id_frames;
    uint16_t last_can_id;
    uint8_t last_dlc;
    uint8_t last_data0;
    j4310_rx_result_t last_result;
} j4310_rx_diagnostics_t;

/* 功能：清空全部 J4310 驱动上下文；用途：在注册电机前复位驱动状态；无返回值表示上下文表已初始化。 */
void J4310_Init(void);
/* 功能：注册一台 J4310 及其模式和物理限制；用途：建立节点运行上下文；返回 true 表示注册成功。 */
bool J4310_AddMotor(uint8_t motor_id,
                    uint16_t master_id,
                    uint8_t feedback_id,
                    j4310_mode_t mode,
                    const j4310_limits_t *limits);
/* 功能：修改已注册 J4310 的控制模式；用途：切换 MIT、位置速度或速度控制；返回 true 表示模式已接受。 */
bool J4310_SetMode(uint8_t motor_id, j4310_mode_t mode);
/* 功能：构造 J4310 使能帧；用途：让目标节点进入可控状态；返回 true 表示构帧成功。 */
bool J4310_BuildEnable(uint8_t motor_id, can_frame_t *frame);
/* 功能：构造 J4310 失能帧并复位在线调参；用途：安全停止目标节点；返回 true 表示构帧成功。 */
bool J4310_BuildDisable(uint8_t motor_id, can_frame_t *frame);
/* 功能：构造 J4310 清除故障帧；用途：请求节点退出故障状态；返回 true 表示构帧成功。 */
bool J4310_BuildClearFault(uint8_t motor_id, can_frame_t *frame);
/* 功能：构造 J4310 保存当前位置为零点的帧；用途：执行机械零点标定；返回 true 表示构帧成功。 */
bool J4310_BuildSaveZero(uint8_t motor_id, can_frame_t *frame);
/* 功能：构造 J4310 MIT 五参数控制帧；用途：发送位置、速度、刚度、阻尼和转矩目标；返回 true 表示参数有效并构帧完成。 */
bool J4310_BuildMit(uint8_t motor_id,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd,
                    float torque_nm,
                    can_frame_t *frame);
/* 功能：构造 J4310 位置速度模式帧；用途：发送位置与速度上限目标；返回 true 表示构帧成功。 */
bool J4310_BuildPositionVelocity(uint8_t motor_id,
                                 float position_rad,
                                 float velocity_rad_s,
                                 can_frame_t *frame);
/* 功能：构造 J4310 纯速度模式帧；用途：发送目标转速；返回 true 表示构帧成功。 */
bool J4310_BuildVelocity(uint8_t motor_id,
                         float velocity_rad_s,
                         can_frame_t *frame);
/* 功能：设置 J4310 软件转矩限制；用途：在构造控制帧时进一步约束输出；返回 true 表示限制合法并已保存。 */
bool J4310_SetTorqueLimit(uint8_t motor_id, float torque_limit_nm);
/* 功能：解析 J4310 CAN 反馈并更新时间和故障信息；用途：维护闭环反馈快照；返回 true 表示该帧属于已注册节点且解析成功。 */
bool J4310_OnFrame(const can_frame_t *frame, uint32_t tick_ms);
/* 功能：读取指定 J4310 的最新反馈快照；用途：供控制、诊断和在线调参使用；返回 true 表示已有有效反馈。 */
bool J4310_GetFeedback(uint8_t motor_id, j4310_feedback_t *feedback);
/* 功能：读取达妙接收诊断快照；用途：向上位机报告有效帧和具体拒绝原因；返回 true 表示参数有效。 */
bool J4310_GetRxDiagnostics(j4310_rx_diagnostics_t *diagnostics);
/* 功能：启用或关闭指定 J4310 的 MIT 在线调参；用途：切换固定和动态 kp、kd；返回 true 表示设置成功。 */
bool J4310_SetOnlineMitEnabled(uint8_t motor_id, bool enabled);
/* 功能：读取指定 J4310 的 MIT 在线调参状态；用途：诊断当前增益和收敛情况；返回 true 表示状态已写出。 */
bool J4310_GetOnlineMitState(uint8_t motor_id,
                             j4310_online_mit_state_t *state);

#endif
