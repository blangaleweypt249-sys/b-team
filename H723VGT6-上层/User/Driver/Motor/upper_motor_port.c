/**
 * @file upper_motor_port.c
 * @brief 实现上层电机管理器到各型号驱动和 CAN 总线的适配。
 */

#include "upper_motor_port.h"

#include <string.h>

#include "bsp_can.h"
#include "DJI/dji_group.h"
#include "DM J4310/j4310.h"
#include "M2006/m2006.h"
#include "M3508/m3508.h"

#define UPPER_MOTOR_PORT_MAX_NODE_ID  255U /**< 电机端口按节点编号建立索引时允许的最大节点号。 */
#define UPPER_CAN_BUS_COUNT            3U /**< 上层电机端口管理的 CAN 总线数量。 */
#define UPPER_DJI_GROUP_COUNT         2U /**< 上层电机端口分别用于 M3508 和 M2006 的 DJI 电流组数量。 */
#define UPPER_DJI_ENCODER_COUNTS      8192LL /**< 电机转子编码器每圈的计数值。 */
#define UPPER_DJI_TWO_PI              6.28318530718f /**< 角度换算使用的二倍圆周率数值。 */
#define UPPER_J4310_PI                3.14159265359f /**< 角度换算使用的圆周率数值。 */
#define UPPER_J4310_TWO_PI            6.28318530718f /**< 角度换算使用的二倍圆周率数值。 */
#define UPPER_J4310_STATE_ENABLED     0x01U /**< J4310 反馈状态中表示电机已经使能的状态值。 */
#define UPPER_J4310_ENABLE_RETRY_MS   20U /**< J4310 使能命令发送失败后的重试间隔，单位：毫秒。 */

#define UPPER_CONTROL_PERIOD_MS                 1U /**< 上层电机控制状态机的执行周期，单位：毫秒。 */
#define UPPER_CONTROL_WATCHDOGS_ENABLED         0U /**< 上层电机端口是否启用命令与反馈超时看门狗。 */
#define UPPER_PC_TIMEOUT_MS                   200U /**< 超过该时间未收到有效上位机数据后判定会话超时，单位：毫秒。 */
#define UPPER_MOTOR_FEEDBACK_TIMEOUT_MS        50U /**< 电机反馈超过该时间未更新后上报离线故障，单位：毫秒。 */

#define UPPER_J4310_POSITION_MAX_RAD           12.5f /**< 电机端口编码 J4310 MIT 位置字段时使用的满量程，单位：弧度。 */
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f /**< 机械臂 J4310 关节 MIT 协议速度字段映射的满量程，单位：弧度每秒。 */
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f /**< 机械臂 J4310 关节 MIT 协议转矩字段映射的满量程，单位：牛米。 */
#define UPPER_J4310_DIRECTION_SIGN             (-1.0f) /**< 机械臂 J4310 关节从电机坐标系换算到机构坐标系时使用的方向系数。 */

#define UPPER_M3508_1_DIRECTION_SIGN            (1.0f) /**< 第一台机械臂 M3508 从电机坐标系换算到机构坐标系的方向系数。 */
#define UPPER_M3508_2_DIRECTION_SIGN           (-1.0f) /**< 第二台机械臂 M3508 输出轴从电机坐标系换算到机构坐标系时使用的方向系数。 */
#define UPPER_M3508_CURRENT_LIMIT_A              3.0f /**< 机械臂 M3508 输出轴软件限制的最大输出电流，单位：安培。 */
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f /**< 机械臂 M3508 输出轴执行位置轨迹时允许的最大速度，单位：弧度每秒。 */
#define UPPER_M3508_POSITION_PID_OUTPUT_LIMIT_RAD_S \
    UPPER_M3508_POSITION_VEL_LIMIT_RAD_S /**< 机械臂 M3508 输出轴位置环输出的目标速度绝对值上限，单位：弧度每秒。 */
#define UPPER_M3508_ACCEL_LIMIT_RAD_S2          62.832f /**< 机械臂 M3508 输出轴轨迹规划使用的最大加速度，单位：弧度每二次方秒。 */
#define UPPER_M3508_SPEED_KP                     1.3988f /**< 机械臂 M3508 输出轴速度环的比例增益。 */
#define UPPER_M3508_SPEED_KI                     0.9325f /**< 机械臂 M3508 输出轴速度环的积分增益。 */
#define UPPER_M3508_SPEED_KD                     0.0f /**< 机械臂 M3508 输出轴速度环的微分增益。 */
#define UPPER_M3508_SPEED_I_LIMIT                6.0f /**< 机械臂 M3508 输出轴速度环积分项的绝对值上限。 */
#define UPPER_M3508_POSITION_KP                 10.4720f /**< 机械臂 M3508 输出轴位置环的比例增益。 */
#define UPPER_M3508_POSITION_KI                  0.0f /**< 机械臂 M3508 输出轴位置环的积分增益。 */
#define UPPER_M3508_POSITION_KD                  0.0f /**< 机械臂 M3508 输出轴位置环的微分增益。 */
#define UPPER_M3508_POSITION_I_LIMIT             0.0f /**< 机械臂 M3508 输出轴位置环积分项的绝对值上限。 */

#define UPPER_M2006_CURRENT_LIMIT_A             10.0f /**< M2006 输出轴软件限制的最大输出电流，单位：安培。 */
#define UPPER_M2006_POSITION_CUTOFF_RAD          6.45771823238f /**< M2006 输出轴的位置积分分离开始生效的误差阈值，单位：弧度。 */
#define UPPER_GRIPPER_M2006_POSITION_CUTOFF_RAD 12.74090353956f /**< 夹爪机构的位置积分分离开始生效的误差阈值，单位：弧度。 */
#define UPPER_M2006_POSITION_VEL_LIMIT_RAD_S     5.235987756f /**< M2006 输出轴执行位置轨迹时允许的最大速度，单位：弧度每秒。 */
#define UPPER_M2006_POSITION_PID_OUTPUT_LIMIT_RAD_S \
    UPPER_M2006_POSITION_VEL_LIMIT_RAD_S /**< M2006 输出轴位置环输出的目标速度绝对值上限，单位：弧度每秒。 */
#define UPPER_M2006_ACCEL_LIMIT_RAD_S2          62.832f /**< M2006 输出轴轨迹规划使用的最大加速度，单位：弧度每二次方秒。 */
#define UPPER_M2006_SPEED_KP                     3.342253805f /**< M2006 输出轴速度环的比例增益。 */
#define UPPER_M2006_SPEED_KI                     2.387324146f /**< M2006 输出轴速度环的积分增益。 */
#define UPPER_M2006_SPEED_KD                     0.0f /**< M2006 输出轴速度环的微分增益。 */
#define UPPER_M2006_SPEED_I_LIMIT                0.052359878f /**< M2006 输出轴速度环积分项的绝对值上限。 */
#define UPPER_M2006_POSITION_KP                 94.247779608f /**< M2006 输出轴位置环的比例增益。 */
#define UPPER_M2006_POSITION_KI                 52.359877560f /**< M2006 输出轴位置环的积分增益。 */
#define UPPER_M2006_POSITION_KD                  0.523598776f /**< M2006 输出轴位置环的微分增益。 */
#define UPPER_M2006_POSITION_I_LIMIT             0.002f /**< M2006 输出轴位置环积分项的绝对值上限。 */
#define UPPER_GATE_M2006_DIRECTION_SIGN         (-1.0f) /**< 挡板机构从电机坐标系换算到机构坐标系时使用的方向系数。 */

/** 标识本次准备发送的 J4310 帧用途。 */
typedef enum
{
    UPPER_J4310_TX_CONTROL = 0U, /**< 本帧下发 J4310 MIT 控制目标。 */
    UPPER_J4310_TX_ENABLE, /**< 本帧请求 J4310 进入使能状态。 */
    UPPER_J4310_TX_DISABLE /**< 本帧请求 J4310 退出使能状态。 */
} upper_j4310_tx_type_t;

static const motor_cfg_t *upper_motor_cfg_ref;
static size_t upper_motor_count;
static uint32_t upper_motor_tick_ms;
static bool upper_motor_initialized;
static bool j4310_enabled[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool j4310_enable_pending[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool j4310_fault_disable_sent[
    UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static uint32_t j4310_enable_last_sent_ms[
    UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static j4310_mode_t j4310_mode[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool j4310_position_valid[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static float j4310_last_raw_position_rad[
    UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static float j4310_continuous_position_rad[
    UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static float j4310_position_offset_rad[
    UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool dji_group_dirty[UPPER_CAN_BUS_COUNT][UPPER_DJI_GROUP_COUNT];
static bool upper_motor_active[MOTOR_MANAGER_MAX_COUNT];
static bool upper_motor_scheduled[MOTOR_MANAGER_MAX_COUNT];
static uint32_t upper_motor_active_since_ms[MOTOR_MANAGER_MAX_COUNT];
static uint8_t j4310_last_fault[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static upper_motor_fault_t upper_motor_pending_fault;
static bool upper_motor_fault_pending;
static uint32_t upper_motor_fault_sequence;
static upper_j4310_tx_diagnostic_t j4310_tx_diagnostic;

/* 功能：判断电机型号是否属于 DJI 分组电流协议；用途：区分 M2006/M3508 与独立帧电机；返回 true 表示是 DJI 型号。 */
static bool UpperMotorPort_IsDjiModel(motor_model_t model /**< 待判断是否属于 DJI 系列的电机型号 */)
{
    return (model == MOTOR_MODEL_M3508) ||
           (model == MOTOR_MODEL_M2006);
}

/* 功能：读取指定电机在机械安装中的方向符号；用途：统一逻辑目标与物理转向；返回值表示目标乘数。 */
static float UpperMotorPort_DirectionSign(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */)
{
    if ((cfg != NULL) && (cfg->model == MOTOR_MODEL_M3508) &&
        (cfg->can_bus == CAN_BUS_ARM_M3508))
    {
        if (cfg->node_id == NODE_ARM_M3508_1)
        {
            return UPPER_M3508_1_DIRECTION_SIGN;
        }
        if (cfg->node_id == NODE_ARM_M3508_2)
        {
            return UPPER_M3508_2_DIRECTION_SIGN;
        }
    }
    if ((cfg != NULL) && (cfg->model == MOTOR_MODEL_M2006) &&
        (cfg->can_bus == CAN_BUS_AUX) &&
        (cfg->node_id == NODE_GATE_M2006))
    {
        return UPPER_GATE_M2006_DIRECTION_SIGN;
    }
    return 1.0f;
}

/* 功能：选择指定 M2006 电机的位置安全阈值；用途：为夹爪和通用机构应用不同限位；返回值表示位置截止值。 */
static float UpperMotorPort_M2006PositionCutoff(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */)
{
    if ((cfg != NULL) && (cfg->model == MOTOR_MODEL_M2006) &&
        (cfg->can_bus == CAN_BUS_AUX) &&
        (cfg->node_id == NODE_GRIPPER_M2006))
    {
        return UPPER_GRIPPER_M2006_POSITION_CUTOFF_RAD;
    }
    return UPPER_M2006_POSITION_CUTOFF_RAD;
}

/* 功能：将 J4310 相邻位置差归一化到半圈范围；用途：处理单圈编码器跨零点变化；返回值表示最短角度增量。 */
static float UpperMotorPort_WrapJ4310Delta(float delta_rad /**< 相邻反馈位置之间的角度增量，单位：弧度 */)
{
    while (delta_rad > UPPER_J4310_PI)
    {
        delta_rad -= UPPER_J4310_TWO_PI;
    }
    while (delta_rad < -UPPER_J4310_PI)
    {
        delta_rad += UPPER_J4310_TWO_PI;
    }
    return delta_rad;
}

/* 在单圈编码器分支变化时保持逻辑关节角连续。
   只要两次接收采样之间的运动小于半圈，该处理就是有效的。 */
/* 功能：根据最新单圈反馈更新 J4310 连续位置；用途：跨越编码器分支时保持逻辑关节角连续；无返回值表示位置状态已更新。 */
static void UpperMotorPort_UpdateJ4310Position(uint8_t can_bus /**< CAN 总线编号 */,
                                                uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    size_t index;

    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg;
        j4310_feedback_t feedback;
        float raw_position_rad;
        float delta_rad;

        cfg = &upper_motor_cfg_ref[index];
        if ((cfg->model != MOTOR_MODEL_J4310) ||
            (cfg->can_bus != can_bus) ||
            !J4310_GetFeedback(cfg->node_id, &feedback) ||
            (feedback.updated_at_ms != tick_ms))
        {
            continue;
        }

        raw_position_rad = feedback.position_rad *
                           UPPER_J4310_DIRECTION_SIGN;
        if (!j4310_position_valid[cfg->node_id])
        {
            j4310_position_valid[cfg->node_id] = true;
            j4310_continuous_position_rad[cfg->node_id] =
                raw_position_rad;
        }
        else
        {
            delta_rad = raw_position_rad -
                        j4310_last_raw_position_rad[cfg->node_id];
            j4310_continuous_position_rad[cfg->node_id] +=
                UpperMotorPort_WrapJ4310Delta(delta_rad);
        }
        j4310_last_raw_position_rad[cfg->node_id] = raw_position_rad;
        j4310_position_offset_rad[cfg->node_id] =
            raw_position_rad -
            j4310_continuous_position_rad[cfg->node_id];
    }
}

/* 功能：清空指定 J4310 的连续位置跟踪状态；用途：电机掉电或重新置零后重新建立基准；无返回值表示跟踪状态已复位。 */
static void UpperMotorPort_ResetJ4310Position(uint8_t node_id /**< 电机协议节点编号 */)
{
    j4310_position_valid[node_id] = false;
    j4310_last_raw_position_rad[node_id] = 0.0f;
    j4310_continuous_position_rad[node_id] = 0.0f;
    j4310_position_offset_rad[node_id] = 0.0f;
}

/* 功能：区分 J4310 运行状态与协议故障码；用途：避免将 0x1 已使能状态误判为故障；返回 true 表示需要失能。 */
static bool UpperMotorPort_IsJ4310Fault(uint8_t state /**< J4310 反馈帧中的电机状态码 */)
{
    return (state == 0x02U) || (state == 0x03U) ||
           (state == 0x05U) || (state == 0x07U) ||
           ((state >= 0x08U) && (state <= 0x0EU));
}

/* 功能：校验上层电机拓扑、地址和协议约束；用途：启动前排除重复节点及不支持组合；返回 true 表示配置可路由。 */
static bool UpperMotorPort_CheckCfg(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                    size_t motor_count /**< 调用方提供的电机配置数量 */)
{
    size_t index;

    if ((cfg == NULL) || (motor_count == 0U) ||
        (motor_count > MOTOR_MANAGER_MAX_COUNT))
    {
        return false;
    }

    for (index = 0U; index < motor_count; index++)
    {
        size_t previous;

        if ((cfg[index].can_bus == 0U) ||
            (cfg[index].can_bus > UPPER_CAN_BUS_COUNT) ||
            (cfg[index].period_ms != UPPER_CONTROL_PERIOD_MS) ||
            (cfg[index].phase_ms != 0U) || !cfg[index].protocol_ready)
        {
            return false;
        }

        if (UpperMotorPort_IsDjiModel(cfg[index].model))
        {
            if ((cfg[index].node_id == 0U) || (cfg[index].node_id > 8U))
            {
                return false;
            }
        }
        else if (cfg[index].model == MOTOR_MODEL_J4310)
        {
            if ((cfg[index].node_id == 0U) ||
                (cfg[index].node_id > 0x0FU))
            {
                return false;
            }
        }
        else
        {
            return false;
        }

        for (previous = 0U; previous < index; previous++)
        {
            if ((cfg[previous].can_bus == cfg[index].can_bus) &&
                (cfg[previous].node_id == cfg[index].node_id) &&
                ((cfg[previous].model == cfg[index].model) ||
                 (UpperMotorPort_IsDjiModel(cfg[previous].model) &&
                  UpperMotorPort_IsDjiModel(cfg[index].model))))
            {
                return false;
            }
        }
    }

    return true;
}

/* 功能：通过 BSP 发送一帧并更新发送计数；用途：统一所有电机协议的实际 CAN 输出；返回 true 表示底层发送成功。 */
static bool UpperMotorPort_SendFrame(uint8_t can_bus /**< CAN 总线编号 */,
                                     const can_frame_t *frame /**< 待发送的 CAN 数据帧 */)
{
    return BspCan_Send(can_bus, frame);
}

/* 功能：查找电机配置指针在当前拓扑中的索引；用途：访问对应运行时状态；返回电机总数表示未找到。 */
static size_t UpperMotorPort_FindCfg(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */)
{
    size_t index;

    if ((cfg == NULL) || (upper_motor_cfg_ref == NULL))
    {
        return upper_motor_count;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if (&upper_motor_cfg_ref[index] == cfg)
        {
            return index;
        }
    }
    return upper_motor_count;
}

/* 功能：判断反馈时间戳是否仍在新鲜窗口内；用途：识别电机离线或反馈超时；返回 true 表示反馈可用于控制。 */
static bool UpperMotorPort_FeedbackFresh(uint32_t updated_at_ms /**< 反馈最近一次更新的系统毫秒时刻 */,
                                         uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    int32_t age_ms;

    age_ms = (int32_t)(tick_ms - updated_at_ms);
    return (age_ms >= -(int32_t)UPPER_CONTROL_PERIOD_MS) &&
           (age_ms <= (int32_t)UPPER_MOTOR_FEEDBACK_TIMEOUT_MS);
}

/* 功能：发送 J4310 帧并记录实际入队结果；用途：证明使能帧是否进入 FDCAN1 TX 队列；返回 true 表示 HAL 已接受该帧。 */
static bool UpperMotorPort_SendJ4310Frame(
    uint8_t can_bus /**< CAN 总线编号 */,
    const can_frame_t *frame /**< 待发送的 J4310 CAN 数据帧 */,
    upper_j4310_tx_type_t type /**< J4310 发送帧的统计类别 */)
{
    bool success;

    if (frame == NULL)
    {
        return false;
    }
    j4310_tx_diagnostic.attempted_frames++;
    j4310_tx_diagnostic.last_can_id = (uint16_t)frame->id;
    j4310_tx_diagnostic.last_dlc = frame->dlc;
    j4310_tx_diagnostic.last_data7 = frame->data[7];
    success = UpperMotorPort_SendFrame(can_bus, frame);
    if (!success)
    {
        j4310_tx_diagnostic.failed_frames++;
        return false;
    }

    j4310_tx_diagnostic.queued_frames++;
    if (type == UPPER_J4310_TX_ENABLE)
    {
        j4310_tx_diagnostic.enable_frames++;
    }
    else if (type == UPPER_J4310_TX_DISABLE)
    {
        j4310_tx_diagnostic.disable_frames++;
    }
    else
    {
        j4310_tx_diagnostic.mit_frames++;
    }
    return true;
}

/* 功能：记录新的电机故障事件；用途：保存待上报故障的型号、地址、错误码和时刻；无返回值表示故障槽已更新。 */
static void UpperMotorPort_RecordFault(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                       uint8_t error_code /**< 本次记录的电机故障码 */,
                                       uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    if ((cfg == NULL) || (error_code == 0U))
    {
        return;
    }

    upper_motor_fault_sequence++;
    upper_motor_pending_fault.model = cfg->model;
    upper_motor_pending_fault.can_bus = cfg->can_bus;
    upper_motor_pending_fault.node_id = cfg->node_id;
    upper_motor_pending_fault.error_code = error_code;
    upper_motor_pending_fault.tick_ms = tick_ms;
    upper_motor_pending_fault.sequence = upper_motor_fault_sequence;
    upper_motor_fault_pending = true;
}

/* 功能：由外部模块注入电机故障；用途：复用端口层的故障去重和上报队列；无返回值表示事件已尝试记录。 */
void UpperMotorPort_RecordExternalFault(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                        uint8_t error_code /**< 外部模块上报的电机故障码 */,
                                        uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    UpperMotorPort_RecordFault(cfg, error_code, tick_ms);
}

/* 功能：标记指定 DJI 四电机组本周期需要发送；用途：把单电机目标汇总为分组帧；返回 true 表示地址合法并已标记。 */
static bool UpperMotorPort_MarkDjiGroup(uint8_t can_bus /**< CAN 总线编号 */, uint8_t node_id /**< 电机协议节点编号 */)
{
    uint32_t group_index;

    if ((can_bus == 0U) || (can_bus > UPPER_CAN_BUS_COUNT) ||
        (node_id == 0U) || (node_id > 8U))
    {
        return false;
    }

    group_index = (uint32_t)(node_id - 1U) / DJI_GROUP_MOTOR_COUNT;
    dji_group_dirty[can_bus - 1U][group_index] = true;
    return true;
}

/* 功能：构造并发送一个已标记的 DJI 电流组帧；用途：一次下发四个槽位的 M2006/M3508 电流；返回 true 表示无需发送或发送成功。 */
static bool UpperMotorPort_FlushDjiGroup(uint8_t can_bus /**< CAN 总线编号 */,
                                         uint32_t group_index /**< DJI 电机控制组的数组下标 */)
{
    can_frame_t frame;
    int16_t current_raw[DJI_GROUP_MOTOR_COUNT] = {0};
    uint8_t start_motor_id;
    size_t motor_index;
    bool group_valid;

    if ((can_bus == 0U) || (can_bus > UPPER_CAN_BUS_COUNT) ||
        (group_index >= UPPER_DJI_GROUP_COUNT))
    {
        return false;
    }
    if (!dji_group_dirty[can_bus - 1U][group_index])
    {
        return true;
    }

    start_motor_id =
        (uint8_t)(group_index * DJI_GROUP_MOTOR_COUNT + 1U);
    group_valid = true;
    for (motor_index = 0U;
         motor_index < upper_motor_count;
         motor_index++)
    {
        const motor_cfg_t *cfg;
        uint32_t slot;

        cfg = &upper_motor_cfg_ref[motor_index];
        if ((cfg->can_bus != can_bus) ||
            (cfg->node_id < start_motor_id) ||
            (cfg->node_id >=
             (uint8_t)(start_motor_id + DJI_GROUP_MOTOR_COUNT)))
        {
            continue;
        }

        /* 未激活的电机必须占用字面值为零电流的槽位，即使驱动上下文仍保留旧的位置命令。 */
        if (!upper_motor_active[motor_index] ||
            !upper_motor_scheduled[motor_index])
        {
            continue;
        }

        slot = (uint32_t)(cfg->node_id - start_motor_id);
        if (cfg->model == MOTOR_MODEL_M3508)
        {
            if (!M3508_CalcCurrentRaw(can_bus,
                                      cfg->node_id,
                                      upper_motor_tick_ms,
                                      &current_raw[slot]))
            {
                group_valid = false;
            }
        }
        else if (cfg->model == MOTOR_MODEL_M2006)
        {
            m2006_feedback_t feedback;
            float position_cutoff_rad;

            position_cutoff_rad = UpperMotorPort_M2006PositionCutoff(cfg);
            if (M2006_GetFeedback(can_bus, cfg->node_id, &feedback) &&
                ((feedback.output_pos_rad <
                  -position_cutoff_rad) ||
                 (feedback.output_pos_rad >
                  position_cutoff_rad)))
            {
                current_raw[slot] = 0;
                continue;
            }
            if (!M2006_CalcCurrentRaw(can_bus,
                                      cfg->node_id,
                                      upper_motor_tick_ms,
                                      &current_raw[slot]))
            {
                group_valid = false;
            }
        }
    }

    if (!group_valid ||
        !DjiGroup_BuildFrame(start_motor_id, current_raw, &frame) ||
        !UpperMotorPort_SendFrame(can_bus, &frame))
    {
        return false;
    }
    dji_group_dirty[can_bus - 1U][group_index] = false;
    return true;
}

/* 功能：按命令模式驱动一台 J4310 并管理使能状态；用途：生成特殊帧或运动控制帧；返回 true 表示本次路由成功。 */
static bool UpperMotorPort_SendJ4310(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                     const motor_cmd_t *cmd /**< 待转换为 J4310 协议帧的控制命令 */)
{
    can_frame_t command_frame;
    can_frame_t state_frame;
    j4310_feedback_t feedback;
    j4310_mode_t desired_mode;
    uint32_t group_index;
    bool feedback_valid;
    bool feedback_fresh;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;

    for (group_index = 0U;
         group_index < UPPER_DJI_GROUP_COUNT;
         group_index++)
    {
        if (!UpperMotorPort_FlushDjiGroup(cfg->can_bus, group_index))
        {
            return false;
        }
    }

    feedback_valid = J4310_GetFeedback(cfg->node_id, &feedback);
    feedback_fresh = feedback_valid &&
                     UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                                  upper_motor_tick_ms);
    if (feedback_fresh && UpperMotorPort_IsJ4310Fault(feedback.fault))
    {
        if (j4310_fault_disable_sent[cfg->node_id])
        {
            return true;
        }
        if (J4310_BuildDisable(cfg->node_id, &state_frame) &&
            UpperMotorPort_SendJ4310Frame(cfg->can_bus,
                                          &state_frame,
                                          UPPER_J4310_TX_DISABLE))
        {
            j4310_enabled[cfg->node_id] = false;
            j4310_enable_pending[cfg->node_id] = false;
            j4310_fault_disable_sent[cfg->node_id] = true;
            return true;
        }
        return false;
    }
    if (feedback_fresh)
    {
        j4310_fault_disable_sent[cfg->node_id] = false;
        if (feedback.fault == 0U)
        {
            j4310_enabled[cfg->node_id] = false;
        }
        else if ((feedback.fault == UPPER_J4310_STATE_ENABLED) &&
                 j4310_enable_pending[cfg->node_id] &&
                 ((int32_t)(feedback.updated_at_ms -
                            j4310_enable_last_sent_ms[cfg->node_id]) >= 0))
        {
            j4310_enabled[cfg->node_id] = true;
            j4310_enable_pending[cfg->node_id] = false;
        }
    }
    else if (j4310_fault_disable_sent[cfg->node_id])
    {
        return true;
    }

    if ((cmd->mode == MOTOR_CMD_STOP) ||
        (cmd->mode == MOTOR_CMD_GLOBAL_STOP))
    {
        if (!j4310_enabled[cfg->node_id] &&
            !j4310_enable_pending[cfg->node_id])
        {
            return true;
        }
        if (J4310_BuildDisable(cfg->node_id, &state_frame) &&
            UpperMotorPort_SendJ4310Frame(cfg->can_bus,
                                          &state_frame,
                                          UPPER_J4310_TX_DISABLE))
        {
            j4310_enabled[cfg->node_id] = false;
            j4310_enable_pending[cfg->node_id] = false;
            return true;
        }
        return false;
    }

    if (cmd->mode == MOTOR_CMD_MIT)
    {
        desired_mode = J4310_MODE_MIT;
    }
    else if (cmd->mode == MOTOR_CMD_POSITION_VELOCITY)
    {
        desired_mode = J4310_MODE_POSITION_VELOCITY;
    }
    else if (cmd->mode == MOTOR_CMD_VELOCITY)
    {
        desired_mode = J4310_MODE_VELOCITY;
    }
    else
    {
        return false;
    }

    if (desired_mode != j4310_mode[cfg->node_id])
    {
        if (j4310_enabled[cfg->node_id] ||
            j4310_enable_pending[cfg->node_id])
        {
            if (!J4310_BuildDisable(cfg->node_id, &state_frame) ||
                !UpperMotorPort_SendJ4310Frame(cfg->can_bus,
                                               &state_frame,
                                               UPPER_J4310_TX_DISABLE))
            {
                return false;
            }
            j4310_enabled[cfg->node_id] = false;
            j4310_enable_pending[cfg->node_id] = false;
        }
        if (!J4310_SetMode(cfg->node_id, desired_mode))
        {
            return false;
        }
        j4310_mode[cfg->node_id] = desired_mode;
        return true;
    }

    if (!j4310_enabled[cfg->node_id])
    {
        if (j4310_enable_pending[cfg->node_id] &&
            ((upper_motor_tick_ms -
              j4310_enable_last_sent_ms[cfg->node_id]) <
             UPPER_J4310_ENABLE_RETRY_MS))
        {
            return true;
        }
        if (!J4310_BuildEnable(cfg->node_id, &state_frame) ||
            !UpperMotorPort_SendJ4310Frame(cfg->can_bus,
                                           &state_frame,
                                           UPPER_J4310_TX_ENABLE))
        {
            return false;
        }
        j4310_enable_pending[cfg->node_id] = true;
        j4310_enable_last_sent_ms[cfg->node_id] = upper_motor_tick_ms;
        return true;
    }

    position_rad = cmd->pos_rad;
    if (j4310_position_valid[cfg->node_id])
    {
        position_rad += j4310_position_offset_rad[cfg->node_id];
    }
    position_rad *= UPPER_J4310_DIRECTION_SIGN;
    velocity_rad_s = cmd->vel_rad_s * UPPER_J4310_DIRECTION_SIGN;
    torque_nm = cmd->torque_nm * UPPER_J4310_DIRECTION_SIGN;

    if (desired_mode == J4310_MODE_MIT)
    {
        if (!J4310_BuildMit(cfg->node_id,
                            position_rad,
                            velocity_rad_s,
                            cmd->kp,
                            cmd->kd,
                            torque_nm,
                            &command_frame))
        {
            return false;
        }
    }
    else if (desired_mode == J4310_MODE_POSITION_VELOCITY)
    {
        if (!J4310_BuildPositionVelocity(cfg->node_id,
                                         position_rad,
                                         velocity_rad_s,
                                         &command_frame))
        {
            return false;
        }
    }
    else if (!J4310_BuildVelocity(cfg->node_id,
                                  velocity_rad_s,
                                  &command_frame))
    {
        return false;
    }

    return UpperMotorPort_SendJ4310Frame(cfg->can_bus,
                                         &command_frame,
                                         UPPER_J4310_TX_CONTROL);
}

/* 功能：计算 M3508 当前电流并写入分组槽位；用途：把统一电机命令转换为 DJI 协议输出；返回 true 表示已成功暂存。 */
static bool UpperMotorPort_SendM3508(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                     const motor_cmd_t *cmd /**< 待转换为 M3508 分组电流的控制命令 */)
{
    m3508_mode_t mode;
    float target;

    switch (cmd->mode)
    {
    case MOTOR_CMD_STOP:
    case MOTOR_CMD_GLOBAL_STOP:
        mode = M3508_MODE_STOP;
        target = 0.0f;
        break;

    case MOTOR_CMD_CURRENT:
        mode = M3508_MODE_CURRENT;
        target = cmd->current_a;
        break;

    case MOTOR_CMD_VELOCITY:
        mode = M3508_MODE_VELOCITY;
        target = cmd->vel_rad_s;
        break;

    case MOTOR_CMD_POSITION:
        mode = M3508_MODE_POSITION;
        target = cmd->pos_rad;
        break;

    default:
        return false;
    }

    target *= UpperMotorPort_DirectionSign(cfg);

    if (!M3508_SetTarget(cfg->can_bus,
                         cfg->node_id,
                         mode,
                         target,
                         upper_motor_tick_ms))
    {
        return false;
    }
    return UpperMotorPort_MarkDjiGroup(cfg->can_bus, cfg->node_id);
}

/* 功能：计算 M2006 当前电流并写入分组槽位；用途：把统一电机命令转换为 DJI 协议输出；返回 true 表示已成功暂存。 */
static bool UpperMotorPort_SendM2006(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                                     const motor_cmd_t *cmd /**< 待转换为 M2006 分组电流的控制命令 */)
{
    m2006_mode_t mode;
    float target;

    switch (cmd->mode)
    {
    case MOTOR_CMD_STOP:
    case MOTOR_CMD_GLOBAL_STOP:
        mode = M2006_MODE_STOP;
        target = 0.0f;
        break;

    case MOTOR_CMD_CURRENT:
        mode = M2006_MODE_CURRENT;
        target = cmd->current_a;
        break;

    case MOTOR_CMD_VELOCITY:
        mode = M2006_MODE_VELOCITY;
        target = cmd->vel_rad_s;
        break;

    case MOTOR_CMD_POSITION:
        mode = M2006_MODE_POSITION;
        target = cmd->pos_rad;
        break;

    default:
        return false;
    }

    target *= UpperMotorPort_DirectionSign(cfg);

    if (!M2006_SetTarget(cfg->can_bus,
                         cfg->node_id,
                         mode,
                         target,
                         upper_motor_tick_ms))
    {
        return false;
    }
    return UpperMotorPort_MarkDjiGroup(cfg->can_bus, cfg->node_id);
}

/* 功能：初始化电机协议驱动、拓扑状态和故障状态；用途：建立上层统一电机端口；返回 true 表示全部型号初始化成功。 */
bool UpperMotorPort_Init(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */, size_t motor_count /**< 调用方提供的电机配置数量 */)
{
    j4310_limits_t j4310_limits;
    m2006_cfg_t m2006_driver_cfg;
    m3508_cfg_t m3508_driver_cfg;
    size_t index;

    upper_motor_initialized = false;
    upper_motor_cfg_ref = NULL;
    upper_motor_count = 0U;
    if (!UpperMotorPort_CheckCfg(cfg, motor_count))
    {
        return false;
    }

    (void)memset(j4310_enabled, 0, sizeof(j4310_enabled));
    (void)memset(j4310_enable_pending, 0, sizeof(j4310_enable_pending));
    (void)memset(j4310_fault_disable_sent,
                 0,
                 sizeof(j4310_fault_disable_sent));
    (void)memset(j4310_enable_last_sent_ms,
                 0,
                 sizeof(j4310_enable_last_sent_ms));
    (void)memset(j4310_mode, 0, sizeof(j4310_mode));
    (void)memset(j4310_position_valid,
                 0,
                 sizeof(j4310_position_valid));
    (void)memset(j4310_last_raw_position_rad,
                 0,
                 sizeof(j4310_last_raw_position_rad));
    (void)memset(j4310_continuous_position_rad,
                 0,
                 sizeof(j4310_continuous_position_rad));
    (void)memset(j4310_position_offset_rad,
                 0,
                 sizeof(j4310_position_offset_rad));
    (void)memset(dji_group_dirty, 0, sizeof(dji_group_dirty));
    (void)memset(upper_motor_active, 0, sizeof(upper_motor_active));
    (void)memset(upper_motor_scheduled, 0, sizeof(upper_motor_scheduled));
    (void)memset(upper_motor_active_since_ms,
                 0,
                 sizeof(upper_motor_active_since_ms));
    (void)memset(j4310_last_fault, 0, sizeof(j4310_last_fault));
    (void)memset(&upper_motor_pending_fault,
                 0,
                 sizeof(upper_motor_pending_fault));
    upper_motor_fault_pending = false;
    upper_motor_fault_sequence = 0U;
    (void)memset(&j4310_tx_diagnostic, 0, sizeof(j4310_tx_diagnostic));
    J4310_Init();

    j4310_limits = (j4310_limits_t)
    {
        UPPER_J4310_POSITION_MAX_RAD,
        UPPER_J4310_VELOCITY_MAX_RAD_S,
        UPPER_J4310_TORQUE_MAP_MAX_NM
    };
    for (index = 0U; index < motor_count; index++)
    {
        if ((cfg[index].model == MOTOR_MODEL_J4310) &&
            (!J4310_AddMotor(cfg[index].node_id,
                             CAN_J4310_MASTER_ID,
                             cfg[index].node_id & 0x0FU,
                             J4310_MODE_MIT,
                             &j4310_limits) ||
             !J4310_SetOnlineMitEnabled(cfg[index].node_id, true)))
        {
            return false;
        }
    }

    m3508_driver_cfg = (m3508_cfg_t)
    {
        .current_limit_a = UPPER_M3508_CURRENT_LIMIT_A,
        .position_vel_limit_rad_s = UPPER_M3508_POSITION_VEL_LIMIT_RAD_S,
        .acceleration_limit_rad_s2 = UPPER_M3508_ACCEL_LIMIT_RAD_S2,
        .feedback_timeout_ms = UPPER_CONTROL_WATCHDOGS_ENABLED ?
                               UPPER_MOTOR_FEEDBACK_TIMEOUT_MS : 0U,
        .command_timeout_ms = UPPER_CONTROL_WATCHDOGS_ENABLED ?
                              UPPER_PC_TIMEOUT_MS : 0U,
        .speed_pid =
        {
            UPPER_M3508_SPEED_KP,
            UPPER_M3508_SPEED_KI,
            UPPER_M3508_SPEED_KD,
            UPPER_M3508_SPEED_I_LIMIT,
            UPPER_M3508_CURRENT_LIMIT_A
        },
        .position_pid =
        {
            UPPER_M3508_POSITION_KP,
            UPPER_M3508_POSITION_KI,
            UPPER_M3508_POSITION_KD,
            UPPER_M3508_POSITION_I_LIMIT,
            UPPER_M3508_POSITION_PID_OUTPUT_LIMIT_RAD_S
        }
    };
    if (!M3508_Init(&m3508_driver_cfg))
    {
        return false;
    }

    m2006_driver_cfg = (m2006_cfg_t)
    {
        .current_limit_a = UPPER_M2006_CURRENT_LIMIT_A,
        .position_vel_limit_rad_s = UPPER_M2006_POSITION_VEL_LIMIT_RAD_S,
        .acceleration_limit_rad_s2 = UPPER_M2006_ACCEL_LIMIT_RAD_S2,
        .feedback_timeout_ms = UPPER_CONTROL_WATCHDOGS_ENABLED ?
                               UPPER_MOTOR_FEEDBACK_TIMEOUT_MS : 0U,
        .command_timeout_ms = UPPER_CONTROL_WATCHDOGS_ENABLED ?
                              UPPER_PC_TIMEOUT_MS : 0U,
        .speed_pid =
        {
            UPPER_M2006_SPEED_KP,
            UPPER_M2006_SPEED_KI,
            UPPER_M2006_SPEED_KD,
            UPPER_M2006_SPEED_I_LIMIT,
            UPPER_M2006_CURRENT_LIMIT_A
        },
        .position_pid =
        {
            UPPER_M2006_POSITION_KP,
            UPPER_M2006_POSITION_KI,
            UPPER_M2006_POSITION_KD,
            UPPER_M2006_POSITION_I_LIMIT,
            UPPER_M2006_POSITION_PID_OUTPUT_LIMIT_RAD_S
        }
    };
    if (!M2006_Init(&m2006_driver_cfg))
    {
        return false;
    }

    upper_motor_cfg_ref = cfg;
    upper_motor_count = motor_count;
    upper_motor_tick_ms = 0U;
    upper_motor_initialized = true;
    return true;
}

/* 功能：开始新的电机发送周期并清空 DJI 分组暂存；用途：保证各槽位只包含本周期命令；无返回值表示周期状态已复位。 */
void UpperMotorPort_BeginCycle(uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    /* 启动和空闲周期只读。只有传递给 UpperMotorPort_Send 的显式命令
     * 才能安排电机 CAN 帧。 */
    upper_motor_tick_ms = tick_ms;
    (void)memset(upper_motor_scheduled, 0, sizeof(upper_motor_scheduled));
}

/* 功能：按配置型号路由统一电机命令；用途：作为 MotorManager 的发送回调；返回 true 表示命令被对应协议接受。 */
bool UpperMotorPort_Send(const motor_cfg_t *cfg /**< 当前电机的型号、总线及节点配置 */,
                         const motor_cmd_t *cmd /**< 待按电机型号路由的统一控制命令 */,
                         void *user_data /**< 调用回调函数时传递的用户上下文 */)
{
    size_t motor_index;
    bool active;

    (void)user_data;
    if (!upper_motor_initialized || (cfg == NULL) || (cmd == NULL))
    {
        return false;
    }

    motor_index = UpperMotorPort_FindCfg(cfg);
    if (motor_index >= upper_motor_count)
    {
        return false;
    }
    active = (cmd->mode != MOTOR_CMD_STOP) &&
             (cmd->mode != MOTOR_CMD_GLOBAL_STOP);
    if (active && !upper_motor_active[motor_index])
    {
        upper_motor_active_since_ms[motor_index] = upper_motor_tick_ms;
    }
    upper_motor_active[motor_index] = active;
    upper_motor_scheduled[motor_index] = true;

    switch (cfg->model)
    {
    case MOTOR_MODEL_J4310:
        return UpperMotorPort_SendJ4310(cfg, cmd);

    case MOTOR_MODEL_M3508:
        return UpperMotorPort_SendM3508(cfg, cmd);

    case MOTOR_MODEL_M2006:
        return UpperMotorPort_SendM2006(cfg, cmd);

    default:
        return false;
    }
}

/* 功能：发送本周期所有待处理 DJI 分组帧；用途：在各电机目标均暂存后统一输出；返回 true 表示所有组发送成功。 */
bool UpperMotorPort_Flush(void)
{
    uint32_t bus_index;
    uint32_t group_index;
    bool success;

    success = true;
    for (bus_index = 0U; bus_index < UPPER_CAN_BUS_COUNT; bus_index++)
    {
        for (group_index = 0U;
             group_index < UPPER_DJI_GROUP_COUNT;
             group_index++)
        {
            if (!UpperMotorPort_FlushDjiGroup(
                    (uint8_t)(bus_index + 1U), group_index))
            {
                success = false;
            }
        }
    }
    return success;
}

/* 功能：按总线和标识符分发电机反馈帧；用途：更新 J4310、M3508 或 M2006 驱动状态；无返回值表示已尝试路由。 */
void UpperMotorPort_OnFrame(uint8_t can_bus /**< CAN 总线编号 */,
                            const can_frame_t *frame /**< 待解析的 CAN 接收帧 */,
                            uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    size_t index;
    bool j4310_attempted;

    if (!upper_motor_initialized || (frame == NULL))
    {
        return;
    }

    j4310_attempted = false;
    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg;

        cfg = &upper_motor_cfg_ref[index];
        if (cfg->can_bus != can_bus)
        {
            continue;
        }

        switch (cfg->model)
        {
        case MOTOR_MODEL_J4310:
            if (!j4310_attempted)
            {
                j4310_attempted = true;
                if (J4310_OnFrame(frame, tick_ms))
                {
                    UpperMotorPort_UpdateJ4310Position(can_bus, tick_ms);
                    return;
                }
            }
            break;

        case MOTOR_MODEL_M3508:
            if (M3508_OnFrame(can_bus,
                              cfg->node_id,
                              frame,
                              tick_ms))
            {
                return;
            }
            break;

        case MOTOR_MODEL_M2006:
            if (M2006_OnFrame(can_bus, cfg->node_id, frame, tick_ms))
            {
                return;
            }
            break;

        default:
            break;
        }
    }
}

/* 仅发送 J4310 协议使能帧，不发送运动目标。 */
/* 功能：仅发送指定 J4310 的协议使能帧；用途：上电恢复时先使能电机而不下发运动目标；返回 true 表示使能帧发送成功。 */
bool UpperMotorPort_EnableJ4310(uint8_t can_bus /**< CAN 总线编号 */, uint8_t node_id /**< 电机协议节点编号 */)
{
    can_frame_t frame;
    size_t index;
    const motor_cfg_t *cfg;
    j4310_feedback_t feedback;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL))
    {
        return false;
    }
    cfg = NULL;
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model == MOTOR_MODEL_J4310) &&
            (upper_motor_cfg_ref[index].can_bus == can_bus) &&
            (upper_motor_cfg_ref[index].node_id == node_id))
        {
            cfg = &upper_motor_cfg_ref[index];
            break;
        }
    }
    if (cfg == NULL)
    {
        return false;
    }
    if (J4310_GetFeedback(node_id, &feedback) &&
        UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                     upper_motor_tick_ms) &&
        (feedback.fault == UPPER_J4310_STATE_ENABLED))
    {
        j4310_enabled[node_id] = true;
        j4310_enable_pending[node_id] = false;
        return true;
    }
    if (!J4310_BuildEnable(node_id, &frame) ||
        !UpperMotorPort_SendJ4310Frame(can_bus,
                                       &frame,
                                       UPPER_J4310_TX_ENABLE))
    {
        return false;
    }
    j4310_enable_pending[node_id] = true;
    j4310_enable_last_sent_ms[node_id] = upper_motor_tick_ms;
    return true;
}

/* 功能：校验反馈并执行 J4310 保存零点序列；用途：安全完成关节机械零位标定；返回 true 表示命令序列发送成功。 */
bool UpperMotorPort_SaveJ4310Zero(uint8_t can_bus /**< CAN 总线编号 */, uint8_t node_id /**< 电机协议节点编号 */)
{
    can_frame_t frame;
    size_t index;
    const motor_cfg_t *cfg;
    j4310_feedback_t feedback;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL))
    {
        return false;
    }

    cfg = NULL;
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model == MOTOR_MODEL_J4310) &&
            (upper_motor_cfg_ref[index].can_bus == can_bus) &&
            (upper_motor_cfg_ref[index].node_id == node_id))
        {
            cfg = &upper_motor_cfg_ref[index];
            break;
        }
    }
    if ((cfg == NULL) || !J4310_GetFeedback(node_id, &feedback) ||
        !UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                      upper_motor_tick_ms) ||
        UpperMotorPort_IsJ4310Fault(feedback.fault))
    {
        return false;
    }

    if (!J4310_BuildDisable(node_id, &frame) ||
        !UpperMotorPort_SendJ4310Frame(can_bus,
                                       &frame,
                                       UPPER_J4310_TX_DISABLE))
    {
        return false;
    }
    j4310_enabled[node_id] = false;
    j4310_enable_pending[node_id] = false;

    if (!J4310_BuildSaveZero(node_id, &frame) ||
        !UpperMotorPort_SendFrame(can_bus, &frame))
    {
        return false;
    }
    UpperMotorPort_ResetJ4310Position(node_id);
    return true;
}

/* 功能：读取 J4310 经方向和减速比换算后的输出轴位置；用途：向应用层提供机械关节角；返回 true 表示反馈新鲜有效。 */
bool UpperMotorPort_GetJ4310OutputPosition(uint8_t can_bus /**< CAN 总线编号 */,
                                           uint8_t node_id /**< 电机协议节点编号 */,
                                           float *position_rad /**< 用于写出 J4310 机构位置的地址，单位：弧度 */)
{
    j4310_feedback_t feedback;
    size_t index;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (position_rad == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model == MOTOR_MODEL_J4310) &&
            (upper_motor_cfg_ref[index].can_bus == can_bus) &&
            (upper_motor_cfg_ref[index].node_id == node_id))
        {
            if (!J4310_GetFeedback(node_id, &feedback) ||
                !UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                              upper_motor_tick_ms))
            {
                return false;
            }
            if (!j4310_position_valid[node_id])
            {
                return false;
            }
            *position_rad = j4310_continuous_position_rad[node_id];
            return true;
        }
    }
    return false;
}

/* 功能：读取 J4310 自动归零所需的位置、速度和状态；用途：仅向上层提供新鲜且无故障的反馈；返回 true 表示可安全用于轨迹控制。 */
bool UpperMotorPort_GetJ4310Feedback(uint8_t can_bus /**< CAN 总线编号 */,
                                     uint8_t node_id /**< 电机协议节点编号 */,
                                     upper_j4310_feedback_t *feedback /**< 用于写出最新 J4310 反馈的对象 */)
{
    j4310_feedback_t driver_feedback;
    size_t index;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (feedback == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model != MOTOR_MODEL_J4310) ||
            (upper_motor_cfg_ref[index].can_bus != can_bus) ||
            (upper_motor_cfg_ref[index].node_id != node_id))
        {
            continue;
        }
        if (!J4310_GetFeedback(node_id, &driver_feedback) ||
            !UpperMotorPort_FeedbackFresh(driver_feedback.updated_at_ms,
                                          upper_motor_tick_ms) ||
            UpperMotorPort_IsJ4310Fault(driver_feedback.fault))
        {
            return false;
        }
        if (!j4310_position_valid[node_id])
        {
            return false;
        }
        feedback->position_rad = j4310_continuous_position_rad[node_id];
        feedback->velocity_rad_s = driver_feedback.velocity_rad_s *
                                   UPPER_J4310_DIRECTION_SIGN;
        feedback->torque_nm = driver_feedback.torque_nm *
                              UPPER_J4310_DIRECTION_SIGN;
        feedback->updated_at_ms = driver_feedback.updated_at_ms;
        feedback->state = driver_feedback.fault;
        return true;
    }
    return false;
}

/* 读取 M2006 的新鲜反馈，并应用已配置的机构方向。 */
bool UpperMotorPort_GetM2006Feedback(uint8_t can_bus /**< CAN 总线编号 */,
                                     uint8_t node_id /**< 电机协议节点编号 */,
                                     upper_m2006_feedback_t *feedback /**< 用于写出最新 M2006 反馈的对象 */)
{
    m2006_feedback_t driver_feedback;
    bool zero_valid;
    size_t index;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (feedback == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg = &upper_motor_cfg_ref[index];
        float direction_sign;

        if ((cfg->model != MOTOR_MODEL_M2006) ||
            (cfg->can_bus != can_bus) || (cfg->node_id != node_id))
        {
            continue;
        }
        if (!M2006_GetFeedbackSnapshot(can_bus,
                                       node_id,
                                       &driver_feedback,
                                       &zero_valid) ||
            !zero_valid ||
            !UpperMotorPort_FeedbackFresh(driver_feedback.updated_at_ms,
                                          upper_motor_tick_ms))
        {
            return false;
        }

        direction_sign = UpperMotorPort_DirectionSign(cfg);
        feedback->position_rad = driver_feedback.output_pos_rad *
                                 direction_sign;
        feedback->velocity_rad_s = driver_feedback.output_vel_rad_s *
                                   direction_sign;
        feedback->current_a = driver_feedback.torque_current_a *
                              direction_sign;
        feedback->updated_at_ms = driver_feedback.updated_at_ms;
        return true;
    }
    return false;
}

/* 功能：读取 J4310 协议接收诊断；用途：区分 FDCAN 有帧但格式、Master ID 或反馈 ID 不匹配；返回 true 表示目标已配置且快照有效。 */
bool UpperMotorPort_GetJ4310RxDiagnostic(
    uint8_t can_bus /**< CAN 总线编号 */,
    uint8_t node_id /**< 电机协议节点编号 */,
    upper_j4310_rx_diagnostic_t *diagnostic /**< 用于写出上层 J4310 接收诊断的对象 */)
{
    j4310_rx_diagnostics_t driver_diagnostic;
    size_t index;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (diagnostic == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model != MOTOR_MODEL_J4310) ||
            (upper_motor_cfg_ref[index].can_bus != can_bus) ||
            (upper_motor_cfg_ref[index].node_id != node_id))
        {
            continue;
        }
        if (!J4310_GetRxDiagnostics(&driver_diagnostic))
        {
            return false;
        }
        diagnostic->frames_seen = driver_diagnostic.frames_seen;
        diagnostic->accepted_frames = driver_diagnostic.accepted_frames;
        diagnostic->rejected_format_frames =
            driver_diagnostic.rejected_format_frames;
        diagnostic->rejected_master_id_frames =
            driver_diagnostic.rejected_master_id_frames;
        diagnostic->rejected_feedback_id_frames =
            driver_diagnostic.rejected_feedback_id_frames;
        diagnostic->last_can_id = driver_diagnostic.last_can_id;
        diagnostic->last_dlc = driver_diagnostic.last_dlc;
        diagnostic->last_data0 = driver_diagnostic.last_data0;
        diagnostic->last_result = (uint8_t)driver_diagnostic.last_result;
        return true;
    }
    return false;
}

/* 功能：读取 J4310 发送与使能确认快照；用途：向上位机区分未发送、TX 入队失败和电机未确认；返回 true 表示节点已配置。 */
bool UpperMotorPort_GetJ4310TxDiagnostic(
    uint8_t can_bus /**< CAN 总线编号 */,
    uint8_t node_id /**< 电机协议节点编号 */,
    upper_j4310_tx_diagnostic_t *diagnostic /**< 用于写出上层 J4310 发送诊断的对象 */)
{
    j4310_feedback_t feedback;
    size_t index;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (diagnostic == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].model != MOTOR_MODEL_J4310) ||
            (upper_motor_cfg_ref[index].can_bus != can_bus) ||
            (upper_motor_cfg_ref[index].node_id != node_id))
        {
            continue;
        }
        *diagnostic = j4310_tx_diagnostic;
        diagnostic->enable_confirmed = j4310_enabled[node_id];
        diagnostic->feedback_state =
            J4310_GetFeedback(node_id, &feedback) ? feedback.fault : 0xFFU;
        return true;
    }
    return false;
}

/* 功能：收集已配置 DJI 电机的诊断快照；用途：上报转子位置、零点、输出位置和反馈新鲜度；返回值表示写入条目数。 */
size_t UpperMotorPort_GetDjiDiagnostics(uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                        upper_dji_diagnostic_t *diagnostics /**< 用于写出 DJI 电机诊断条目数组 */,
                                        size_t capacity /**< 调用方提供的数组最大条目数 */)
{
    size_t index;
    size_t count;

    if (!upper_motor_initialized || (upper_motor_cfg_ref == NULL) ||
        (diagnostics == NULL))
    {
        return 0U;
    }
    count = 0U;
    for (index = 0U; (index < upper_motor_count) && (count < capacity);
         index++)
    {
        const motor_cfg_t *cfg;
        upper_dji_diagnostic_t *diagnostic;
        int64_t zero_counts;

        cfg = &upper_motor_cfg_ref[index];
        if (!UpperMotorPort_IsDjiModel(cfg->model))
        {
            continue;
        }
        diagnostic = &diagnostics[count++];
        (void)memset(diagnostic, 0, sizeof(*diagnostic));
        diagnostic->model = cfg->model;
        diagnostic->can_bus = cfg->can_bus;
        diagnostic->node_id = cfg->node_id;
        zero_counts = 0;
        if (cfg->model == MOTOR_MODEL_M3508)
        {
            m3508_feedback_t feedback;

            diagnostic->feedback_received = M3508_GetFeedbackSnapshot(
                cfg->can_bus, cfg->node_id, &feedback,
                &diagnostic->zero_valid);
            if (diagnostic->feedback_received)
            {
                diagnostic->rotor_position_rad =
                    (float)feedback.rotor_encoder * UPPER_DJI_TWO_PI /
                    (float)UPPER_DJI_ENCODER_COUNTS;
                zero_counts = feedback.zero_encoder_counts;
                diagnostic->relative_output_position_rad =
                    feedback.output_pos_rad *
                    UpperMotorPort_DirectionSign(cfg);
                diagnostic->feedback_fresh =
                    UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                                  tick_ms);
            }
        }
        else
        {
            m2006_feedback_t feedback;

            diagnostic->feedback_received = M2006_GetFeedbackSnapshot(
                cfg->can_bus, cfg->node_id, &feedback,
                &diagnostic->zero_valid);
            if (diagnostic->feedback_received)
            {
                diagnostic->rotor_position_rad =
                    (float)feedback.rotor_encoder * UPPER_DJI_TWO_PI /
                    (float)UPPER_DJI_ENCODER_COUNTS;
                zero_counts = feedback.zero_encoder_counts;
                diagnostic->relative_output_position_rad =
                    feedback.output_pos_rad *
                    UpperMotorPort_DirectionSign(cfg);
                diagnostic->feedback_fresh =
                    UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                                  tick_ms);
            }
        }
        if (diagnostic->zero_valid)
        {
            int64_t wrapped_zero;

            wrapped_zero = zero_counts % UPPER_DJI_ENCODER_COUNTS;
            if (wrapped_zero < 0)
            {
                wrapped_zero += UPPER_DJI_ENCODER_COUNTS;
            }
            diagnostic->zero_rotor_position_rad =
                (float)wrapped_zero * UPPER_DJI_TWO_PI /
                (float)UPPER_DJI_ENCODER_COUNTS;
        }
    }
    return count;
}

/* 功能：读取当前待上报电机故障；用途：供应用层构造故障消息；返回 true 表示存在尚未确认的故障。 */
bool UpperMotorPort_GetPendingFault(upper_motor_fault_t *fault /**< 用于写出待上报的电机故障记录 */)
{
    if (!upper_motor_fault_pending || (fault == NULL))
    {
        return false;
    }
    *fault = upper_motor_pending_fault;
    return true;
}

/* 功能：确认指定序号的故障已成功发送；用途：释放待上报故障槽；仅序号匹配时清除状态。 */
void UpperMotorPort_MarkFaultSent(uint32_t sequence /**< 用于匹配请求和响应的消息序号 */)
{
    if (upper_motor_fault_pending &&
        (upper_motor_pending_fault.sequence == sequence))
    {
        upper_motor_fault_pending = false;
    }
}

/* 功能：检查全部电机的反馈超时和协议故障；用途：形成整机健康结论并记录新故障；返回 true 表示所有电机健康。 */
bool UpperMotorPort_GetHealth(uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                              upper_motor_health_t *health /**< 用于写出电机在线状态和故障信息 */)
{
    size_t index;

    if (!upper_motor_initialized || (health == NULL))
    {
        return false;
    }

    (void)memset(health, 0, sizeof(*health));
    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg;
        uint32_t mask;
        bool feedback_valid;
        uint32_t updated_at_ms;
        bool faulted;

        cfg = &upper_motor_cfg_ref[index];
        if ((cfg->model == MOTOR_MODEL_J4310) && cfg->protocol_ready)
        {
            j4310_feedback_t feedback;

            if (J4310_GetFeedback(cfg->node_id, &feedback))
            {
                uint8_t fault;

                fault = UpperMotorPort_IsJ4310Fault(feedback.fault) ?
                        feedback.fault : 0U;
                if ((fault != 0U) &&
                    (fault != j4310_last_fault[cfg->node_id]))
                {
                    UpperMotorPort_RecordFault(cfg,
                                               fault,
                                               feedback.updated_at_ms);
                }
                j4310_last_fault[cfg->node_id] = fault;
            }
        }
        if (!upper_motor_active[index])
        {
            continue;
        }
        mask = 1UL << index;
        health->active_mask |= mask;
        if (!cfg->protocol_ready)
        {
            health->protocol_block_mask |= mask;
            continue;
        }

        feedback_valid = false;
        updated_at_ms = 0U;
        faulted = false;
        switch (cfg->model)
        {
        case MOTOR_MODEL_J4310:
        {
            j4310_feedback_t feedback;

            feedback_valid = J4310_GetFeedback(cfg->node_id, &feedback);
            updated_at_ms = feedback.updated_at_ms;
            faulted = UpperMotorPort_IsJ4310Fault(feedback.fault);
            break;
        }

        case MOTOR_MODEL_M3508:
        {
            m3508_feedback_t feedback;

            feedback_valid = M3508_GetFeedback(cfg->can_bus,
                                               cfg->node_id,
                                               &feedback);
            updated_at_ms = feedback.updated_at_ms;
            break;
        }

        case MOTOR_MODEL_M2006:
        {
            m2006_feedback_t feedback;

            feedback_valid = M2006_GetFeedback(cfg->can_bus,
                                               cfg->node_id,
                                               &feedback);
            updated_at_ms = feedback.updated_at_ms;
            break;
        }

        default:
            health->protocol_block_mask |= mask;
            continue;
        }

        if (faulted)
        {
            health->fault_mask |= mask;
        }
        if ((!feedback_valid &&
             ((tick_ms - upper_motor_active_since_ms[index]) >
               UPPER_MOTOR_FEEDBACK_TIMEOUT_MS)) ||
            (feedback_valid &&
             !UpperMotorPort_FeedbackFresh(updated_at_ms, tick_ms)))
        {
            health->offline_mask |= mask;
        }
    }
    return true;
}

/* 功能：查询指定总线、节点和 DJI 型号是否存在于拓扑；用途：验证调试或诊断路由；返回 true 表示配置匹配。 */
bool UpperMotorPort_IsDjiConfigured(uint8_t can_bus /**< CAN 总线编号 */,
                                    motor_model_t model /**< 待匹配拓扑配置的 DJI 电机型号 */,
                                    uint8_t node_id /**< 电机协议节点编号 */)
{
    size_t index;

    if (!UpperMotorPort_IsDjiModel(model) ||
        (upper_motor_cfg_ref == NULL))
    {
        return false;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if ((upper_motor_cfg_ref[index].can_bus == can_bus) &&
            (upper_motor_cfg_ref[index].node_id == node_id) &&
            (upper_motor_cfg_ref[index].model == model))
        {
            return true;
        }
    }
    return false;
}
