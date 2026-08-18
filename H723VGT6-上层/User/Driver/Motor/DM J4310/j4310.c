/**
 * @file j4310.c
 * @brief 实现达妙 J4310 电机命令编码、反馈解析和在线整定。
 */

#include "j4310.h"

#include <float.h>
#include <string.h>

#include "motor_online_tune.h"

#define J4310_CAN_STD_ID_MAX  0x7FFU
#define J4310_FEEDBACK_ID_MAX 0x0FU
#define J4310_KP_MAX          500.0f
#define J4310_KD_MAX          5.0f
#define J4310_ONLINE_MIT_KP_MAX 49.0f
#define J4310_ONLINE_MIT_KD_MAX 0.95f
#define J4310_ONLINE_MIT_NEAR_ERROR_RAD 0.01745329252f
#define J4310_ONLINE_MIT_FAR_ERROR_RAD  0.17453292520f
#define J4310_CMD_CLEAR_FAULT 0xFBU
#define J4310_CMD_ENABLE      0xFCU
#define J4310_CMD_DISABLE     0xFDU
#define J4310_CMD_SAVE_ZERO   0xFEU

typedef struct
{
    bool used;
    uint8_t motor_id;
    uint16_t master_id;
    uint8_t feedback_id;
    j4310_mode_t mode;
    /* 协议映射值存储在电机中，不属于可编辑的安全限值。 */
    j4310_limits_t limits;
    float software_torque_limit_nm;
    motor_online_mit_t mit_tuner;
    uint32_t online_last_feedback_ms;
    volatile uint32_t sequence;
    volatile j4310_feedback_t feedback;
} j4310_context_t;

static j4310_context_t j4310_context[J4310_MAX_MOTOR_COUNT];
static volatile uint32_t j4310_rx_diagnostic_sequence;
static volatile j4310_rx_diagnostics_t j4310_rx_diagnostics;

/* 功能：记录达妙解析结果和最近一帧帧头；用途：区分物理收帧与协议匹配失败；无返回值表示诊断快照已更新。 */
static void J4310_RecordRxDiagnostic(const can_frame_t *frame,
                                     j4310_rx_result_t result)
{
    uint32_t sequence;

    sequence = j4310_rx_diagnostic_sequence;
    j4310_rx_diagnostic_sequence = sequence + 1U;
    j4310_rx_diagnostics.frames_seen++;
    if (result == J4310_RX_ACCEPTED)
    {
        j4310_rx_diagnostics.accepted_frames++;
    }
    else if (result == J4310_RX_REJECTED_FORMAT)
    {
        j4310_rx_diagnostics.rejected_format_frames++;
    }
    else if (result == J4310_RX_REJECTED_MASTER_ID)
    {
        j4310_rx_diagnostics.rejected_master_id_frames++;
    }
    else if (result == J4310_RX_REJECTED_FEEDBACK_ID)
    {
        j4310_rx_diagnostics.rejected_feedback_id_frames++;
    }
    j4310_rx_diagnostics.last_can_id = (uint16_t)frame->id;
    j4310_rx_diagnostics.last_dlc = frame->dlc;
    j4310_rx_diagnostics.last_data0 = frame->data[0];
    j4310_rx_diagnostics.last_result = result;
    j4310_rx_diagnostic_sequence = sequence + 2U;
}

/* 功能：判断浮点数是否有限；用途：过滤 J4310 命令中的 NaN 和无穷值；返回 true 表示数值可用。 */
static bool J4310_IsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/* 功能：把数值限制在给定区间；用途：约束位置、速度、转矩和增益；返回值表示限幅结果。 */
static float J4310_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

/* 功能：检查 J4310 控制模式枚举；用途：拒绝驱动不支持的模式；返回 true 表示模式合法。 */
static bool J4310_ModeValid(j4310_mode_t mode)
{
    return (mode == J4310_MODE_MIT) ||
           (mode == J4310_MODE_POSITION_VELOCITY) ||
           (mode == J4310_MODE_VELOCITY);
}

/* 功能：将物理浮点量线性映射为指定比特的无符号整数；用途：编码 MIT 控制字段；返回值表示量化后的协议值。 */
static uint16_t J4310_FloatToUint(float value,
                                  float min,
                                  float max,
                                  uint8_t bits)
{
    uint32_t scale;
    float normalized;

    scale = (1UL << bits) - 1UL;
    value = J4310_Clamp(value, min, max);
    normalized = (value - min) * (float)scale / (max - min);
    return (uint16_t)normalized;
}

/* 功能：把单精度浮点数按小端位模式写入字节；用途：编码位置速度和速度模式帧；结果写入 data。 */
static void J4310_WriteFloatLe(uint8_t *data, float value)
{
    uint32_t raw;

    (void)memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)(raw >> 16U);
    data[3] = (uint8_t)(raw >> 24U);
}

/* 功能：将协议无符号整数线性还原为物理浮点量；用途：解析 MIT 反馈；返回值表示对应的物理值。 */
static float J4310_UintToFloat(uint16_t value,
                               float min,
                               float max,
                               uint8_t bits)
{
    uint32_t scale;

    scale = (1UL << bits) - 1UL;
    return ((float)value * (max - min) / (float)scale) + min;
}

/* 功能：按节点号查找已注册的 J4310 上下文；用途：访问模式、限制和反馈；返回 NULL 表示电机未注册。 */
static j4310_context_t *J4310_Find(uint8_t motor_id)
{
    uint32_t index;

    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (j4310_context[index].used &&
            (j4310_context[index].motor_id == motor_id))
        {
            return &j4310_context[index];
        }
    }
    return NULL;
}

/* 功能：根据电机限制配置 MIT 在线调参器；用途：建立刚度和阻尼的动态调整边界；返回 true 表示配置成功。 */
static bool J4310_ConfigureOnlineMit(j4310_context_t *context,
                                     bool enabled)
{
    motor_online_mit_cfg_t cfg;

    cfg.minimum_kp = 0.0f;
    cfg.maximum_kp = J4310_ONLINE_MIT_KP_MAX;
    cfg.minimum_kd = 0.0f;
    cfg.maximum_kd = J4310_ONLINE_MIT_KD_MAX;
    cfg.near_error = J4310_ONLINE_MIT_NEAR_ERROR_RAD;
    cfg.far_error = J4310_ONLINE_MIT_FAR_ERROR_RAD;
    if (cfg.far_error <= cfg.near_error)
    {
        cfg.far_error = cfg.near_error + 0.1f;
    }
    cfg.velocity_scale = J4310_Clamp(
        context->limits.velocity_max_rad_s * 0.10f, 1.0f, 3.0f);
    cfg.diverging_rate = -0.05f;
    cfg.stalled_rate = 0.01f;
    cfg.stalled_velocity = 0.10f;
    cfg.smoothing = 0.20f;
    context->online_last_feedback_ms = 0U;
    return MotorOnlineMit_Init(&context->mit_tuner, &cfg, enabled);
}

/* 功能：构造 J4310 使能、停机、清错或置零特殊帧；用途：复用特殊命令格式；返回 true 表示构帧成功。 */
static bool J4310_BuildSpecial(uint8_t motor_id,
                               uint8_t command,
                               can_frame_t *frame)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL))
    {
        return false;
    }

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = (uint32_t)context->motor_id + (uint32_t)context->mode;
    frame->dlc = 8U;
    (void)memset(frame->data, 0xFF, 7U);
    frame->data[7] = command;
    return true;
}

/* 功能：清空全部 J4310 驱动上下文；用途：在注册电机前复位驱动状态；无返回值表示上下文表已初始化。 */
void J4310_Init(void)
{
    (void)memset(j4310_context, 0, sizeof(j4310_context));
    j4310_rx_diagnostic_sequence = 0U;
    (void)memset((void *)&j4310_rx_diagnostics,
                 0,
                 sizeof(j4310_rx_diagnostics));
}

/* 功能：注册一台 J4310 及其模式和物理限制；用途：建立节点运行上下文；返回 true 表示注册成功。 */
bool J4310_AddMotor(uint8_t motor_id,
                    uint16_t master_id,
                    uint8_t feedback_id,
                    j4310_mode_t mode,
                    const j4310_limits_t *limits)
{
    uint32_t index;

    if ((motor_id == 0U) || (motor_id > J4310_FEEDBACK_ID_MAX) ||
        (master_id > J4310_CAN_STD_ID_MAX) ||
        (feedback_id > J4310_FEEDBACK_ID_MAX) ||
        !J4310_ModeValid(mode) ||
        ((uint32_t)motor_id + (uint32_t)mode > J4310_CAN_STD_ID_MAX) ||
        (limits == NULL) ||
        !J4310_IsFinite(limits->position_max_rad) ||
        !J4310_IsFinite(limits->velocity_max_rad_s) ||
        !J4310_IsFinite(limits->torque_max_nm) ||
        (limits->position_max_rad <= 0.0f) ||
        (limits->velocity_max_rad_s <= 0.0f) ||
        (limits->torque_max_nm <= 0.0f) ||
        (J4310_Find(motor_id) != NULL))
    {
        return false;
    }

    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (!j4310_context[index].used)
        {
            j4310_context[index].used = true;
            j4310_context[index].motor_id = motor_id;
            j4310_context[index].master_id = master_id;
            j4310_context[index].feedback_id = feedback_id;
            j4310_context[index].mode = mode;
            j4310_context[index].limits = *limits;
            j4310_context[index].software_torque_limit_nm =
                limits->torque_max_nm;
            if (!J4310_ConfigureOnlineMit(&j4310_context[index], false))
            {
                (void)memset(&j4310_context[index],
                             0,
                             sizeof(j4310_context[index]));
                return false;
            }
            return true;
        }
    }
    return false;
}

/* 功能：修改已注册 J4310 的控制模式；用途：切换 MIT、位置速度或速度控制；返回 true 表示模式已接受。 */
bool J4310_SetMode(uint8_t motor_id, j4310_mode_t mode)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || !J4310_ModeValid(mode) ||
        ((uint32_t)motor_id + (uint32_t)mode > J4310_CAN_STD_ID_MAX))
    {
        return false;
    }
    context->mode = mode;
    context->online_last_feedback_ms = 0U;
    return true;
}

/* 功能：构造 J4310 使能帧；用途：让目标节点进入可控状态；返回 true 表示构帧成功。 */
bool J4310_BuildEnable(uint8_t motor_id, can_frame_t *frame)
{
    return J4310_BuildSpecial(motor_id, J4310_CMD_ENABLE, frame);
}

/* 功能：构造 J4310 失能帧并复位在线调参；用途：安全停止目标节点；返回 true 表示构帧成功。 */
bool J4310_BuildDisable(uint8_t motor_id, can_frame_t *frame)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if (context != NULL)
    {
        MotorOnlineMit_SetEnabled(&context->mit_tuner,
                                  context->mit_tuner.enabled != 0U);
        context->online_last_feedback_ms = 0U;
    }
    return J4310_BuildSpecial(motor_id, J4310_CMD_DISABLE, frame);
}

/* 功能：构造 J4310 清除故障帧；用途：请求节点退出故障状态；返回 true 表示构帧成功。 */
bool J4310_BuildClearFault(uint8_t motor_id, can_frame_t *frame)
{
    return J4310_BuildSpecial(motor_id, J4310_CMD_CLEAR_FAULT, frame);
}

/* 功能：构造 J4310 保存当前位置为零点的帧；用途：执行机械零点标定；返回 true 表示构帧成功。 */
bool J4310_BuildSaveZero(uint8_t motor_id, can_frame_t *frame)
{
    return J4310_BuildSpecial(motor_id, J4310_CMD_SAVE_ZERO, frame);
}

/* 功能：构造 J4310 MIT 五参数控制帧；用途：发送位置、速度、刚度、阻尼和转矩目标；返回 true 表示参数有效并构帧完成。 */
bool J4310_BuildMit(uint8_t motor_id,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd,
                    float torque_nm,
                    can_frame_t *frame)
{
    j4310_context_t *context;
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque;
    j4310_feedback_t feedback;
    float applied_kp;
    float applied_kd;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL) ||
        (context->mode != J4310_MODE_MIT) ||
        !J4310_IsFinite(position_rad) ||
        !J4310_IsFinite(velocity_rad_s) || !J4310_IsFinite(kp) ||
        !J4310_IsFinite(kd) || !J4310_IsFinite(torque_nm))
    {
        return false;
    }

    position_rad = J4310_Clamp(position_rad,
                               -context->limits.position_max_rad,
                               context->limits.position_max_rad);
    velocity_rad_s = J4310_Clamp(velocity_rad_s,
                                 -context->limits.velocity_max_rad_s,
                                 context->limits.velocity_max_rad_s);
    torque_nm = J4310_Clamp(torque_nm,
                             -context->software_torque_limit_nm,
                             context->software_torque_limit_nm);
    kp = J4310_Clamp(kp, 0.0f, J4310_KP_MAX);
    kd = J4310_Clamp(kd, 0.0f, J4310_KD_MAX);
    if ((kp != context->mit_tuner.base_kp) ||
        (kd != context->mit_tuner.base_kd))
    {
        if (!MotorOnlineMit_SetCommand(&context->mit_tuner, kp, kd))
        {
            return false;
        }
        context->online_last_feedback_ms = 0U;
    }
    if ((context->mit_tuner.enabled != 0U) &&
        J4310_GetFeedback(motor_id, &feedback) &&
        (feedback.updated_at_ms != context->online_last_feedback_ms))
    {
        float dt_s;

        dt_s = 0.001f;
        if (context->online_last_feedback_ms != 0U)
        {
            dt_s = (float)(uint32_t)(feedback.updated_at_ms -
                                     context->online_last_feedback_ms) /
                   1000.0f;
            dt_s = J4310_Clamp(dt_s, 0.001f, 0.10f);
        }
        MotorOnlineMit_Update(
            &context->mit_tuner,
            position_rad - feedback.position_rad,
            velocity_rad_s - feedback.velocity_rad_s,
            feedback.velocity_rad_s,
            dt_s,
            &context->mit_tuner.applied_kp,
            &context->mit_tuner.applied_kd);
        context->online_last_feedback_ms = feedback.updated_at_ms;
    }
    applied_kp = context->mit_tuner.applied_kp;
    applied_kd = context->mit_tuner.applied_kd;

    position = J4310_FloatToUint(position_rad,
                                 -context->limits.position_max_rad,
                                 context->limits.position_max_rad,
                                 16U);
    velocity = J4310_FloatToUint(velocity_rad_s,
                                 -context->limits.velocity_max_rad_s,
                                 context->limits.velocity_max_rad_s,
                                 12U);
    kp_raw = J4310_FloatToUint(applied_kp,
                               0.0f,
                               J4310_KP_MAX,
                               12U);
    kd_raw = J4310_FloatToUint(applied_kd,
                               0.0f,
                               J4310_KD_MAX,
                               12U);
    torque = J4310_FloatToUint(torque_nm,
                               -context->limits.torque_max_nm,
                               context->limits.torque_max_nm,
                               12U);

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = (uint32_t)context->motor_id + (uint32_t)context->mode;
    frame->dlc = 8U;
    frame->data[0] = (uint8_t)(position >> 8U);
    frame->data[1] = (uint8_t)position;
    frame->data[2] = (uint8_t)(velocity >> 4U);
    frame->data[3] = (uint8_t)(((velocity & 0x0FU) << 4U) |
                               (kp_raw >> 8U));
    frame->data[4] = (uint8_t)kp_raw;
    frame->data[5] = (uint8_t)(kd_raw >> 4U);
    frame->data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) |
                               (torque >> 8U));
    frame->data[7] = (uint8_t)torque;
    return true;
}

/* 功能：构造 J4310 位置速度模式帧；用途：发送位置与速度上限目标；返回 true 表示构帧成功。 */
bool J4310_BuildPositionVelocity(uint8_t motor_id,
                                 float position_rad,
                                 float velocity_rad_s,
                                 can_frame_t *frame)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL) ||
        (context->mode != J4310_MODE_POSITION_VELOCITY) ||
        !J4310_IsFinite(position_rad) ||
        !J4310_IsFinite(velocity_rad_s))
    {
        return false;
    }
    (void)memset(frame, 0, sizeof(*frame));
    frame->id = (uint32_t)context->motor_id + (uint32_t)context->mode;
    frame->dlc = 8U;
    J4310_WriteFloatLe(&frame->data[0], position_rad);
    J4310_WriteFloatLe(&frame->data[4], velocity_rad_s);
    return true;
}

/* 功能：构造 J4310 纯速度模式帧；用途：发送目标转速；返回 true 表示构帧成功。 */
bool J4310_BuildVelocity(uint8_t motor_id,
                         float velocity_rad_s,
                         can_frame_t *frame)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (frame == NULL) ||
        (context->mode != J4310_MODE_VELOCITY) ||
        !J4310_IsFinite(velocity_rad_s))
    {
        return false;
    }
    (void)memset(frame, 0, sizeof(*frame));
    frame->id = (uint32_t)context->motor_id + (uint32_t)context->mode;
    frame->dlc = 4U;
    J4310_WriteFloatLe(frame->data, velocity_rad_s);
    return true;
}

/* 功能：设置 J4310 软件转矩限制；用途：在构造控制帧时进一步约束输出；返回 true 表示限制合法并已保存。 */
bool J4310_SetTorqueLimit(uint8_t motor_id, float torque_limit_nm)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || !J4310_IsFinite(torque_limit_nm) ||
        (torque_limit_nm <= 0.0f) ||
        (torque_limit_nm > context->limits.torque_max_nm))
    {
        return false;
    }
    context->software_torque_limit_nm = torque_limit_nm;
    return true;
}

/* 功能：解析 J4310 CAN 反馈并更新时间和故障信息；用途：维护闭环反馈快照；返回 true 表示该帧属于已注册节点且解析成功。 */
bool J4310_OnFrame(const can_frame_t *frame, uint32_t tick_ms)
{
    j4310_context_t *context;
    volatile j4310_feedback_t *feedback;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint32_t index;
    uint32_t sequence;
    bool master_id_matched;

    if (frame == NULL)
    {
        return false;
    }
    if (frame->extended || (frame->dlc != 8U) ||
        (frame->id > J4310_CAN_STD_ID_MAX))
    {
        J4310_RecordRxDiagnostic(frame, J4310_RX_REJECTED_FORMAT);
        return false;
    }

    context = NULL;
    master_id_matched = false;
    for (index = 0U; index < J4310_MAX_MOTOR_COUNT; index++)
    {
        if (!j4310_context[index].used ||
            (frame->id != j4310_context[index].master_id))
        {
            continue;
        }
        master_id_matched = true;
        if ((frame->data[0] & 0x0FU) ==
            j4310_context[index].feedback_id)
        {
            context = &j4310_context[index];
            break;
        }
    }
    if (context == NULL)
    {
        J4310_RecordRxDiagnostic(
            frame,
            master_id_matched ? J4310_RX_REJECTED_FEEDBACK_ID :
                                J4310_RX_REJECTED_MASTER_ID);
        return false;
    }

    position = (uint16_t)(((uint16_t)frame->data[1] << 8U) |
                          frame->data[2]);
    velocity = (uint16_t)(((uint16_t)frame->data[3] << 4U) |
                          (frame->data[4] >> 4U));
    torque = (uint16_t)((((uint16_t)frame->data[4] & 0x0FU) << 8U) |
                        frame->data[5]);

    feedback = &context->feedback;
    sequence = context->sequence;
    context->sequence = sequence + 1U;
    feedback->position_rad = J4310_UintToFloat(
        position, -context->limits.position_max_rad,
        context->limits.position_max_rad, 16U);
    feedback->velocity_rad_s = J4310_UintToFloat(
        velocity, -context->limits.velocity_max_rad_s,
        context->limits.velocity_max_rad_s, 12U);
    feedback->torque_nm = J4310_UintToFloat(
        torque, -context->limits.torque_max_nm,
        context->limits.torque_max_nm, 12U);
    feedback->mos_temperature_c = frame->data[6];
    feedback->rotor_temperature_c = frame->data[7];
    feedback->fault = frame->data[0] >> 4U;
    feedback->updated_at_ms = tick_ms;
    feedback->rx_frames++;
    context->sequence = sequence + 2U;
    J4310_RecordRxDiagnostic(frame, J4310_RX_ACCEPTED);
    return true;
}

/* 功能：读取指定 J4310 的最新反馈快照；用途：供控制、诊断和在线调参使用；返回 true 表示已有有效反馈。 */
bool J4310_GetFeedback(uint8_t motor_id, j4310_feedback_t *feedback)
{
    const j4310_context_t *context;
    uint32_t before;
    uint32_t after;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (feedback == NULL))
    {
        return false;
    }

    for (;;)
    {
        before = context->sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        *feedback = context->feedback;
        after = context->sequence;
        if (before == after)
        {
            break;
        }
    }

    return feedback->rx_frames != 0U;
}

/* 功能：读取达妙接收诊断快照；用途：向上位机报告有效帧和具体拒绝原因；返回 true 表示参数有效。 */
bool J4310_GetRxDiagnostics(j4310_rx_diagnostics_t *diagnostics)
{
    uint32_t before;
    uint32_t after;

    if (diagnostics == NULL)
    {
        return false;
    }
    for (;;)
    {
        before = j4310_rx_diagnostic_sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        diagnostics->frames_seen = j4310_rx_diagnostics.frames_seen;
        diagnostics->accepted_frames =
            j4310_rx_diagnostics.accepted_frames;
        diagnostics->rejected_format_frames =
            j4310_rx_diagnostics.rejected_format_frames;
        diagnostics->rejected_master_id_frames =
            j4310_rx_diagnostics.rejected_master_id_frames;
        diagnostics->rejected_feedback_id_frames =
            j4310_rx_diagnostics.rejected_feedback_id_frames;
        diagnostics->last_can_id = j4310_rx_diagnostics.last_can_id;
        diagnostics->last_dlc = j4310_rx_diagnostics.last_dlc;
        diagnostics->last_data0 = j4310_rx_diagnostics.last_data0;
        diagnostics->last_result = j4310_rx_diagnostics.last_result;
        after = j4310_rx_diagnostic_sequence;
        if ((before == after) && ((after & 1U) == 0U))
        {
            return true;
        }
    }
}

/* 功能：启用或关闭指定 J4310 的 MIT 在线调参；用途：切换固定和动态 kp、kd；返回 true 表示设置成功。 */
bool J4310_SetOnlineMitEnabled(uint8_t motor_id, bool enabled)
{
    j4310_context_t *context;

    context = J4310_Find(motor_id);
    if (context == NULL)
    {
        return false;
    }
    MotorOnlineMit_SetEnabled(&context->mit_tuner, enabled);
    context->online_last_feedback_ms = 0U;
    return true;
}

/* 功能：读取指定 J4310 的 MIT 在线调参状态；用途：诊断当前增益和收敛情况；返回 true 表示状态已写出。 */
bool J4310_GetOnlineMitState(uint8_t motor_id,
                             j4310_online_mit_state_t *state)
{
    const j4310_context_t *context;

    context = J4310_Find(motor_id);
    if ((context == NULL) || (state == NULL))
    {
        return false;
    }
    state->enabled = context->mit_tuner.enabled != 0U;
    state->base_kp = context->mit_tuner.base_kp;
    state->base_kd = context->mit_tuner.base_kd;
    state->applied_kp = context->mit_tuner.applied_kp;
    state->applied_kd = context->mit_tuner.applied_kd;
    return true;
}
