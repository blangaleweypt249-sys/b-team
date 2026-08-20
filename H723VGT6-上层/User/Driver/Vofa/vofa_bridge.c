/**
 * @file vofa_bridge.c
 * @brief 实现非生产 VOFA 调试命令解析、电机控制和遥测数据发送。
 */

#include "vofa_bridge.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_can.h"
#include "comm_runtime.h"
#include "DJI/dji_group.h"
#include "M2006/m2006.h"
#include "M3508/m3508.h"
#include "upper_motor_port.h"

#define VOFA_LINE_SIZE                  192U
#define VOFA_TOKEN_COUNT                 24U
#define VOFA_DJI_MOTOR_COUNT              8U
#define VOFA_CAN_BUS_COUNT                3U
#define VOFA_CHANNEL_COUNT               16U
#define VOFA_DEFAULT_PERIOD_MS           20U
#define VOFA_MIN_PERIOD_MS                5U
#define VOFA_MAX_PERIOD_MS             1000U
#define VOFA_FEEDBACK_TIMEOUT_MS         200U
#define VOFA_JUSTFLOAT_TAIL       0x7F800000UL
#define VOFA_PI                    3.14159265358979323846f
#define VOFA_TWO_PI                6.28318530717958647692f
#define VOFA_DEG_TO_RAD           (VOFA_PI / 180.0f)
#define VOFA_RPM_TO_RAD_S         (VOFA_TWO_PI / 60.0f)
#define VOFA_RAD_S_TO_RPM         (60.0f / VOFA_TWO_PI)
#define VOFA_M2006_RAW_MAX             10000.0f
#define VOFA_M2006_CURRENT_MAX_A          10.0f
#define VOFA_M3508_RAW_MAX             16384.0f
#define VOFA_M3508_CURRENT_MAX_A          20.0f
#define VOFA_DEFAULT_M2006_CURRENT_LIMIT_A 10.0f
#define VOFA_DEFAULT_M3508_CURRENT_LIMIT_A  3.0f

#define VOFA_STATUS_INVALID       (-1)
#define VOFA_STATUS_BUSY          (-2)
#define VOFA_STATUS_EXITING       (-3)
#define VOFA_STATUS_TUNE_FAILED   (-4)
#define VOFA_STATUS_TIMEOUT       (-5)
#define VOFA_STATUS_OFFLINE         0
#define VOFA_STATUS_ONLINE          1
#define VOFA_STATUS_TUNING          4
#define VOFA_STATUS_TUNED           5

typedef enum
{
    VOFA_FAMILY_M2006,
    VOFA_FAMILY_M3508,
    VOFA_FAMILY_COUNT
} vofa_family_t;

typedef enum
{
    VOFA_MODE_STOP,
    VOFA_MODE_CURRENT,
    VOFA_MODE_SPEED,
    VOFA_MODE_POSITION
} vofa_mode_t;

typedef struct
{
    bool active;
    vofa_family_t family;
    uint8_t can_bus;
    uint8_t motor_ids[VOFA_DJI_MOTOR_COUNT];
    uint8_t motor_count;
    vofa_mode_t mode;
    uint32_t telemetry_period_ms;
    uint32_t next_telemetry_ms;
    float current_limit_a;
    float target_current_a;
    float target_speed_rpm;
    float target_position_rad;
    float position_speed_rpm;
    float acceleration_rpm_s;
    int16_t current_raw[VOFA_DJI_MOTOR_COUNT];
    int32_t command_status;
    bool auto_tune_active;
} vofa_session_t;

#define VOFA_SESSION_COUNT (VOFA_FAMILY_COUNT * VOFA_CAN_BUS_COUNT)

static vofa_session_t vofa_sessions[VOFA_SESSION_COUNT];
static vofa_session_t *vofa_telemetry_session;
static char vofa_rx_line[VOFA_LINE_SIZE];
static uint16_t vofa_rx_index;
static bool vofa_rx_candidate;
static bool vofa_rx_overflow;
static uint32_t vofa_last_sequence;
static int32_t vofa_last_sequence_status;
static bool vofa_last_sequence_valid;
static uint8_t vofa_telemetry_frame[
    VOFA_DJI_MOTOR_COUNT * VOFA_CHANNEL_COUNT * sizeof(float) +
    sizeof(uint32_t)];

/* 功能：忽略字母大小写比较两个字符串；用途：识别 VOFA 文本命令关键字；返回 true 表示内容相同。 */
static bool Vofa_StringEquals(const char *left, const char *right)
{
    char left_value;
    char right_value;

    if ((left == NULL) || (right == NULL))
    {
        return false;
    }
    while ((*left != '\0') && (*right != '\0'))
    {
        left_value = *left;
        right_value = *right;
        if ((left_value >= 'a') && (left_value <= 'z'))
        {
            left_value = (char)(left_value - 'a' + 'A');
        }
        if ((right_value >= 'a') && (right_value <= 'z'))
        {
            right_value = (char)(right_value - 'a' + 'A');
        }
        if (left_value != right_value)
        {
            return false;
        }
        left++;
        right++;
    }
    return (*left == '\0') && (*right == '\0');
}

/* 功能：将文本解析为指定范围内的无符号整数；用途：读取总线号、电机号和周期；返回 true 表示格式及范围合法。 */
static bool Vofa_ParseU32(const char *text,
                          uint32_t minimum,
                          uint32_t maximum,
                          uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }
    parsed = strtoul(text, &end, 0);
    if ((*end != '\0') || (parsed < minimum) || (parsed > maximum))
    {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

/* 功能：将文本解析为有限浮点数；用途：读取目标值和 PID 参数；返回 true 表示整段文本均为合法数值。 */
static bool Vofa_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }
    parsed = strtof(text, &end);
    if ((*end != '\0') || (parsed != parsed) ||
        (parsed > FLT_MAX) || (parsed < -FLT_MAX))
    {
        return false;
    }
    *value = parsed;
    return true;
}

/* 功能：按空格和制表符拆分命令行；用途：为命令解析建立参数数组；返回值表示得到的字段数量。 */
static uint8_t Vofa_Tokenize(char *line, char *tokens[VOFA_TOKEN_COUNT])
{
    uint8_t count;
    char *token;

    count = 0U;
    token = strtok(line, " \t");
    while ((token != NULL) && (count < VOFA_TOKEN_COUNT))
    {
        tokens[count++] = token;
        token = strtok(NULL, " \t");
    }
    if (token != NULL)
    {
        return 0U;
    }
    return count;
}

/* 功能：取得指定电机族的默认 CAN 总线；用途：简化未显式指定总线的 VOFA 命令；返回值为默认总线号。 */
static uint8_t Vofa_DefaultCanBus(vofa_family_t family)
{
    return (family == VOFA_FAMILY_M2006) ? 3U : 2U;
}

/* 功能：定位电机族与 CAN 总线对应的 VOFA 会话；用途：访问固定会话表；返回值为该会话对象。 */
static vofa_session_t *Vofa_GetSession(vofa_family_t family,
                                       uint8_t can_bus)
{
    uint32_t index;

    index = (uint32_t)family * VOFA_CAN_BUS_COUNT + (can_bus - 1U);
    return &vofa_sessions[index];
}

/* 功能：取得电机族允许的最大物理电流；用途：校验命令并换算原始电流；返回值单位为安培。 */
static float Vofa_CurrentMaximum(vofa_family_t family)
{
    return (family == VOFA_FAMILY_M2006) ?
           VOFA_M2006_CURRENT_MAX_A : VOFA_M3508_CURRENT_MAX_A;
}

/* 功能：取得电机族协议中的最大原始电流值；用途：在安培和 CAN 原始量之间换算；返回值为协议量程。 */
static float Vofa_RawMaximum(vofa_family_t family)
{
    return (family == VOFA_FAMILY_M2006) ?
           VOFA_M2006_RAW_MAX : VOFA_M3508_RAW_MAX;
}

/* 功能：按会话电流限制约束原始电流命令；用途：保护被调试电机；返回值表示限幅后的 CAN 电流值。 */
static int16_t Vofa_ClampCurrentRaw(const vofa_session_t *session,
                                    int16_t current_raw)
{
    float raw_limit;

    raw_limit = session->current_limit_a *
                Vofa_RawMaximum(session->family) /
                Vofa_CurrentMaximum(session->family);
    if ((float)current_raw > raw_limit)
    {
        return (int16_t)raw_limit;
    }
    if ((float)current_raw < -raw_limit)
    {
        return (int16_t)(-raw_limit);
    }
    return current_raw;
}

/* 功能：向会话内所有电机应用运动与电流限制配置；用途：在启动调试前统一控制参数；返回 true 表示全部应用成功。 */
static bool Vofa_ApplyProfile(const vofa_session_t *session)
{
    uint8_t index;
    bool success;

    success = true;
    for (index = 0U; index < session->motor_count; index++)
    {
        uint8_t motor_id;

        motor_id = session->motor_ids[index];
        if (session->family == VOFA_FAMILY_M2006)
        {
            success = M2006_SetCurrentLimit(
                          session->can_bus,
                          motor_id,
                          session->current_limit_a) && success;
            success = M2006_SetPositionVelocityLimit(
                          session->can_bus,
                          motor_id,
                          session->position_speed_rpm *
                          VOFA_RPM_TO_RAD_S) && success;
            success = M2006_SetAccelerationLimit(
                          session->can_bus,
                          motor_id,
                          session->acceleration_rpm_s *
                          VOFA_RPM_TO_RAD_S) && success;
        }
        else
        {
            success = M3508_SetCurrentLimit(
                          session->can_bus,
                          motor_id,
                          session->current_limit_a) && success;
            success = M3508_SetPositionVelocityLimit(
                          session->can_bus,
                          motor_id,
                          session->position_speed_rpm *
                          VOFA_RPM_TO_RAD_S) && success;
            success = M3508_SetAccelerationLimit(
                          session->can_bus,
                          motor_id,
                          session->acceleration_rpm_s *
                          VOFA_RPM_TO_RAD_S) && success;
        }
    }
    return success;
}

/* 功能：停止会话内全部电机并清理调试状态；用途：响应 STOP、超时或错误；无返回值表示会话已失活。 */
static void Vofa_StopSession(vofa_session_t *session, uint32_t tick_ms)
{
    uint8_t index;

    for (index = 0U; index < session->motor_count; index++)
    {
        uint8_t motor_id;

        motor_id = session->motor_ids[index];
        if (session->family == VOFA_FAMILY_M2006)
        {
            (void)M2006_CancelAutoTune(session->can_bus, motor_id);
            (void)M2006_SetTarget(session->can_bus,
                                  motor_id,
                                  M2006_MODE_STOP,
                                  0.0f,
                                  tick_ms);
        }
        else
        {
            (void)M3508_CancelAutoTune(session->can_bus, motor_id);
            (void)M3508_SetTarget(session->can_bus,
                                  motor_id,
                                  M3508_MODE_STOP,
                                  0.0f,
                                  tick_ms);
        }
        session->current_raw[index] = 0;
    }
    session->mode = VOFA_MODE_STOP;
    session->target_current_a = 0.0f;
    session->target_speed_rpm = 0.0f;
    session->auto_tune_active = false;
    session->command_status = VOFA_STATUS_OFFLINE;
}

/* 功能：检查活动会话是否占用相同 CAN 电机地址；用途：防止两个会话同时控制同一电机；返回 true 表示存在冲突。 */
static bool Vofa_SessionsConflict(const vofa_session_t *session,
                                  uint8_t can_bus,
                                  const uint8_t *motor_ids,
                                  uint8_t motor_count)
{
    uint8_t left;
    uint8_t right;

    if (!session->active || (session->can_bus != can_bus))
    {
        return false;
    }
    for (left = 0U; left < session->motor_count; left++)
    {
        for (right = 0U; right < motor_count; right++)
        {
            if (session->motor_ids[left] == motor_ids[right])
            {
                return true;
            }
        }
    }
    return false;
}

/* 功能：遍历会话表检查目标路由冲突；用途：启动新会话前保证控制权唯一；返回 true 表示至少一处地址被占用。 */
static bool Vofa_RouteConflicts(const vofa_session_t *target,
                                uint8_t can_bus,
                                const uint8_t *motor_ids,
                                uint8_t motor_count)
{
    uint8_t session_index;

    for (session_index = 0U;
         session_index < VOFA_SESSION_COUNT;
         session_index++)
    {
        const vofa_session_t *session;

        session = &vofa_sessions[session_index];
        if ((session != target) &&
            Vofa_SessionsConflict(session,
                                  can_bus,
                                  motor_ids,
                                  motor_count))
        {
            return true;
        }
    }
    return false;
}

/* 功能：配置并启动一个 VOFA 电机调试会话；用途：绑定电机、周期和初始状态；返回 true 表示会话已激活。 */
static bool Vofa_StartSession(vofa_session_t *session,
                              uint8_t can_bus,
                              const uint8_t *motor_ids,
                              uint8_t motor_count,
                              uint32_t period_ms,
                              uint32_t tick_ms)
{
    uint8_t index;

    if (session->active)
    {
        Vofa_StopSession(session, tick_ms);
    }
    session->active = true;
    session->can_bus = can_bus;
    session->motor_count = motor_count;
    (void)memset(session->motor_ids, 0, sizeof(session->motor_ids));
    (void)memcpy(session->motor_ids, motor_ids, motor_count);
    session->mode = VOFA_MODE_STOP;
    session->telemetry_period_ms = period_ms;
    session->next_telemetry_ms = tick_ms;
    session->target_current_a = 0.0f;
    session->target_speed_rpm = 0.0f;
    session->target_position_rad = 0.0f;
    session->auto_tune_active = false;
    session->command_status = VOFA_STATUS_OFFLINE;
    for (index = 0U; index < motor_count; index++)
    {
        session->current_raw[index] = 0;
    }
    vofa_telemetry_session = session;
    if (!Vofa_ApplyProfile(session))
    {
        session->active = false;
        return false;
    }
    return true;
}

/* 功能：解析 START 命令中的电机列表和发送周期；用途：形成会话启动参数；返回 true 表示所有字段合法。 */
static bool Vofa_ParseStart(char *const *tokens,
                            uint8_t token_count,
                            uint8_t start_index,
                            uint8_t *motor_ids,
                            uint8_t *motor_count,
                            uint32_t *period_ms)
{
    uint32_t parsed;
    uint8_t index;

    if (start_index >= token_count)
    {
        return false;
    }
    *period_ms = VOFA_DEFAULT_PERIOD_MS;
    if (!Vofa_StringEquals(tokens[start_index], "IDS"))
    {
        if (!Vofa_ParseU32(tokens[start_index], 1U, 8U, &parsed))
        {
            return false;
        }
        motor_ids[0] = (uint8_t)parsed;
        *motor_count = 1U;
        if ((uint8_t)(start_index + 1U) < token_count)
        {
            if (!Vofa_ParseU32(tokens[start_index + 1U],
                               VOFA_MIN_PERIOD_MS,
                               VOFA_MAX_PERIOD_MS,
                               period_ms) ||
                ((uint8_t)(start_index + 2U) != token_count))
            {
                return false;
            }
        }
        return true;
    }

    if (((uint8_t)(start_index + 2U) >= token_count) ||
        !Vofa_ParseU32(tokens[start_index + 1U], 1U, 8U, &parsed))
    {
        return false;
    }
    *motor_count = (uint8_t)parsed;
    if ((uint32_t)start_index + 4U + *motor_count != token_count)
    {
        return false;
    }
    for (index = 0U; index < *motor_count; index++)
    {
        uint8_t previous;

        if (!Vofa_ParseU32(tokens[start_index + 2U + index],
                           1U,
                           8U,
                           &parsed))
        {
            return false;
        }
        motor_ids[index] = (uint8_t)parsed;
        for (previous = 0U; previous < index; previous++)
        {
            if (motor_ids[previous] == motor_ids[index])
            {
                return false;
            }
        }
    }
    if (!Vofa_StringEquals(tokens[start_index + 2U + *motor_count],
                           "PERIOD") ||
        !Vofa_ParseU32(tokens[start_index + 3U + *motor_count],
                       VOFA_MIN_PERIOD_MS,
                       VOFA_MAX_PERIOD_MS,
                       period_ms))
    {
        return false;
    }
    return true;
}

/* 功能：把会话目标应用到各电机控制器；用途：周期执行电流、速度或位置调试命令；返回 true 表示全部设置成功。 */
static bool Vofa_ApplyTarget(vofa_session_t *session, uint32_t tick_ms)
{
    uint8_t index;
    bool success;

    if (session->auto_tune_active)
    {
        return true;
    }
    success = true;
    for (index = 0U; index < session->motor_count; index++)
    {
        uint8_t motor_id;
        float target;

        motor_id = session->motor_ids[index];
        if (session->mode == VOFA_MODE_CURRENT)
        {
            target = session->target_current_a;
        }
        else if (session->mode == VOFA_MODE_SPEED)
        {
            target = session->target_speed_rpm * VOFA_RPM_TO_RAD_S;
        }
        else if (session->mode == VOFA_MODE_POSITION)
        {
            target = session->target_position_rad;
        }
        else
        {
            target = 0.0f;
        }

        if (session->family == VOFA_FAMILY_M2006)
        {
            m2006_mode_t mode;

            mode = (session->mode == VOFA_MODE_CURRENT) ?
                   M2006_MODE_CURRENT :
                   (session->mode == VOFA_MODE_SPEED) ?
                   M2006_MODE_VELOCITY :
                   (session->mode == VOFA_MODE_POSITION) ?
                   M2006_MODE_POSITION : M2006_MODE_STOP;
            success = M2006_SetTarget(session->can_bus,
                                      motor_id,
                                      mode,
                                      target,
                                      tick_ms) && success;
        }
        else
        {
            m3508_mode_t mode;

            mode = (session->mode == VOFA_MODE_CURRENT) ?
                   M3508_MODE_CURRENT :
                   (session->mode == VOFA_MODE_SPEED) ?
                   M3508_MODE_VELOCITY :
                   (session->mode == VOFA_MODE_POSITION) ?
                   M3508_MODE_POSITION : M3508_MODE_STOP;
            success = M3508_SetTarget(session->can_bus,
                                      motor_id,
                                      mode,
                                      target,
                                      tick_ms) && success;
        }
    }
    return success;
}

/* 功能：向会话内电机应用速度环或位置环 PID；用途：支持 VOFA 在线整定；返回 true 表示全部参数设置成功。 */
static bool Vofa_ApplyPid(vofa_session_t *session,
                          bool speed_loop,
                          const float values[5])
{
    uint8_t index;
    bool success;
    float current_scale;

    success = true;
    current_scale = Vofa_CurrentMaximum(session->family) /
                    Vofa_RawMaximum(session->family);
    for (index = 0U; index < session->motor_count; index++)
    {
        uint8_t motor_id;

        motor_id = session->motor_ids[index];
        if (session->family == VOFA_FAMILY_M2006)
        {
            m2006_pid_cfg_t cfg;

            if (speed_loop)
            {
                cfg.kp = values[0] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.ki = values[1] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.kd = values[2] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.integral_limit = values[3] * VOFA_RPM_TO_RAD_S;
                cfg.output_limit = values[4] * current_scale;
                success = M2006_SetSpeedPid(session->can_bus,
                                            motor_id,
                                            &cfg) && success;
            }
            else
            {
                cfg.kp = values[0] * VOFA_RPM_TO_RAD_S;
                cfg.ki = values[1] * VOFA_RPM_TO_RAD_S;
                cfg.kd = values[2] * VOFA_RPM_TO_RAD_S;
                cfg.integral_limit = values[3];
                cfg.output_limit = values[4] * VOFA_RPM_TO_RAD_S;
                success = M2006_SetPositionPid(session->can_bus,
                                               motor_id,
                                               &cfg) && success;
            }
        }
        else
        {
            m3508_pid_cfg_t cfg;

            if (speed_loop)
            {
                cfg.kp = values[0] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.ki = values[1] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.kd = values[2] * current_scale * VOFA_RAD_S_TO_RPM;
                cfg.integral_limit = values[3] * VOFA_RPM_TO_RAD_S;
                cfg.output_limit = values[4] * current_scale;
                success = M3508_SetSpeedPid(session->can_bus,
                                            motor_id,
                                            &cfg) && success;
            }
            else
            {
                cfg.kp = values[0] * VOFA_RPM_TO_RAD_S;
                cfg.ki = values[1] * VOFA_RPM_TO_RAD_S;
                cfg.kd = values[2] * VOFA_RPM_TO_RAD_S;
                cfg.integral_limit = values[3];
                cfg.output_limit = values[4] * VOFA_RPM_TO_RAD_S;
                success = M3508_SetPositionPid(session->can_bus,
                                               motor_id,
                                               &cfg) && success;
            }
        }
    }
    return success;
}

/* 功能：解析并执行一条 VOFA 业务命令；用途：处理 START、目标、PID、自动整定和停止操作；返回值为协议状态码。 */
static int32_t Vofa_ProcessCommand(char *line, uint32_t tick_ms)
{
    char *tokens[VOFA_TOKEN_COUNT];
    uint8_t token_count;
    uint8_t index;
    uint8_t can_bus;
    uint32_t parsed;
    vofa_family_t family;
    vofa_session_t *session;
    const char *verb;

    token_count = Vofa_Tokenize(line, tokens);
    if ((token_count < 3U) || !Vofa_StringEquals(tokens[0], "VOFA"))
    {
        return VOFA_STATUS_INVALID;
    }
    index = 1U;
    if (Vofa_StringEquals(tokens[index], "SEQ"))
    {
        index = (uint8_t)(index + 2U);
        if (index >= token_count)
        {
            return VOFA_STATUS_INVALID;
        }
    }

    if (Vofa_StringEquals(tokens[index], "M2006"))
    {
        family = VOFA_FAMILY_M2006;
    }
    else if (Vofa_StringEquals(tokens[index], "M3508"))
    {
        family = VOFA_FAMILY_M3508;
    }
    else
    {
        return VOFA_STATUS_INVALID;
    }
    index++;
    can_bus = Vofa_DefaultCanBus(family);
    if ((index < token_count) && Vofa_StringEquals(tokens[index], "CAN"))
    {
        if (((uint8_t)(index + 1U) >= token_count) ||
            !Vofa_ParseU32(tokens[index + 1U],
                           1U,
                           VOFA_CAN_BUS_COUNT,
                           &parsed))
        {
            return VOFA_STATUS_INVALID;
        }
        can_bus = (uint8_t)parsed;
        index = (uint8_t)(index + 2U);
    }
    else if ((index < token_count) &&
             (strlen(tokens[index]) >= 4U) &&
             ((tokens[index][0] == 'C') || (tokens[index][0] == 'c')) &&
             ((tokens[index][1] == 'A') || (tokens[index][1] == 'a')) &&
             ((tokens[index][2] == 'N') || (tokens[index][2] == 'n')) &&
             (tokens[index][3] != '\0'))
    {
        if (!Vofa_ParseU32(&tokens[index][3],
                           1U,
                           VOFA_CAN_BUS_COUNT,
                           &parsed))
        {
            return VOFA_STATUS_INVALID;
        }
        can_bus = (uint8_t)parsed;
        index++;
    }
    if (index >= token_count)
    {
        return VOFA_STATUS_INVALID;
    }
    session = Vofa_GetSession(family, can_bus);
    verb = tokens[index++];

    if (Vofa_StringEquals(verb, "START"))
    {
        uint8_t motor_ids[VOFA_DJI_MOTOR_COUNT];
        uint8_t motor_count;
        uint32_t period_ms;

        if (!Vofa_ParseStart(tokens,
                             token_count,
                             index,
                             motor_ids,
                             &motor_count,
                             &period_ms))
        {
            return VOFA_STATUS_INVALID;
        }
        if (Vofa_RouteConflicts(session,
                                can_bus,
                                motor_ids,
                                motor_count))
        {
            return VOFA_STATUS_BUSY;
        }
        return Vofa_StartSession(session,
                                 can_bus,
                                 motor_ids,
                                 motor_count,
                                 period_ms,
                                 tick_ms) ?
               VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
    }

    if (!session->active || (can_bus != session->can_bus))
    {
        return VOFA_STATUS_INVALID;
    }
    vofa_telemetry_session = session;

    if (Vofa_StringEquals(verb, "FOCUS"))
    {
        return (index == token_count) ? VOFA_STATUS_OFFLINE :
               VOFA_STATUS_INVALID;
    }

    if (Vofa_StringEquals(verb, "CURRENT"))
    {
        float value;

        if ((index + 1U != token_count) ||
            !Vofa_ParseFloat(tokens[index], &value) ||
            (value < -session->current_limit_a) ||
            (value > session->current_limit_a))
        {
            return VOFA_STATUS_INVALID;
        }
        session->target_current_a = value;
        session->target_speed_rpm = 0.0f;
        session->mode = VOFA_MODE_CURRENT;
        session->auto_tune_active = false;
        session->command_status = VOFA_STATUS_OFFLINE;
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "SPEED"))
    {
        float value;

        if ((index + 1U != token_count) ||
            !Vofa_ParseFloat(tokens[index], &value))
        {
            return VOFA_STATUS_INVALID;
        }
        session->target_speed_rpm = value;
        session->target_current_a = 0.0f;
        session->mode = VOFA_MODE_SPEED;
        session->auto_tune_active = false;
        session->command_status = VOFA_STATUS_OFFLINE;
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "ANGLE") ||
        Vofa_StringEquals(verb, "POSITION"))
    {
        float value;

        if ((index + 1U != token_count) ||
            !Vofa_ParseFloat(tokens[index], &value))
        {
            return VOFA_STATUS_INVALID;
        }
        session->target_position_rad = Vofa_StringEquals(verb, "ANGLE") ?
                                       value * VOFA_DEG_TO_RAD : value;
        session->target_current_a = 0.0f;
        session->target_speed_rpm = 0.0f;
        session->mode = VOFA_MODE_POSITION;
        session->auto_tune_active = false;
        session->command_status = VOFA_STATUS_OFFLINE;
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "PROFILE"))
    {
        float speed;
        float acceleration;

        if ((index + 2U != token_count) ||
            !Vofa_ParseFloat(tokens[index], &speed) ||
            !Vofa_ParseFloat(tokens[index + 1U], &acceleration) ||
            (speed <= 0.0f) || (acceleration <= 0.0f))
        {
            return VOFA_STATUS_INVALID;
        }
        session->position_speed_rpm = speed;
        session->acceleration_rpm_s = acceleration;
        return Vofa_ApplyProfile(session) ?
               VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
    }
    if (Vofa_StringEquals(verb, "PID"))
    {
        float values[5];
        uint8_t value_index;
        bool speed_loop;

        if ((index + 6U != token_count) ||
            (!Vofa_StringEquals(tokens[index], "SPEED") &&
             !Vofa_StringEquals(tokens[index], "POSITION")))
        {
            return VOFA_STATUS_INVALID;
        }
        speed_loop = Vofa_StringEquals(tokens[index], "SPEED");
        for (value_index = 0U; value_index < 5U; value_index++)
        {
            if (!Vofa_ParseFloat(tokens[index + 1U + value_index],
                                 &values[value_index]) ||
                (values[value_index] < 0.0f))
            {
                return VOFA_STATUS_INVALID;
            }
        }
        if ((values[4] <= 0.0f) ||
            !Vofa_ApplyPid(session, speed_loop, values))
        {
            return VOFA_STATUS_INVALID;
        }
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "ONLINE"))
    {
        bool enabled;
        uint8_t motor_index;
        bool success;

        if (index + 1U != token_count)
        {
            return VOFA_STATUS_INVALID;
        }
        if (Vofa_StringEquals(tokens[index], "ON") ||
            Vofa_StringEquals(tokens[index], "ENABLE") ||
            Vofa_StringEquals(tokens[index], "1"))
        {
            enabled = true;
        }
        else if (Vofa_StringEquals(tokens[index], "OFF") ||
                 Vofa_StringEquals(tokens[index], "DISABLE") ||
                 Vofa_StringEquals(tokens[index], "0"))
        {
            enabled = false;
        }
        else
        {
            return VOFA_STATUS_INVALID;
        }
        success = true;
        for (motor_index = 0U;
             motor_index < session->motor_count;
             motor_index++)
        {
            if (family == VOFA_FAMILY_M2006)
            {
                success = M2006_SetOnlinePidEnabled(
                              session->can_bus,
                              session->motor_ids[motor_index],
                              enabled) && success;
            }
            else
            {
                success = M3508_SetOnlinePidEnabled(
                              session->can_bus,
                              session->motor_ids[motor_index],
                              enabled) && success;
            }
        }
        return success ? VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
    }
    if (Vofa_StringEquals(verb, "AUTOTUNE"))
    {
        uint8_t motor_id;

        if (session->motor_count != 1U)
        {
            return VOFA_STATUS_INVALID;
        }
        motor_id = session->motor_ids[0];
        if ((index + 1U == token_count) &&
            Vofa_StringEquals(tokens[index], "CANCEL"))
        {
            bool success;

            success = (family == VOFA_FAMILY_M2006) ?
                      M2006_CancelAutoTune(can_bus, motor_id) :
                      M3508_CancelAutoTune(can_bus, motor_id);
            if (success)
            {
                session->mode = VOFA_MODE_STOP;
                session->auto_tune_active = false;
                session->command_status = VOFA_STATUS_OFFLINE;
            }
            return success ? VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
        }
        if ((index + 4U == token_count) &&
            Vofa_StringEquals(tokens[index], "SPEED"))
        {
            float relay_current_a;
            float hysteresis_rpm;
            float safety_speed_rpm;
            bool success;

            if (!Vofa_ParseFloat(tokens[index + 1U], &relay_current_a) ||
                !Vofa_ParseFloat(tokens[index + 2U], &hysteresis_rpm) ||
                !Vofa_ParseFloat(tokens[index + 3U], &safety_speed_rpm))
            {
                return VOFA_STATUS_INVALID;
            }
            success = (family == VOFA_FAMILY_M2006) ?
                      M2006_StartSpeedAutoTune(
                          can_bus,
                          motor_id,
                          relay_current_a,
                          hysteresis_rpm * VOFA_RPM_TO_RAD_S,
                          safety_speed_rpm * VOFA_RPM_TO_RAD_S,
                          tick_ms) :
                      M3508_StartSpeedAutoTune(
                          can_bus,
                          motor_id,
                          relay_current_a,
                          hysteresis_rpm * VOFA_RPM_TO_RAD_S,
                          safety_speed_rpm * VOFA_RPM_TO_RAD_S,
                          tick_ms);
            if (success)
            {
                session->mode = VOFA_MODE_STOP;
                session->auto_tune_active = true;
                session->command_status = VOFA_STATUS_TUNING;
            }
            return success ? VOFA_STATUS_TUNING : VOFA_STATUS_INVALID;
        }
        return VOFA_STATUS_INVALID;
    }
    if (Vofa_StringEquals(verb, "LIMIT"))
    {
        float value;

        if ((index + 1U != token_count) ||
            !Vofa_ParseFloat(tokens[index], &value) || (value <= 0.0f) ||
            (value > Vofa_CurrentMaximum(family)))
        {
            return VOFA_STATUS_INVALID;
        }
        session->current_limit_a = value;
        return Vofa_ApplyProfile(session) ?
               VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
    }
    if (Vofa_StringEquals(verb, "ZERO"))
    {
        uint8_t motor_index;
        bool success;

        if ((index != token_count) || (session->mode != VOFA_MODE_STOP))
        {
            return VOFA_STATUS_INVALID;
        }
        success = true;
        for (motor_index = 0U;
             motor_index < session->motor_count;
             motor_index++)
        {
            if (family == VOFA_FAMILY_M2006)
            {
                success = M2006_ZeroPosition(
                              session->can_bus,
                              session->motor_ids[motor_index]) && success;
            }
            else
            {
                success = M3508_ZeroPosition(
                              session->can_bus,
                              session->motor_ids[motor_index]) && success;
            }
        }
        session->target_position_rad = 0.0f;
        return success ? VOFA_STATUS_OFFLINE : VOFA_STATUS_INVALID;
    }
    if (Vofa_StringEquals(verb, "STOP") ||
        Vofa_StringEquals(verb, "MONITOR"))
    {
        if (index != token_count)
        {
            return VOFA_STATUS_INVALID;
        }
        Vofa_StopSession(session, tick_ms);
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "PERIOD"))
    {
        if ((index + 1U != token_count) ||
            !Vofa_ParseU32(tokens[index],
                           VOFA_MIN_PERIOD_MS,
                           VOFA_MAX_PERIOD_MS,
                           &parsed))
        {
            return VOFA_STATUS_INVALID;
        }
        session->telemetry_period_ms = parsed;
        return VOFA_STATUS_OFFLINE;
    }
    if (Vofa_StringEquals(verb, "EXIT"))
    {
        if (index != token_count)
        {
            return VOFA_STATUS_INVALID;
        }
        Vofa_StopSession(session, tick_ms);
        session->active = false;
        if (vofa_telemetry_session == session)
        {
            vofa_telemetry_session = NULL;
        }
        return VOFA_STATUS_EXITING;
    }
    return VOFA_STATUS_INVALID;
}

/* 功能：发送文本格式的 VOFA 命令确认；用途：反馈命令序号和执行状态；无返回值表示发送结果由通信层处理。 */
static void Vofa_SendAck(uint32_t sequence, int32_t status)
{
    char response[48];
    int length;

    length = snprintf(response,
                      sizeof(response),
                      "VOFA ACK %lu %ld\r\n",
                      (unsigned long)sequence,
                      (long)status);
    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        (void)CommRuntime_PcTransmitCopy((const uint8_t *)response,
                                         (uint16_t)length);
    }
}

/* 功能：识别并处理一行完整 VOFA 文本；用途：解析可选序号并发送 ACK；返回 true 表示该行属于 VOFA 协议。 */
static bool Vofa_ProcessLine(char *line, uint32_t tick_ms)
{
    char sequence_copy[VOFA_LINE_SIZE];
    char *tokens[VOFA_TOKEN_COUNT];
    uint8_t token_count;
    uint32_t sequence;
    int32_t status;
    bool sequenced;

    (void)memcpy(sequence_copy, line, strlen(line) + 1U);
    token_count = Vofa_Tokenize(sequence_copy, tokens);
    if ((token_count == 0U) || !Vofa_StringEquals(tokens[0], "VOFA"))
    {
        return false;
    }

    sequenced = (token_count >= 3U) &&
                Vofa_StringEquals(tokens[1], "SEQ") &&
                Vofa_ParseU32(tokens[2], 1U, UINT32_MAX, &sequence);
    if (sequenced && vofa_last_sequence_valid &&
        (sequence == vofa_last_sequence))
    {
        Vofa_SendAck(sequence, vofa_last_sequence_status);
        return true;
    }

    status = Vofa_ProcessCommand(line, tick_ms);
    if (sequenced)
    {
        if ((status != VOFA_STATUS_BUSY) &&
            (status != VOFA_STATUS_TIMEOUT))
        {
            vofa_last_sequence = sequence;
            vofa_last_sequence_status = status;
            vofa_last_sequence_valid = true;
        }
        Vofa_SendAck(sequence, status);
    }
    return true;
}

/* 功能：在会话电机列表中查找节点号；用途：把 CAN 反馈映射到遥测槽位；返回负数表示未找到。 */
static int8_t Vofa_FindMotorIndex(const vofa_session_t *session,
                                  uint8_t motor_id)
{
    uint8_t index;

    for (index = 0U; index < session->motor_count; index++)
    {
        if (session->motor_ids[index] == motor_id)
        {
            return (int8_t)index;
        }
    }
    return -1;
}

/* 功能：汇总并发送各 CAN 总线的 DJI 四电机电流组帧；用途：统一下发 M2006/M3508 调试输出；无返回值表示完成本周期发送。 */
static void Vofa_SendGroups(uint32_t tick_ms)
{
    uint8_t can_bus;
    uint8_t group_index;

    for (can_bus = 1U; can_bus <= VOFA_CAN_BUS_COUNT; can_bus++)
    {
        for (group_index = 0U; group_index < 2U; group_index++)
        {
            int16_t currents[DJI_GROUP_MOTOR_COUNT] = {0};
            uint8_t start_motor_id;
            uint8_t slot;
            bool group_active;
            can_frame_t frame;

            start_motor_id = (uint8_t)(group_index * 4U + 1U);
            group_active = false;
            for (slot = 0U; slot < DJI_GROUP_MOTOR_COUNT; slot++)
            {
                uint8_t motor_id;
                uint8_t session_index;

                motor_id = (uint8_t)(start_motor_id + slot);
                for (session_index = 0U;
                     session_index < VOFA_SESSION_COUNT;
                     session_index++)
                {
                    vofa_session_t *session;
                    int8_t motor_index;
                    int16_t current_raw;
                    bool calculated;

                    session = &vofa_sessions[session_index];
                    if (!session->active ||
                        (session->can_bus != can_bus))
                    {
                        continue;
                    }
                    motor_index = Vofa_FindMotorIndex(session, motor_id);
                    if (motor_index < 0)
                    {
                        continue;
                    }
                    group_active = true;
                    current_raw = 0;
                    if (session->family == VOFA_FAMILY_M2006)
                    {
                        calculated = M2006_CalcCurrentRaw(
                                         can_bus,
                                         motor_id,
                                         tick_ms,
                                         &current_raw);
                    }
                    else
                    {
                        calculated = M3508_CalcCurrentRaw(
                                         can_bus,
                                         motor_id,
                                         tick_ms,
                                         &current_raw);
                    }
                    if (!calculated)
                    {
                        session->command_status = VOFA_STATUS_INVALID;
                        current_raw = 0;
                    }
                    current_raw = Vofa_ClampCurrentRaw(session, current_raw);
                    currents[slot] = current_raw;
                    session->current_raw[(uint8_t)motor_index] = current_raw;
                }
            }
            if (group_active &&
                DjiGroup_BuildFrame(start_motor_id, currents, &frame))
            {
                (void)BspCan_Send(can_bus, &frame);
            }
        }
    }
}

/* 功能：填充单台电机的 VOFA 遥测通道；用途：汇集目标、反馈、误差和自动整定状态；结果写入 channels。 */
static void Vofa_FillTelemetry(const vofa_session_t *session,
                               uint8_t motor_index,
                               uint32_t tick_ms,
                               float channels[VOFA_CHANNEL_COUNT])
{
    uint8_t motor_id;
    bool feedback_fresh;
    float actual_position_rad;
    float actual_speed_rpm;
    float feedback_current_a;
    float final_position_rad;
    float reference_position_rad;
    float final_speed_rpm;
    float reference_speed_rpm;
    float reference_acceleration_rpm_s;
    float current_command_a;
    float position_error_rad;
    float speed_error_rpm;
    int32_t auto_tune_status;

    (void)memset(channels, 0, VOFA_CHANNEL_COUNT * sizeof(float));
    motor_id = session->motor_ids[motor_index];
    feedback_fresh = false;
    actual_position_rad = 0.0f;
    actual_speed_rpm = 0.0f;
    feedback_current_a = 0.0f;
    final_position_rad = 0.0f;
    reference_position_rad = 0.0f;
    final_speed_rpm = 0.0f;
    reference_speed_rpm = 0.0f;
    reference_acceleration_rpm_s = 0.0f;
    current_command_a = 0.0f;
    position_error_rad = 0.0f;
    speed_error_rpm = 0.0f;
    auto_tune_status = VOFA_STATUS_OFFLINE;
    if (session->family == VOFA_FAMILY_M2006)
    {
        m2006_feedback_t feedback;
        m2006_online_pid_state_t online;
        m2006_motion_state_t motion;
        m2006_autotune_state_t auto_tune;

        (void)memset(&feedback, 0, sizeof(feedback));
        (void)memset(&online, 0, sizeof(online));
        (void)memset(&motion, 0, sizeof(motion));
        (void)memset(&auto_tune, 0, sizeof(auto_tune));
        feedback_fresh = M2006_GetFeedback(session->can_bus,
                                           motor_id,
                                           &feedback) &&
                         ((tick_ms - feedback.updated_at_ms) <=
                          VOFA_FEEDBACK_TIMEOUT_MS);
        actual_position_rad = feedback.output_pos_rad;
        actual_speed_rpm = feedback.output_vel_rad_s * VOFA_RAD_S_TO_RPM;
        feedback_current_a = feedback.torque_current_a;
        (void)M2006_GetOnlinePidState(session->can_bus,
                                      motor_id,
                                      &online);
        (void)M2006_GetMotionState(session->can_bus,
                                   motor_id,
                                   &motion);
        (void)M2006_GetAutoTuneState(session->can_bus,
                                     motor_id,
                                     &auto_tune);
        final_position_rad = motion.final_position_rad;
        reference_position_rad = motion.reference_position_rad;
        final_speed_rpm = motion.final_velocity_rad_s * VOFA_RAD_S_TO_RPM;
        reference_speed_rpm = motion.reference_velocity_rad_s *
                              VOFA_RAD_S_TO_RPM;
        reference_acceleration_rpm_s =
            motion.reference_acceleration_rad_s2 * VOFA_RAD_S_TO_RPM;
        current_command_a = motion.current_command_a;
        position_error_rad = motion.position_error_rad;
        speed_error_rpm = motion.velocity_error_rad_s * VOFA_RAD_S_TO_RPM;
        if (auto_tune.status == M2006_AUTOTUNE_RUNNING)
        {
            auto_tune_status = VOFA_STATUS_TUNING;
        }
        else if (auto_tune.status == M2006_AUTOTUNE_COMPLETE)
        {
            auto_tune_status = VOFA_STATUS_TUNED;
        }
        else if (auto_tune.status == M2006_AUTOTUNE_FAILED)
        {
            auto_tune_status = VOFA_STATUS_TUNE_FAILED;
        }
        channels[12] = online.applied_kp /
                       (VOFA_M2006_CURRENT_MAX_A /
                        VOFA_M2006_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[13] = online.applied_ki /
                       (VOFA_M2006_CURRENT_MAX_A /
                        VOFA_M2006_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[14] = online.applied_kd /
                       (VOFA_M2006_CURRENT_MAX_A /
                        VOFA_M2006_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[15] = online.enabled ?
                       (float)online.strategy +
                       0.1f * (float)online.active_rule : 0.0f;
    }
    else
    {
        m3508_feedback_t feedback;
        m3508_online_pid_state_t online;
        m3508_motion_state_t motion;
        m3508_autotune_state_t auto_tune;

        (void)memset(&feedback, 0, sizeof(feedback));
        (void)memset(&online, 0, sizeof(online));
        (void)memset(&motion, 0, sizeof(motion));
        (void)memset(&auto_tune, 0, sizeof(auto_tune));
        feedback_fresh = M3508_GetFeedback(session->can_bus,
                                           motor_id,
                                           &feedback) &&
                         ((tick_ms - feedback.updated_at_ms) <=
                          VOFA_FEEDBACK_TIMEOUT_MS);
        actual_position_rad = feedback.output_pos_rad;
        actual_speed_rpm = feedback.output_vel_rad_s * VOFA_RAD_S_TO_RPM;
        feedback_current_a = feedback.torque_current_a;
        (void)M3508_GetOnlinePidState(session->can_bus,
                                      motor_id,
                                      &online);
        (void)M3508_GetMotionState(session->can_bus,
                                   motor_id,
                                   &motion);
        (void)M3508_GetAutoTuneState(session->can_bus,
                                     motor_id,
                                     &auto_tune);
        final_position_rad = motion.final_position_rad;
        reference_position_rad = motion.reference_position_rad;
        final_speed_rpm = motion.final_velocity_rad_s * VOFA_RAD_S_TO_RPM;
        reference_speed_rpm = motion.reference_velocity_rad_s *
                              VOFA_RAD_S_TO_RPM;
        reference_acceleration_rpm_s =
            motion.reference_acceleration_rad_s2 * VOFA_RAD_S_TO_RPM;
        current_command_a = motion.current_command_a;
        position_error_rad = motion.position_error_rad;
        speed_error_rpm = motion.velocity_error_rad_s * VOFA_RAD_S_TO_RPM;
        if (auto_tune.status == M3508_AUTOTUNE_RUNNING)
        {
            auto_tune_status = VOFA_STATUS_TUNING;
        }
        else if (auto_tune.status == M3508_AUTOTUNE_COMPLETE)
        {
            auto_tune_status = VOFA_STATUS_TUNED;
        }
        else if (auto_tune.status == M3508_AUTOTUNE_FAILED)
        {
            auto_tune_status = VOFA_STATUS_TUNE_FAILED;
        }
        channels[12] = online.applied_kp /
                       (VOFA_M3508_CURRENT_MAX_A /
                        VOFA_M3508_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[13] = online.applied_ki /
                       (VOFA_M3508_CURRENT_MAX_A /
                        VOFA_M3508_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[14] = online.applied_kd /
                       (VOFA_M3508_CURRENT_MAX_A /
                        VOFA_M3508_RAW_MAX * VOFA_RAD_S_TO_RPM);
        channels[15] = online.enabled ?
                       (float)online.strategy +
                       0.1f * (float)online.active_rule : 0.0f;
    }

    channels[0] = final_position_rad;
    channels[1] = reference_position_rad;
    channels[2] = actual_position_rad;
    channels[3] = final_speed_rpm;
    channels[4] = reference_speed_rpm;
    channels[5] = actual_speed_rpm;
    channels[6] = reference_acceleration_rpm_s;
    channels[7] = current_command_a;
    channels[8] = feedback_current_a;
    channels[9] = position_error_rad;
    channels[10] = speed_error_rpm;
    channels[11] = (auto_tune_status != VOFA_STATUS_OFFLINE) ?
                   (float)auto_tune_status :
                   (session->command_status < 0) ?
                   (float)session->command_status :
                   (feedback_fresh ?
                    (float)VOFA_STATUS_ONLINE :
                    (float)VOFA_STATUS_OFFLINE);
}

/* 功能：打包并发送一个会话的 JustFloat 遥测；用途：供 VOFA 实时绘图和调参观察；无返回值表示更新发送统计与时刻。 */
static void Vofa_SendTelemetry(vofa_session_t *session, uint32_t tick_ms)
{
    uint8_t motor_index;
    uint16_t payload_size;
    uint32_t tail;

    payload_size = (uint16_t)(session->motor_count *
                              VOFA_CHANNEL_COUNT * sizeof(float));
    for (motor_index = 0U;
         motor_index < session->motor_count;
         motor_index++)
    {
        float channels[VOFA_CHANNEL_COUNT];

        Vofa_FillTelemetry(session, motor_index, tick_ms, channels);
        (void)memcpy(&vofa_telemetry_frame[
                         motor_index * VOFA_CHANNEL_COUNT * sizeof(float)],
                     channels,
                     sizeof(channels));
    }
    tail = VOFA_JUSTFLOAT_TAIL;
    (void)memcpy(&vofa_telemetry_frame[payload_size],
                 &tail,
                 sizeof(tail));
    (void)CommRuntime_TelemetryTransmit(
        vofa_telemetry_frame,
        (uint16_t)(payload_size + sizeof(tail)));
}

/* 功能：初始化全部 VOFA 会话和接收状态；用途：建立调试桥默认参数；无返回值表示所有会话被复位。 */
void VofaBridge_Init(void)
{
    uint8_t family_index;
    uint8_t can_bus;

    (void)memset(vofa_sessions, 0, sizeof(vofa_sessions));
    for (family_index = 0U;
         family_index < VOFA_FAMILY_COUNT;
         family_index++)
    {
        for (can_bus = 1U; can_bus <= VOFA_CAN_BUS_COUNT; can_bus++)
        {
            vofa_session_t *session;

            session = Vofa_GetSession((vofa_family_t)family_index, can_bus);
            session->family = (vofa_family_t)family_index;
            session->can_bus = can_bus;
            if (session->family == VOFA_FAMILY_M2006)
            {
                session->current_limit_a =
                    VOFA_DEFAULT_M2006_CURRENT_LIMIT_A;
                session->position_speed_rpm = 100.0f;
                session->acceleration_rpm_s = 300.0f;
            }
            else
            {
                session->current_limit_a =
                    VOFA_DEFAULT_M3508_CURRENT_LIMIT_A;
                session->position_speed_rpm = 150.0f;
                session->acceleration_rpm_s = 500.0f;
            }
        }
    }
    vofa_telemetry_session = NULL;
    vofa_rx_index = 0U;
    vofa_rx_candidate = false;
    vofa_rx_overflow = false;
    vofa_last_sequence = 0U;
    vofa_last_sequence_status = VOFA_STATUS_OFFLINE;
    vofa_last_sequence_valid = false;
}

/* 功能：接收字节流并按行解析 VOFA 命令；用途：从 UART 数据中提取文本控制消息；返回 true 表示至少消费了 VOFA 数据。 */
bool VofaBridge_Receive(const uint8_t *data,
                        size_t size,
                        uint32_t tick_ms)
{
    size_t data_index;
    bool consumed;

    if ((data == NULL) || (size == 0U))
    {
        return false;
    }
    consumed = vofa_rx_candidate;
    for (data_index = 0U; data_index < size; data_index++)
    {
        uint8_t value;

        value = data[data_index];
        if (!vofa_rx_candidate && (vofa_rx_index == 0U))
        {
            if ((value == '\r') || (value == '\n') ||
                (value == ' ') || (value == '\t'))
            {
                continue;
            }
            if ((value != 'V') && (value != 'v'))
            {
                return consumed;
            }
            vofa_rx_candidate = true;
            consumed = true;
        }

        if ((value == '\r') || (value == '\n'))
        {
            if (vofa_rx_candidate && (vofa_rx_index > 0U) &&
                !vofa_rx_overflow)
            {
                vofa_rx_line[vofa_rx_index] = '\0';
                (void)Vofa_ProcessLine(vofa_rx_line, tick_ms);
            }
            vofa_rx_index = 0U;
            vofa_rx_candidate = false;
            vofa_rx_overflow = false;
            continue;
        }
        if ((value != '\t') && ((value < 0x20U) || (value > 0x7EU)))
        {
            vofa_rx_index = 0U;
            vofa_rx_candidate = false;
            vofa_rx_overflow = false;
            continue;
        }
        if (!vofa_rx_overflow)
        {
            if (vofa_rx_index < (VOFA_LINE_SIZE - 1U))
            {
                vofa_rx_line[vofa_rx_index++] = (char)value;
            }
            else
            {
                vofa_rx_overflow = true;
            }
        }
    }
    return consumed;
}

/* 功能：执行 VOFA 桥的 1 ms 周期任务；用途：处理会话超时、目标应用、分组发送和遥测；无返回值表示完成一次调度。 */
void VofaBridge_Control1ms(uint32_t tick_ms)
{
    uint8_t session_index;

    for (session_index = 0U;
         session_index < VOFA_SESSION_COUNT;
         session_index++)
    {
        vofa_session_t *session;

        session = &vofa_sessions[session_index];
        if (!session->active)
        {
            continue;
        }
        if (!Vofa_ApplyTarget(session, tick_ms))
        {
            session->command_status = VOFA_STATUS_INVALID;
        }
    }
    Vofa_SendGroups(tick_ms);

    if ((vofa_telemetry_session != NULL) &&
        vofa_telemetry_session->active &&
        ((int32_t)(tick_ms -
                   vofa_telemetry_session->next_telemetry_ms) >= 0))
    {
        Vofa_SendTelemetry(vofa_telemetry_session, tick_ms);
        vofa_telemetry_session->next_telemetry_ms =
            tick_ms + vofa_telemetry_session->telemetry_period_ms;
    }
}

/* 功能：把 DJI CAN 反馈路由到对应 VOFA 会话；用途：更新调试电机反馈和在线整定输入；无返回值表示匹配帧已记录。 */
void VofaBridge_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms)
{
    uint8_t motor_id;
    uint8_t session_index;

    if ((frame == NULL) || frame->extended || (frame->dlc != 8U) ||
        (frame->id < 0x201U) || (frame->id > 0x208U))
    {
        return;
    }
    motor_id = (uint8_t)(frame->id - 0x200U);
    for (session_index = 0U;
         session_index < VOFA_SESSION_COUNT;
         session_index++)
    {
        vofa_session_t *session;
        motor_model_t model;

        session = &vofa_sessions[session_index];
        if (!session->active || (session->can_bus != can_bus) ||
            (Vofa_FindMotorIndex(session, motor_id) < 0))
        {
            continue;
        }
        model = (session->family == VOFA_FAMILY_M2006) ?
                MOTOR_MODEL_M2006 : MOTOR_MODEL_M3508;
        if (UpperMotorPort_IsDjiConfigured(can_bus, model, motor_id))
        {
            continue;
        }
        if (session->family == VOFA_FAMILY_M2006)
        {
            (void)M2006_OnFrame(can_bus, motor_id, frame, tick_ms);
        }
        else
        {
            (void)M3508_OnFrame(can_bus, motor_id, frame, tick_ms);
        }
    }
}

/* 功能：查询是否存在活动 VOFA 会话；用途：判断调试桥是否正在占用电机控制；返回 true 表示至少一个会话已启动。 */
bool VofaBridge_IsActive(void)
{
    uint8_t session_index;

    for (session_index = 0U;
         session_index < VOFA_SESSION_COUNT;
         session_index++)
    {
        if (vofa_sessions[session_index].active)
        {
            return true;
        }
    }
    return false;
}
