#include "imu_main.h"

#include "imu.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define IMU_BOOT_DELAY_MS          100U
#define IMU_CAL_CMD_DELAY_MS       50U
#define IMU_GYRO_CAL_WAIT_MS       4000U
#define IMU_CONFIG_START_DELAY_MS  100U
#define IMU_CONFIG_CMD_DELAY_MS    20U
#define IMU_GYRO_BIAS_SAMPLE_COUNT 200U
#define IMU_ONLINE_TIMEOUT_MS      100U
#define IMU_YAW_KALMAN_Q           0.02f
#define IMU_YAW_KALMAN_R           3.0f
#define IMU_ANGLE_HALF_RANGE_DEG   180.0f
#define IMU_ANGLE_RANGE_DEG        360.0f
#define IMU_YAW_TX_PERIOD_MS       50U
#define IMU_YAW_TX_SCALE           100.0f
#define IMU_YAW_FRAME_HEADER_0     0xA5U
#define IMU_YAW_FRAME_HEADER_1     0x5CU
#define IMU_YAW_FRAME_LENGTH       5U

#define IMU_YAW_PERIOD_MS          5U
#define IMU_YAW_CMD_THRESHOLD      5
#define IMU_YAW_LINEAR_THRESHOLD   5
#define IMU_YAW_DEADZONE_DEG       0.5f
#define IMU_YAW_I_ACTIVE_DEG       10.0f
#define IMU_YAW_I_DECAY            0.90f
#define IMU_YAW_GYRO_K             5.0f
#define IMU_GYRO_FILTER_Q          0.1f
#define IMU_GYRO_FILTER_R          2.0f

//航向环
#define IMU_YAW_MOVE_KP            1.8f
#define IMU_YAW_MOVE_KI            0.25f
#define IMU_YAW_MOVE_KD            1.8f
#define IMU_YAW_MOVE_I_MAX         8.0f
#define IMU_YAW_MOVE_OUT_MAX       1000.0f

#define IMU_YAW_STOP_KP            1.0f
#define IMU_YAW_STOP_KI            0.18f
#define IMU_YAW_STOP_KD            1.8f
#define IMU_YAW_STOP_I_MAX         12.0f
#define IMU_YAW_STOP_OUT_MAX       250.0f

typedef enum
{
    IMU_INIT_WAIT_BOOT,
    IMU_INIT_SEND_CAL,
    IMU_INIT_WAIT_CAL,
    IMU_INIT_WAIT_CONFIG,
    IMU_INIT_SEND_CONFIG,
    IMU_INIT_SAMPLE_BIAS,
    IMU_INIT_COMPLETE,
    IMU_INIT_ERROR
} imu_init_step_t;

typedef struct
{
    const uint8_t *data;
    uint8_t length;
} imu_command_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float integral;
    float last_error;
    bool first_run;
} imu_yaw_pid_t;

typedef struct
{
    float estimate;
    float covariance;
    bool valid;
} imu_gyro_filter_t;

static const uint8_t cmd_enter_setup[] = {0xAAU, 0x06U, 0x01U, 0x0DU};
static const uint8_t cmd_gyro_cal[] = {0xAAU, 0x03U, 0x02U, 0x0DU};
static const uint8_t cmd_set_output_485[] = {0xAAU, 0x0AU, 0x01U, 0x0DU};
static const uint8_t cmd_set_baud_921600[] = {
    0xAAU, 0x0DU, 0x01U, 0x05U, 0x0DU
};
static const uint8_t cmd_enable_active[] = {
    0xAAU, 0x01U, 0x13U, 0x0DU
};
static const uint8_t cmd_open_gyro[] = {0xAAU, 0x01U, 0x15U, 0x0DU};
static const uint8_t cmd_open_euler[] = {0xAAU, 0x01U, 0x16U, 0x0DU};
static const uint8_t cmd_close_quat[] = {0xAAU, 0x01U, 0x07U, 0x0DU};
static const uint8_t cmd_exit_setup[] = {0xAAU, 0x06U, 0x00U, 0x0DU};

static const imu_command_t config_commands[] = {
    {cmd_enter_setup, sizeof(cmd_enter_setup)},
    {cmd_set_output_485, sizeof(cmd_set_output_485)},
    {cmd_set_baud_921600, sizeof(cmd_set_baud_921600)},
    {cmd_enable_active, sizeof(cmd_enable_active)},
    {cmd_open_gyro, sizeof(cmd_open_gyro)},
    {cmd_open_euler, sizeof(cmd_open_euler)},
    {cmd_close_quat, sizeof(cmd_close_quat)},
    {cmd_exit_setup, sizeof(cmd_exit_setup)}
};

static imu_data_t imu_data;
static imu_init_step_t init_step;
static uint32_t next_action_ms;
static uint32_t last_gyro_sequence;
static uint32_t last_yaw_sequence;
static float gyro_bias_sum_deg_s;
static uint16_t gyro_bias_sample_count;
static float yaw_zero_ref_deg;
static bool yaw_zero_ref_valid;
static float yaw_kalman_estimate_deg;
static float yaw_kalman_p;
static bool yaw_kalman_valid;
static uint8_t config_index;
static bool initialized;
static imu_yaw_pid_t yaw_move_pid = {
    .kp = IMU_YAW_MOVE_KP,
    .ki = IMU_YAW_MOVE_KI,
    .kd = IMU_YAW_MOVE_KD,
    .i_max = IMU_YAW_MOVE_I_MAX,
    .out_max = IMU_YAW_MOVE_OUT_MAX,
    .first_run = true
};
static imu_yaw_pid_t yaw_stop_pid = {
    .kp = IMU_YAW_STOP_KP,
    .ki = IMU_YAW_STOP_KI,
    .kd = IMU_YAW_STOP_KD,
    .i_max = IMU_YAW_STOP_I_MAX,
    .out_max = IMU_YAW_STOP_OUT_MAX,
    .first_run = true
};
static imu_gyro_filter_t gyro_filter;
static uint32_t last_yaw_control_ms;
static uint32_t last_yaw_tx_ms;
static bool yaw_target_valid;

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

static float normalize_angle(float angle_deg)
{
    while (angle_deg >= IMU_ANGLE_HALF_RANGE_DEG)
    {
        angle_deg -= IMU_ANGLE_RANGE_DEG;
    }
    while (angle_deg < -IMU_ANGLE_HALF_RANGE_DEG)
    {
        angle_deg += IMU_ANGLE_RANGE_DEG;
    }

    return angle_deg;
}

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static void reset_yaw_pid(imu_yaw_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->first_run = true;
}

static void reset_yaw_control(void)
{
    reset_yaw_pid(&yaw_move_pid);
    reset_yaw_pid(&yaw_stop_pid);
    memset(&gyro_filter, 0, sizeof(gyro_filter));
    yaw_target_valid = false;
    last_yaw_control_ms = 0U;
    imu_data.target_yaw_deg = 0.0f;
    imu_data.yaw_error_deg = 0.0f;
    imu_data.omega_output = 0;
    imu_data.yaw_hold_active = false;
}

static float filter_gyro(float gyro_deg_s)
{
    float gain;

    if (!gyro_filter.valid)
    {
        gyro_filter.estimate = gyro_deg_s;
        gyro_filter.covariance = 1.0f;
        gyro_filter.valid = true;
    }
    else
    {
        gyro_filter.covariance += IMU_GYRO_FILTER_Q;
        gain = gyro_filter.covariance /
               (gyro_filter.covariance + IMU_GYRO_FILTER_R);
        gyro_filter.estimate += gain *
                                (gyro_deg_s - gyro_filter.estimate);
        gyro_filter.covariance *= 1.0f - gain;
    }

    return roundf(gyro_filter.estimate * 10.0f) / 10.0f;
}

static float calculate_yaw_pid(imu_yaw_pid_t *pid, float error_deg)
{
    float error_delta;
    float output;

    if (fabsf(error_deg) <= IMU_YAW_I_ACTIVE_DEG)
    {
        pid->integral += pid->ki * error_deg;
        pid->integral = limit_float(pid->integral,
                                    -pid->i_max, pid->i_max);
    }
    else
    {
        pid->integral *= IMU_YAW_I_DECAY;
    }

    if (pid->first_run)
    {
        pid->last_error = error_deg;
        pid->first_run = false;
    }

    error_delta = normalize_angle(error_deg - pid->last_error);
    output = pid->kp * error_deg + pid->integral +
             pid->kd * error_delta;
    pid->last_error = error_deg;
    return limit_float(output, -pid->out_max, pid->out_max);
}

static float filter_yaw(float measured_yaw_deg)
{
    float innovation_deg;
    float kalman_gain;

    // 标量卡尔曼参数，并对跨越正负 180 度的误差归一化
    if (!yaw_kalman_valid)
    {
        yaw_kalman_estimate_deg = normalize_angle(measured_yaw_deg);
        yaw_kalman_p = 1.0f;
        yaw_kalman_valid = true;
        return yaw_kalman_estimate_deg;
    }

    yaw_kalman_p += IMU_YAW_KALMAN_Q;
    innovation_deg = normalize_angle(measured_yaw_deg -
                                     yaw_kalman_estimate_deg);
    kalman_gain = yaw_kalman_p / (yaw_kalman_p + IMU_YAW_KALMAN_R);
    yaw_kalman_estimate_deg = normalize_angle(
        yaw_kalman_estimate_deg + kalman_gain * innovation_deg);
    yaw_kalman_p = (1.0f - kalman_gain) * yaw_kalman_p;
    return yaw_kalman_estimate_deg;
}

static void reset_yaw(void)
{
    yaw_zero_ref_deg = 0.0f;
    yaw_zero_ref_valid = false;
    yaw_kalman_estimate_deg = 0.0f;
    yaw_kalman_p = 1.0f;
    yaw_kalman_valid = false;
    imu_data.yaw_deg = 0.0f;
    imu_data.yaw_valid = false;
    reset_yaw_control();
}

static void set_error_state(void)
{
    init_step = IMU_INIT_ERROR;
    imu_data.state = IMU_STATE_ERROR;
    imu_data.gyro_valid = false;
    imu_data.yaw_valid = false;
    reset_yaw_control();
}

static void start_bias_sampling(const imu_raw_data_t *raw_data)
{
    gyro_bias_sum_deg_s = 0.0f;
    gyro_bias_sample_count = 0U;
    last_gyro_sequence = raw_data->gyro_sequence;
    last_yaw_sequence = raw_data->yaw_sequence;
    imu_data.gyro_bias_deg_s = 0.0f;
    imu_data.gyro_z_deg_s = 0.0f;
    imu_data.gyro_valid = false;
    reset_yaw();
    init_step = IMU_INIT_SAMPLE_BIAS;
    imu_data.state = IMU_STATE_BIAS_SAMPLING;
}

static void run_init_state(uint32_t now_ms, const imu_raw_data_t *raw_data)
{
    const imu_command_t *command;

    if (!time_reached(now_ms, next_action_ms))
    {
        return;
    }

    // 固定顺序：硬件校准 -> 配置主动输出 -> 软件零偏采样
    switch (init_step)
    {
    case IMU_INIT_WAIT_BOOT:
        if (Imu_Send(cmd_enter_setup, sizeof(cmd_enter_setup)) != HAL_OK)
        {
            set_error_state();
            break;
        }
        init_step = IMU_INIT_SEND_CAL;
        next_action_ms = now_ms + IMU_CAL_CMD_DELAY_MS;
        break;

    case IMU_INIT_SEND_CAL:
        if (Imu_Send(cmd_gyro_cal, sizeof(cmd_gyro_cal)) != HAL_OK)
        {
            set_error_state();
            break;
        }
        init_step = IMU_INIT_WAIT_CAL;
        next_action_ms = now_ms + IMU_GYRO_CAL_WAIT_MS;
        break;

    case IMU_INIT_WAIT_CAL:
        init_step = IMU_INIT_WAIT_CONFIG;
        imu_data.state = IMU_STATE_CONFIGURING;
        next_action_ms = now_ms + IMU_CONFIG_START_DELAY_MS;
        break;

    case IMU_INIT_WAIT_CONFIG:
        config_index = 0U;
        init_step = IMU_INIT_SEND_CONFIG;
        next_action_ms = now_ms;
        break;

    case IMU_INIT_SEND_CONFIG:
        command = &config_commands[config_index];
        if (Imu_Send(command->data, command->length) != HAL_OK)
        {
            set_error_state();
            break;
        }
        config_index++;
        if (config_index >=
            (uint8_t)(sizeof(config_commands) / sizeof(config_commands[0])))
        {
            start_bias_sampling(raw_data);
        }
        else
        {
            next_action_ms = now_ms + IMU_CONFIG_CMD_DELAY_MS;
        }
        break;

    case IMU_INIT_SAMPLE_BIAS:
    case IMU_INIT_COMPLETE:
    case IMU_INIT_ERROR:
    default:
        break;
    }
}

static void process_gyro(const imu_raw_data_t *raw_data)
{
    if (!raw_data->gyro_valid ||
        (raw_data->gyro_sequence == last_gyro_sequence))
    {
        return;
    }
    last_gyro_sequence = raw_data->gyro_sequence;

    if (init_step == IMU_INIT_SAMPLE_BIAS)
    {
        // 延续旧工程做法，静止采集 200 帧 Z 轴角速度作为软件零偏
        gyro_bias_sum_deg_s += raw_data->gyro_z_deg_s;
        gyro_bias_sample_count++;
        if (gyro_bias_sample_count >= IMU_GYRO_BIAS_SAMPLE_COUNT)
        {
            imu_data.gyro_bias_deg_s = gyro_bias_sum_deg_s /
                                       (float)gyro_bias_sample_count;
            imu_data.gyro_z_deg_s = 0.0f;
            imu_data.gyro_valid = true;
            last_yaw_sequence = raw_data->yaw_sequence;
            reset_yaw();
            init_step = IMU_INIT_COMPLETE;
            imu_data.state = IMU_STATE_READY;
        }
        return;
    }

    if (init_step == IMU_INIT_COMPLETE)
    {
        imu_data.gyro_z_deg_s = -(raw_data->gyro_z_deg_s -
                                  imu_data.gyro_bias_deg_s);
        imu_data.gyro_valid = true;
    }
}

static void process_yaw(const imu_raw_data_t *raw_data)
{
    float signed_yaw_deg;
    float zeroed_yaw_deg;

    if ((init_step != IMU_INIT_COMPLETE) || !raw_data->yaw_valid ||
        (raw_data->yaw_sequence == last_yaw_sequence))
    {
        return;
    }
    last_yaw_sequence = raw_data->yaw_sequence;
    signed_yaw_deg = normalize_angle(-raw_data->yaw_deg);

    if (!yaw_zero_ref_valid)
    {
        yaw_zero_ref_deg = signed_yaw_deg;
        yaw_zero_ref_valid = true;
        imu_data.yaw_deg = 0.0f;
        imu_data.yaw_valid = true;
        return;
    }

    zeroed_yaw_deg = normalize_angle(signed_yaw_deg - yaw_zero_ref_deg);
    imu_data.yaw_deg = filter_yaw(zeroed_yaw_deg);
    imu_data.yaw_valid = true;
}

static void update_stats(uint32_t now_ms)
{
    imu_stats_t stats;

    if (!Imu_GetStats(&stats))
    {
        return;
    }

    imu_data.last_rx_ms = stats.last_valid_ms;
    imu_data.valid_frame_count = stats.valid_frame_count;
    imu_data.invalid_frame_count = stats.invalid_frame_count;
    imu_data.rx_overflow_count = stats.rx_overflow_count;
    imu_data.uart_error_count = stats.uart_error_count;
    imu_data.online = (stats.last_valid_ms != 0U) &&
                      ((now_ms - stats.last_valid_ms) <=
                       IMU_ONLINE_TIMEOUT_MS);
}

/**
 * @brief 初始化 USART1 上的 DM-IMU L1 上层逻辑
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_Init(void)
{
    if (initialized)
    {
        return HAL_OK;
    }

    memset(&imu_data, 0, sizeof(imu_data));
    if (Imu_Init(&huart1) != HAL_OK)
    {
        imu_data.state = IMU_STATE_ERROR;
        return HAL_ERROR;
    }

    init_step = IMU_INIT_WAIT_BOOT;
    imu_data.state = IMU_STATE_CALIBRATING;
    imu_data.yaw_hold_enabled = true;
    next_action_ms = HAL_GetTick() + IMU_BOOT_DELAY_MS;
    initialized = true;
    return HAL_OK;
}

/**
 * @brief 每 1 ms 执行数据解析、初始化状态机和零偏修正
 * @retval None
 */
void ImuMain_Run1ms(void)
{
    imu_raw_data_t raw_data;
    uint32_t now_ms;

    if (!initialized)
    {
        return;
    }

    Imu_Process();
    if (!Imu_GetRawData(&raw_data))
    {
        set_error_state();
        return;
    }

    now_ms = HAL_GetTick();
    run_init_state(now_ms, &raw_data);
    process_gyro(&raw_data);
    process_yaw(&raw_data);
    update_stats(now_ms);
}

/**
 * @brief 将下一帧欧拉角设为新的偏航角零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_ZeroYaw(void)
{
    if (!initialized || (init_step != IMU_INIT_COMPLETE))
    {
        return HAL_ERROR;
    }

    reset_yaw();
    return HAL_OK;
}

/**
 * @brief 根据当前航向计算底盘旋转输出
 * @param vx X 方向速度指令
 * @param vy Y 方向速度指令
 * @param omega 手动旋转指令
 * @retval 修正后的底盘旋转指令
 */
int16_t ImuMain_CalcOmega(int16_t vx, int16_t vy, int16_t omega)
{
    imu_yaw_pid_t *active_pid;
    float filtered_gyro_deg_s;
    float output;
    uint32_t now_ms;
    bool stopped;

    if (!initialized || !imu_data.yaw_hold_enabled ||
        (imu_data.state != IMU_STATE_READY) || !imu_data.online ||
        !imu_data.yaw_valid || !imu_data.gyro_valid)
    {
        reset_yaw_control();
        imu_data.omega_output = omega;
        return omega;
    }

    now_ms = HAL_GetTick();
    if ((last_yaw_control_ms != 0U) &&
        ((now_ms - last_yaw_control_ms) < IMU_YAW_PERIOD_MS))
    {
        return imu_data.omega_output;
    }
    last_yaw_control_ms = now_ms;
    filtered_gyro_deg_s = filter_gyro(imu_data.gyro_z_deg_s);

    // 第一次进入闭环时保持当前位置，避免使能瞬间突然旋转
    if (!yaw_target_valid)
    {
        imu_data.target_yaw_deg = imu_data.yaw_deg;
        yaw_target_valid = true;
    }

    // 手动旋转优先，旋转过程中持续记录当前航向，松手后原地保持
    if ((omega > IMU_YAW_CMD_THRESHOLD) ||
        (omega < -IMU_YAW_CMD_THRESHOLD))
    {
        imu_data.target_yaw_deg = imu_data.yaw_deg;
        imu_data.yaw_error_deg = 0.0f;
        imu_data.omega_output = omega;
        imu_data.yaw_hold_active = false;
        reset_yaw_pid(&yaw_move_pid);
        reset_yaw_pid(&yaw_stop_pid);
        return omega;
    }

    stopped = (abs((int)vx) <= IMU_YAW_LINEAR_THRESHOLD) &&
              (abs((int)vy) <= IMU_YAW_LINEAR_THRESHOLD);
    active_pid = stopped ? &yaw_stop_pid : &yaw_move_pid;
    if (stopped)
    {
        reset_yaw_pid(&yaw_move_pid);
    }
    else
    {
        reset_yaw_pid(&yaw_stop_pid);
    }

    imu_data.yaw_error_deg = normalize_angle(imu_data.target_yaw_deg -
                                             imu_data.yaw_deg);
    imu_data.yaw_hold_active = true;
    if (fabsf(imu_data.yaw_error_deg) <= IMU_YAW_DEADZONE_DEG)
    {
        reset_yaw_pid(active_pid);
        imu_data.omega_output = 0;
        return 0;
    }

    output = calculate_yaw_pid(active_pid, imu_data.yaw_error_deg) -
             filtered_gyro_deg_s * IMU_YAW_GYRO_K;
    output = limit_float(output, -active_pid->out_max,
                         active_pid->out_max);
    imu_data.omega_output = (int16_t)output;
    return imu_data.omega_output;
}

/**
 * @brief 设置航向保持目标角
 * @param target_yaw_deg 目标偏航角，单位为 deg
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg)
{
    if (!initialized || isnan(target_yaw_deg) || isinf(target_yaw_deg))
    {
        return HAL_ERROR;
    }

    imu_data.target_yaw_deg = normalize_angle(target_yaw_deg);
    imu_data.yaw_error_deg = 0.0f;
    yaw_target_valid = true;
    reset_yaw_pid(&yaw_move_pid);
    reset_yaw_pid(&yaw_stop_pid);
    return HAL_OK;
}

/**
 * @brief 设置航向保持使能状态
 * @param enabled 是否使能
 * @retval None
 */
void ImuMain_EnableYawHold(bool enabled)
{
    if (!initialized)
    {
        return;
    }

    imu_data.yaw_hold_enabled = enabled;
    reset_yaw_control();
}

bool ImuMain_GetData(imu_data_t *data)
{
    if (!initialized || (data == NULL))
    {
        return false;
    }

    *data = imu_data;
    return true;
}

/**
 * @brief 通过上位机串口回传当前 yaw 角
 * @param uart 上位机串口句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SendYaw(UART_HandleTypeDef *uart)
{
    uint8_t frame[IMU_YAW_FRAME_LENGTH];
    int16_t yaw_cdeg;
    float scaled_yaw;
    uint32_t now_ms;

    if ((uart == NULL) || !initialized || !imu_data.online ||
        !imu_data.yaw_valid)
    {
        return HAL_ERROR;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - last_yaw_tx_ms) < IMU_YAW_TX_PERIOD_MS)
    {
        return HAL_BUSY;
    }
    last_yaw_tx_ms = now_ms;

    scaled_yaw = imu_data.yaw_deg * IMU_YAW_TX_SCALE;
    yaw_cdeg = (int16_t)(scaled_yaw +
                        ((scaled_yaw >= 0.0f) ? 0.5f : -0.5f));
    frame[0] = IMU_YAW_FRAME_HEADER_0;
    frame[1] = IMU_YAW_FRAME_HEADER_1;
    frame[2] = (uint8_t)((uint16_t)yaw_cdeg & 0xFFU);
    frame[3] = (uint8_t)(((uint16_t)yaw_cdeg >> 8) & 0xFFU);
    frame[4] = frame[2] ^ frame[3];

    return HAL_UART_Transmit(uart, frame, sizeof(frame), 1U);
}

void ImuMain_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    Imu_HandleRxEvent(uart, size);
}

void ImuMain_HandleUartError(UART_HandleTypeDef *uart)
{
    Imu_HandleUartError(uart);
}
