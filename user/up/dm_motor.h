#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define DM_J4310_P_MAX 12.5f    // 位置上限 ±12.5 弧度（约 ±716°）
#define DM_J4310_V_MAX 30.0f    // 速度上限 ±30 弧度/秒
#define DM_J4310_T_MAX 10.0f    // 力矩上限 ±10 牛米
#define DM_MIT_KP_MAX  500.0f   // MIT协议KP增益上限 500
#define DM_MIT_KD_MAX  5.0f     // MIT协议KD增益上限 5

typedef enum
{
    DM_OK = 0,
    DM_BAD_ARG,
    DM_BUSY,
    DM_IO_ERROR
} dm_result_t;

typedef enum
{
    DM_FAULT_NONE = 0x0,
    DM_FAULT_OVER_VOLTAGE = 0x8,
    DM_FAULT_UNDER_VOLTAGE = 0x9,
    DM_FAULT_OVER_CURRENT = 0xA,
    DM_FAULT_MOS_OVER_TEMP = 0xB,
    DM_FAULT_COIL_OVER_TEMP = 0xC,
    DM_FAULT_COMM_LOST = 0xD,
    DM_FAULT_OVERLOAD = 0xE
} dm_fault_t;

// 结构体：MIT控制命令
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float kp;
    float kd;
} dm_mit_cmd_t;

// 结构体：电机物理限制
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
} dm_limits_t;

// 结构体：电机反馈状态
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temp_c;
    uint8_t rotor_temp_c;
    dm_fault_t fault;
    uint32_t rx_tick_ms;
    uint32_t rx_count;
} dm_state_t;

// 结构体：CAN数据帧
typedef struct
{
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} dm_frame_t;

// 结构体：电机实例（包含配置、状态、同步机制）
typedef struct
{
    uint16_t tx_id;
    uint16_t master_id;
    uint8_t feedback_id;
    dm_limits_t limits;
    volatile uint32_t state_seq;
    volatile dm_state_t state;
} dm_motor_t;

// 初始化电机实例
dm_result_t DM_MotorInit(dm_motor_t *motor, uint16_t tx_id, uint16_t master_id,
                         uint8_t feedback_id);
// 设置电机物理限制
dm_result_t DM_MotorSetLimits(dm_motor_t *motor, float p_max, float v_max,
                              float t_max);
// 构建使能帧                             
dm_result_t DM_MotorBuildEnable(const dm_motor_t *motor, dm_frame_t *frame);
// 构建失能帧
dm_result_t DM_MotorBuildDisable(const dm_motor_t *motor, dm_frame_t *frame);
// 构建归零帧（设置当前位置为机械零点）
dm_result_t DM_MotorBuildZero(const dm_motor_t *motor, dm_frame_t *frame);
// 构建MIT控制帧（核心控制命令）
dm_result_t DM_MotorBuildMit(const dm_motor_t *motor, const dm_mit_cmd_t *cmd,
                             dm_frame_t *frame);
// 解析电机反馈帧（从中断接收的数据中提取状态）
bool DM_MotorParseFeedback(dm_motor_t *motor, uint16_t id, const uint8_t *data,
                           uint8_t length, uint32_t tick_ms);
// 安全获取电机状态                          
bool DM_MotorGetState(const dm_motor_t *motor, dm_state_t *state);


#endif
