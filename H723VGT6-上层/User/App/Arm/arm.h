/**
 * @file arm.h
 * @brief 定义机械臂目标、J4310 控制状态和统一控制接口。
 */

#ifndef ARM_H
#define ARM_H /**< 防止 arm.h 被重复包含。 */

#include <stdbool.h>
#include <stdint.h>

#include "upper_motor_port.h"
#include "upper_pid.h"

#define UPPER_ARM_M3508_COUNT                  2U /**< 机械臂机构中参与控制的 M3508 电机数量。 */

#define UPPER_J4310_POSITION_MAX_RAD           12.5f /**< 机械臂 J4310 关节命令允许使用的位置映射满量程，单位：弧度。 */
#define UPPER_J4310_VELOCITY_MAX_RAD_S         30.0f /**< 机械臂 J4310 关节 MIT 协议速度字段映射的满量程，单位：弧度每秒。 */
#define UPPER_J4310_TORQUE_MAP_MAX_NM          10.0f /**< 机械臂 J4310 关节 MIT 协议转矩字段映射的满量程，单位：牛米。 */
#define UPPER_J4310_KP_MAX                     49.0f /**< 机械臂 J4310 关节控制命令允许使用的最大比例增益。 */
#define UPPER_J4310_KD_MAX                      0.95f /**< 机械臂 J4310 关节控制命令允许使用的最大微分增益。 */

#define UPPER_M3508_POSITION_MIN_RAD            0.0f /**< 机械臂 M3508 输出轴允许接收的最小位置目标，单位：弧度。 */
#define UPPER_M3508_POSITION_MAX_RAD            20.9439510239f /**< 机械臂 M3508 输出轴允许接收的最大位置目标，单位：弧度。 */
#define UPPER_M3508_POSITION_VEL_LIMIT_RAD_S    15.708f /**< 机械臂 M3508 输出轴执行位置轨迹时允许的最大速度，单位：弧度每秒。 */

/** 描述机械臂在当前控制周期内需要达到的目标。 */
typedef struct
{
    bool enabled; /**< 本周期是否使能机械臂机构。 */
    bool j4310_commanded; /**< 本周期是否需要向机械臂 J4310 下发控制命令。 */
    bool m3508_enabled; /**< 机械臂两台 M3508 是否允许输出。 */
    bool position_mode; /**< J4310 与 M3508 是否采用位置控制模式。 */
    float grip_pos_rad; /**< J4310 关节目标位置，单位：弧度。 */
    float grip_vel_rad_s; /**< J4310 关节目标速度，单位：弧度每秒。 */
    float grip_kp; /**< 夹持关节 MIT 位置项的比例增益。 */
    float grip_kd; /**< 夹持关节 MIT 速度项的微分增益。 */
    float grip_torque_nm; /**< 夹持关节 MIT 命令要求的前馈转矩，单位：牛米。 */
    float grip_torque_limit_nm; /**< J4310 关节允许的转矩上限，单位：牛米。 */
    float m3508_vel_rad_s[UPPER_ARM_M3508_COUNT]; /**< 两台机械臂 M3508 的目标速度，单位：弧度每秒。 */
    float m3508_pos_rad[UPPER_ARM_M3508_COUNT]; /**< 两台机械臂 M3508 的目标位置，单位：弧度。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    upper_pid_cfg_t m3508_speed_pid; /**< 机械臂 M3508 速度环 PID 参数。 */
    upper_pid_cfg_t m3508_position_pid; /**< 机械臂 M3508 位置环 PID 参数。 */
} arm_target_t;

/** 保存机械臂目标校验并转换后的电机命令。 */
typedef struct
{
    bool enabled; /**< 校验后的机械臂命令是否允许输出。 */
    bool m3508_enabled; /**< 机械臂两台 M3508 是否允许输出。 */
    motor_cmd_t j4310; /**< 已经转换完成的机械臂 J4310 电机命令。 */
    motor_cmd_t m3508[UPPER_ARM_M3508_COUNT]; /**< 已经转换完成的机械臂 M3508 电机命令。 */
    bool pid_update; /**< 本次命令是否同时更新 PID 参数。 */
    float j4310_torque_limit_nm; /**< 校验后的 J4310 转矩上限，单位：牛米。 */
    upper_pid_cfg_t m3508_speed_pid; /**< 校验后的机械臂 M3508 速度环 PID 参数。 */
    upper_pid_cfg_t m3508_position_pid; /**< 校验后的机械臂 M3508 位置环 PID 参数。 */
} arm_output_t;

/* 功能：校验机械臂目标并转换为各电机命令；用途：生成单周期可统一下发的输出快照；返回 true 表示目标合法且转换完成。 */
bool Arm_Calc(const arm_target_t *target /**< 本周期机械臂关节控制目标 */, arm_output_t *output /**< 用于写出机械臂电机命令的对象 */);
/* 功能：应用机械臂输出及可选 PID 更新；用途：把 J4310 和 M3508 命令提交给电机管理器；返回 true 表示全部设置成功。 */
bool Arm_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */, const arm_output_t *output /**< 待下发的机械臂电机命令 */);


/* J4310 自动回位控制。 */
/** 标识 J4310 自动回位流程当前执行到的阶段。 */
typedef enum
{
    J4310_AUTO_RETURN_DISABLED = 0, /**< 自动回位功能已关闭，不接管 J4310。 */
    J4310_AUTO_RETURN_ARMED, /**< 已检测到掉线，等待 J4310 重新上线。 */
    J4310_AUTO_RETURN_RUNNING, /**< 当前正在执行该流程。 */
    J4310_AUTO_RETURN_HOLDING /**< 已回到目标位置并保持当前关节角。 */
} j4310_auto_return_stage_t;

/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    bool enabled; /**< J4310 自动回位功能是否启用。 */
    bool seen_online; /**< 自动回位控制器是否曾观察到 J4310 在线。 */
    bool online; /**< J4310 当前反馈是否在线。 */
    bool reconnect_armed; /**< J4310 重新上线时是否允许触发自动回位。 */
    bool owns_control; /**< 自动回位流程当前是否占用 J4310 控制权。 */
    j4310_auto_return_stage_t stage; /**< J4310 自动回位状态机当前阶段。 */
    uint32_t trajectory_start_ms; /**< 当前轨迹开始执行的系统毫秒时刻。 */
    uint32_t trajectory_duration_ms; /**< 当前轨迹计划执行的总时间，单位：毫秒。 */
    float trajectory_start_position_rad; /**< J4310的轨迹起始位置，单位：弧度。 */
    float target_position_rad; /**< J4310的目标位置，单位：弧度。 */
    float target_velocity_rad_s; /**< J4310的目标速度，单位：弧度每秒。 */
} j4310_auto_return_t;

/* 功能：初始化 J4310 自动回零控制器；用途：设置使能状态并清空历史状态；无返回值表示控制器已复位。 */
void J4310AutoReturn_Init(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                          bool enabled /**< 是否启用 J4310 自动回零流程 */);
/* 功能：重新配置自动回零使能和反馈在线状态；用途：在系统启动或配置切换时重建状态机；无返回值表示配置已生效。 */
void J4310AutoReturn_Configure(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                               bool enabled /**< 是否启用 J4310 自动回零流程 */,
                               bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */);
/* 功能：取消正在进行或等待中的自动回零；用途：在人工控制或停机时释放控制权；无返回值表示回零目标已清除。 */
void J4310AutoReturn_Cancel(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */);
/* 功能：按反馈在线状态推进自动回零状态机；用途：检测掉线重连并生成回零目标；无返回值表示本周期状态已更新。 */
void J4310AutoReturn_Update(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                            uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                            bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */,
                            float position_rad /**< 当前实测 J4310 关节位置，单位：弧度 */,
                            float velocity_rad_s /**< 当前实测 J4310 关节速度，单位：弧度每秒 */,
                            bool control_allowed /**< 当前周期是否允许 J4310 自动回零输出控制命令 */);
/* 功能：判断自动回零控制器是否持有控制权；用途：决定上层是否采用回零目标；返回 true 表示自动回零正在生效。 */
bool J4310AutoReturn_IsActive(const j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */);

/* J4310 位置轨迹与重力补偿控制。 */
/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    bool initialized; /**< 控制状态是否已经初始化。 */
    bool trajectory_active; /**< 当前是否正在执行平滑位置轨迹。 */
    bool feedback_seen; /**< 控制器是否至少收到过一帧 J4310 有效反馈。 */
    bool gravity_position_valid; /**< 重力模型使用的关节角是否已经由有效反馈初始化。 */
    bool gravity_learning_target_valid; /**< 重力辨识目标位置是否已经设置。 */
    bool gravity_learning_locked; /**< 重力模型参数是否已锁定并停止继续学习。 */
    uint32_t trajectory_start_ms; /**< 当前轨迹开始执行的系统毫秒时刻。 */
    uint32_t trajectory_duration_ms; /**< 当前轨迹计划执行的总时间，单位：毫秒。 */
    uint32_t last_feedback_ms; /**< 最近一次使用 J4310 有效反馈的系统毫秒时刻。 */
    uint32_t gravity_settle_feedback_count; /**< J4310 位置连续满足重力辨识条件的反馈帧数。 */
    uint32_t gravity_settle_required_count; /**< 开始更新重力模型前要求连续稳定的反馈帧数。 */
    float trajectory_start_position_rad; /**< J4310的轨迹起始位置，单位：弧度。 */
    float trajectory_target_position_rad; /**< J4310 当前轨迹最终需要达到的位置，单位：弧度。 */
    float target_position_rad; /**< J4310 本周期轨迹采样得到的目标位置，单位：弧度。 */
    float target_velocity_rad_s; /**< J4310 本周期轨迹采样得到的目标速度，单位：弧度每秒。 */
    float max_velocity_rad_s; /**< J4310的最大允许速度，单位：弧度每秒。 */
    float max_acceleration_rad_s2; /**< J4310 平滑位置轨迹允许的最大加速度，单位：弧度每二次方秒。 */
    float gravity_model_limit_nm; /**< 重力模型允许输出的最大补偿转矩，单位：牛米。 */
    float gravity_learning_rate; /**< 每个有效样本更新重力模型参数的步长。 */
    float gravity_compensation_gain; /**< 重力模型估计值施加到控制命令的比例。 */
    float gravity_disable_half_width_rad; /**< 零位附近关闭重力补偿区域的半宽，单位：弧度。 */
    float gravity_settle_error_rad; /**< 允许重力辨识采样的位置误差，单位：弧度。 */
    float gravity_torque_rate_limit_nm_s; /**< J4310 重力补偿转矩允许的最大变化率，单位：牛米每秒。 */
    float gravity_learning_target_rad; /**< 当前重力辨识采样点的目标关节角，单位：弧度。 */
    float gravity_actual_position_rad; /**< J4310的实际位置，单位：弧度。 */
    float gravity_reference_position_rad; /**< J4310的轨迹参考位置，单位：弧度。 */
    float gravity_cos_nm; /**< 重力转矩模型中余弦基函数的系数，单位：牛米。 */
    float gravity_sin_nm; /**< 重力转矩模型中正弦基函数的系数，单位：牛米。 */
    float gravity_filtered_torque_nm; /**< 低通滤波后的 J4310 反馈转矩，单位：牛米。 */
    float gravity_torque_nm; /**< 本周期实际施加的 J4310 重力补偿转矩，单位：牛米。 */
} j4310_position_control_t;

/* 功能：校验参数并初始化 J4310 位置控制器；用途：建立轨迹和重力补偿运行状态；返回 true 表示初始化成功。 */
bool J4310PositionControl_Init(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                               float max_velocity_rad_s /**< 轨迹允许的最大速度，单位：弧度每秒 */,
                               float max_acceleration_rad_s2 /**< 轨迹允许的最大加速度，单位：弧度每二次方秒 */,
                               float gravity_model_limit_nm /**< 允许设置的转矩上限，单位：牛米 */,
                               float gravity_learning_rate /**< 每个有效样本更新重力模型参数的步长 */,
                               float gravity_compensation_gain /**< 重力模型估计值施加到控制命令的比例 */,
                               float gravity_disable_half_width_rad /**< 零位附近关闭重力补偿区域的半宽，单位：弧度 */,
                               float gravity_settle_error_rad /**< 允许重力辨识采样的位置误差，单位：弧度 */,
                               uint32_t gravity_settle_required_count /**< 开始学习前位置连续稳定所需的反馈帧数 */,
                               float gravity_torque_rate_limit_nm_s /**< 重力补偿转矩允许的最大变化率，单位：牛米每秒 */);
/* 功能：启动从当前位置到目标位置的五次轨迹；用途：生成平滑且受限的关节运动；返回 true 表示轨迹已建立。 */
bool J4310PositionControl_Start(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                                uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                float start_position_rad /**< 轨迹起始关节角，单位：弧度 */,
                                float target_position_rad /**< 轨迹目标关节角，单位：弧度 */);
/* 功能：取消运动轨迹并保持指定位置；用途：在启动或模式切换时建立静止目标；无返回值表示保持目标已更新。 */
void J4310PositionControl_Hold(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                               float position_rad /**< 需要保持的关节目标位置，单位：弧度 */);
/* 功能：取消当前 J4310 位置轨迹；用途：停止轨迹推进并清除目标速度；无返回值表示轨迹已停用。 */
void J4310PositionControl_CancelTrajectory(
    j4310_position_control_t *control /**< J4310 位置轨迹控制器 */);
/* 功能：按当前时刻采样 J4310 五次位置轨迹；用途：输出连续的位置和速度目标；无返回值表示采样结果已写入输出参数。 */
void J4310PositionControl_Sample(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                                 uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                 float *position_rad /**< 用于写出轨迹目标位置的地址，单位：弧度 */,
                                 float *velocity_rad_s /**< 用于写出轨迹目标速度的地址，单位：弧度每秒 */);
/* 功能：学习重力模型并合成最终 J4310 扭矩；用途：叠加请求扭矩、重力补偿和安全限幅；返回值表示最终扭矩命令。 */
float J4310PositionControl_ComposeTorque(
    j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
    bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */,
    uint32_t feedback_ms /**< 当前反馈对应的系统毫秒时刻 */,
    float actual_position_rad /**< J4310 当前实测关节角，单位：弧度 */,
    float actual_velocity_rad_s /**< J4310 当前实测关节速度，单位：弧度每秒 */,
    float feedback_torque_nm /**< J4310 当前反馈转矩，单位：牛米 */,
    float desired_position_rad /**< J4310 当前期望关节角，单位：弧度 */,
    float desired_velocity_rad_s /**< J4310 当前期望关节速度，单位：弧度每秒 */,
    float requested_torque_nm /**< 位置控制器请求输出的转矩，单位：牛米 */,
    float torque_limit_nm /**< 允许设置的转矩上限，单位：牛米 */);

#endif
