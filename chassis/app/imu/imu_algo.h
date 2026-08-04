/**
 * @file    imu_algo.h
 * @brief   IMU 传感器核心姿态与二次积分位置解算算法头文件
 * @note    独立算法模块，涵盖处理链、在线零偏、姿态变换、二阶Butterworth、ZUPT/ZARU与梯形双重积分
 */

#ifndef IMU_ALGO_H
#define IMU_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>

#define IMU_ALGO_HISTORY_SIZE   100U

/**
 * @brief  二阶 Butterworth 低通滤波器状态结构体
 */
typedef struct
{
    float x1, x2;               // 滤波历史输入项
    float y1, y2;               // 滤波历史输出项
} biquad_lpf_t;

/**
 * @brief  IMU 核心算法状态与历史缓存结构体
 */
typedef struct
{
    /* 二状态 Yaw-Bias 卡尔曼滤波向量 [yaw, bias]^T 与协方差矩阵 P[2x2] */
    float x_yaw;                // 状态0：偏航角 (deg)
    float x_bias;               // 状态1：陀螺仪零漂 (deg/s)
    float p00, p01;             // 协方差 P00, P01
    float p10, p11;             // 协方差 P10, P11
    uint8_t kalman_inited;      // 卡尔曼是否已初始化

    /* 自适应角速度滤波与梯形积分 */
    float gyro_filtered;        // 经自适应一阶低通滤波后的角速度 (deg/s)
    float gyro_last_int;        // 上一帧参与积分的角速度 (deg/s)
    uint8_t has_last_int;       // 是否有上一帧积分记录
    float gyro_last_raw;        // 上一帧原始角速度值 (deg/s)
    uint8_t has_last_raw;       // 是否有上一帧原始值记录

    /* 动态零偏估计与静态在线检测 (100帧滑动窗口) */
    float gyro_history[IMU_ALGO_HISTORY_SIZE];
    uint16_t history_idx;       // 循环写入索引
    uint16_t history_count;     // 缓存内有效帧数
    uint8_t is_stationary;      // 是否静止标定标记 (1:静止, 0:运动)

    /* =================== 加速度双重积分、二阶滤波、ZUPT 与卡尔曼 =================== */
    /* 二阶 Butterworth 低通滤波状态 (X 与 Y 轴) */
    biquad_lpf_t lpf_acc_x;
    biquad_lpf_t lpf_acc_y;
    float acc_x_filt;           // 机体坐标系滤波后 X 轴加速度 (m/s²)
    float acc_y_filt;           // 机体坐标系滤波后 Y 轴加速度 (m/s²)

    /* 在线零偏跟踪估计 (X 与 Y 轴) */
    float acc_bias_x;           // X 轴加速度零偏 (m/s²)
    float acc_bias_y;           // Y 轴加速度零偏 (m/s²)

    /* 姿态旋转与世界坐标系线性加速度 */
    float acc_world_x;          // 世界坐标系 X 轴线性加速度 (m/s²)
    float acc_world_y;          // 世界坐标系 Y 轴线性加速度 (m/s²)
    float acc_world_last_x;     // 上一帧参与积分的世界坐标加速度 X
    float acc_world_last_y;     // 上一帧参与积分的世界坐标加速度 Y
    uint8_t has_last_acc_world;
    float acc_last_raw_x;       // 上一帧采样 X 加速度
    float acc_last_raw_y;       // 上一帧采样 Y 加速度
    uint8_t has_last_acc_raw;

    /* 速度梯形积分与 ZUPT 控制 */
    float vel_world_x;          // 世界坐标系 X 轴速度 (m/s)
    float vel_world_y;          // 世界坐标系 Y 轴速度 (m/s)
    float vel_world_last_x;     // 上一帧速度 X
    float vel_world_last_y;     // 上一帧速度 Y
    uint8_t has_last_vel_world;
    uint8_t zupt_active;        // ZUPT 静止驻停触发标记 (1:触发ZUPT, 0:运动中)

    /* 位置梯形双重积分 (X 与 Y 轴) */
    float pos_world_x;          // 世界坐标系 X 轴累计位置 (m)
    float pos_world_y;          // 世界坐标系 Y 轴累计位置 (m)

    /* 三状态 [pos, vel, acc_bias]^T 卡尔曼滤波协方差矩阵 P (X与Y两独立正交轴) */
    float p_acc_x[3][3];        // X 轴三状态卡尔曼协方差矩阵
    float p_acc_y[3][3];        // Y 轴三状态卡尔曼协方差矩阵
    uint8_t pos_kalman_inited;  // 位置卡尔曼是否初始化

    /* 滑动窗口标准差与振动判断 (100帧窗) */
    float acc_history_x[IMU_ALGO_HISTORY_SIZE];
    float acc_history_y[IMU_ALGO_HISTORY_SIZE];
    uint16_t acc_history_idx;
    uint16_t acc_history_count;

    /* 微秒级高精度 dt (基于 DWT 周期计数器) */
    uint32_t last_dwt_cycles;   // 上一次调用的 DWT 周期值
    uint8_t dwt_inited;         // DWT 是否初始化
} imu_algo_t;

/**
 * @brief  初始化算法模块各个参数与结构体
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_Init(imu_algo_t *algo);

/**
 * @brief  异常角速度检验保护 (超过上限或单帧阶跃过大视为错帧)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前读到的原始角速度 (deg/s)
 * @retval 1:有效正常数据, 0:异常错帧需丢弃
 */
uint8_t ImuAlgo_CheckGyroValid(imu_algo_t *algo, float gyro_raw);

/**
 * @brief  角速度噪声自适应一阶滤波 (静止强滤波 alpha=0.02, 运动弱滤波 alpha=0.3)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前有效角速度 (deg/s)
 * @retval 滤波计算后得到的平滑角速度 (deg/s)
 */
float ImuAlgo_AdaptiveFilterGyro(imu_algo_t *algo, float gyro_raw);

/**
 * @brief  滑动窗口在线零偏检测与跟踪 (检查100帧内均值<0.2且标准差<0.05则更新bias)
 * @param  algo          算法实例指针
 * @param  gyro_filtered 当前滤波后的角速度 (deg/s)
 * @retval None
 */
void ImuAlgo_UpdateStationary(imu_algo_t *algo, float gyro_filtered);

/**
 * @brief  获取基于 DWT 微秒级时间计数的帧时间差 dt
 * @param  algo 算法实例指针
 * @retval dt (秒)，有效范围 [0.00001, 0.02]，超时默认 0.005
 */
float ImuAlgo_GetDtSeconds(imu_algo_t *algo);

/**
 * @brief  二状态卡尔曼滤波预测步 + 梯形积分预测偏航角
 * @param  algo          算法实例指针
 * @param  gyro_filtered 当前平滑角速度 (deg/s)
 * @param  dt            当前帧微秒级差 (秒)
 * @retval 当前卡尔曼预测的偏航角 (deg)
 */
float ImuAlgo_PredictAndIntegrate(imu_algo_t *algo, float gyro_filtered, float dt);

/**
 * @brief  二状态卡尔曼滤波测量校正步 (融合 IMU 硬件回传的绝对偏航角度)
 * @param  algo         算法实例指针
 * @param  yaw_measured 经零 reference 修正后的采样偏航角 (deg)
 * @retval 滤波校正后最终输出的偏航角 (deg)
 */
float ImuAlgo_UpdateYawMeasurement(imu_algo_t *algo, float yaw_measured);

/**
 * @brief  获取最终最优偏航角输出 (Yaw)
 * @param  algo 算法实例指针
 * @retval 最优偏航角 (-180.0 ~ 180.0 deg)
 */
float ImuAlgo_GetYaw(const imu_algo_t *algo);

/**
 * @brief  获取当前系统在线跟踪评估的最优角速度零偏 (Bias)
 * @param  algo 算法实例指针
 * @retval 最优角速度零偏 (deg/s)
 */
float ImuAlgo_GetBias(const imu_algo_t *algo);

/**
 * @brief  获取当前是否判定为车体静止
 * @param  algo 算法实例指针
 * @retval 1:静止, 0:运动中
 */
uint8_t ImuAlgo_IsStationary(const imu_algo_t *algo);

/* =================== 加速度处理链、二阶滤波、ZUPT与梯形二次积分 =================== */

/**
 * @brief  第 4 层：异常线性加速度检测保护 (判断采样是否超过最大额定量程)
 * @param  algo      算法实例指针
 * @param  acc_raw_x 采样 X 加速度 (m/s²)
 * @param  acc_raw_y 采样 Y 加速度 (m/s²)
 * @retval 1:有效数据, 0:错报尖峰包需抛弃
 */
uint8_t ImuAlgo_CheckAccValid(imu_algo_t *algo, float acc_raw_x, float acc_raw_y);

/**
 * @brief  第 4 层：二阶 Butterworth 低通滤波 (20~30Hz 截止频率，强力滤除电机机械振动)
 * @param  algo       算法实例指针
 * @param  acc_raw_x  原始有效 X 轴加速度
 * @param  acc_raw_y  原始有效 Y 轴加速度
 * @param  acc_filt_x 输出二阶滤波后的 X 轴加速度
 * @param  acc_filt_y 输出二阶滤波后的 Y 轴加速度
 * @retval None
 */
void ImuAlgo_BiquadFilterAcc(imu_algo_t *algo, float acc_raw_x, float acc_raw_y, float *acc_filt_x, float *acc_filt_y);

/**
 * @brief  第 7 层 & 第 5/6/2 层：滑动窗口振动判断、ZUPT 零速驻停与加速度零漂在线收敛
 * @param  algo       算法实例指针
 * @param  acc_filt_x 滤波后 X 轴加速度 (m/s²)
 * @param  acc_filt_y 滤波后 Y 轴加速度 (m/s²)
 * @retval None
 */
void ImuAlgo_UpdateZuptAndBias(imu_algo_t *algo, float acc_filt_x, float acc_filt_y);

/**
 * @brief  第 3 层：姿态变换与重力补偿 —— 减去在线估算零飘，根据偏航角将机体加速度向世界坐标系正交变换
 * @param  algo       算法实例指针
 * @param  acc_filt_x 机体滤波 X 加速度
 * @param  acc_filt_y 机体滤波 Y 加速度
 * @param  yaw_deg    当前修正后偏航角 (deg)
 * @retval None
 */
void ImuAlgo_RotateAndCompensateAcc(imu_algo_t *algo, float acc_filt_x, float acc_filt_y, float yaw_deg);

/**
 * @brief  第 11 层：地面差速/直行机器人约束运动模型 (侧滑速度阻尼限幅)
 * @param  algo    算法实例指针
 * @param  yaw_deg 当前偏航角 (deg)
 * @retval None
 */
void ImuAlgo_ApplyNonHolonomicConstraint(imu_algo_t *algo, float yaw_deg);

/**
 * @brief  第 8 层 & 第 9 层：梯形二次双重积分 (a -> v -> pos) + 三状态 [pos, vel, a_bias] 协方差预测
 * @param  algo 算法实例指针
 * @param  dt   当前时间差 (秒)
 * @retval None
 */
void ImuAlgo_PredictDoubleIntegral(imu_algo_t *algo, float dt);

/**
 * @brief  第 5/9 层：ZUPT 零速度触发时的 3 状态卡尔曼测量校正 (以 z=0 更新速度、坐标趋势与零偏)
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_UpdateZuptKalman(imu_algo_t *algo);

/**
 * @brief  获取姿态变换去偏后的世界坐标系 X 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldX(const imu_algo_t *algo);

/**
 * @brief  获取姿态变换去偏后的世界坐标系 Y 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldY(const imu_algo_t *algo);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 X 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelX(const imu_algo_t *algo);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 Y 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelY(const imu_algo_t *algo);

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 X 轴位置坐标 (Position X)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosX(const imu_algo_t *algo);

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 Y 轴位置坐标 (Position Y)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosY(const imu_algo_t *algo);

/**
 * @brief  获取实时在线跟踪收敛出的 X 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasX(const imu_algo_t *algo);

/**
 * @brief  获取实时在线跟踪收敛出的 Y 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasY(const imu_algo_t *algo);

/**
 * @brief  获取当前是否处于 ZUPT 零速度驻停模式
 * @retval 1:静止触发ZUPT, 0:运动
 */
uint8_t ImuAlgo_IsZuptActive(const imu_algo_t *algo);

/**
 * @brief  一键复位清零位置坐标和线速度
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_ResetPosition(imu_algo_t *algo);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ALGO_H */
