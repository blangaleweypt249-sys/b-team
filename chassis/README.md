# 底盘控制模块 (Chassis Module)

比赛机器人底盘模块：负责 **电机驱动(VESC/CAN)**、**编码器+IMU 里程计融合**、**串口命令控制台**。

- **输入**：4 路 VESC 的 CAN status（转速/电流/码盘）、IMU 的 yaw/gyro（USART6/485）、上位机串口命令。
- **输出**：世界系里程计 (x, y, yaw)、车体速度 (vx, vy)、各轮转速/电流、在线状态。
- **在系统中的位置**：上层（决策/导航）通过串口命令按轮下发目标，本模块负责执行驱动 + 状态反馈；里程计供上层定位使用。

> 目前上层按 **每个轮子** 下发目标（R/C/B 串口命令）。**底盘级速度指令 (vx, vy, wz → 四轮)** 为待加入接口。

---

## 1. 文件结构

```
app/
├── chassis/
│   ├── chassis_motors.{c,h}   轮位注册表 (id ↔ LF/RF/LR/RR) + 在线检测
│   ├── odom_fusion.{c,h}      编码器+IMU 里程计融合 (正运动学 + ZUPT + 异常剔除)
│   ├── soft_start.{c,h}       电机缓启动 (线性爬坡)
│   └── bsp/
│       ├── can_rx.{c,h}       CAN 接收 (在线检测 + status 解码 → 编码器)
│       └── vesc_can.{c,h}     VESC CAN 协议 (set erpm/current/duty/brake)
├── imu/
│   ├── imu.{c,h} / imu_algo.{c,h}   IMU 驱动 + 姿态算法 (外部引入，未改)
│   └── encoder.{c,h}                编码器反馈解码 (STATUS_1 rpm/cur/duty, STATUS_5 tach)
├── console/
│   ├── serial_cmd.{c,h}       串口命令解析与分发
│   └── usart.{c,h}            USART1 控制台 + USART6 (IMU/485)
└── util/
    ├── pid.{c,h}              通用 PID
    └── print_util.{c,h}       统一打印 (put_i32 / put_float)
```

---

## 2. 对外接口（C API）

### 2.1 驱动电机（经缓启动）

```c
void soft_start_set_target(CAN_HandleTypeDef *hcan, uint8_t id,
                           soft_start_type_t type, int32_t value);
```
- 功能：设置某轮目标值，自动线性爬坡到位
- 参数：`hcan = &hcan1`；`id` = VESC 控制器 id；`type` = `SOFT_START_TYPE_RPM / _CURRENT / _DUTY / _BRAKE`；`value` = 对应数值
- 频率：由 `soft_start_tick()` 每 20 ms 重发，保持 VESC 控制不断

不经缓启动直接发 CAN：`vesc_can_set_erpm/current/duty/brake(hcan, id, value)`

### 2.2 里程计（核心输出）

| 函数 | 返回 | 单位 | 说明 |
|---|---|---|---|
| `void odom_tick(void)` | — | — | 周期调用（~100 Hz），推进融合 |
| `float odom_get_x_m(void)` | x | m | 世界系位置 X |
| `float odom_get_y_m(void)` | y | m | 世界系位置 Y |
| `float odom_get_yaw_deg(void)` | yaw | deg | 航向（来自 IMU） |
| `float odom_get_vx_mps(void)` | vx | m/s | 车体系速度 X |
| `float odom_get_vy_mps(void)` | vy | m/s | 车体系速度 Y |
| `float odom_get_gyro_bias_deg_s(void)` | bias | deg/s | 残余陀螺零偏 |
| `uint8_t odom_is_calibrated(void)` | 0/1 | — | 启动零偏标定是否完成 |

### 2.3 编码器 / IMU / 注册表

```c
int32_t  encoder_get_erpm(chassis_wheel_t w);        /* 电机 eRPM */
int32_t  encoder_get_tachometer(chassis_wheel_t w);  /* 累计码盘值 */
float    encoder_get_distance_m(chassis_wheel_t w);  /* 单轮距离 m */
uint32_t encoder_get_age_ms(chassis_wheel_t w);      /* 距上次收帧 ms */

float    Imu_GetYaw(void);     /* deg，已校零漂 */
float    Imu_GetGyroZ(void);   /* deg/s，已校零漂 */

const chassis_motor_t *chassis_find_by_id(uint8_t id);  /* id → 轮位 */
uint8_t chassis_id_of(chassis_wheel_t wheel);           /* 轮位 → id */
```

---

## 3. 数据接口（模块需要什么）

### 电机数据
- **来源**：VESC CAN status
  - `STATUS_1`（包 9）：rpm / current / duty
  - `STATUS_5`（包 27）：tachometer（累计位置）
- **单位**：rpm = 电气 RPM；current = mA；duty = ×1e5；tach = 电气转累计
- **要求**：VESC Tool 开启 CAN status，**Rate 1 = 50 Hz，勾选 status 1 + 5**；更新频率 ≥ 50 Hz

### IMU 数据
- **来源**：USART6 / RS485，IMU 上电自动流式
- **数据**：`yaw`（deg，−180~180）、`gyro_z`（deg/s）
- **要求**：连续接收，波特 921600 8N1

---

## 4. 输出状态（上层可读）

| 量 | 单位 | 来源 | 接口 |
|---|---|---|---|
| odom_x / odom_y | m | 融合 | `odom_get_x_m/y_m` |
| odom_yaw | deg | IMU | `odom_get_yaw_deg` |
| velocity_x / velocity_y | m/s (车体) | 编码器正解 | `odom_get_vx_mps/vy_mps` |
| 各轮 erpm | rpm | VESC status_1 | `encoder_get_erpm` |
| 各轮 tach / distance | 转 / m | VESC status_5 | `encoder_get_tachometer/distance_m` |

---

## 5. 参数配置（宏，重点）

> 几何参数**必须按你实际车体改**，否则里程计不准。

### 轮位 id（`chassis_motors.h`）
| 宏 | 作用 | 默认 |
|---|---|---|
| `CHASSIS_ID_LF/RF/LR/RR` | 四轮 VESC 的 CAN id | 67/68/69/70 |

### 里程计几何（`odom_fusion.h`）
| 宏 | 作用 | 单位 | 修改影响 |
|---|---|---|---|
| `ODOM_WHEEL_RADIUS_M` | 轮半径 | m | 影响速度/距离换算 |
| `ODOM_WHEEL_BASE_M` | 轮心距 L | m | 影响旋转分量 |
| `ODOM_POLE_PAIRS` | 电机极对数 | — | eRPM↔机械转速 |
| `ODOM_xx_BETA_DEG / GAMMA_DEG` | 各轮安装角/位置角 | deg | 装配方式变了必须改 |

### 融合调参（`odom_fusion.h`）
| 宏 | 作用 | 默认 | 修改影响 |
|---|---|---|---|
| `ODOM_ZUPT_WHEEL_THRESH` | 静止判定轮速阈值 | 0.05 m/s | 太大→运动中误判静止；太小→停不稳才触发 |
| `ODOM_ZUPT_BIAS_ALPHA` | ZUPT 零漂 EMA 系数 | 0.01 | 大→修得快但易被运动带偏 |
| `ODOM_CALIB_TICKS` | 启动零偏标定采样数 | 200 | 约 2 s @100 Hz |
| `ODOM_OUTLIER_THRESH` | 轮速异常剔除阈值 | 0.5 m/s | 超过的轮被剔出拟合 |

### 编码器（`encoder.h`）
| 宏 | 作用 |
|---|---|
| `ENC_POLE_PAIRS` / `ENC_WHEEL_RADIUS_M` | 单轮距离换算 |
| `ENC_STATUS5_PACKET` | STATUS_5 包号（默认 27，位置不更新就核对固件） |

### 通信（`usart.h` / `soft_start.h`）
| 宏 | 作用 | 默认 |
|---|---|---|
| `USART1_BAUDRATE` | 控制台波特率 | 115200 |
| `SOFT_START_DEFAULT_RAMP_MS` | 缓启动爬坡时间 | 500 |

---

## 6. 参数调试（现象 → 检查 → 改）

| 现象 | 检查 | 修改 |
|---|---|---|
| 旋转方向反 / 转弯角度不对 | IMU yaw 正负、轮安装角 | yaw 符号 或 `BETA/GAMMA` |
| 直线跑偏 | 轮距 / 轮径 / 安装角 | `ODOM_WHEEL_BASE/RADIUS/BETA` |
| 停车时角度慢慢漂 | 陀螺零漂 / ZUPT 阈值 | `ODOM_ZUPT_*` |
| 里程(tach)不更新 | VESC 没发 STATUS_5 / 包号不对 | VESC 开 status 5、`ENC_STATUS5_PACKET` |
| 收数时断时续 | CAN 被电机干扰 / 终端电阻 | 硬件：双绞+终端电阻+共地 |

---

## 7. 串口命令（115200 8N1，快速控制/查看）

| 命令 | 功能 |
|---|---|
| `R,<id>,<erpm>` | 设转速（电气 RPM） |
| `C,<id>,<mA>` | 设电流（mA） |
| `B,<id>,<mA>` | 设刹车电流 |
| `S,<0\|1>[,<ms>]` | 缓启动 开/关 [+爬坡ms] |
| `M[,<id>]` | 查轮子在线（ping） |
| `E[,<id>]` | 看编码器（rpm/电流/占空比/码盘） |
| `I` | 流式打印 IMU（yaw/gyro/pos，×50） |
| `O` | 流式打印里程计（x/y/yaw/v，×50） |
| `D` | RX 帧率诊断（STATUS_1/5 计数 + Hz） |
| `P,...` | PID（占位，未实现） |

> ⚠️ 已知问题：`D` 当前是 RX 诊断，占用了原"设占空比"命令位（占空比暂不可用，待修）。

---

## 8. 使用流程

```c
/* 初始化（顺序不能乱，USART6/IMU 要在 odom 之前） */
MX_CAN1_Init();
vesc_can_start(&hcan1);
can_rx_init(&hcan1);
MX_USART1_Init();   usart1_start_rx();   serial_cmd_init();
soft_start_init();
MX_USART6_UART_Init();   Imu_Init();
odom_init();

/* 主循环 */
while (1) {
    serial_cmd_poll();   /* 解析串口命令 */
    soft_start_tick(&hcan1);  /* ~20ms 重发目标 + 推进缓启动 */
    Imu_Update();        /* IMU 解算 */
    odom_tick();         /* ~100Hz 里程计融合 */
}
```

---

## 9. 注意事项

- **改底盘尺寸后必须重新标定**几何宏（轮径/轮距/安装角）。
- **IMU 安装方向不能改变**；改了要同步改 yaw 符号 / 安装角宏。
- **电机方向修改**（线序/正反）要同步改参数，否则运动学/里程计全反。
- **CAN id 必须匹配** VESC Tool 里设的 id（默认 67–70）。
- VESC 侧必须 **开启 CAN status 1 + 5 @ 50 Hz**，否则没有编码器/位置数据。
- 电机运行会干扰 CAN（实测）；CAN 线要 **双绞 + 远离电机相线 + 两端 120Ω + 共地**。
