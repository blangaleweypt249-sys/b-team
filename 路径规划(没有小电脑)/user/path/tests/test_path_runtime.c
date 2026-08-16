#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"
#include "path_localization.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

static path_line_imu_data_t mock_odometry;
static imu_data_t mock_imu;
static int16_t mock_chassis_vx;
static int16_t mock_chassis_vy;
static int16_t mock_chassis_z;
static uint32_t mock_set_count;
static uint32_t mock_stop_count;
static uint32_t mock_yaw_target_count;

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    mock_chassis_vx = vx;
    mock_chassis_vy = vy;
    mock_chassis_z = z;
    mock_set_count++;
    return HAL_OK;
}

void Chassis_StopAll(void)
{
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    mock_chassis_z = 0;
    mock_stop_count++;
}

bool PathLineImu_GetData(path_line_imu_data_t *data)
{
    *data = mock_odometry;
    return true;
}

bool ImuMain_GetData(imu_data_t *data)
{
    *data = mock_imu;
    return true;
}

void ImuMain_EnableYawHold(bool enabled)
{
    mock_imu.yaw_hold_enabled = enabled;
}

HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg)
{
    mock_imu.target_yaw_deg = target_yaw_deg;
    mock_yaw_target_count++;
    return HAL_OK;
}

static void submit_and_run(int16_t raw_vx, int16_t raw_vy,
                           int16_t raw_z, uint8_t buttons,
                           uint32_t now_ms)
{
    Path_SubmitRemoteCommand(&raw_vx, &raw_vy, &raw_z, buttons, now_ms);
    assert(raw_z == 0);
    Path_Run1ms(now_ms);
}

static void set_front_laser(uint16_t distance_cm, uint32_t now_ms)
{
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = distance_cm;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_F_INDEX].online = 1U;
}

static void set_left_laser(uint16_t distance_cm, uint32_t now_ms)
{
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = distance_cm;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 1U;
}

static void test_initial_position_median_and_anchor(void)
{
    path_diagnostics_t diagnostics;
    float initial_x_m;
    float initial_y_m;
    float yaw_rad = 10.0f * 3.14159265358979323846f / 180.0f;

    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);

    /* 前 DT35 保持无样本/离线；同一左光时间戳不能伪造 3 个样本。 */
    assert(dt35_link[SENSOR_LINK_F_INDEX].online == 0U);
    mock_imu.yaw_deg = 10.0f;
    set_left_laser(15U, 1U);
    Path_Run1ms(1U);
    Path_Run1ms(2U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.front_laser_online);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 1U);

    set_left_laser(17U, 2U);
    Path_Run1ms(2U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 2U);

    set_left_laser(16U, 3U);
    Path_Run1ms(3U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 3U);
    assert(fabsf(diagnostics.front_initial_distance_m) < 0.0002f);
    assert(fabsf(diagnostics.left_initial_distance_m - 0.16f) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_x_m -
                 (PATH_MAP_WEST_INNER_X_M + 0.335f * cosf(yaw_rad))) <
           0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - 0.3085f) < 0.0002f);
    assert(fabsf(diagnostics.front_wall_hit_x_m) < 0.0002f);
    assert(fabsf(diagnostics.initial_yaw_deg - 10.0f) < 0.0002f);
    assert(diagnostics.segment_index == 0U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);
    initial_x_m = diagnostics.initial_map_x_m;
    initial_y_m = diagnostics.initial_map_y_m;

    /* 定点后新激光不再改原点，地图位置仅累加融合里程计位移。 */
    mock_imu.yaw_deg = 0.0f;
    mock_odometry.fused_position_x_m = 0.10f;
    mock_odometry.fused_position_y_m = 0.05f;
    set_front_laser(5U, 4U);
    set_left_laser(5U, 4U);
    Path_Run1ms(4U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_x_m - (initial_x_m + 0.10f)) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - (initial_y_m + 0.05f)) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_x_m - initial_x_m) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - initial_y_m) < 0.0002f);

    mock_odometry.fused_position_x_m = 0.0f;
    mock_odometry.fused_position_y_m = 0.0f;
    set_front_laser(20U, 5U);
    set_left_laser(20U, 5U);
    Path_Run1ms(5U);
    set_front_laser(20U, 6U);
    set_left_laser(20U, 6U);
    Path_Run1ms(6U);
    set_front_laser(20U, 7U);
    set_left_laser(20U, 7U);
    Path_Run1ms(7U);
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
}

static void test_manual_axis_and_auto_cancel(void)
{
    path_diagnostics_t diagnostics;

    /* 第一个松键帧为按键沿重新布防。 */
    submit_and_run(0, 0, 10, 0U, 10U);
    submit_and_run(40, 60, 10, PATH_REMOTE_MODE_BUTTON_BIT, 20U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 60);
    assert(mock_chassis_z == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.single_axis_enabled);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);

    /* 按住键不会反复切换。 */
    submit_and_run(40, 60, -10, PATH_REMOTE_MODE_BUTTON_BIT, 30U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 60);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.manual_toggle_count == 1U);

    /* 松开后再次按键，人工解除当前单轴限制。 */
    submit_and_run(40, 60, 0, 0U, 40U);
    submit_and_run(40, 60, 0, PATH_REMOTE_MODE_BUTTON_BIT, 50U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.single_axis_enabled);
    assert(diagnostics.raw_vy == 60);

    /* 再次打开首个 Y 段并越过 1.65 m，自动取消、停车并等待回中。 */
    submit_and_run(0, 0, 0, 0U, 60U);
    submit_and_run(50, 20, 0, PATH_REMOTE_MODE_BUTTON_BIT, 70U);
    assert(Path_GetDiagnostics(&diagnostics));
    mock_odometry.fused_position_y_m =
        1.650f - diagnostics.initial_map_y_m;
    submit_and_run(50, 20, 0, PATH_REMOTE_MODE_BUTTON_BIT, 80U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(mock_stop_count >= 1U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.single_axis_enabled);
    assert(diagnostics.neutral_rearm_required);
    assert(diagnostics.segment_index == 1U);
    assert(diagnostics.automatic_cancel_count == 1U);

    submit_and_run(30, 0, 0, 0U, 90U);
    assert(mock_chassis_vx == 0);
    submit_and_run(0, 0, 0, 0U, 100U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.neutral_rearm_required);

    /* 为后续独立激光安全测试移到墙 B 右侧，不影响初始锚点。 */
    mock_odometry.fused_position_x_m =
        2.480f - diagnostics.initial_map_x_m;
}

static void test_laser_direction_and_offline_policy(void)
{
    path_diagnostics_t diagnostics;
    int16_t vx;
    int16_t vy;
    int16_t z;

    /* 已在墙 B 右侧；前 DT35 的 20 cm 量程会按动态制动距离限速。 */
    set_front_laser(20U, 110U);
    submit_and_run(0, 150, 0, 0U, 110U);
    assert(mock_chassis_vy > 0);
    assert(mock_chassis_vy < 150);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_speed_limited);
    assert(diagnostics.front_laser_online);

    /* 三点保守最小值对距离下降首帧生效；0 同样按 5 cm 处理。 */
    set_front_laser(10U, 120U);
    submit_and_run(0, 150, 0, 0U, 120U);
    assert(mock_chassis_vy == 0);
    set_front_laser(10U, 130U);
    submit_and_run(0, 150, 0, 0U, 130U);
    set_front_laser(10U, 140U);
    submit_and_run(0, 150, 0, 0U, 140U);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_hard_blocked);

    set_front_laser(0U, 150U);
    submit_and_run(0, 100, 0, 0U, 150U);
    set_front_laser(0U, 160U);
    submit_and_run(0, 100, 0, 0U, 160U);
    set_front_laser(0U, 170U);
    submit_and_run(0, 100, 0, 0U, 170U);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_distance_cm == 5U);

    /* 距离增大必须用三帧覆盖近距离历史后才释放。 */
    set_front_laser(20U, 171U);
    submit_and_run(0, 100, 0, 0U, 171U);
    assert(mock_chassis_vy == 0);
    set_front_laser(20U, 172U);
    submit_and_run(0, 100, 0, 0U, 172U);
    assert(mock_chassis_vy == 0);
    set_front_laser(20U, 173U);
    submit_and_run(0, 100, 0, 0U, 173U);
    assert(mock_chassis_vy > 0);
    assert(mock_chassis_vy < 100);

    /* 离线只更新诊断，不因离线停车。 */
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    submit_and_run(0, 100, 0, 0U, 180U);
    assert(mock_chassis_vy == 100);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.front_laser_online);

    /* 离线后重新在线的近障碍必须首帧生效，不能沿用旧的远距离。 */
    set_front_laser(5U, 185U);
    submit_and_run(0, 100, 0, 0U, 185U);
    assert(mock_chassis_vy == 0);

    /* 横移不能掩盖仍在逼近前障碍的实测惯性；后退恢复仍可用。 */
    mock_odometry.encoder_body_velocity_y_mps = 0.20f;
    submit_and_run(30, 0, 0, 0U, 186U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    submit_and_run(0, -50, 0, 0U, 187U);
    assert(mock_chassis_vy < 0);
    mock_odometry.encoder_body_velocity_y_mps = 0.0f;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;

    /* 左 DT35 只限制 -X；+X 逃离方向保持可用。 */
    set_left_laser(5U, 190U);
    submit_and_run(-100, 0, 0, 0U, 190U);
    assert(mock_chassis_vx == 0);
    submit_and_run(100, 0, 0, 0U, 200U);
    assert(mock_chassis_vx > 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.left_speed_limited);

    /* 上位机来源不能旁路人工安全输出，且旋转始终为 0。 */
    vx = -999;
    vy = -999;
    z = 999;
    Path_ReplaceNonRemoteCommand(&vx, &vy, &z);
    assert(vx == mock_chassis_vx);
    assert(vy == 0);
    assert(z == 0);
}

static void test_yaw_zero_and_remote_timeout(void)
{
    path_diagnostics_t diagnostics;

    assert(mock_yaw_target_count >= 1U);
    assert(mock_imu.target_yaw_deg == 0.0f);
    Path_Run1ms(401U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.remote_online);
    assert(!diagnostics.single_axis_enabled);
}

/*
 * 回程：去程走完按回程键进入回程模式（yaw 目标 180°），掉头旋转
 * 期间忽略平移命令（地图坐标实时跟随里程计，不做冻结）；yaw 对齐
 * 后按回程路线推进。注意 vx/vy 是车体系：掉头后车体系相对地图系
 * 取反，向地图 -Y（回起点）走要推车体 +Y，向地图 +X 走要推车体
 * -X。前光装在车头，永远只限制车体 +Y；左光装在左侧，永远只限制
 * 车体 -X；地图净空按实测 yaw 把命令旋到地图系后查询。最后一段用
 * 前光距离实时校正 Y 漂移（基准为起始贴靠面 y=0），前光读数降到
 * 8 cm（半车长-前光偏移，车头贴面时的读数下限）判定回到起点并
 * 强制归零；再按回程键退出回程重新开始去程。
 */
static void test_return_trip(void)
{
    path_diagnostics_t diagnostics;

    /* 重新初始化：常规侧锚定 (0.374, 0.3085)。 */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(700U);
    set_left_laser(15U, 701U);
    Path_Run1ms(701U);
    set_left_laser(15U, 702U);
    Path_Run1ms(702U);
    set_left_laser(15U, 703U);
    Path_Run1ms(703U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);

    /* 去程路线 X 非单调，逐步推进 6 段直到终点 (0.5, 3.7)。 */
    mock_odometry.fused_position_y_m = 1.651f - 0.3085f;
    Path_Run1ms(704U);
    mock_odometry.fused_position_x_m = 2.481f - 0.374f;
    Path_Run1ms(705U);
    mock_odometry.fused_position_y_m = 2.601f - 0.3085f;
    Path_Run1ms(706U);
    mock_odometry.fused_position_x_m = 0.359f - 0.374f;
    Path_Run1ms(707U);
    mock_odometry.fused_position_y_m = 3.701f - 0.3085f;
    Path_Run1ms(708U);
    mock_odometry.fused_position_x_m = 0.501f - 0.374f;
    Path_Run1ms(709U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(diagnostics.segment_index == 6U);
    assert(!diagnostics.return_mode);

    /* 回到精确终点位置，等待回程键。 */
    mock_odometry.fused_position_x_m = 0.500f - 0.374f;
    mock_odometry.fused_position_y_m = 3.700f - 0.3085f;
    Path_Run1ms(710U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_x_m - 0.500f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 3.700f) < 0.0002f);

    /* 回程键按下沿：进入回程，yaw 目标切到 180°，要求停车回中。 */
    submit_and_run(0, 0, 0, 0U, 711U);
    submit_and_run(0, 0, 0, PATH_REMOTE_RETURN_BUTTON_BIT, 712U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_mode);
    assert(!diagnostics.return_yaw_aligned);
    assert(mock_imu.target_yaw_deg == 180.0f);
    assert(mock_stop_count >= 1U);

    /* 旋转未对齐期间平移命令被忽略；地图坐标实时跟随里程计
       （原地旋转不产生平移位移，坐标自然不变，无需冻结）。 */
    submit_and_run(50, 50, 0, 0U, 713U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(fabsf(diagnostics.map_x_m - 0.500f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 3.700f) < 0.0002f);

    /* IMU 掉头完成：对齐后开始回程段 0。 */
    mock_imu.yaw_deg = 180.0f;
    Path_Run1ms(714U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_yaw_aligned);
    assert(diagnostics.segment_index == 0U);
    assert(fabsf(diagnostics.map_x_m - 0.500f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 3.700f) < 0.0002f);

    /* 回程段 0：地图 X- 到 0.36（单轴 X）。yaw=180，向地图 -X
       前进需要推车体 +X；单轴在地图系清除 Y 分量。 */
    submit_and_run(0, 0, 0, 0U, 715U);
    submit_and_run(100, 60, 0, PATH_REMOTE_MODE_BUTTON_BIT, 716U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.single_axis_enabled);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_X);
    /* 真实运动方向为地图 -X，接近西侧净空线：限速但方向不变。 */
    assert(mock_chassis_vx > 0);
    assert(mock_chassis_vx < 100);
    assert(diagnostics.map_speed_limited);
    assert(mock_chassis_vy == 0);
    mock_odometry.fused_position_x_m = 0.359f - 0.374f;
    Path_Run1ms(717U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    /* 段 0 自动取消单轴后要求回中，先回中再发新命令。 */
    submit_and_run(0, 0, 0, 0U, 718U);
    submit_and_run(0, 0, 0, 0U, 719U);

    /* 回程段 1：地图 Y- 到 2.6 = 车体 +Y 前进；前光永远保护车体
       +Y：20 cm 饱和限速、10 cm 停车。 */
    set_front_laser(20U, 719U);
    Path_Run1ms(719U);
    submit_and_run(0, 150, 0, 0U, 720U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_speed_limited);
    assert(mock_chassis_vy > 0);
    assert(mock_chassis_vy < 150);
    set_front_laser(10U, 721U);
    submit_and_run(0, 150, 0, 0U, 721U);
    assert(mock_chassis_vy == 0);
    mock_odometry.fused_position_y_m = 2.599f - 0.3085f;
    Path_Run1ms(722U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);

    /* 回程段 2：地图 X+ 到 2.48 = 车体 -X，朝左光一侧运动；左光
       永远保护车体 -X（掉头后该侧物理上朝地图 +X）。 */
    mock_imu.yaw_deg = 180.0f;
    set_front_laser(20U, 723U);
    set_left_laser(8U, 723U);
    submit_and_run(-150, 0, 0, 0U, 723U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_hard_blocked);
    assert(mock_chassis_vx == 0);
    /* 车体 +X（远离左光、地图 -X）不受左光限制，但 x=0.36 处向
       地图 -X 会触发西侧净空限速，方向仍为车体正。 */
    set_left_laser(8U, 724U);
    submit_and_run(150, 0, 0, 0U, 724U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.left_speed_limited);
    assert(diagnostics.map_speed_limited);
    assert(mock_chassis_vx > 0);
    assert(mock_chassis_vx < 150);
    /* 回程常规侧左光 20 cm 饱和（面向空旷东侧）跳过限速；
       距离滤波需三帧覆盖 8 cm 历史后释放。 */
    set_left_laser(20U, 725U);
    Path_Run1ms(725U);
    set_left_laser(20U, 726U);
    mock_odometry.fused_position_x_m = 2.481f - 0.374f;
    Path_Run1ms(726U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 3U);
    set_left_laser(20U, 727U);
    mock_odometry.fused_position_y_m = 1.599f - 0.3085f;
    Path_Run1ms(727U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 4U);
    /*
     * 进入段 5 前先用两帧 20 cm 刷新前光三点窗口（覆盖段 1 遗留的
     * 10 cm 旧样本）：真实场景中段 4 期间前光面向 1.3 m 外的空旷
     * 南向，读数必然饱和 20 cm。
     */
    set_front_laser(20U, 728U);
    Path_Run1ms(728U);
    set_front_laser(20U, 729U);
    Path_Run1ms(729U);
    mock_odometry.fused_position_x_m = 0.0f;
    submit_and_run(150, 0, 0, 0U, 730U);
    assert(Path_GetDiagnostics(&diagnostics));
    /* 回程常规侧左光面向空旷东侧，20 cm 饱和跳过左光限速。 */
    assert(!diagnostics.left_speed_limited);
    /* x=0.374 处向地图 -X 受西侧净空限速，但方向保持车体正。 */
    assert(mock_chassis_vx > 0);
    assert(diagnostics.segment_index == 5U);

    /*
     * 回程段 5：前光朝起始贴靠面，距离 < 20 cm 时实时校正 Y 漂移。
     * 故意把 odometry Y 多走 5 cm：校正前 map_y 应为 0.3585，
     * 前光 11 cm 校正后 map_y 应为 0+0.225+0.11 = 0.335
     * （基准为起始贴靠面 y=0；11 cm 未到判定阈值，只校正不归零）。
     */
    mock_odometry.fused_position_y_m = 0.05f;
    set_front_laser(11U, 731U);
    Path_Run1ms(731U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_y_m - 0.335f) < 0.0002f);
    assert(!diagnostics.return_complete);

    /*
     * 前光降到 10 cm（= 8.35 cm 传感器凹进量按 5° 掉头残差斜置
     * 投影后的贴面读数上限）= 回到起点：物理兜底触发，强制归零
     * 到初始锚定位置吸收残余误差。
     */
    set_front_laser(10U, 732U);
    Path_Run1ms(732U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_complete);
    assert(fabsf(diagnostics.map_x_m - 0.374f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 0.3085f) < 0.0002f);

    /* 回程完成后再次按键：退出回程，重新开始去程（yaw 目标 0°）。 */
    submit_and_run(0, 0, 0, 0U, 740U);
    submit_and_run(0, 0, 0, PATH_REMOTE_RETURN_BUTTON_BIT, 741U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.return_mode);
    assert(diagnostics.segment_index == 0U);
    assert(!diagnostics.route_complete);
    assert(mock_imu.target_yaw_deg == 0.0f);
}

/*
 * 任务后回程：去程走完后不立即回程，先继续前进（模拟做任务），
 * 之后按回程键；验证回程路线能从任务点（去程终点之外的任意走廊
 * 位置）引导回起点。
 */
static void test_return_after_task(void)
{
    path_diagnostics_t diagnostics;

    /* 重新初始化：常规侧锚定 (0.374, 0.3085)。 */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(900U);
    set_left_laser(15U, 901U);
    Path_Run1ms(901U);
    set_left_laser(15U, 902U);
    Path_Run1ms(902U);
    set_left_laser(15U, 903U);
    Path_Run1ms(903U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);

    /* 去程逐步推进到终点 (0.5, 3.7)。 */
    mock_odometry.fused_position_y_m = 1.651f - 0.3085f;
    Path_Run1ms(904U);
    mock_odometry.fused_position_x_m = 2.481f - 0.374f;
    Path_Run1ms(905U);
    mock_odometry.fused_position_y_m = 2.601f - 0.3085f;
    Path_Run1ms(906U);
    mock_odometry.fused_position_x_m = 0.359f - 0.374f;
    Path_Run1ms(907U);
    mock_odometry.fused_position_y_m = 3.701f - 0.3085f;
    Path_Run1ms(908U);
    mock_odometry.fused_position_x_m = 0.501f - 0.374f;
    Path_Run1ms(909U);
    mock_odometry.fused_position_x_m = 0.500f - 0.374f;
    mock_odometry.fused_position_y_m = 3.700f - 0.3085f;
    Path_Run1ms(910U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(!diagnostics.return_mode);

    /* 做任务：继续沿走廊前进到 (0.5, 4.5)，不回程。 */
    mock_odometry.fused_position_y_m = 4.500f - 0.3085f;
    Path_Run1ms(911U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(!diagnostics.return_mode);
    assert(fabsf(diagnostics.map_y_m - 4.500f) < 0.0002f);

    /* 任务完成，按回程键：进入回程，yaw 目标 180°，掉头。 */
    submit_and_run(0, 0, 0, 0U, 912U);
    submit_and_run(0, 0, 0, PATH_REMOTE_RETURN_BUTTON_BIT, 913U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_mode);
    assert(mock_imu.target_yaw_deg == 180.0f);
    /* 地图坐标仍实时跟随任务点位置，不做任何冻结。 */
    assert(fabsf(diagnostics.map_x_m - 0.500f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 4.500f) < 0.0002f);

    /* 掉头完成，从任务点开始回程段 0（X- 到 0.36，在 y=4.5 层走）。 */
    mock_imu.yaw_deg = 180.0f;
    Path_Run1ms(914U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_yaw_aligned);
    assert(diagnostics.segment_index == 0U);
    assert(fabsf(diagnostics.map_y_m - 4.500f) < 0.0002f);
    mock_odometry.fused_position_x_m = 0.359f - 0.374f;
    Path_Run1ms(915U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);

    /* 回程段 1：Y- 到 2.6（y 4.5 → 2.6）。 */
    mock_odometry.fused_position_y_m = 2.599f - 0.3085f;
    Path_Run1ms(916U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);

    /* 回程段 2/3/4：X+ 2.48、Y- 1.60、X- 0.374。 */
    mock_odometry.fused_position_x_m = 2.481f - 0.374f;
    Path_Run1ms(917U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 3U);
    mock_odometry.fused_position_y_m = 1.599f - 0.3085f;
    Path_Run1ms(918U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 4U);
    mock_odometry.fused_position_x_m = 0.373f - 0.374f;
    Path_Run1ms(919U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 5U);

    /* 回程段 5：前光 11 cm 校正 Y（贴靠面基准），10 cm 贴面归零。 */
    set_front_laser(11U, 920U);
    mock_odometry.fused_position_y_m = 0.05f;
    Path_Run1ms(920U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_y_m - 0.335f) < 0.0002f);
    set_front_laser(10U, 921U);
    Path_Run1ms(921U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.return_complete);
    assert(fabsf(diagnostics.map_x_m - 0.374f) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - 0.3085f) < 0.0002f);
}

/* 镜像侧回程最后一段：左光朝东墙，读到真实距离时实时校正 X。 */
static void test_mirrored_return_x_correction(void)
{
    path_diagnostics_t diagnostics;

    /* 重新初始化：前光撞墙自动识别镜像侧并锚定。 */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(800U);
    mock_odometry.fused_position_y_m = 1.20f;
    set_front_laser(19U, 801U);
    Path_Run1ms(801U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_y_m - 1.660f) < 0.0002f);

    /* 镜像去程逐步推进到终点 (2.5, 3.7)。 */
    mock_odometry.fused_position_y_m = 1.651f - 0.460f;
    Path_Run1ms(802U);
    mock_odometry.fused_position_x_m = 0.519f - 2.626f;
    Path_Run1ms(803U);
    mock_odometry.fused_position_y_m = 2.601f - 0.460f;
    Path_Run1ms(804U);
    mock_odometry.fused_position_x_m = 2.641f - 2.626f;
    Path_Run1ms(805U);
    mock_odometry.fused_position_y_m = 3.701f - 0.460f;
    Path_Run1ms(806U);
    mock_odometry.fused_position_x_m = 2.499f - 2.626f;
    Path_Run1ms(807U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);

    /* 回程键 + 掉头 + 回程段推进到段 5。 */
    submit_and_run(0, 0, 0, 0U, 810U);
    submit_and_run(0, 0, 0, PATH_REMOTE_RETURN_BUTTON_BIT, 811U);
    mock_imu.yaw_deg = 180.0f;
    Path_Run1ms(812U);
    mock_odometry.fused_position_x_m = 2.641f - 2.626f;
    Path_Run1ms(813U);
    mock_odometry.fused_position_y_m = 2.599f - 0.460f;
    Path_Run1ms(814U);
    mock_odometry.fused_position_x_m = 0.519f - 2.626f;
    Path_Run1ms(815U);
    mock_odometry.fused_position_y_m = 1.599f - 0.460f;
    Path_Run1ms(816U);
    mock_odometry.fused_position_x_m = 0.0f;
    Path_Run1ms(817U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 5U);

    /*
     * 段 5：左光读到东墙 15 cm，实时校正 X 漂移。
     * 故意让 odometry X 偏 5 cm（map_x 本应为 2.576），
     * 校正后 map_x 应为 2.951-0.175-0.15 = 2.626。
     */
    mock_odometry.fused_position_x_m = -0.05f;
    set_left_laser(15U, 820U);
    set_front_laser(20U, 820U);
    Path_Run1ms(820U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_x_m - 2.626f) < 0.0002f);
}

/*
 * 对抗镜像侧：左光超量程饱和（20 cm、无目标）不参与初始定点；
 * 前光首次扫到墙时左光仍无目标 → 判定镜像侧，用假定贴东墙起点 +
 * 前光到墙 B 距离锚定，并切换到镜像地图；镜像侧左光无目标时
 * -X 运动不受左光限速，出现真实目标时限速恢复。
 */
static void test_mirrored_side_detection_and_anchor(void)
{
    path_diagnostics_t diagnostics;

    /* 模拟镜像侧重新上电：重新初始化模块并复位外部模拟量。 */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(500U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);

    /* 左光 20 cm 饱和视为无目标：不采集、不按常规侧锚定。 */
    set_left_laser(20U, 501U);
    Path_Run1ms(501U);
    Path_Run1ms(502U);
    Path_Run1ms(503U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(diagnostics.left_initial_sample_count == 0U);

    /* 机器人向 +Y 行进，前光首次扫到墙（读数首次降到 20 cm 以下）。 */
    mock_odometry.fused_position_y_m = 1.20f;
    set_front_laser(19U, 504U);
    Path_Run1ms(504U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_x_m -
                 PATH_MAP_MIRRORED_START_X_M) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m -
                 (PATH_MAP_MIRRORED_FIRST_WALL_Y_M -
                  PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M - 0.19f)) <
           0.0002f);
    assert(fabsf(diagnostics.front_initial_distance_m - 0.19f) < 0.0002f);
    assert(fabsf(diagnostics.left_initial_distance_m) < 0.0002f);
    /* 撞墙点已在第 0 段终点（1.65 m）之后，直接进入镜像侧 X− 段。 */
    assert(diagnostics.segment_index == 1U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_X);

    /* 镜像侧左光无目标：-X 运动既不受左光限速也不受地图限速。 */
    set_left_laser(20U, 510U);
    submit_and_run(-200, 0, 0, 0U, 510U);
    assert(mock_chassis_vx == -200);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.left_speed_limited);
    assert(!diagnostics.left_hard_blocked);

    /* 镜像侧左光出现真实目标时，左光限速仍生效。 */
    set_left_laser(15U, 511U);
    submit_and_run(-200, 0, 0, 0U, 511U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_speed_limited);
    assert(mock_chassis_vx > -200);
    assert(mock_chassis_vx < 0);
    submit_and_run(0, 0, 0, 0U, 512U);

    /* 在第一条横向通道内继续行进，仍处于镜像侧第 1 段（X−）。 */
    mock_odometry.fused_position_y_m = 1.30f;
    Path_Run1ms(520U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_X);

    /* 镜像侧第 1 段终点为 x = 0.52 m（对应贴东墙起点的左向通道）。 */
    mock_odometry.fused_position_x_m = 0.53f - PATH_MAP_MIRRORED_START_X_M;
    Path_Run1ms(521U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    mock_odometry.fused_position_x_m = 0.52f - PATH_MAP_MIRRORED_START_X_M;
    Path_Run1ms(522U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);

    /*
     * 反向场景：前光首次撞墙时左光有目标 → 判定为常规侧，不切换
     * 镜像地图；yaw 恢复就绪后仍按常规左光锚定完成。
     */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.state = IMU_STATE_CALIBRATING;
    mock_imu.yaw_deg = 0.0f;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(600U);
    set_left_laser(15U, 601U);
    Path_Run1ms(601U);
    set_left_laser(15U, 602U);
    Path_Run1ms(602U);
    set_left_laser(15U, 603U);
    Path_Run1ms(603U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);

    mock_odometry.fused_position_y_m = 1.00f;
    set_front_laser(15U, 604U);
    Path_Run1ms(604U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);

    /* IMU 就绪后常规侧左光锚定照常完成，仍不是镜像侧。 */
    mock_imu.state = IMU_STATE_READY;
    mock_imu.yaw_deg = 0.0f;
    set_left_laser(15U, 605U);
    Path_Run1ms(605U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_x_m - 0.374f) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - 0.3085f) < 0.0002f);
}

int main(void)
{
    (void)memset((void *)dt35_link, 0, sizeof(dt35_link));
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    (void)memset(&mock_imu, 0, sizeof(mock_imu));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;

    Path_Init();
    Path_Run1ms(0U);
    test_initial_position_median_and_anchor();
    test_manual_axis_and_auto_cancel();
    test_laser_direction_and_offline_policy();
    test_yaw_zero_and_remote_timeout();
    test_mirrored_side_detection_and_anchor();
    test_return_trip();
    test_return_after_task();
    test_mirrored_return_x_correction();
    puts("path runtime host tests: PASS");
    return 0;
}
