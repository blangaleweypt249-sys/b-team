/**
 * @file test_j4310_position_control.c
 * @brief 验证 J4310 位置轨迹和重力补偿控制。
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "j4310_position_control.h"

/* 功能：判断两个浮点数是否在给定误差内接近；用途：辅助验证数值控制结果；返回 true 表示比较通过。 */
static bool Test_Close(float actual /* 测试或判断使用的实际值 */, float expected /* 测试期望得到的参考值 */, float tolerance /* 比较实际值与期望值时允许的误差 */)
{
    return fabsf(actual - expected) <= tolerance;
}

/* 功能：运行本文件的 J4310 位置轨迹和重力补偿控制测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    j4310_position_control_t control;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float learned_sin_nm;
    uint32_t index;

    assert(J4310PositionControl_Init(&control, 2.0f, 5.0f, 3.0f,
                                      0.01f, 1.0f, 0.0872664626f,
                                      0.01745329252f, 100U, 10000.0f));
    assert(J4310PositionControl_Start(&control, 100U, 0.0f, 2.0f));
    J4310PositionControl_Sample(&control, 100U,
                                 &position_rad, &velocity_rad_s);
    assert(position_rad == 0.0f);
    assert(velocity_rad_s == 0.0f);

    J4310PositionControl_Sample(&control, 1038U,
                                 &position_rad, &velocity_rad_s);
    assert(position_rad > 0.99f && position_rad < 1.01f);
    assert(velocity_rad_s > 1.99f && velocity_rad_s <= 2.001f);

    J4310PositionControl_Sample(&control, 1975U,
                                 &position_rad, &velocity_rad_s);
    assert(position_rad == 2.0f);
    assert(velocity_rad_s == 0.0f);
    assert(!control.trajectory_active);

    assert(J4310PositionControl_Start(&control, 2000U, 2.0f, -1.0f));
    J4310PositionControl_Sample(&control, 2000U,
                                 &position_rad, &velocity_rad_s);
    assert(position_rad == 2.0f);
    assert(velocity_rad_s == 0.0f);
    J4310PositionControl_Sample(
        &control, 2000U + control.trajectory_duration_ms,
        &position_rad, &velocity_rad_s);
    assert(position_rad == -1.0f);
    assert(velocity_rad_s == 0.0f);

    /* 新的反馈时间戳只学习一次。在 q=90 度时，正弦模型收敛到按电流换算的静态关节转矩。 */
    for (index = 1U; index <= 1000U; index++)
    {
        torque_nm = J4310PositionControl_ComposeTorque(
            &control, true, index, 1.57079632679f, 0.0f, 2.0f,
            1.60570291183f, 0.0f, 0.0f, 5.0f);
    }
    assert(torque_nm > 1.94f && torque_nm < 2.01f);
    learned_sin_nm = control.gravity_sin_nm;

    (void)J4310PositionControl_ComposeTorque(
        &control, true, 1000U, 1.57079632679f, 0.0f, -2.0f,
        1.60570291183f, 0.0f, 0.0f, 5.0f);
    assert(control.gravity_sin_nm == learned_sin_nm);

    /* 动态反馈不会被解释为重力。 */
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1001U, 4.71238898038f, 1.0f, -2.0f,
        0.0f, 1.0f, 0.0f, 5.0f);
    assert(control.gravity_sin_nm == learned_sin_nm);
    assert(Test_Close(torque_nm, -learned_sin_nm, 0.001f));

    /* 重力补偿实时跟随当前角度，但在 0 度和 180 度附近的 +/-5 度区域内除外。 */
    control.gravity_cos_nm = 0.5f;
    control.gravity_sin_nm = 1.2f;
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1002U, 0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1003U, 0.01f, 1.0f, 0.0f,
        0.01f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1004U, 1.57079632679f, 1.0f, 0.0f,
        1.57079632679f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 1.2f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1005U, 3.14159265359f, 1.0f, 0.0f,
        3.14159265359f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1006U, 2.96705972839f, 1.0f, 0.0f,
        2.96705972839f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm,
                      0.5f * cosf(2.96705972839f) +
                      1.2f * sinf(2.96705972839f),
                      0.0001f));

    /* 即使 2 度模型参考死区尚未移动，最新原始角度仍控制禁用区域；
     * 即使反馈短暂过期，该判断仍然有效。 */
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1007U, 0.10471975512f, 1.0f, 0.0f,
        0.10471975512f, 1.0f, 0.0f, 5.0f);
    assert(fabsf(torque_nm) > 0.1f);
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1008U, 0.07853981634f, 1.0f, 0.0f,
        0.07853981634f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, false, 0U, 0.0f, 0.0f, 0.0f,
        0.10471975512f, 0.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));

    /* 禁用区域内的反馈既不输出也不学习重力。 */
    learned_sin_nm = control.gravity_sin_nm;
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1009U, 6.28318530718f, 0.0f, 4.0f,
        6.28318530718f, 0.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.0f, 0.0001f));
    assert(control.gravity_sin_nm == learned_sin_nm);

    /* 请求的 tau 具有优先级，重力补偿只能使用剩余的软件转矩预算。 */
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, false, 0U, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.8f, 1.0f);
    assert(Test_Close(torque_nm, 0.8f, 0.0001f));
    assert(Test_Close(control.gravity_torque_nm, 0.0f, 0.0001f));

    /* 目标在 +/-1 度范围内保持足够的反馈次数后，重力学习会冻结，直到请求目标改变。 */
    assert(J4310PositionControl_Init(&control, 4.0f, 20.0f, 8.0f,
                                      0.02f, 1.0f, 0.0872664626f,
                                      0.01745329252f, 3U, 10000.0f));
    for (index = 1U; index <= 3U; index++)
    {
        (void)J4310PositionControl_ComposeTorque(
            &control, true, index, 1.57079632679f, 0.0f, 2.0f,
            1.57079632679f, 0.0f, 0.0f, 5.0f);
    }
    assert(control.gravity_learning_locked);
    learned_sin_nm = control.gravity_sin_nm;
    for (index = 4U; index <= 20U; index++)
    {
        (void)J4310PositionControl_ComposeTorque(
            &control, true, index, 1.57079632679f, 0.0f, -2.0f,
            1.57079632679f, 0.0f, 0.0f, 5.0f);
    }
    assert(control.gravity_sin_nm == learned_sin_nm);
    (void)J4310PositionControl_ComposeTorque(
        &control, true, 21U, 1.04719755120f, 0.0f, 1.0f,
        1.04719755120f, 0.0f, 0.0f, 5.0f);
    assert(!control.gravity_learning_locked);
    assert(control.gravity_sin_nm != learned_sin_nm);

    /* 生产重力增益会将学习模型输出提高 50%。 */
    assert(J4310PositionControl_Init(&control, 4.0f, 20.0f, 8.0f,
                                      0.01f, 1.5f, 0.0872664626f,
                                      0.01745329252f, 100U, 10000.0f));
    control.gravity_sin_nm = 1.0f;
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1U, 1.57079632679f, 1.0f, 0.0f,
        1.57079632679f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 1.5f, 0.0001f));

    /* 生产转矩变化率限制可防止重力补偿出现阶跃。 */
    assert(J4310PositionControl_Init(&control, 4.0f, 20.0f, 8.0f,
                                      0.01f, 1.5f, 0.0872664626f,
                                      0.01745329252f, 100U, 10.0f));
    control.gravity_sin_nm = 1.0f;
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1U, 1.57079632679f, 1.0f, 0.0f,
        1.57079632679f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.01f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 2U, 1.57079632679f, 1.0f, 0.0f,
        1.57079632679f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.02f, 0.0001f));

    /* 生产限值使 90 度运动耗时约 0.80 秒。 */
    assert(J4310PositionControl_Init(&control, 3.7f, 15.0f, 8.0f,
                                      0.02f, 1.5f, 0.0872664626f,
                                      0.01745329252f, 100U, 10.0f));
    assert(J4310PositionControl_Start(&control, 0U, 0.0f,
                                       1.57079632679f));
    assert(control.trajectory_duration_ms >= 790U);
    assert(control.trajectory_duration_ms <= 810U);
    return 0;
}
