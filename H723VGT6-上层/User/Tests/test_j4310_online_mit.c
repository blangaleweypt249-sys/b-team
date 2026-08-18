/**
 * @file test_j4310_online_mit.c
 * @brief 验证 J4310 MIT 在线整定器的更新行为。
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "motor_online_tune.h"

/* 功能：判断两个浮点数是否在给定误差内接近；用途：辅助验证数值控制结果；返回 true 表示比较通过。 */
static bool Test_Close(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

/* 功能：运行本文件的 J4310 MIT 在线整定器的更新行为测试；用途：集中执行断言用例；返回 0 表示全部测试通过。 */
int main(void)
{
    motor_online_mit_cfg_t cfg =
    {
        .minimum_kp = 0.0f,
        .maximum_kp = 49.0f,
        .minimum_kd = 0.0f,
        .maximum_kd = 0.95f,
        .near_error = 0.01745329252f,
        .far_error = 0.17453292520f,
        .velocity_scale = 3.0f,
        .diverging_rate = -0.05f,
        .stalled_rate = 0.01f,
        .stalled_velocity = 0.10f,
        .smoothing = 0.20f
    };
    motor_online_mit_t tuner;
    float kp;
    float kd;

    assert(MotorOnlineMit_Init(&tuner, &cfg, true));
    assert(MotorOnlineMit_SetCommand(&tuner, 30.0f, 0.5f));

    MotorOnlineMit_Update(&tuner, 0.01f, 0.0f, 0.0f, 0.001f,
                          &kp, &kd);
    assert(Test_Close(kp, 30.0f, 0.0001f));
    assert(Test_Close(kd, 0.5f, 0.0001f));

    MotorOnlineMit_Update(&tuner, 0.17453292520f, 0.0f, 0.0f, 0.001f,
                          &kp, &kd);
    assert(kp > 30.0f);
    assert(kd >= 0.5f);
    assert(kp < 50.0f);
    assert(kd < 1.0f);

    MotorOnlineMit_Update(&tuner, 0.01f, 0.0f, 0.0f, 0.001f,
                          &kp, &kd);
    assert(Test_Close(kp, 30.0f, 0.0001f));
    assert(Test_Close(kd, 0.5f, 0.0001f));

    MotorOnlineMit_SetEnabled(&tuner, false);
    MotorOnlineMit_Update(&tuner, 0.17453292520f, 2.0f, 2.0f, 0.001f,
                          &kp, &kd);
    assert(Test_Close(kp, 30.0f, 0.0001f));
    assert(Test_Close(kd, 0.5f, 0.0001f));
    return 0;
}
