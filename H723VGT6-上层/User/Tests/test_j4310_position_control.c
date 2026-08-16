#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "j4310_position_control.h"

static bool Test_Close(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    j4310_position_control_t control;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float learned_cos_nm;
    uint32_t index;

    assert(J4310PositionControl_Init(&control, 2.0f, 5.0f, 3.0f,
                                      0.01f));
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

    /* A new feedback timestamp is learned once. At q=0 the cosine model
     * converges to the current-derived static joint torque. */
    for (index = 1U; index <= 1000U; index++)
    {
        torque_nm = J4310PositionControl_ComposeTorque(
            &control, true, index, 0.0f, 0.0f, 2.0f,
            0.0f, 0.0f, 0.0f, 5.0f);
    }
    assert(torque_nm > 1.94f && torque_nm < 2.01f);
    learned_cos_nm = control.gravity_cos_nm;

    (void)J4310PositionControl_ComposeTorque(
        &control, true, 1000U, 0.0f, 0.0f, -2.0f,
        0.0f, 0.0f, 0.0f, 5.0f);
    assert(control.gravity_cos_nm == learned_cos_nm);

    /* Dynamic feedback is not interpreted as gravity. */
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1001U, 4.71238898038f, 1.0f, -2.0f,
        0.0f, 1.0f, 0.0f, 5.0f);
    assert(control.gravity_cos_nm == learned_cos_nm);
    assert(fabsf(torque_nm) < 0.001f);

    /* Gravity compensation follows the current angle in real time, while a
     * small movement inside the deadband does not change the reference. */
    control.gravity_cos_nm = 0.5f;
    control.gravity_sin_nm = 1.2f;
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1002U, 0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.5f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1003U, 0.01f, 1.0f, 0.0f,
        0.01f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 0.5f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1004U, 1.57079632679f, 1.0f, 0.0f,
        1.57079632679f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, 1.2f, 0.0001f));
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, true, 1005U, 3.14159265359f, 1.0f, 0.0f,
        3.14159265359f, 1.0f, 0.0f, 5.0f);
    assert(Test_Close(torque_nm, -0.5f, 0.0001f));

    /* Requested tau keeps priority and gravity uses only the remaining
     * software torque budget. */
    torque_nm = J4310PositionControl_ComposeTorque(
        &control, false, 0U, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.8f, 1.0f);
    assert(Test_Close(torque_nm, 0.6f, 0.0001f));
    assert(Test_Close(control.gravity_torque_nm, -0.2f, 0.0001f));

    /* Production limits make a 90-degree move about 0.59 seconds. */
    assert(J4310PositionControl_Init(&control, 5.0f, 30.0f, 8.0f,
                                      0.002f));
    assert(J4310PositionControl_Start(&control, 0U, 0.0f,
                                       1.57079632679f));
    assert(control.trajectory_duration_ms >= 580U);
    assert(control.trajectory_duration_ms <= 600U);
    return 0;
}
