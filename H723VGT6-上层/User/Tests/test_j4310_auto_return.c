#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "j4310_auto_return.h"

static bool Test_Close(float actual, float expected, float tolerance)
{
    float error = actual - expected;

    return (error >= -tolerance) && (error <= tolerance);
}

int main(void)
{
    j4310_auto_return_t control;
    float positive_midpoint;

    J4310AutoReturn_Init(&control, false);
    J4310AutoReturn_Update(&control, 10U, true, 1.0f, 0.0f, true);
    assert(!control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_DISABLED);

    J4310AutoReturn_Init(&control, true);
    assert(control.reconnect_armed);
    J4310AutoReturn_Update(&control, 20U, true, 2.0f, 0.0f, true);
    assert(control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_RUNNING);
    assert(Test_Close(control.target_position_rad, 2.0f, 0.0001f));
    J4310AutoReturn_Update(&control, 1020U, true, 1.0f, -0.5f, true);
    positive_midpoint = control.target_position_rad;
    assert(positive_midpoint > 0.0f && positive_midpoint < 2.0f);
    assert(control.target_velocity_rad_s < 0.0f);

    J4310AutoReturn_Update(&control, 2020U, false, 0.0f, 0.0f, true);
    assert(!control.owns_control);
    assert(control.reconnect_armed);
    J4310AutoReturn_Update(&control, 2030U, true, -1.5f, 0.0f, true);
    assert(control.owns_control);
    assert(Test_Close(control.target_position_rad, -1.5f, 0.0001f));
    J4310AutoReturn_Update(&control, 3030U, true, -0.7f, 0.5f, true);
    assert(control.target_position_rad > -1.5f);
    assert(control.target_position_rad < 0.0f);
    assert(control.target_velocity_rad_s > 0.0f);

    J4310AutoReturn_Update(&control, 5000U, true, 0.01f, 0.05f, true);
    assert(control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_HOLDING);
    assert(control.target_position_rad == 0.0f);
    assert(control.target_velocity_rad_s == 0.0f);

    J4310AutoReturn_Cancel(&control);
    assert(!control.owns_control);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 5010U, true, 0.5f, 0.0f, true);
    assert(!control.owns_control);
    J4310AutoReturn_Update(&control, 5020U, false, 0.0f, 0.0f, true);
    J4310AutoReturn_Update(&control, 5030U, true, 0.5f, 0.0f, true);
    assert(control.owns_control);

    J4310AutoReturn_Configure(&control, true, true);
    assert(!control.owns_control);
    assert(!control.reconnect_armed);
    J4310AutoReturn_Update(&control, 6000U, true, 0.8f, 0.0f, true);
    assert(!control.owns_control);
    J4310AutoReturn_Update(&control, 6010U, false, 0.0f, 0.0f, true);
    J4310AutoReturn_Update(&control, 6020U, true, 0.8f, 0.0f, true);
    assert(control.owns_control);

    J4310AutoReturn_Update(&control, 6030U, true, 0.7f, 0.0f, false);
    assert(!control.owns_control);
    assert(control.stage == J4310_AUTO_RETURN_ARMED);

    J4310AutoReturn_Configure(&control, false, true);
    assert(!control.enabled);
    assert(control.stage == J4310_AUTO_RETURN_DISABLED);
    return 0;
}
