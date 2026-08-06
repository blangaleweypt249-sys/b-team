#include "action_api.h"

#include "up_main.h"

/* 动作命令使用的固定机械位置，单位为度。 */
#define LEG_HOME_ANGLE_DEG      0.0f
#define LEG_LIFT_ANGLE_DEG      180.0f
#define LEG_FOLD_ANGLE_DEG      270.0f
#define LEG_FLAT_ANGLE_DEG      450.0f
#define LEG_DOWN_ANGLE_DEG      540.0f
#define M2006_FORWARD_ANGLE_DEG 360.0f

volatile action_cmd_t action_pending = ACTION_CMD_NONE;
HAL_StatusTypeDef action_last_result = HAL_OK;

/**
 * @brief 提交一个机构动作
 * @param action 动作命令
 * @retval HAL 状态
 */
HAL_StatusTypeDef Action_Request(action_cmd_t action)
{
    if (action > ACTION_CMD_REAR_DOWN)
    {
        return HAL_ERROR;
    }

    action_pending = action;
    return HAL_OK;
}

/**
 * @brief 在抬升任务中执行待处理动作
 * @retval None
 */
void Action_Run1ms(void)
{
    action_cmd_t action;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    action = action_pending;
    action_pending = ACTION_CMD_NONE;
    if (primask == 0U)
    {
        __enable_irq();
    }

    switch (action)
    {
    case ACTION_CMD_LOWER:
        action_last_result = Up_SetMotorPos(
            LEG_HOME_ANGLE_DEG, LEG_HOME_ANGLE_DEG,
            LEG_HOME_ANGLE_DEG, LEG_HOME_ANGLE_DEG);
        break;

    case ACTION_CMD_LIFT:
        action_last_result = Up_SetMotorPos(
            LEG_LIFT_ANGLE_DEG, LEG_LIFT_ANGLE_DEG,
            LEG_LIFT_ANGLE_DEG, LEG_LIFT_ANGLE_DEG);
        break;

    case ACTION_CMD_M2006_FORWARD:
        action_last_result = Up_MoveM2006(M2006_FORWARD_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_FOLD:
        action_last_result = Up_SetRsPos(LEG_FOLD_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_FLAT:
        // 450 deg 与机械角 90 deg 等效，保证从 270 deg 沿正方向越过零点。
        action_last_result = Up_SetRsPos(LEG_FLAT_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_DOWN:
        action_last_result = Up_SetRsPos(LEG_DOWN_ANGLE_DEG);
        break;

    case ACTION_CMD_REAR_FOLD:
        action_last_result = Up_SetDmPos(LEG_FOLD_ANGLE_DEG);
        break;

    case ACTION_CMD_REAR_FLAT:
        // 达妙位置范围支持 450 deg，与机械角 90 deg 等效。
        action_last_result = Up_SetDmPos(LEG_FLAT_ANGLE_DEG);
        break;

    case ACTION_CMD_REAR_DOWN:
        action_last_result = Up_SetDmPos(LEG_DOWN_ANGLE_DEG);
        break;

    case ACTION_CMD_NONE:
    default:
        break;
    }
}
