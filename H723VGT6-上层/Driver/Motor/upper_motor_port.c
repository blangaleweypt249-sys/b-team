#include "upper_motor_port.h"

#include "main.h"

__weak bool UpperMotorPort_SendMg5010(const motor_cfg_t *cfg,
                                      const motor_cmd_t *cmd)
{
    (void)cfg;
    (void)cmd;
    return false;
}

__weak bool UpperMotorPort_SendJ4310(const motor_cfg_t *cfg,
                                     const motor_cmd_t *cmd)
{
    (void)cfg;
    (void)cmd;
    return false;
}

__weak bool UpperMotorPort_SendDji(const motor_cfg_t *cfg,
                                   const motor_cmd_t *cmd)
{
    (void)cfg;
    (void)cmd;
    return false;
}

__weak bool UpperMotorPort_SendDjm4216(const motor_cfg_t *cfg,
                                       const motor_cmd_t *cmd)
{
    (void)cfg;
    (void)cmd;
    return false;
}

__weak void UpperMotorPort_OnMg5010Frame(const can_frame_t *frame)
{
    (void)frame;
}

__weak void UpperMotorPort_OnJ4310Frame(const can_frame_t *frame)
{
    (void)frame;
}

__weak void UpperMotorPort_OnDjiFrame(uint8_t can_bus,
                                      const can_frame_t *frame)
{
    (void)can_bus;
    (void)frame;
}

__weak void UpperMotorPort_OnDjm4216Frame(const can_frame_t *frame)
{
    (void)frame;
}

bool UpperMotorPort_Send(const motor_cfg_t *cfg,
                         const motor_cmd_t *cmd,
                         void *user_data)
{
    (void)user_data;
    if ((cfg == NULL) || (cmd == NULL))
    {
        return false;
    }

    switch (cfg->model)
    {
    case MOTOR_MODEL_MG5010:
        return UpperMotorPort_SendMg5010(cfg, cmd);

    case MOTOR_MODEL_J4310:
        return UpperMotorPort_SendJ4310(cfg, cmd);

    case MOTOR_MODEL_M3508:
    case MOTOR_MODEL_M2006:
        return UpperMotorPort_SendDji(cfg, cmd);

    case MOTOR_MODEL_DJM4216:
        return UpperMotorPort_SendDjm4216(cfg, cmd);

    default:
        return false;
    }
}

void UpperMotorPort_OnFrame(uint8_t can_bus, const can_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    switch (can_bus)
    {
    case 1U:
        UpperMotorPort_OnMg5010Frame(frame);
        UpperMotorPort_OnJ4310Frame(frame);
        break;

    case 2U:
        UpperMotorPort_OnDjiFrame(can_bus, frame);
        break;

    case 3U:
        UpperMotorPort_OnDjiFrame(can_bus, frame);
        UpperMotorPort_OnDjm4216Frame(frame);
        break;

    default:
        break;
    }
}
