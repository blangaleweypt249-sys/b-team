#include "upper_entry.h"

#include "cmsis_os2.h"
#include "comm_runtime.h"
#include "flash_tool.h"
#include "upper_config.h"
#include "upper_motor_port.h"
#include "upper_pc_link.h"
#include "vofa_bridge.h"

#define UPPER_CMD_QUEUE_DEPTH  4U
#define UPPER_STATE_PERIOD_MS  50U
#define UPPER_TX_BUFFER_SIZE   160U

static upper_robot_t upper_robot;
static upper_pc_link_t upper_pc_link;
static osMessageQueueId_t upper_cmd_queue;
static volatile bool upper_estop_pending;
static bool upper_vofa_suspended;
static uint32_t upper_cmd_drop_count;
__ALIGNED(32) static uint8_t upper_tx_buffer[UPPER_TX_BUFFER_SIZE];

static void UpperEntry_OnPcCmd(const upper_target_t *target, void *user_data)
{
    (void)user_data;
    if (osMessageQueuePut(upper_cmd_queue, target, 0U, 0U) != osOK)
    {
        upper_cmd_drop_count++;
    }
}

static void UpperEntry_OnPcEStop(void *user_data)
{
    (void)user_data;
    upper_estop_pending = true;
}

static void UpperEntry_OnUart(comm_uart_channel_t channel,
                              const uint8_t *data,
                              size_t size,
                              void *user_data)
{
    (void)user_data;
    if ((uint32_t)channel < (uint32_t)COMM_UART_RS485_1)
    {
        CommRuntime_SetPcChannel(channel);
        if (VofaBridge_Receive(data,
                               size,
                               CommRuntime_GetTickMs()))
        {
            return;
        }
        if (!FlashTool_Receive(data, size))
        {
            UpperEntry_OnPcData(data, size, CommRuntime_GetTickMs());
        }
    }
}

static void UpperEntry_OnCan(uint8_t can_bus,
                             const can_frame_t *frame,
                             void *user_data)
{
    (void)user_data;
    UpperEntry_OnCanFrame(can_bus, frame, CommRuntime_GetTickMs());
}

bool UpperEntry_Init(void)
{
    if (!UpperMotorPort_Init(upper_motor_cfg, UPPER_MOTOR_COUNT))
    {
        return false;
    }
    VofaBridge_Init();
    upper_vofa_suspended = false;

    upper_cmd_queue = osMessageQueueNew(UPPER_CMD_QUEUE_DEPTH,
                                        sizeof(upper_target_t),
                                        NULL);
    if (upper_cmd_queue == NULL)
    {
        return false;
    }

    UpperPcLink_Init(&upper_pc_link,
                     UpperEntry_OnPcCmd,
                     UpperEntry_OnPcEStop,
                     NULL);
    CommRuntime_SetHandlers(UpperEntry_OnUart, UpperEntry_OnCan, NULL);
    return UpperRobot_Init(&upper_robot, UpperMotorPort_Send, NULL);
}

static void UpperEntry_ProcessCmd(void)
{
    upper_target_t target;
    bool received;

    received = false;
    while (osMessageQueueGet(upper_cmd_queue, &target, NULL, 0U) == osOK)
    {
        received = true;
    }

    if (received)
    {
        UpperRobot_SetTarget(&upper_robot, &target);
        UpperRobot_Start(&upper_robot);
    }
}

static void UpperEntry_SendState(uint32_t tick_ms)
{
    size_t frame_size;

    if (FlashTool_IsActive() || VofaBridge_IsActive())
    {
        return;
    }
    if ((tick_ms % UPPER_STATE_PERIOD_MS) != 0U)
    {
        return;
    }

    if (!CommRuntime_PcTxReady())
    {
        return;
    }

    frame_size = UpperPcLink_BuildState(&upper_pc_link,
                                        &upper_robot,
                                        tick_ms,
                                        upper_tx_buffer,
                                        sizeof(upper_tx_buffer));
    if (frame_size > 0U)
    {
        (void)CommRuntime_PcTransmit(upper_tx_buffer, (uint16_t)frame_size);
    }
}

void UpperEntry_Control1ms(uint32_t tick_ms)
{
    UpperMotorPort_BeginCycle(tick_ms);
    if (VofaBridge_IsActive())
    {
        if (!upper_vofa_suspended)
        {
            UpperRobot_Stop(&upper_robot);
            upper_vofa_suspended = true;
        }
        VofaBridge_Control1ms(tick_ms);
        return;
    }
    upper_vofa_suspended = false;
    UpperEntry_ProcessCmd();
    if (upper_estop_pending)
    {
        upper_estop_pending = false;
        UpperRobot_EStop(&upper_robot);
    }
    else if (UpperPcLink_IsTimedOut(&upper_pc_link, tick_ms) &&
             (upper_robot.state == ROBOT_RUN))
    {
        UpperRobot_Stop(&upper_robot);
    }

    UpperRobot_Control1ms(&upper_robot, tick_ms);
    if (!UpperMotorPort_Flush())
    {
        upper_robot.motor_manager.send_fail_count++;
    }
    UpperEntry_SendState(tick_ms);
}

void UpperEntry_OnPcData(const uint8_t *data,
                         size_t size,
                         uint32_t tick_ms)
{
    UpperPcLink_Push(&upper_pc_link, data, size, tick_ms);
}

void UpperEntry_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms)
{
    UpperMotorPort_OnFrame(can_bus, frame, tick_ms);
    VofaBridge_OnCanFrame(can_bus, frame, tick_ms);
}

void App_Control1ms(void)
{
    UpperEntry_Control1ms(CommRuntime_GetTickMs());
}
