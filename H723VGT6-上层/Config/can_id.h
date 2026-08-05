#ifndef CAN_ID_H
#define CAN_ID_H

/* Logical node IDs. Each device driver converts these to protocol CAN IDs. */
#define CAN_BUS_ARM_PRIMARY    1U
#define CAN_BUS_ARM_SECONDARY  2U
#define CAN_BUS_AUX            3U

#define NODE_FDCAN1_M3508_1    1U
#define NODE_FDCAN1_M3508_2    2U
#define NODE_ARM_J4310         3U
/* Must match the J4310 Master ID stored by the DAMIAO setup tool. */
#define CAN_J4310_MASTER_ID     0x000U

#define NODE_FDCAN2_M3508_1    1U
#define NODE_FDCAN2_M3508_2    2U

#define NODE_CONVEYOR_M2006     1U
#define NODE_GRIPPER_M2006      2U

#define CAN_FILTER_ARM_J4310_ID       CAN_J4310_MASTER_ID
#define CAN_FILTER_DJI_NODE_1_ID      0x201U
#define CAN_FILTER_DJI_NODE_2_ID      0x202U
#define CAN_FILTER_AUX_CONVEYOR_ID    (0x200U + NODE_CONVEYOR_M2006)
#define CAN_FILTER_AUX_GRIPPER_ID     (0x200U + NODE_GRIPPER_M2006)

#endif
