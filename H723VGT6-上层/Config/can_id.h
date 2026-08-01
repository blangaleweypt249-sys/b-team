#ifndef CAN_ID_H
#define CAN_ID_H

/* Logical node IDs. Each device driver converts these to protocol CAN IDs. */
#define CAN_BUS_ARM             1U
#define CAN_BUS_MOVE            2U
#define CAN_BUS_AUX             3U

#define NODE_ARM_MG5010         1U
#define NODE_ARM_J4310          2U
/* Must match the J4310 Master ID stored by the DAMIAO setup tool. */
#define CAN_J4310_MASTER_ID     0x000U

#define NODE_MOVE_M3508_L       1U
#define NODE_MOVE_M3508_R       2U
#define NODE_MOVE_M2006_L       3U
#define NODE_MOVE_M2006_R       4U

#define NODE_ORE_M2006          1U
#define NODE_GATE_M2006         2U
#define NODE_CONVEYOR_M4216_L   3U
#define NODE_CONVEYOR_M4216_R   4U

#endif
