#ifndef CHASSIS_MOTORS_H
#define CHASSIS_MOTORS_H

#include "main.h"

/*
 * Chassis motor registry + presence check.
 *
 * Registers each VESC ESC to a wheel position so (later) chassis kinematics
 * can map a body velocity to per-wheel eRPM/current. Kinematics are NOT
 * implemented yet.
 *
 * The 'M' command does an ACTIVE presence check: it sends a CAN ping to each
 * registered id, then collects replies for CHASSIS_PROBE_TIMEOUT_MS. An id that
 * replies (status broadcast or pong) is "online"; a registered id with no reply
 * within the window is reported "LOST" (失联). This requires CAN RX (can_rx.c).
 *
 * This module only ADDS code; it does not modify VESC/CAN TX/soft-start/clock.
 */

/* ---- Wheel positions (order is also the registry index) ---- */
typedef enum {
    CHASSIS_WHEEL_LF = 0,   /* left-front  / 左前 */
    CHASSIS_WHEEL_RF = 1,   /* right-front / 右前 */
    CHASSIS_WHEEL_LR = 2,   /* left-rear   / 左后 */
    CHASSIS_WHEEL_RR = 3,   /* right-rear  / 右后 */
    CHASSIS_WHEEL_COUNT
} chassis_wheel_t;

/*
 * VESC controller IDs per wheel. CHANGE THESE to match the CAN IDs set in
 * VESC Tool. Defaults assume sequential ids 67..70.
 */
#define CHASSIS_ID_LF   67U     /* left-front  / 左前 (from spec) */
#define CHASSIS_ID_RF   68U     /* right-front / 右前 */
#define CHASSIS_ID_LR   69U     /* left-rear   / 左后 */
#define CHASSIS_ID_RR   70U     /* right-rear  / 右后 */

/* How long to wait for a reply during a presence probe (ms). */
#define CHASSIS_PROBE_TIMEOUT_MS   500U

/* A registered motor: which VESC id sits at which wheel. */
typedef struct {
    uint8_t         id;
    chassis_wheel_t wheel;
    const char     *name;
} chassis_motor_t;

#define CHASSIS_MOTOR_COUNT   ((uint8_t)CHASSIS_WHEEL_COUNT)

/* Registry access (read-only). */
const chassis_motor_t *chassis_get_motors(uint8_t *count);
const chassis_motor_t *chassis_find_by_id(uint8_t id);
const chassis_motor_t *chassis_find_by_wheel(chassis_wheel_t wheel);
uint8_t chassis_id_of(chassis_wheel_t wheel);

/*
 * Mark a controller id as "seen" (received a frame from it). Called from the
 * CAN RX interrupt for every received frame that originates from `controller_id`.
 */
void chassis_mark_seen(uint8_t controller_id);

/* Active presence probe over USART1: pings and reports each id online / LOST. */
void chassis_print_all(void);      /* probe all registered motors */
void chassis_print_id(uint8_t id); /* probe one id (or "not registered") */

#endif /* CHASSIS_MOTORS_H */
