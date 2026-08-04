#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include "main.h"

/*
 * Serial command bridge.
 *
 * Parses ASCII command lines received over USART1 and translates them into the
 * corresponding VESC CAN "set-point" frames on CAN1. Existing VESC / CAN code
 * is left untouched; this only ADDS new send helpers and calls them.
 *
 * Line format (comma separated, terminated by '\n'):
 *
 *   R,<id>,<erpm>          set electrical RPM          (CAN_PACKET_SET_RPM, 3)
 *   C,<id>,<current_mA>    set current in mA           (CAN_PACKET_SET_CURRENT, 1)
 *   D,<id>,<duty_1e5>      set duty (duty*100000)      (CAN_PACKET_SET_DUTY, 0)
 *   B,<id>,<brake_mA>      set brake current in mA     (CAN_PACKET_SET_CURRENT_BRAKE, 2)
 *   P,<id>,<kp>,<ki>,<kd>  set speed PID (PLACEHOLDER) -> multi-frame COMM, reserved
 *
 *   <id>           : VESC controller id, 0..255
 *   <erpm>         : signed 32-bit electrical RPM (motor_rpm * pole_pairs)
 *   <current_mA>   : signed 32-bit, e.g. 2000 = 2 A (VESC wires current*1000)
 *   <duty_1e5>     : signed 32-bit, e.g. 50000 = 0.50 (50%)
 *
 * Examples:
 *   R,67,21000\n      -> controller 67 to 21000 eRPM
 *   C,67,2000\n       -> controller 67 to 2 A
 *   C,67,-2000\n      -> controller 67 to -2 A (reverse)
 *   D,67,50000\n      -> controller 67 to 50% duty
 *
 * Each valid line is answered with "OK" or "ERR ..." over USART1.
 */

void serial_cmd_init(void);   /* print usage banner */
void serial_cmd_poll(void);   /* call from the main loop */

#endif /* SERIAL_CMD_H */
