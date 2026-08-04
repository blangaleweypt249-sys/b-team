#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "chassis_motors.h"   /* chassis_wheel_t, chassis_find_by_id */

/*
 * Wheel encoder feedback over VESC CAN.
 *
 * The AS5047P is the VESC's encoder; the VESC derives motor speed/position
 * from it and broadcasts CAN status frames. This module decodes those frames
 * per registered wheel:
 *   - STATUS_1 (CAN_PACKET_STATUS, packet 9): electrical RPM (for PID feedback),
 *     current, duty.
 *   - STATUS_5 (CAN_PACKET_STATUS_5, tachometer): accumulated position (for
 *     odometry).
 *
 * The decoded values are stored per wheel and exposed via getters. This module
 * only ADDS code and is updated passively from the CAN RX interrupt.
 *
 * NOTE: ENC_STATUS5_PACKET defaults to 27 (recent VESC enum). If wheel position
 * never updates, your VESC firmware's STATUS_5 number likely differs - check it
 * and adjust the macro. STATUS_1 (9) is stable.
 */

#define ENC_STATUS1_PACKET      9U    /* CAN_PACKET_STATUS: rpm, current, duty */
#define ENC_STATUS5_PACKET     27U    /* CAN_PACKET_STATUS_5: tachometer, v_in */

/* Unit conversion (edit for your motor/wheel). */
#define ENC_POLE_PAIRS         21.0f
#define ENC_WHEEL_RADIUS_M     0.050f
#define ENC_TACH_PER_MECH_REV  ENC_POLE_PAIRS   /* tachometer counts electrical revs */

/* Feed a received status frame (from the CAN RX callback). Safe to call for any
 * packet; only STATUS_1 / STATUS_5 are decoded. */
void encoder_on_rx(uint8_t controller_id, uint8_t packet,
                   const uint8_t *data, uint8_t len);

/* Per-wheel access. Return 0 / 0.0 if never received. */
int32_t  encoder_get_erpm(chassis_wheel_t w);
float    encoder_get_current_a(chassis_wheel_t w);
float    encoder_get_duty(chassis_wheel_t w);
int32_t  encoder_get_tachometer(chassis_wheel_t w);
float    encoder_get_distance_m(chassis_wheel_t w);
uint32_t encoder_get_age_ms(chassis_wheel_t w);
uint8_t  encoder_has_data(chassis_wheel_t w);

/* Serial print (USART1). */
void encoder_print_all(void);
void encoder_print(uint8_t controller_id);

/* RX frame-rate diagnostic: count STATUS_1 / STATUS_5 frames per wheel.
 * 'D' shows the per-second rate over the window since the last 'D' call. */
void encoder_diag_print_all(void);
void encoder_diag_print(uint8_t controller_id);

#endif /* ENCODER_H */
