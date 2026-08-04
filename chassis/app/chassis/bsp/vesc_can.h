#ifndef VESC_CAN_H
#define VESC_CAN_H

#include "stm32f4xx_hal.h"

/*
 * Standard VESC CAN "set-point" packet identifiers (Vedder firmware).
 * These are single extended frames with a 4-byte big-endian signed payload.
 * (CAN_PACKET_SET_RPM == 3 is still defined locally in vesc_can.c and is left
 * untouched; the values below are the additional ones used by the new helpers.)
 */
#define VESC_CAN_PACKET_SET_DUTY            0U
#define VESC_CAN_PACKET_SET_CURRENT         1U
#define VESC_CAN_PACKET_SET_CURRENT_BRAKE   2U

HAL_StatusTypeDef vesc_can_start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef vesc_can_set_erpm(CAN_HandleTypeDef *hcan,
                                    uint8_t controller_id,
                                    int32_t erpm);

/* New helpers (added for the USART command bridge; existing code unchanged). */

/* Set motor current. current_mA is wired as-is (VESC payload = current*1000). */
HAL_StatusTypeDef vesc_can_set_current(CAN_HandleTypeDef *hcan,
                                       uint8_t controller_id,
                                       int32_t current_mA);

/* Set duty cycle. duty_100k is wired as-is (VESC payload = duty*100000,
 * e.g. 50000 -> 0.50). */
HAL_StatusTypeDef vesc_can_set_duty(CAN_HandleTypeDef *hcan,
                                    uint8_t controller_id,
                                    int32_t duty_100k);

/* Set braking current (positive value). current_mA wired as-is. */
HAL_StatusTypeDef vesc_can_set_brake_current(CAN_HandleTypeDef *hcan,
                                             uint8_t controller_id,
                                             int32_t current_mA);

/*
 * Set speed-loop PID. PLACEHOLDER: VESC PID configuration is not a single CAN
 * frame; it must be sent via the multi-frame COMM-over-CAN protocol
 * (COMM_SET_MCCONF using fill_rx_buffer / process_rx_buffer). Returns HAL_ERROR
 * until that protocol is implemented. Declared so the serial command layer and
 * higher-level code can already target it.
 */
HAL_StatusTypeDef vesc_can_set_speed_pid(CAN_HandleTypeDef *hcan,
                                         uint8_t controller_id,
                                         float kp, float ki, float kd);

#endif
