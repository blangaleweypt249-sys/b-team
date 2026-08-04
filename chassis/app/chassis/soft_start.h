#ifndef SOFT_START_H
#define SOFT_START_H

#include "main.h"

/*
 * Soft-start (linear ramp) for VESC CAN set-points.
 *
 * Instead of applying a commanded value instantly, the applied value ramps
 * linearly from its current level to the new target over a configurable time,
 * which avoids sudden current / torque steps when starting or changing speed.
 *
 * Behavior:
 *  - Applies to ALL set-point types (RPM / current / duty / brake current).
 *  - When ENABLED and the command TYPE is unchanged, the value ramps from the
 *    current value to the target over `ramp_ms`. The very first command ramps
 *    from 0 (soft start from standstill).
 *  - When DISABLED, or when the command TYPE changes (e.g. current -> rpm), the
 *    new value is applied at once (no cross-type ramp, which would be unit-
 *    meaningless).
 *  - The set-point is (re)sent on CAN at a fixed cadence so the VESC control
 *    stays alive, matching the original 20 ms periodic send.
 *
 * This module only ADDS code; it calls the existing, untouched vesc_can_*
 * helpers. It is inert until wired into serial_cmd.c / main.c (see review).
 */

/* Default ramp duration at init(). */
#define SOFT_START_DEFAULT_RAMP_MS   500U
/* Upper clamp on ramp duration (keeps the int64 ramp math comfortably in range). */
#define SOFT_START_MAX_RAMP_MS       60000U
/* Set-point resend cadence (ms). Keeps VESC control alive, same as demo loop. */
#define SOFT_START_SEND_PERIOD_MS    20U

/* Which VESC set-point the active target refers to. Mirrors the serial verbs. */
typedef enum {
    SOFT_START_TYPE_RPM     = 0,
    SOFT_START_TYPE_CURRENT = 1,
    SOFT_START_TYPE_DUTY    = 2,
    SOFT_START_TYPE_BRAKE   = 3
} soft_start_type_t;

/* Reset to defaults: enabled, ramp_ms = 500, no active target. */
void soft_start_init(void);

/*
 * Register a new target. Ramps when enabled & same type; snaps when disabled or
 * on a type change. The actual CAN frame is sent later by soft_start_tick().
 */
void soft_start_set_target(CAN_HandleTypeDef *hcan,
                           uint8_t controller_id,
                           soft_start_type_t type,
                           int32_t target);

/* Runtime configuration (driven by the serial 'S' command after integration). */
void    soft_start_set_enabled(uint8_t enabled);   /* 1 = on, 0 = off */
void    soft_start_set_ramp_ms(uint32_t ramp_ms);  /* clamped to [1, MAX] */
uint8_t soft_start_get_enabled(void);
uint32_t soft_start_get_ramp_ms(void);

/*
 * Advance the ramp and (re)send the current set-point on `hcan` (CAN1) at the
 * fixed cadence. Call this every main-loop iteration. No-op until a target has
 * been registered.
 */
void soft_start_tick(CAN_HandleTypeDef *hcan);

#endif /* SOFT_START_H */
