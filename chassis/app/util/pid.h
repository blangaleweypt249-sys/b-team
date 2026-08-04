#ifndef PID_H
#define PID_H

#include "main.h"

/*
 * Generic PID controller (not wired to anything yet).
 *
 * Usage:
 *   pid_t c;
 *   pid_init(&c, kp, ki, kd, int_limit, out_min, out_max);
 *   ...
 *   out = pid_step(&c, setpoint, measured, dt_seconds);
 *
 * - Output and integral are clamped (anti-windup).
 * - Derivative is on error (uses previous error); call pid_reset() when the
 *   setpoint jumps to avoid a derivative kick.
 *
 * This module only ADDS code; it does not modify anything else.
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;     /* accumulated integral term */
    float prev_error;   /* previous error, for derivative */
    float int_limit;    /* max |integral|; <= 0 means unlimited */
    float out_min;      /* output lower clamp */
    float out_max;      /* output upper clamp */
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float int_limit, float out_min, float out_max);
void  pid_reset(pid_t *p);                       /* clear integral + prev_error */
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);
float pid_step(pid_t *p, float setpoint, float measured, float dt);

#endif /* PID_H */
