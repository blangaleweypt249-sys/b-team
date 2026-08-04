#include "soft_start.h"
#include "vesc_can.h"

/* ---- state ---- */
static uint8_t           s_enabled;       /* 1 = ramp on */
static uint32_t          s_ramp_ms;       /* ramp duration */

static uint8_t           s_have_target;   /* a command received at least once */
static soft_start_type_t s_type;          /* active set-point type */
static uint8_t           s_id;            /* VESC controller id */

static int32_t           s_cur;           /* currently applied value */
static int32_t           s_target;        /* commanded target */
static int32_t           s_ramp_start;    /* value at the start of the ramp */
static uint32_t          s_t0;            /* HAL tick at ramp start */
static uint8_t           s_ramping;       /* 1 while a ramp is in progress */

static uint32_t          s_last_send;     /* HAL tick of last CAN send */

void soft_start_init(void)
{
  s_enabled     = 1U;
  s_ramp_ms     = SOFT_START_DEFAULT_RAMP_MS;
  s_have_target = 0U;
  s_type        = SOFT_START_TYPE_RPM;
  s_id          = 0U;
  s_cur         = 0;
  s_target      = 0;
  s_ramp_start  = 0;
  s_t0          = 0U;
  s_ramping     = 0U;
  s_last_send   = 0U;
}

void soft_start_set_enabled(uint8_t enabled)
{
  s_enabled = enabled ? 1U : 0U;
  /* When disabled the target should take effect immediately. */
  if (!s_enabled)
  {
    s_cur     = s_target;
    s_ramping = 0U;
  }
}

void soft_start_set_ramp_ms(uint32_t ramp_ms)
{
  if (ramp_ms == 0U)
  {
    ramp_ms = 1U;                       /* avoid div-by-zero (1 ms ~= snap) */
  }
  if (ramp_ms > SOFT_START_MAX_RAMP_MS)
  {
    ramp_ms = SOFT_START_MAX_RAMP_MS;
  }
  s_ramp_ms = ramp_ms;
  /* The active ramp is re-evaluated each tick with the new duration, so a
   * mid-ramp change rescales smoothly. */
}

uint8_t  soft_start_get_enabled(void)  { return s_enabled; }
uint32_t soft_start_get_ramp_ms(void)  { return s_ramp_ms; }

void soft_start_set_target(CAN_HandleTypeDef *hcan,
                           uint8_t controller_id,
                           soft_start_type_t type,
                           int32_t target)
{
  (void)hcan;   /* CAN frames are sent from soft_start_tick() */

  s_id = controller_id;

  if (!s_enabled)
  {
    /* Disabled: apply at once. */
    s_cur = target;
    s_target = target;
    s_ramp_start = target;
    s_ramping = 0U;
  }
  else if (s_have_target && (type != s_type))
  {
    /* Variable changed: ramping across units is meaningless -> apply at once. */
    s_cur = target;
    s_target = target;
    s_ramp_start = target;
    s_ramping = 0U;
  }
  else
  {
    /* Enabled, same type (or first command): ramp from the current value.
     * First command ramps from 0 (s_cur starts at 0). */
    s_ramp_start = s_cur;
    s_target     = target;
    s_t0         = HAL_GetTick();
    s_ramping    = 1U;
  }

  s_type        = type;
  s_have_target = 1U;
}

void soft_start_tick(CAN_HandleTypeDef *hcan)
{
  uint32_t now;
  uint32_t elapsed;

  if (hcan == NULL || !s_have_target)
  {
    return;
  }

  now = HAL_GetTick();

  /* Advance the ramp. */
  if (s_ramping)
  {
    elapsed = now - s_t0;
    if (elapsed >= s_ramp_ms)
    {
      s_cur     = s_target;
      s_ramping = 0U;
    }
    else
    {
      /* Linear interpolation: cur = start + (target - start) * elapsed / ramp.
       * int64 avoids overflow for large eRPM / mA values. */
      int64_t delta = (int64_t)(s_target - s_ramp_start) * (int64_t)elapsed;
      s_cur = s_ramp_start + (int32_t)(delta / (int64_t)s_ramp_ms);
    }
  }

  /* Resend the set-point at the fixed cadence to keep VESC control alive. */
  if ((now - s_last_send) >= SOFT_START_SEND_PERIOD_MS)
  {
    s_last_send = now;
    switch (s_type)
    {
      case SOFT_START_TYPE_RPM:
        vesc_can_set_erpm(hcan, s_id, s_cur);
        break;
      case SOFT_START_TYPE_CURRENT:
        vesc_can_set_current(hcan, s_id, s_cur);
        break;
      case SOFT_START_TYPE_DUTY:
        vesc_can_set_duty(hcan, s_id, s_cur);
        break;
      case SOFT_START_TYPE_BRAKE:
        vesc_can_set_brake_current(hcan, s_id, s_cur);
        break;
      default:
        break;
    }
  }
}
