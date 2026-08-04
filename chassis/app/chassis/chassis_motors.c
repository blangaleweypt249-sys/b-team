#include "chassis_motors.h"
#include "usart.h"
#include "can.h"          /* hcan1 (for sending the ping) */
#include "soft_start.h"   /* soft_start_tick (keep motor alive during probe) */

/* VESC ping packet (VESC 6.x enum). PONG reply is parsed in can_rx.c. */
#define VESC_CAN_PACKET_PING   17U

/* The chassis registry: id -> wheel position. Order matches chassis_wheel_t. */
static const chassis_motor_t s_motors[CHASSIS_MOTOR_COUNT] = {
    { CHASSIS_ID_LF, CHASSIS_WHEEL_LF, "LF / left-front"  },
    { CHASSIS_ID_RF, CHASSIS_WHEEL_RF, "RF / right-front" },
    { CHASSIS_ID_LR, CHASSIS_WHEEL_LR, "LR / left-rear"   },
    { CHASSIS_ID_RR, CHASSIS_WHEEL_RR, "RR / right-rear"  },
};

/* Presence flags, one per registered motor. Written from the CAN RX interrupt,
 * read/cleared from the main-loop probe. Byte-sized -> no tearing. */
static volatile uint8_t s_seen[CHASSIS_MOTOR_COUNT];

const chassis_motor_t *chassis_get_motors(uint8_t *count)
{
  if (count != NULL)
  {
    *count = CHASSIS_MOTOR_COUNT;
  }
  return s_motors;
}

const chassis_motor_t *chassis_find_by_id(uint8_t id)
{
  uint8_t i;
  for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
  {
    if (s_motors[i].id == id)
    {
      return &s_motors[i];
    }
  }
  return NULL;
}

const chassis_motor_t *chassis_find_by_wheel(chassis_wheel_t wheel)
{
  if ((uint8_t)wheel < (uint8_t)CHASSIS_WHEEL_COUNT)
  {
    return &s_motors[(uint8_t)wheel];
  }
  return NULL;
}

uint8_t chassis_id_of(chassis_wheel_t wheel)
{
  const chassis_motor_t *m = chassis_find_by_wheel(wheel);
  return (m != NULL) ? m->id : 0U;
}

void chassis_mark_seen(uint8_t controller_id)
{
  uint8_t i;
  for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
  {
    if (s_motors[i].id == controller_id)
    {
      s_seen[i] = 1U;
      return;
    }
  }
}

/* ---- minimal unsigned-decimal printer (avoids pulling in printf) ---- */
static void put_decimal(uint32_t v)
{
  char tmp[11];
  char out[12];
  uint8_t n = 0U;
  uint8_t i;

  if (v == 0U)
  {
    usart1_puts("0");
    return;
  }

  while (v != 0U && n < (uint8_t)sizeof(tmp))
  {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  }
  for (i = 0U; i < n; i++)
  {
    out[i] = tmp[n - 1U - i];
  }
  out[n] = '\0';
  usart1_puts(out);
}

/* Send a VESC ping to `target_id`. data[0] = sender id (0 = this MCU); the
 * target replies with a PONG whose data[0] is its own controller id. */
static void send_ping(uint8_t target_id)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  uint8_t data[1] = {0U};

  header.ExtId = ((uint32_t)VESC_CAN_PACKET_PING << 8) | (uint32_t)target_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 1U;
  (void)HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

/* Probe a single registry slot: clear its flag, ping, wait (keeping the motor
 * set-point alive), then return 1 if a reply arrived. */
static uint8_t probe_slot(uint8_t idx)
{
  uint32_t start;

  s_seen[idx] = 0U;
  send_ping(s_motors[idx].id);

  start = HAL_GetTick();
  while ((HAL_GetTick() - start) < CHASSIS_PROBE_TIMEOUT_MS)
  {
    /* Keep the soft-start set-point alive so a running motor is not left
     * without commands during the probe window. */
    soft_start_tick(&hcan1);
  }

  return s_seen[idx];
}

void chassis_print_all(void)
{
  uint8_t i;

  usart1_puts("probing ");
  put_decimal(CHASSIS_MOTOR_COUNT);
  usart1_puts(" motors...\r\n");

  for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
  {
    uint8_t online = probe_slot(i);
    usart1_puts("  id ");
    put_decimal(s_motors[i].id);
    usart1_puts(" ");
    usart1_puts(s_motors[i].name);
    usart1_puts(online ? "  [online]\r\n" : "  [LOST]\r\n");
  }
}

void chassis_print_id(uint8_t id)
{
  const chassis_motor_t *m = chassis_find_by_id(id);

  if (m == NULL)
  {
    usart1_puts("id ");
    put_decimal(id);
    usart1_puts(" not registered\r\n");
    return;
  }

  {
    uint8_t idx    = (uint8_t)(m - s_motors);
    uint8_t online = probe_slot(idx);

    usart1_puts("id ");
    put_decimal(m->id);
    usart1_puts(" ");
    usart1_puts(m->name);
    usart1_puts(online ? "  [online]\r\n" : "  [LOST]\r\n");
  }
}
