#include "serial_cmd.h"
#include "usart.h"
#include "can.h"
#include "vesc_can.h"
#include "soft_start.h"
#include "chassis_motors.h"
#include "encoder.h"
#include "imu.h"
#include "odom_fusion.h"
#include "print_util.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

static long parse_dec(const char *s, int *ok)
{
  char *end = NULL;
  long v;

  if (s == NULL || *s == '\0')
  {
    *ok = 0;
    return 0;
  }

  v = strtol(s, &end, 10);
  *ok = (end != s) && (*end == '\0');
  return v;
}

static void cmd_err(const char *msg)
{
  usart1_puts("ERR ");
  usart1_puts(msg);
  usart1_puts("\r\n");
}

void serial_cmd_init(void)
{
  usart1_puts("\r\n");
  usart1_puts("U12 VESC serial console ready.\r\n");
  usart1_puts("  R,<id>,<erpm>\r\n");
  usart1_puts("  C,<id>,<current_mA>\r\n");
  usart1_puts("  D,<id>,<duty*1e5>\r\n");
  usart1_puts("  B,<id>,<brake_mA>\r\n");
  usart1_puts("  P,<id>,<kp>,<ki>,<kd>  (PID stub, not implemented)\r\n");
  usart1_puts("  S,<0|1>[,<ms>]         soft-start on/off [+ramp ms, def 500]\r\n");
  usart1_puts("  M                      list chassis motors (id -> wheel)\r\n");
  usart1_puts("  M,<id>                 query one motor's wheel position\r\n");
  usart1_puts("  E                      list wheel encoders (rpm/cur/duty/pos)\r\n");
  usart1_puts("  E,<id>                 query one wheel's encoder\r\n");
  usart1_puts("  D                      RX diag: STATUS_1/5 frame counts + Hz\r\n");
  usart1_puts("  D,<id>                 RX diag for one wheel\r\n");
  usart1_puts("  I                      stream IMU yaw/gyro/pos x50\r\n");
  usart1_puts("  O                      stream odometry (enc+IMU) x50\r\n");
}

/* Runs in main-loop context (NOT an ISR), so non-reentrant strtok() is safe. */
void serial_cmd_poll(void)
{
  static char line[USART1_RX_LINE_MAX];
  char *tok_cmd;
  char *tok_id;
  char *tok_v1;
  int   ok;
  long  id;
  long  v1;
  char  cmd;

  if (!usart1_getline(line, sizeof(line)))
  {
    return;
  }

  tok_cmd = strtok(line, ",");
  if (tok_cmd == NULL || tok_cmd[0] == '\0')
  {
    cmd_err("empty");
    return;
  }

  cmd = (char)toupper((unsigned char)tok_cmd[0]);

  tok_id = strtok(NULL, ",");
  tok_v1 = strtok(NULL, ",");

  /* Chassis motor query: 'M' lists all; 'M,<id>' queries one id. Responds on
   * serial with the registered id -> wheel position. */
  if (cmd == 'M')
  {
    if (tok_id == NULL)
    {
      chassis_print_all();
    }
    else
    {
      long qid = parse_dec(tok_id, &ok);
      if (!ok || qid < 0 || qid > 255)
      {
        cmd_err("bad id");
        return;
      }
      chassis_print_id((uint8_t)qid);
    }
    return;
  }

  /* Encoder feedback: 'E' lists all wheels; 'E,<id>' queries one. */
  if (cmd == 'E')
  {
    if (tok_id == NULL)
    {
      encoder_print_all();
    }
    else
    {
      long qid = parse_dec(tok_id, &ok);
      if (!ok || qid < 0 || qid > 255)
      {
        cmd_err("bad id");
        return;
      }
      encoder_print((uint8_t)qid);
    }
    return;
  }

  /* RX diagnostic: 'D' counts STATUS_1/STATUS_5 frames per wheel (rate since last D). */
  if (cmd == 'D')
  {
    if (tok_id == NULL)
    {
      encoder_diag_print_all();
    }
    else
    {
      long qid = parse_dec(tok_id, &ok);
      if (!ok || qid < 0 || qid > 255)
      {
        cmd_err("bad id");
        return;
      }
      encoder_diag_print((uint8_t)qid);
    }
    return;
  }

  /* IMU data: 'I' streams yaw/gyro/pos 50 times (~50ms apart). */
  if (cmd == 'I')
  {
    int n;
    for (n = 0; n < 50; n++)
    {
      usart1_puts("imu: yaw ");
      put_float(Imu_GetYaw(), 2);
      usart1_puts(" deg, gyro_z ");
      put_float(Imu_GetGyroZ(), 2);
      usart1_puts(" deg/s, pos (");
      put_float(Imu_GetPosX(), 3);
      usart1_puts(", ");
      put_float(Imu_GetPosY(), 3);
      usart1_puts(") m\r\n");
      { uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 50U) { soft_start_tick(&hcan1); } }
    }
    return;
  }

  /* Odometry (encoder+IMU fusion): 'O' streams x/y/yaw/vel 50 times. */
  if (cmd == 'O')
  {
    int n;
    for (n = 0; n < 50; n++)
    {
      odom_print();
      { uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 50U) { soft_start_tick(&hcan1); odom_tick(); } }
    }
    return;
  }

  if (tok_id == NULL)
  {
    cmd_err("missing id");
    return;
  }

  id = parse_dec(tok_id, &ok);
  if (!ok || id < 0 || id > 255)
  {
    cmd_err("bad id");
    return;
  }

  /* Soft-start configuration: S,<0|1>[,<ms>]  e.g. S,1,500 / S,0.
   * tok_id is the enable flag (already parsed as `id`); tok_v1 is the optional
   * ramp time in ms. Placed before the value-presence check so ms is optional. */
  if (cmd == 'S')
  {
    if (tok_v1 != NULL)
    {
      long ms = parse_dec(tok_v1, &ok);
      if (!ok || ms < 0 || ms > 60000)
      {
        cmd_err("bad ramp ms");
        return;
      }
      soft_start_set_ramp_ms((uint32_t)ms);
    }
    soft_start_set_enabled(id ? 1U : 0U);
    usart1_puts("OK soft\r\n");
    return;
  }

  /* PID is a multi-frame VESC COMM command; reserved as a stub for now. */
  if (cmd == 'P')
  {
    usart1_puts("WARN PID stub: not implemented\r\n");
    return;
  }

  if (tok_v1 == NULL)
  {
    cmd_err("missing value");
    return;
  }

  v1 = parse_dec(tok_v1, &ok);
  if (!ok)
  {
    cmd_err("bad value");
    return;
  }

  /* Route the set-point through the soft-start ramp. The actual CAN frame is
   * sent periodically by soft_start_tick() from the main loop. */
  switch (cmd)
  {
    case 'R':
      soft_start_set_target(&hcan1, (uint8_t)id, SOFT_START_TYPE_RPM, (int32_t)v1);
      break;
    case 'C':
      soft_start_set_target(&hcan1, (uint8_t)id, SOFT_START_TYPE_CURRENT, (int32_t)v1);
      break;
    case 'D':
      soft_start_set_target(&hcan1, (uint8_t)id, SOFT_START_TYPE_DUTY, (int32_t)v1);
      break;
    case 'B':
      soft_start_set_target(&hcan1, (uint8_t)id, SOFT_START_TYPE_BRAKE, (int32_t)v1);
      break;
    default:
      cmd_err("unknown cmd");
      return;
  }

  usart1_puts("OK\r\n");
}
