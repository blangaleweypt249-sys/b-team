#include "vesc_can.h"

#define VESC_CAN_PACKET_SET_RPM 3U

HAL_StatusTypeDef vesc_can_start(CAN_HandleTypeDef *hcan)
{
  return hcan != NULL ? HAL_CAN_Start(hcan) : HAL_ERROR;
}

HAL_StatusTypeDef vesc_can_set_erpm(CAN_HandleTypeDef *hcan,
                                    uint8_t controller_id,
                                    int32_t erpm)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  uint32_t value = (uint32_t)erpm;
  uint8_t data[4] = {
    (uint8_t)(value >> 24),
    (uint8_t)(value >> 16),
    (uint8_t)(value >> 8),
    (uint8_t)value
  };

  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  header.ExtId = (VESC_CAN_PACKET_SET_RPM << 8) | controller_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = sizeof(data);

  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

/* ===========================================================================
 *  New helpers for the USART command bridge.
 *  The functions above are untouched; everything below is additive.
 * ===========================================================================*/

/* Send a 4-byte big-endian signed payload as an extended CAN frame with the
 * VESC ID convention: ExtId = (packet_id << 8) | controller_id. */
static HAL_StatusTypeDef vesc_can_send_int32(CAN_HandleTypeDef *hcan,
                                             uint8_t packet_id,
                                             uint8_t controller_id,
                                             int32_t value)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  uint32_t u = (uint32_t)value;
  uint8_t data[4];

  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  data[0] = (uint8_t)(u >> 24);
  data[1] = (uint8_t)(u >> 16);
  data[2] = (uint8_t)(u >> 8);
  data[3] = (uint8_t)(u);

  header.ExtId = ((uint32_t)packet_id << 8) | (uint32_t)controller_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = sizeof(data);

  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef vesc_can_set_current(CAN_HandleTypeDef *hcan,
                                       uint8_t controller_id,
                                       int32_t current_mA)
{
  return vesc_can_send_int32(hcan, VESC_CAN_PACKET_SET_CURRENT,
                             controller_id, current_mA);
}

HAL_StatusTypeDef vesc_can_set_duty(CAN_HandleTypeDef *hcan,
                                    uint8_t controller_id,
                                    int32_t duty_100k)
{
  return vesc_can_send_int32(hcan, VESC_CAN_PACKET_SET_DUTY,
                             controller_id, duty_100k);
}

HAL_StatusTypeDef vesc_can_set_brake_current(CAN_HandleTypeDef *hcan,
                                             uint8_t controller_id,
                                             int32_t current_mA)
{
  return vesc_can_send_int32(hcan, VESC_CAN_PACKET_SET_CURRENT_BRAKE,
                             controller_id, current_mA);
}

HAL_StatusTypeDef vesc_can_set_speed_pid(CAN_HandleTypeDef *hcan,
                                         uint8_t controller_id,
                                         float kp, float ki, float kd)
{
  /* TODO: implement via VESC COMM-over-CAN (fill_rx_buffer + process_rx_buffer
   * carrying COMM_SET_MCCONF). A single CAN frame cannot carry PID config, so
   * this stays a stub until that multi-frame path is added. */
  (void)hcan;
  (void)controller_id;
  (void)kp;
  (void)ki;
  (void)kd;
  return HAL_ERROR;
}
