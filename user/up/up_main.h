#ifndef UP_MAIN_H
#define UP_MAIN_H

#include "c610_2006.h"
#include "dm_app.h"
#include "rs_app.h"

#include <stdbool.h>
#include <stdint.h>

#define RS_MOTOR_L_ID 39U
#define RS_MOTOR_F_ID 40U
#define DM_MOTOR_L_ID 0x05U
#define DM_MOTOR_F_ID 0x07U
#define M2006_MOTOR_L_ID 1U
#define M2006_MOTOR_F_ID 2U

HAL_StatusTypeDef Up_MainInit(void);
HAL_StatusTypeDef Up_MainInitializeMotors(void);
void Up_MainRun1ms(void);
void Up_MainStopAll(void);
bool Up_MainGetRsStatus(uint8_t id, rs_app_status_t *status);
bool Up_MainGetDmStatus(uint16_t id, dm_app_status_t *status);
bool Up_MainGetM2006Status(uint8_t id, m2006_status_t *status);

#endif
