#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

typedef struct
{
    uint16_t left_shoulder;
    uint16_t right_shoulder;
    uint16_t left_x;
    uint16_t left_y;
    uint16_t right_x;
    uint16_t right_y;
} remote_data_t;

extern volatile remote_data_t remote_data;
extern uint8_t remote_adc_error;

void Remote_Init(void);
void Remote_Update(void);

#endif
