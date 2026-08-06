#ifndef MYMAIN_H
#define MYMAIN_H

#include "dt35.h"

extern DT35_Data dt35_data;
extern HAL_StatusTypeDef dt35_state;

void MyMain_Init(void);
void MyMain_Loop(void);

#endif
