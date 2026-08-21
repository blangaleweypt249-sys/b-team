#ifndef ROAD_H
#define ROAD_H

#include <stdbool.h>

typedef struct
{
    float x_m;
    float y_m;
    float distance_m;
    bool valid;
} road_data_t;

void Road_Init(void);
void Road_Run(void);
void Road_Reset(void);
bool Road_GetData(road_data_t *data);

#endif
