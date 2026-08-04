#ifndef PRINT_UTIL_H
#define PRINT_UTIL_H

#include "main.h"

/* Unified console print helpers (all go out over USART1 via usart1_puts).
 * Used everywhere instead of per-file copies. */
void put_i32(int32_t v);
void put_float(float v, uint8_t decimals);

#endif /* PRINT_UTIL_H */
