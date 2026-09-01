/**
 * @file my_main.h
 * @brief 声明用户应用基础硬件初始化接口。
 */

#ifndef MY_MAIN_H
/** 防止 my_main.h 被重复包含。 */
#define MY_MAIN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 功能：执行用户自定义硬件初始化。 */
void My_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_MAIN_H */
