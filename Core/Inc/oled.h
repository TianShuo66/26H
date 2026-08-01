#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef OLED_Init(void);
HAL_StatusTypeDef OLED_ShowElapsedTime(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif
