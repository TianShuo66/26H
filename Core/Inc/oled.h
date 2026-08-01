#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef OLED_Init(void);
HAL_StatusTypeDef OLED_ShowUptime(uint32_t uptime_ms);

#ifdef __cplusplus
}
#endif

#endif
