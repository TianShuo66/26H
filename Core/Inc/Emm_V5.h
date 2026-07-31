#ifndef __EMM_V5_H
#define __EMM_V5_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "usart.h"

typedef enum
{
  S_VER = 0,
  S_RL = 1,
  S_PID = 2,
  S_VBUS = 3,
  S_CPHA = 5,
  S_ENCL = 7,
  S_TPOS = 8,
  S_VEL = 9,
  S_CPOS = 10,
  S_PERR = 11,
  S_FLAG = 13,
  S_CONF = 14,
  S_STATE = 15,
  S_ORG = 16
} SysParams_t;

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t param);
bool Emm_V5_Read_Current_Position(uint8_t addr, int32_t *position_counts);
bool Emm_V5_Request_Current_Position(uint8_t addr);
bool Emm_V5_Get_Current_Position_Result(int32_t *position_counts);
bool Emm_V5_Current_Position_Request_Failed(void);
void Emm_V5_UartTxCpltCallback(UART_HandleTypeDef *huart);
void Emm_V5_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Emm_V5_UartErrorCallback(UART_HandleTypeDef *huart);
void Emm_V5_En_Control(uint8_t addr, bool state, bool sync);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, bool sync);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, uint32_t pulses, bool absolute,
                        bool sync);
void Emm_V5_Stop_Now(uint8_t addr, bool sync);

#ifdef __cplusplus
}
#endif

#endif
