/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

/* USER CODE BEGIN Private defines */

extern volatile int16_t vision_x_deci_cm;
extern volatile int16_t vision_y_deci_cm;
extern volatile uint8_t vision_data_valid;
extern volatile uint8_t vision_no_detection;
extern volatile uint32_t vision_last_update_ms;
extern volatile uint32_t vision_frame_counter;
extern volatile uint32_t vision_measurement_counter;

typedef enum
{
  CLOSED_LOOP_REJECT_NO_VISION = 0U,
  CLOSED_LOOP_REJECT_VISION_TIMEOUT,
  CLOSED_LOOP_REJECT_POSITION_READ,
  CLOSED_LOOP_REJECT_LOWER_LIMIT,
  CLOSED_LOOP_REJECT_UPPER_LIMIT
} ClosedLoopRejectReason_t;

typedef enum
{
  TASK_EVENT_START = 0U,
  TASK_EVENT_REVERSE,
  TASK_EVENT_COMPLETE,
  TASK_EVENT_HOLD,
  TASK_EVENT_TIMEOUT,
  TASK_EVENT_START_POSITION
} TaskEvent_t;

typedef enum
{
  BALANCE_DEBUG_ENTER = 0U,
  BALANCE_DEBUG_STEP,
  BALANCE_DEBUG_SAVE,
  BALANCE_DEBUG_EXIT,
  BALANCE_DEBUG_POSITION_READ_FAILED
} BalanceDebugEvent_t;

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_UART5_Init(void);
HAL_StatusTypeDef Vision_StartReception(void);
HAL_StatusTypeDef Debug_StartCommandReception(void);
HAL_StatusTypeDef Vehicle_StartCommandReception(void);
uint8_t Debug_GetCommand(void);
uint8_t Vehicle_TakeStartCommand(void);
HAL_StatusTypeDef Vision_DebugPrintLatest(void);
HAL_StatusTypeDef Debug_PrintMotorPosition(int32_t position_counts,
                                           int32_t position_centi_degrees);
HAL_StatusTypeDef Debug_PrintControlState(int16_t ball_x_deci_cm,
                                           int16_t ball_y_deci_cm,
                                           int16_t target_x_deci_cm,
                                           int32_t motor_pulse,
                                           int32_t motor_tilt_target_pulse,
                                           int32_t position_error_deci_cm,
                                           int32_t ball_velocity_deci_cm_per_s,
                                           int32_t velocity_reference_deci_cm_per_s,
                                           int32_t adaptive_tilt_pulse,
                                           uint8_t fast_tilt_tracking,
                                           uint8_t static_compensation_active,
                                           uint8_t micro_adjust_active,
                                           uint8_t capture_braking_active,
                                           int32_t position_counts,
                                           int32_t position_centi_degrees,
                                           uint32_t task_elapsed_ms);
HAL_StatusTypeDef Debug_PrintClosedLoopEnabled(void);
HAL_StatusTypeDef Debug_PrintClosedLoopRejected(ClosedLoopRejectReason_t reason);
HAL_StatusTypeDef Debug_PrintTaskEvent(TaskEvent_t event,
                                       uint32_t task_elapsed_ms);
HAL_StatusTypeDef Debug_PrintBalanceDebug(BalanceDebugEvent_t event,
                                          int32_t position_counts,
                                          int32_t relative_pulse);
HAL_StatusTypeDef Debug_PrintCalibrationState(uint8_t phase,
                                              int32_t target_pulse,
                                              int16_t ball_x_deci_cm,
                                              int32_t ball_velocity_deci_cm_per_s,
                                              int32_t motor_pulse,
                                              uint32_t task_elapsed_ms);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

