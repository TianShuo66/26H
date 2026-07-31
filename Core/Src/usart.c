/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "Emm_V5.h"

/* USER CODE BEGIN 0 */

static uint8_t uart4_rx_byte;
static uint8_t usart1_rx_byte;
static uint8_t vision_rx_line[24];
static uint8_t vision_rx_length;
static uint8_t debug_control_buffer[144];
static volatile uint8_t debug_control_tx_busy;
static volatile uint8_t debug_command;

volatile int16_t vision_x_deci_cm;
volatile int16_t vision_y_deci_cm;
volatile uint8_t vision_data_valid;
volatile uint8_t vision_no_detection;
volatile uint32_t vision_last_update_ms;
volatile uint32_t vision_frame_counter;
volatile uint32_t vision_measurement_counter;

static uint8_t Vision_ParseInt16(uint8_t *index, int16_t *value)
{
  int32_t result = 0;
  int32_t sign = 1;
  uint8_t has_digit = 0U;

  if (vision_rx_line[*index] == '-')
  {
    sign = -1;
    (*index)++;
  }
  while ((*index < vision_rx_length) && (vision_rx_line[*index] >= '0')
         && (vision_rx_line[*index] <= '9'))
  {
    result = result * 10 + (vision_rx_line[*index] - '0');
    if (result > 32768)
    {
      return 0U;
    }
    has_digit = 1U;
    (*index)++;
  }
  if (has_digit == 0U)
  {
    return 0U;
  }
  result *= sign;
  if ((result < -32768) || (result > 32767))
  {
    return 0U;
  }
  *value = (int16_t)result;
  return 1U;
}

static void Vision_ParseLine(void)
{
  uint8_t index = 2U;
  int16_t x;
  int16_t y;

  if ((vision_rx_length == 1U) && (vision_rx_line[0] == 'N'))
  {
    vision_no_detection = 1U;
    vision_frame_counter++;
    return;
  }
  if ((vision_rx_length < 5U) || (vision_rx_line[0] != 'B')
      || (vision_rx_line[1] != ',') || (Vision_ParseInt16(&index, &x) == 0U)
      || (index >= vision_rx_length) || (vision_rx_line[index++] != ',')
      || (Vision_ParseInt16(&index, &y) == 0U) || (index != vision_rx_length))
  {
    return;
  }
  vision_x_deci_cm = x;
  vision_y_deci_cm = y;
  vision_last_update_ms = HAL_GetTick();
  vision_data_valid = 1U;
  vision_no_detection = 0U;
  vision_frame_counter++;
  vision_measurement_counter++;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Emm_V5_UartRxCpltCallback(huart);
    return;
  }
  if (huart->Instance == USART1)
  {
    if (((usart1_rx_byte >= 'A') && (usart1_rx_byte <= 'D'))
        || ((usart1_rx_byte >= 'a') && (usart1_rx_byte <= 'd'))
        || (usart1_rx_byte == 'X') || (usart1_rx_byte == 'x'))
    {
      debug_command = usart1_rx_byte;
    }
    (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
    return;
  }
  if (huart->Instance != UART4)
  {
    return;
  }
  if (uart4_rx_byte == '\n')
  {
    Vision_ParseLine();
    vision_rx_length = 0U;
  }
  else if (uart4_rx_byte != '\r')
  {
    if (vision_rx_length < sizeof(vision_rx_line))
    {
      vision_rx_line[vision_rx_length++] = uart4_rx_byte;
    }
    else
    {
      vision_rx_length = 0U;
    }
  }
  (void)HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Emm_V5_UartTxCpltCallback(huart);
  }
  else if (huart->Instance == USART1)
  {
    debug_control_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Emm_V5_UartErrorCallback(huart);
  }
  else if (huart->Instance == USART1)
  {
    debug_control_tx_busy = 0U;
    (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
  }
}

HAL_StatusTypeDef Vision_StartReception(void)
{
  vision_rx_length = 0U;
  vision_data_valid = 0U;
  vision_no_detection = 1U;
  vision_measurement_counter = 0U;
  return HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1U);
}

HAL_StatusTypeDef Debug_StartCommandReception(void)
{
  debug_command = 0U;
  return HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1U);
}

uint8_t Debug_GetCommand(void)
{
  uint8_t command = debug_command;

  debug_command = 0U;
  return command;
}

HAL_StatusTypeDef Vision_DebugPrintLatest(void)
{
  uint8_t buffer[20];
  uint8_t length = 0U;
  int16_t values[2];
  uint8_t i;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  if (vision_data_valid == 0U)
  {
    buffer[0] = 'N';
    buffer[1] = '\r';
    buffer[2] = '\n';
    return HAL_UART_Transmit(&huart1, buffer, 3U, 10U);
  }

  values[0] = vision_x_deci_cm;
  values[1] = vision_y_deci_cm;
  buffer[length++] = 'B';
  for (i = 0U; i < 2U; i++)
  {
    uint16_t magnitude;
    uint8_t digits[5];
    uint8_t digit_count = 0U;

    buffer[length++] = ',';
    if (values[i] < 0)
    {
      buffer[length++] = '-';
      magnitude = (uint16_t)(-(int32_t)values[i]);
    }
    else
    {
      magnitude = (uint16_t)values[i];
    }
    do
    {
      digits[digit_count++] = (uint8_t)('0' + (magnitude % 10U));
      magnitude /= 10U;
    } while (magnitude > 0U);
    while (digit_count > 0U)
    {
      buffer[length++] = digits[--digit_count];
    }
  }
  buffer[length++] = '\r';
  buffer[length++] = '\n';
  return HAL_UART_Transmit(&huart1, buffer, length, 10U);
}

HAL_StatusTypeDef Debug_PrintClosedLoopEnabled(void)
{
  static const char message[] = "CLOSED_LOOP,ENABLED\r\n";

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  return HAL_UART_Transmit(&huart1, (uint8_t *)message,
                           sizeof(message) - 1U, 10U);
}

HAL_StatusTypeDef Debug_PrintClosedLoopRejected(ClosedLoopRejectReason_t reason)
{
  const char *message;
  uint16_t length;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  switch (reason)
  {
    case CLOSED_LOOP_REJECT_NO_VISION:
      message = "CLOSED_LOOP,REJECT,NO_VISION\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,NO_VISION\r\n") - 1U;
      break;
    case CLOSED_LOOP_REJECT_VISION_TIMEOUT:
      message = "CLOSED_LOOP,REJECT,VISION_TIMEOUT\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,VISION_TIMEOUT\r\n") - 1U;
      break;
    case CLOSED_LOOP_REJECT_POSITION_READ:
      message = "CLOSED_LOOP,REJECT,POSITION_READ\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,POSITION_READ\r\n") - 1U;
      break;
    case CLOSED_LOOP_REJECT_LOWER_LIMIT:
      message = "CLOSED_LOOP,REJECT,LOWER_LIMIT\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,LOWER_LIMIT\r\n") - 1U;
      break;
    case CLOSED_LOOP_REJECT_UPPER_LIMIT:
      message = "CLOSED_LOOP,REJECT,UPPER_LIMIT\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,UPPER_LIMIT\r\n") - 1U;
      break;
    default:
      message = "CLOSED_LOOP,REJECT,UNKNOWN\r\n";
      length = sizeof("CLOSED_LOOP,REJECT,UNKNOWN\r\n") - 1U;
      break;
  }
  return HAL_UART_Transmit(&huart1, (uint8_t *)message, length, 10U);
}

static void Debug_AppendSignedInt32(uint8_t *buffer, uint8_t *length,
                                    int32_t value)
{
  uint8_t digits[10];
  uint8_t digit_count = 0U;
  uint32_t magnitude;

  if (value < 0)
  {
    buffer[(*length)++] = '-';
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }
  do
  {
    digits[digit_count++] = (uint8_t)('0' + (magnitude % 10U));
    magnitude /= 10U;
  } while (magnitude > 0U);
  while (digit_count > 0U)
  {
    buffer[(*length)++] = digits[--digit_count];
  }
}

static void Debug_AppendUnsignedInt32(uint8_t *buffer, uint8_t *length,
                                      uint32_t value)
{
  uint8_t digits[10];
  uint8_t digit_count = 0U;

  do
  {
    digits[digit_count++] = (uint8_t)('0' + (value % 10U));
    value /= 10U;
  } while (value > 0U);
  while (digit_count > 0U)
  {
    buffer[(*length)++] = digits[--digit_count];
  }
}

static void Debug_AppendDeciCm(uint8_t *buffer, uint8_t *length, int16_t value)
{
  uint16_t magnitude;

  if (value < 0)
  {
    buffer[(*length)++] = '-';
    magnitude = (uint16_t)(-(int32_t)value);
  }
  else
  {
    magnitude = (uint16_t)value;
  }
  Debug_AppendSignedInt32(buffer, length, magnitude / 10U);
  buffer[(*length)++] = '.';
  buffer[(*length)++] = (uint8_t)('0' + (magnitude % 10U));
}

HAL_StatusTypeDef Debug_PrintTaskEvent(TaskEvent_t event,
                                       uint32_t task_elapsed_ms)
{
  const char *event_name;
  uint8_t event_length;
  uint8_t *buffer = debug_control_buffer;
  uint8_t length = 0U;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  switch (event)
  {
    case TASK_EVENT_START:
      event_name = "START";
      event_length = sizeof("START") - 1U;
      break;
    case TASK_EVENT_REVERSE:
      event_name = "REVERSE";
      event_length = sizeof("REVERSE") - 1U;
      break;
    case TASK_EVENT_COMPLETE:
      event_name = "COMPLETE";
      event_length = sizeof("COMPLETE") - 1U;
      break;
    case TASK_EVENT_TIMEOUT:
      event_name = "TIMEOUT";
      event_length = sizeof("TIMEOUT") - 1U;
      break;
    case TASK_EVENT_START_POSITION:
      event_name = "REJECT_START_POSITION";
      event_length = sizeof("REJECT_START_POSITION") - 1U;
      break;
    default:
      event_name = "UNKNOWN";
      event_length = sizeof("UNKNOWN") - 1U;
      break;
  }
  buffer[length++] = 'T';
  buffer[length++] = 'A';
  buffer[length++] = 'S';
  buffer[length++] = 'K';
  buffer[length++] = ',';
  while (event_length > 0U)
  {
    buffer[length++] = (uint8_t)*event_name++;
    event_length--;
  }
  buffer[length++] = ',';
  buffer[length++] = 'M';
  buffer[length++] = 'S';
  buffer[length++] = ',';
  Debug_AppendUnsignedInt32(buffer, &length, task_elapsed_ms);
  buffer[length++] = '\r';
  buffer[length++] = '\n';
  debug_control_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart1, buffer, length) != HAL_OK)
  {
    debug_control_tx_busy = 0U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Debug_PrintMotorPosition(int32_t position_counts,
                                           int32_t position_centi_degrees)
{
  uint8_t buffer[48];
  uint8_t length = 0U;
  int32_t whole_degrees = position_centi_degrees / 100;
  int32_t decimal = position_centi_degrees % 100;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  buffer[length++] = 'M';
  buffer[length++] = 'O';
  buffer[length++] = 'T';
  buffer[length++] = 'O';
  buffer[length++] = 'R';
  buffer[length++] = ',';
  buffer[length++] = 'R';
  buffer[length++] = 'A';
  buffer[length++] = 'W';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, position_counts);
  buffer[length++] = ',';
  buffer[length++] = 'A';
  buffer[length++] = 'N';
  buffer[length++] = 'G';
  buffer[length++] = 'L';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  if ((position_centi_degrees < 0) && (whole_degrees == 0))
  {
    buffer[length++] = '-';
  }
  Debug_AppendSignedInt32(buffer, &length, whole_degrees);
  if (decimal < 0)
  {
    decimal = -decimal;
  }
  buffer[length++] = '.';
  buffer[length++] = (uint8_t)('0' + (decimal / 10));
  buffer[length++] = (uint8_t)('0' + (decimal % 10));
  buffer[length++] = '\r';
  buffer[length++] = '\n';
  return HAL_UART_Transmit(&huart1, buffer, length, 10U);
}

HAL_StatusTypeDef Debug_PrintControlState(int16_t ball_x_deci_cm,
                                           int16_t ball_y_deci_cm,
                                           int16_t target_x_deci_cm,
                                           int32_t motor_pulse,
                                           int32_t motor_tilt_target_pulse,
                                           int32_t predicted_stop_distance_deci_cm,
                                           int32_t predicted_error_deci_cm,
                                           uint8_t predicted_brake_active,
                                           uint8_t recovery_active,
                                           int32_t position_counts,
                                           int32_t position_centi_degrees,
                                           uint32_t task_elapsed_ms)
{
  uint8_t *buffer = debug_control_buffer;
  uint8_t length = 0U;
  int32_t whole_degrees = position_centi_degrees / 100;
  int32_t decimal = position_centi_degrees % 100;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  buffer[length++] = 'C';
  buffer[length++] = 'O';
  buffer[length++] = 'N';
  buffer[length++] = 'T';
  buffer[length++] = 'R';
  buffer[length++] = 'O';
  buffer[length++] = 'L';
  buffer[length++] = ',';
  buffer[length++] = 'X';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, ball_x_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'Y';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, ball_y_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'R';
  buffer[length++] = 'E';
  buffer[length++] = 'F';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, target_x_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'S';
  buffer[length++] = 'T';
  buffer[length++] = 'O';
  buffer[length++] = 'P';
  buffer[length++] = '_';
  buffer[length++] = 'D';
  buffer[length++] = 'I';
  buffer[length++] = 'S';
  buffer[length++] = 'T';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, predicted_stop_distance_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'P';
  buffer[length++] = 'R';
  buffer[length++] = 'E';
  buffer[length++] = 'D';
  buffer[length++] = '_';
  buffer[length++] = 'E';
  buffer[length++] = 'R';
  buffer[length++] = 'R';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, predicted_error_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'B';
  buffer[length++] = 'R';
  buffer[length++] = 'A';
  buffer[length++] = 'K';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  Debug_AppendUnsignedInt32(buffer, &length, predicted_brake_active);
  buffer[length++] = ',';
  buffer[length++] = 'R';
  buffer[length++] = 'E';
  buffer[length++] = 'C';
  buffer[length++] = 'O';
  buffer[length++] = 'V';
  buffer[length++] = 'E';
  buffer[length++] = 'R';
  buffer[length++] = 'Y';
  buffer[length++] = ',';
  Debug_AppendUnsignedInt32(buffer, &length, recovery_active);
  buffer[length++] = ',';
  buffer[length++] = 'P';
  buffer[length++] = 'U';
  buffer[length++] = 'L';
  buffer[length++] = 'S';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, motor_pulse);
  buffer[length++] = ',';
  buffer[length++] = 'T';
  buffer[length++] = 'I';
  buffer[length++] = 'L';
  buffer[length++] = 'T';
  buffer[length++] = '_';
  buffer[length++] = 'R';
  buffer[length++] = 'E';
  buffer[length++] = 'F';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, motor_tilt_target_pulse);
  buffer[length++] = ',';
  buffer[length++] = 'R';
  buffer[length++] = 'A';
  buffer[length++] = 'W';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, position_counts);
  buffer[length++] = ',';
  buffer[length++] = 'A';
  buffer[length++] = 'N';
  buffer[length++] = 'G';
  buffer[length++] = 'L';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  if ((position_centi_degrees < 0) && (whole_degrees == 0))
  {
    buffer[length++] = '-';
  }
  Debug_AppendSignedInt32(buffer, &length, whole_degrees);
  if (decimal < 0)
  {
    decimal = -decimal;
  }
  buffer[length++] = '.';
  buffer[length++] = (uint8_t)('0' + (decimal / 10));
  buffer[length++] = (uint8_t)('0' + (decimal % 10));
  buffer[length++] = ',';
  buffer[length++] = 'T';
  buffer[length++] = 'A';
  buffer[length++] = 'S';
  buffer[length++] = 'K';
  buffer[length++] = '_';
  buffer[length++] = 'M';
  buffer[length++] = 'S';
  buffer[length++] = ',';
  Debug_AppendUnsignedInt32(buffer, &length, task_elapsed_ms);
  buffer[length++] = '\r';
  buffer[length++] = '\n';
  debug_control_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart1, buffer, length) != HAL_OK)
  {
    debug_control_tx_busy = 0U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Debug_PrintCalibrationState(uint8_t phase,
                                              int32_t target_pulse,
                                              int16_t ball_x_deci_cm,
                                              int32_t ball_velocity_deci_cm_per_s,
                                              int32_t motor_pulse,
                                              uint32_t task_elapsed_ms)
{
  uint8_t *buffer = debug_control_buffer;
  uint8_t length = 0U;

  if (debug_control_tx_busy != 0U)
  {
    return HAL_BUSY;
  }
  buffer[length++] = 'C';
  buffer[length++] = 'A';
  buffer[length++] = 'L';
  buffer[length++] = ',';
  buffer[length++] = 'P';
  buffer[length++] = 'H';
  buffer[length++] = 'A';
  buffer[length++] = 'S';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  buffer[length++] = phase;
  buffer[length++] = ',';
  buffer[length++] = 'T';
  buffer[length++] = 'A';
  buffer[length++] = 'R';
  buffer[length++] = 'G';
  buffer[length++] = 'E';
  buffer[length++] = 'T';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, target_pulse);
  buffer[length++] = ',';
  buffer[length++] = 'X';
  buffer[length++] = ',';
  Debug_AppendDeciCm(buffer, &length, ball_x_deci_cm);
  buffer[length++] = ',';
  buffer[length++] = 'V';
  buffer[length++] = 'E';
  buffer[length++] = 'L';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, ball_velocity_deci_cm_per_s);
  buffer[length++] = ',';
  buffer[length++] = 'P';
  buffer[length++] = 'U';
  buffer[length++] = 'L';
  buffer[length++] = 'S';
  buffer[length++] = 'E';
  buffer[length++] = ',';
  Debug_AppendSignedInt32(buffer, &length, motor_pulse);
  buffer[length++] = ',';
  buffer[length++] = 'M';
  buffer[length++] = 'S';
  buffer[length++] = ',';
  Debug_AppendUnsignedInt32(buffer, &length, task_elapsed_ms);
  buffer[length++] = '\r';
  buffer[length++] = '\n';
  debug_control_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart1, buffer, length) != HAL_OK)
  {
    debug_control_tx_busy = 0U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

/* USER CODE END 1 */
