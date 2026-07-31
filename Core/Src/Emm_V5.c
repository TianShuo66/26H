#include "Emm_V5.h"

#define EMM_TX_TIMEOUT_MS  20U
#define EMM_ACK_TIMEOUT_MS 20U
#define EMM_POSITION_RESPONSE_TIMEOUT_MS 25U

typedef enum
{
  EMM_POSITION_REQUEST_IDLE = 0,
  EMM_POSITION_REQUEST_TX_PENDING,
  EMM_POSITION_REQUEST_RX_PENDING,
  EMM_POSITION_REQUEST_READY,
  EMM_POSITION_REQUEST_FAILED
} EmmPositionRequestState_t;

static uint8_t emm_position_command[3];
static uint8_t emm_position_response[8];
static uint32_t emm_position_request_start_ms;
static volatile EmmPositionRequestState_t emm_position_request_state;

static void Emm_V5_Send(const uint8_t *command, uint16_t length,
                        bool receive_ack)
{
  uint8_t ack[4];

  if (HAL_UART_Transmit(&huart2, (uint8_t *)command, length,
                        EMM_TX_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  if (receive_ack)
  {
    (void)HAL_UART_Receive(&huart2, ack, sizeof(ack), EMM_ACK_TIMEOUT_MS);
  }
}

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t param)
{
  uint8_t command[4];
  uint16_t length = 0U;

  command[length++] = addr;
  switch (param)
  {
    case S_VER:   command[length++] = 0x1FU; break;
    case S_RL:    command[length++] = 0x20U; break;
    case S_PID:   command[length++] = 0x21U; break;
    case S_VBUS:  command[length++] = 0x24U; break;
    case S_CPHA:  command[length++] = 0x27U; break;
    case S_ENCL:  command[length++] = 0x31U; break;
    case S_TPOS:  command[length++] = 0x33U; break;
    case S_VEL:   command[length++] = 0x35U; break;
    case S_CPOS:  command[length++] = 0x36U; break;
    case S_PERR:  command[length++] = 0x37U; break;
    case S_FLAG:  command[length++] = 0x3AU; break;
    case S_ORG:   command[length++] = 0x3BU; break;
    case S_CONF:
      command[length++] = 0x42U;
      command[length++] = 0x6CU;
      break;
    case S_STATE:
      command[length++] = 0x43U;
      command[length++] = 0x7AU;
      break;
    default:
      return;
  }
  command[length++] = 0x6BU;
  Emm_V5_Send(command, length, false);
}

bool Emm_V5_Read_Current_Position(uint8_t addr, int32_t *position_counts)
{
  const uint8_t command[3] = {addr, 0x36U, 0x6BU};
  uint8_t response[8];
  uint32_t raw_position;

  if ((position_counts == NULL)
      || (HAL_UART_Transmit(&huart2, (uint8_t *)command, sizeof(command),
                            EMM_TX_TIMEOUT_MS) != HAL_OK)
      || (HAL_UART_Receive(&huart2, response, sizeof(response),
                           EMM_ACK_TIMEOUT_MS) != HAL_OK)
      || (response[0] != addr) || (response[1] != 0x36U)
      || (response[7] != 0x6BU))
  {
    return false;
  }

  raw_position = ((uint32_t)response[3] << 24)
               | ((uint32_t)response[4] << 16)
               | ((uint32_t)response[5] << 8)
               | (uint32_t)response[6];
  if (raw_position > 0x7FFFFFFFUL)
  {
    return false;
  }
  *position_counts = response[2] ? -(int32_t)raw_position : (int32_t)raw_position;
  return true;
}

bool Emm_V5_Request_Current_Position(uint8_t addr)
{
  if ((emm_position_request_state == EMM_POSITION_REQUEST_READY)
      || (emm_position_request_state == EMM_POSITION_REQUEST_FAILED))
  {
    /* A completed response from a stopped task must not start the next task. */
    emm_position_request_state = EMM_POSITION_REQUEST_IDLE;
  }
  if (emm_position_request_state != EMM_POSITION_REQUEST_IDLE)
  {
    return false;
  }
  emm_position_command[0] = addr;
  emm_position_command[1] = 0x36U;
  emm_position_command[2] = 0x6BU;
  emm_position_request_state = EMM_POSITION_REQUEST_TX_PENDING;
  emm_position_request_start_ms = HAL_GetTick();
  if (HAL_UART_Transmit_IT(&huart2, emm_position_command,
                           sizeof(emm_position_command)) != HAL_OK)
  {
    emm_position_request_state = EMM_POSITION_REQUEST_FAILED;
    return false;
  }
  return true;
}

bool Emm_V5_Get_Current_Position_Result(int32_t *position_counts)
{
  uint32_t raw_position;

  if ((position_counts == NULL)
      || (emm_position_request_state != EMM_POSITION_REQUEST_READY))
  {
    return false;
  }
  emm_position_request_state = EMM_POSITION_REQUEST_IDLE;
  if ((emm_position_response[0] != emm_position_command[0])
      || (emm_position_response[1] != 0x36U)
      || (emm_position_response[7] != 0x6BU))
  {
    return false;
  }
  raw_position = ((uint32_t)emm_position_response[3] << 24)
               | ((uint32_t)emm_position_response[4] << 16)
               | ((uint32_t)emm_position_response[5] << 8)
               | (uint32_t)emm_position_response[6];
  if (raw_position > 0x7FFFFFFFUL)
  {
    return false;
  }
  *position_counts = emm_position_response[2] ? -(int32_t)raw_position
                                               : (int32_t)raw_position;
  return true;
}

bool Emm_V5_Current_Position_Request_Failed(void)
{
  if ((emm_position_request_state == EMM_POSITION_REQUEST_TX_PENDING)
      || (emm_position_request_state == EMM_POSITION_REQUEST_RX_PENDING))
  {
    if ((HAL_GetTick() - emm_position_request_start_ms)
        > EMM_POSITION_RESPONSE_TIMEOUT_MS)
    {
      (void)HAL_UART_Abort_IT(&huart2);
      emm_position_request_state = EMM_POSITION_REQUEST_FAILED;
    }
  }
  if (emm_position_request_state != EMM_POSITION_REQUEST_FAILED)
  {
    return false;
  }
  emm_position_request_state = EMM_POSITION_REQUEST_IDLE;
  return true;
}

void Emm_V5_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != &huart2)
      || (emm_position_request_state != EMM_POSITION_REQUEST_TX_PENDING))
  {
    return;
  }
  emm_position_request_state = EMM_POSITION_REQUEST_RX_PENDING;
  if (HAL_UART_Receive_IT(&huart2, emm_position_response,
                          sizeof(emm_position_response)) != HAL_OK)
  {
    emm_position_request_state = EMM_POSITION_REQUEST_FAILED;
  }
}

void Emm_V5_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == &huart2)
      && (emm_position_request_state == EMM_POSITION_REQUEST_RX_PENDING))
  {
    emm_position_request_state = EMM_POSITION_REQUEST_READY;
  }
}

void Emm_V5_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == &huart2)
      && (emm_position_request_state != EMM_POSITION_REQUEST_IDLE))
  {
    emm_position_request_state = EMM_POSITION_REQUEST_FAILED;
  }
}

void Emm_V5_En_Control(uint8_t addr, bool state, bool sync)
{
  const uint8_t command[6] = {
    addr, 0xF3U, 0xABU, (uint8_t)state, (uint8_t)sync, 0x6BU
  };
  Emm_V5_Send(command, sizeof(command), true);
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, bool sync)
{
  const uint8_t command[8] = {
    addr,
    0xF6U,
    dir,
    (uint8_t)(vel >> 8),
    (uint8_t)vel,
    acc,
    (uint8_t)sync,
    0x6BU
  };
  Emm_V5_Send(command, sizeof(command), true);
}

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, uint32_t pulses, bool absolute,
                        bool sync)
{
  const uint8_t command[13] = {
    addr,
    0xFDU,
    dir,
    (uint8_t)(vel >> 8),
    (uint8_t)vel,
    acc,
    (uint8_t)(pulses >> 24),
    (uint8_t)(pulses >> 16),
    (uint8_t)(pulses >> 8),
    (uint8_t)pulses,
    (uint8_t)absolute,
    (uint8_t)sync,
    0x6BU
  };
  Emm_V5_Send(command, sizeof(command), true);
}

void Emm_V5_Stop_Now(uint8_t addr, bool sync)
{
  const uint8_t command[5] = {
    addr, 0xFEU, 0x98U, (uint8_t)sync, 0x6BU
  };
  Emm_V5_Send(command, sizeof(command), true);
}
