#include "oled.h"
#include "i2c.h"

#include <string.h>

#define OLED_I2C_ADDRESS       (0x3CU << 1)
#define OLED_WIDTH             128U
#define OLED_TIME_X            16U
#define OLED_TIME_WIDTH        96U
#define OLED_TIME_FIRST_PAGE   3U
#define OLED_TIME_PAGE_COUNT   3U

static uint8_t oled_buffer[OLED_WIDTH * 8U];

static const uint8_t oled_digit_font[11][5] =
{
  {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
  {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
  {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
  {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
  {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
  {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
  {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
  {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
  {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
  {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
  {0x00U, 0x00U, 0x14U, 0x00U, 0x00U}
};

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
  uint8_t packet[2] = {0x00U, command};

  return HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, packet,
                                 sizeof(packet), 10U);
}

static HAL_StatusTypeDef OLED_WritePage(uint8_t page, uint8_t column,
                                        const uint8_t *data, uint8_t length)
{
  uint8_t packet[1U + OLED_WIDTH];

  if ((OLED_WriteCommand((uint8_t)(0xB0U + page)) != HAL_OK)
      || (OLED_WriteCommand((uint8_t)(column & 0x0FU)) != HAL_OK)
      || (OLED_WriteCommand((uint8_t)(0x10U | (column >> 4U))) != HAL_OK))
  {
    return HAL_ERROR;
  }
  packet[0] = 0x40U;
  (void)memcpy(&packet[1], data, length);
  return HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, packet,
                                 (uint16_t)length + 1U, 10U);
}

static void OLED_SetPixel(uint8_t x, uint8_t y)
{
  if ((x < OLED_WIDTH) && (y < 64U))
  {
    oled_buffer[(uint16_t)(y >> 3U) * OLED_WIDTH + x] |= (uint8_t)(1U << (y & 7U));
  }
}

static void OLED_DrawTimeCharacter(uint8_t character, uint8_t x)
{
  uint8_t glyph = (character == ':') ? 10U : (uint8_t)(character - '0');
  uint8_t column;
  uint8_t row;
  uint8_t dx;
  uint8_t dy;

  for (column = 0U; column < 5U; column++)
  {
    for (row = 0U; row < 7U; row++)
    {
      if ((oled_digit_font[glyph][column] & (1U << row)) == 0U)
      {
        continue;
      }
      for (dx = 0U; dx < 2U; dx++)
      {
        for (dy = 0U; dy < 3U; dy++)
        {
          OLED_SetPixel((uint8_t)(x + column * 2U + dx),
                        (uint8_t)(24U + row * 3U + dy));
        }
      }
    }
  }
}

HAL_StatusTypeDef OLED_Init(void)
{
  uint8_t page;
  uint8_t clear_line[OLED_WIDTH] = {0};

  HAL_Delay(50U);
  if ((OLED_WriteCommand(0xAEU) != HAL_OK)
      || (OLED_WriteCommand(0x20U) != HAL_OK)
      || (OLED_WriteCommand(0x02U) != HAL_OK)
      || (OLED_WriteCommand(0x81U) != HAL_OK)
      || (OLED_WriteCommand(0x7FU) != HAL_OK)
      || (OLED_WriteCommand(0xA1U) != HAL_OK)
      || (OLED_WriteCommand(0xC8U) != HAL_OK)
      || (OLED_WriteCommand(0xA8U) != HAL_OK)
      || (OLED_WriteCommand(0x3FU) != HAL_OK)
      || (OLED_WriteCommand(0xD3U) != HAL_OK)
      || (OLED_WriteCommand(0x00U) != HAL_OK)
      || (OLED_WriteCommand(0xD5U) != HAL_OK)
      || (OLED_WriteCommand(0x80U) != HAL_OK)
      || (OLED_WriteCommand(0xD9U) != HAL_OK)
      || (OLED_WriteCommand(0xF1U) != HAL_OK)
      || (OLED_WriteCommand(0xDAU) != HAL_OK)
      || (OLED_WriteCommand(0x12U) != HAL_OK)
      || (OLED_WriteCommand(0xDBU) != HAL_OK)
      || (OLED_WriteCommand(0x40U) != HAL_OK)
      || (OLED_WriteCommand(0x8DU) != HAL_OK)
      || (OLED_WriteCommand(0x14U) != HAL_OK)
      || (OLED_WriteCommand(0xA4U) != HAL_OK)
      || (OLED_WriteCommand(0xA6U) != HAL_OK)
      || (OLED_WriteCommand(0xAFU) != HAL_OK))
  {
    return HAL_ERROR;
  }
  for (page = 0U; page < 8U; page++)
  {
    if (OLED_WritePage(page, 0U, clear_line, OLED_WIDTH) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef OLED_ShowUptime(uint32_t uptime_ms)
{
  uint32_t total_seconds = uptime_ms / 1000U;
  uint8_t time_text[8];
  uint8_t page;
  uint8_t index;

  total_seconds %= 360000U;
  time_text[0] = (uint8_t)('0' + ((total_seconds / 36000U) % 10U));
  time_text[1] = (uint8_t)('0' + ((total_seconds / 3600U) % 10U));
  time_text[2] = ':';
  time_text[3] = (uint8_t)('0' + ((total_seconds / 600U) % 6U));
  time_text[4] = (uint8_t)('0' + ((total_seconds / 60U) % 10U));
  time_text[5] = ':';
  time_text[6] = (uint8_t)('0' + ((total_seconds / 10U) % 6U));
  time_text[7] = (uint8_t)('0' + (total_seconds % 10U));

  for (page = 0U; page < OLED_TIME_PAGE_COUNT; page++)
  {
    (void)memset(&oled_buffer[(uint16_t)(OLED_TIME_FIRST_PAGE + page)
                              * OLED_WIDTH + OLED_TIME_X], 0, OLED_TIME_WIDTH);
  }
  for (index = 0U; index < sizeof(time_text); index++)
  {
    OLED_DrawTimeCharacter(time_text[index],
                           (uint8_t)(OLED_TIME_X + index * 12U));
  }
  for (page = 0U; page < OLED_TIME_PAGE_COUNT; page++)
  {
    if (OLED_WritePage((uint8_t)(OLED_TIME_FIRST_PAGE + page), OLED_TIME_X,
                       &oled_buffer[(uint16_t)(OLED_TIME_FIRST_PAGE + page)
                                    * OLED_WIDTH + OLED_TIME_X],
                       OLED_TIME_WIDTH) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}
