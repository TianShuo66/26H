#include "i2c.h"

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000U;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0U;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0U;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c_handle)
{
  GPIO_InitTypeDef gpio_init = {0};

  if (i2c_handle->Instance != I2C2)
  {
    return;
  }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  gpio_init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  gpio_init.Mode = GPIO_MODE_AF_OD;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio_init);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2c_handle)
{
  if (i2c_handle->Instance != I2C2)
  {
    return;
  }
  __HAL_RCC_I2C2_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
}
