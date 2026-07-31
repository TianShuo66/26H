/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Emm_V5.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  CALIBRATION_IDLE = 0,
  CALIBRATION_MOVE,
  CALIBRATION_HOLD,
  CALIBRATION_RETURN
} CalibrationState_t;

typedef enum
{
  TASK_IDLE = 0,
  TASK_TO_POSITIVE,
  TASK_TO_NEGATIVE,
  TASK_COMPLETE
} TaskState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 电机通讯地址
#define MOTOR_ADDRESS             1U
// 电机每转脉冲数（驱动器脉冲当量）
#define MOTOR_PULSES_PER_REVOLUTION 3200U
// 电机位置反馈寄存器一圈计数值（编码器/总线位置单位）
#define MOTOR_POSITION_COUNTS_PER_REVOLUTION 65536L
// 驱动器上电后位置反馈从零开始；机构上电位置即物理水平零点。
#define MOTOR_FIXED_ZERO_COUNTS              0L
// 电机位置查询周期 40ms
#define MOTOR_POSITION_QUERY_PERIOD_MS 40U
// 位置数据最大有效时长，超过视为过期 120ms
#define MOTOR_POSITION_MAX_AGE_MS     120U
// 电机控制运算周期 40ms
#define MOTOR_CONTROL_PERIOD_MS       40U
// 短距离微动最大脉冲
#define MOTOR_SHORT_STEP_MAX_PULSES   12
// 短步运动转速 1000RPM
#define MOTOR_SHORT_STEP_SPEED_RPM    1000U
// 短步加减速参数
#define MOTOR_SHORT_STEP_ACCELERATION 100U
// 方向反转时的反制动速度与加减速参数
#define MOTOR_REVERSE_BRAKE_SPEED_RPM 1800U
#define MOTOR_REVERSE_BRAKE_ACCELERATION 200U
#define MOTOR_REVERSE_BRAKE_MAX_STEP_PULSES 24
// 软件行程限位脉冲
#define MOTOR_SOFTWARE_LIMIT_PULSES   230
// 倾斜目标脉冲上限
#define MOTOR_TILT_TARGET_LIMIT_PULSES 160
// 静态维持倾斜脉冲
#define MOTOR_STATIC_TILT_PULSES       100
// 静态偏置调节阈值脉冲
#define MOTOR_STATIC_BIAS_LIMIT_PULSES 60
// 静态偏置单次调节脉冲步长
#define MOTOR_STATIC_BIAS_STEP_PULSES  10
// 静态偏置调节周期 80ms
#define MOTOR_STATIC_BIAS_PERIOD_MS    80U

// 视觉图像超时阈值，无目标判定失效 180ms
#define VISION_TIMEOUT_MS             500U
// 小球相邻帧最大位移阈值 单位：0.1cm
#define BALL_MAX_FRAME_CHANGE_DECI_CM 30
// 小球允许最大速度 单位：0.1cm/s
#define BALL_MAX_VELOCITY_DECI_CM_S   400L
// 小球位置死区 0.1cm
#define BALL_POSITION_DEADBAND_DECI_CM 10
// 小球速度死区 0.1cm/s
#define BALL_VELOCITY_DEADBAND_DECI_CM_S 20L
#define BALL_SETTLED_VELOCITY_DECI_CM_S 5L
#define BALL_STATIC_TRIGGER_DECI_CM   10
// 位置环增益分子
#define BALL_POSITION_GAIN_NUMERATOR  100L
// 速度环增益分子
#define BALL_VELOCITY_GAIN_NUMERATOR  45L
// 倾斜控制增益分母
#define BALL_TILT_GAIN_DIVISOR        100L
// 钢球速度超过该值视为运动状态，单位：0.1cm/s
#define BALL_MOTION_THRESHOLD_DECI_CM_S 10L
// 终点小误差下的最大倾角，避免低速来回冲击
#define MOTOR_TERMINAL_TILT_LIMIT_PULSES 80
#define TASK_POSITIVE_TARGET_DECI_CM    50
#define TASK_NEGATIVE_TARGET_DECI_CM   (-50)
#define TASK_POSITIVE_REVERSE_DECI_CM   45
#define TASK_START_POSITION_TOLERANCE_DECI_CM 15
#define TASK_SETTLED_POSITION_TOLERANCE_DECI_CM 10
#define TASK_SETTLED_VELOCITY_DECI_CM_S 5L
#define TASK_SETTLED_HOLD_MS            120U
#define TASK_MAX_DURATION_MS           5000U
#define TASK_REVERSE_BOOST_MS            900U
#define CALIBRATION_HOLD_MS            1000U
#define CALIBRATION_MOVE_TIMEOUT_MS    1500U
#define CALIBRATION_PULSE_TOLERANCE    2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t ReadPressedKeys(void)
{
  uint8_t keys = 0U;

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) { keys |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET) { keys |= 0x02U; }
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET) { keys |= 0x04U; }
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) { keys |= 0x08U; }
  return keys;
}

static int32_t ClampInt32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static int32_t AbsInt32(int32_t value)
{
  if (value < 0)
  {
    return -value;
  }
  return value;
}

static void EnableMotor(uint8_t *motor_enabled)
{
  if (*motor_enabled == 0U)
  {
    Emm_V5_En_Control(MOTOR_ADDRESS, true, false);
    *motor_enabled = 1U;
  }
}

static void StopAndDisableMotor(uint8_t *motor_enabled)
{
  Emm_V5_Stop_Now(MOTOR_ADDRESS, false);
  if (*motor_enabled != 0U)
  {
    Emm_V5_En_Control(MOTOR_ADDRESS, false, false);
    *motor_enabled = 0U;
  }
}

static int32_t MotorPositionToPulse(int32_t position_counts,
                                    int32_t motor_zero_counts)
{
  return ((position_counts - motor_zero_counts)
          * (int32_t)MOTOR_PULSES_PER_REVOLUTION)
         / MOTOR_POSITION_COUNTS_PER_REVOLUTION;
}

static uint8_t SendShortRelativePulse(int32_t current_pulse, int32_t step,
                                      uint8_t reverse_braking)
{
  uint32_t pulses;
  uint8_t direction;

  if ((step == 0) || ((current_pulse + step) > MOTOR_SOFTWARE_LIMIT_PULSES)
      || ((current_pulse + step) < -MOTOR_SOFTWARE_LIMIT_PULSES))
  {
    return 0U;
  }

  direction = (step > 0) ? 0U : 1U;
  pulses = (uint32_t)AbsInt32(step);
  Emm_V5_Pos_Control(MOTOR_ADDRESS, direction,
                      (reverse_braking != 0U) ? MOTOR_REVERSE_BRAKE_SPEED_RPM
                                               : MOTOR_SHORT_STEP_SPEED_RPM,
                      (reverse_braking != 0U)
                        ? MOTOR_REVERSE_BRAKE_ACCELERATION
                        : MOTOR_SHORT_STEP_ACCELERATION,
                      pulses, false, false);
  return 1U;
}

static uint8_t UpdateBallEstimate(uint32_t *last_measurement_counter,
                                  uint32_t *last_sample_ms,
                                  uint8_t *ball_state_valid,
                                  int16_t *ball_x_est_deci_cm,
                                  int32_t *ball_velocity_deci_cm_per_s)
{
  uint32_t measurement_counter;
  uint32_t sample_ms;
  uint32_t dt_ms;
  int16_t x_meas;
  int32_t x_change;
  int32_t velocity_raw;

  measurement_counter = vision_measurement_counter;
  if (measurement_counter == *last_measurement_counter)
  {
    return 0U;
  }
  x_meas = vision_x_deci_cm;
  sample_ms = vision_last_update_ms;
  if (measurement_counter != vision_measurement_counter)
  {
    return 0U;
  }

  *last_measurement_counter = measurement_counter;
  if (*ball_state_valid == 0U)
  {
    *ball_x_est_deci_cm = x_meas;
    *ball_velocity_deci_cm_per_s = 0;
    *last_sample_ms = sample_ms;
    *ball_state_valid = 1U;
    return 1U;
  }
  dt_ms = sample_ms - *last_sample_ms;
  if ((dt_ms == 0U) || (dt_ms > 200U))
  {
    *ball_x_est_deci_cm = x_meas;
    *ball_velocity_deci_cm_per_s = 0;
    *last_sample_ms = sample_ms;
    return 1U;
  }
  x_change = (int32_t)x_meas - *ball_x_est_deci_cm;
  if (AbsInt32(x_change) > BALL_MAX_FRAME_CHANGE_DECI_CM)
  {
    return 0U;
  }
  velocity_raw = ClampInt32((x_change * 1000L) / (int32_t)dt_ms,
                             -BALL_MAX_VELOCITY_DECI_CM_S,
                             BALL_MAX_VELOCITY_DECI_CM_S);
  *ball_velocity_deci_cm_per_s = ((*ball_velocity_deci_cm_per_s * 65L)
                                  + (velocity_raw * 35L)) / 100L;
  *ball_x_est_deci_cm = x_meas;
  *last_sample_ms = sample_ms;
  return 1U;
}

static int32_t CalibrationTargetFromCommand(uint8_t command)
{
  switch (command)
  {
    case 'A': case 'a': return 20;
    case 'B': case 'b': return -20;
    case 'C': case 'c': return 40;
    case 'D': case 'd': return -40;
    default: return 0;
  }
}

static uint8_t CalibrationPhaseCode(CalibrationState_t state)
{
  switch (state)
  {
    case CALIBRATION_MOVE: return 'M';
    case CALIBRATION_HOLD: return 'H';
    case CALIBRATION_RETURN: return 'R';
    default: return 'I';
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t last_key_action = 0U;
  uint32_t last_control_update = 0U;
  uint32_t closed_loop_start_ms = 0U;
  uint32_t last_position_query = 0U;
  uint32_t last_motor_position_ms = 0U;
  uint32_t last_vision_measurement_counter = 0U;
  uint32_t last_ball_sample_ms = 0U;
  uint32_t last_static_bias_update = 0U;
  uint32_t task_start_ms = 0U;
  uint32_t task_settled_start_ms = 0U;
  uint32_t task_reverse_boost_until_ms = 0U;
  uint32_t calibration_start_ms = 0U;
  uint32_t calibration_phase_start_ms = 0U;
  int32_t motor_position_counts = 0;
  int32_t motor_zero_counts = MOTOR_FIXED_ZERO_COUNTS;
  int32_t motor_pulse_est = 0;
  int32_t motor_tilt_target_pulse = 0;
  int32_t motor_static_bias_pulse = 0;
  int32_t ball_velocity_deci_cm_per_s = 0;
  int16_t ball_x_est_deci_cm = 0;
  int16_t target_x_deci_cm = 0;
  int32_t calibration_target_pulse = 0;
  uint8_t previous_keys;
  uint8_t motor_enabled = 0U;
  uint8_t motor_position_valid = 0U;
  uint8_t closed_loop_enabled = 0U;
  uint8_t ball_state_valid = 0U;
  uint8_t motor_position_request_pending = 0U;
  uint8_t previous_command_direction = 0U;
  uint8_t previous_command_active = 0U;
  CalibrationState_t calibration_state = CALIBRATION_IDLE;
  TaskState_t task_state = TASK_IDLE;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  if (Vision_StartReception() != HAL_OK)
  {
    Error_Handler();
  }
  if (Debug_StartCommandReception() != HAL_OK)
  {
    Error_Handler();
  }

  previous_keys = ReadPressedKeys();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    (void)UpdateBallEstimate(&last_vision_measurement_counter,
                             &last_ball_sample_ms, &ball_state_valid,
                             &ball_x_est_deci_cm,
                             &ball_velocity_deci_cm_per_s);

    if (Emm_V5_Current_Position_Request_Failed())
    {
      motor_position_request_pending = 0U;
      if (calibration_state != CALIBRATION_IDLE)
      {
        StopAndDisableMotor(&motor_enabled);
        calibration_state = CALIBRATION_IDLE;
      }
      else if (closed_loop_enabled != 0U)
      {
        StopAndDisableMotor(&motor_enabled);
        closed_loop_enabled = 0U;
        previous_command_active = 0U;
      }
    }
    else if (Emm_V5_Get_Current_Position_Result(&motor_position_counts))
    {
      int32_t position_centi_degrees;

      motor_position_request_pending = 0U;
      motor_position_valid = 1U;
      last_motor_position_ms = now_ms;
      if (calibration_state != CALIBRATION_IDLE)
      {
        motor_pulse_est = MotorPositionToPulse(motor_position_counts,
                                                motor_zero_counts);
        (void)Debug_PrintCalibrationState(CalibrationPhaseCode(calibration_state),
                                          calibration_target_pulse,
                                          ball_x_est_deci_cm,
                                          ball_velocity_deci_cm_per_s,
                                          motor_pulse_est,
                                          now_ms - calibration_start_ms);
      }
      else if (closed_loop_enabled != 0U)
      {
        motor_pulse_est = MotorPositionToPulse(motor_position_counts,
                                                motor_zero_counts);
        position_centi_degrees = ((motor_position_counts
                                   / MOTOR_POSITION_COUNTS_PER_REVOLUTION) * 36000L)
                               + (((motor_position_counts
                                    % MOTOR_POSITION_COUNTS_PER_REVOLUTION) * 36000L)
                                  / MOTOR_POSITION_COUNTS_PER_REVOLUTION);
        (void)Debug_PrintControlState(vision_x_deci_cm, vision_y_deci_cm,
                                      target_x_deci_cm, motor_pulse_est,
                                      motor_tilt_target_pulse,
                                      (int32_t)target_x_deci_cm
                                          - ball_x_est_deci_cm,
                                      ball_velocity_deci_cm_per_s,
                                      motor_position_counts,
                                      position_centi_degrees,
                                      now_ms - closed_loop_start_ms);
      }
    }
    {
      uint8_t calibration_command = Debug_GetCommand();
      int32_t requested_calibration_target;

      if ((calibration_command == 'X') || (calibration_command == 'x'))
      {
        if (calibration_state != CALIBRATION_IDLE)
        {
          StopAndDisableMotor(&motor_enabled);
          calibration_state = CALIBRATION_IDLE;
          motor_position_request_pending = 0U;
        }
      }
      else
      {
        requested_calibration_target = CalibrationTargetFromCommand(
                                       calibration_command);
        if ((requested_calibration_target != 0)
            && (calibration_state == CALIBRATION_IDLE)
            && (closed_loop_enabled == 0U))
        {
          if ((vision_data_valid == 0U) || (ball_state_valid == 0U))
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_NO_VISION);
          }
          else if ((now_ms - last_ball_sample_ms) > VISION_TIMEOUT_MS)
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_VISION_TIMEOUT);
          }
          else if (!Emm_V5_Read_Current_Position(MOTOR_ADDRESS,
                                                  &motor_position_counts))
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_POSITION_READ);
          }
          else
          {
            EnableMotor(&motor_enabled);
            motor_zero_counts = MOTOR_FIXED_ZERO_COUNTS;
            motor_pulse_est = 0;
            calibration_target_pulse = requested_calibration_target;
            calibration_state = CALIBRATION_MOVE;
            calibration_start_ms = now_ms;
            calibration_phase_start_ms = now_ms;
            last_control_update = now_ms;
            last_position_query = now_ms;
            last_motor_position_ms = now_ms;
            motor_position_valid = 1U;
            motor_position_request_pending = 0U;
            previous_command_active = 0U;
          }
        }
      }
    }
    {
      uint8_t current_keys = ReadPressedKeys();
      uint8_t pressed_keys = current_keys & (uint8_t)~previous_keys;

      previous_keys = current_keys;
      if ((pressed_keys != 0U) && ((HAL_GetTick() - last_key_action) >= 50U))
      {
        uint8_t start_requested = 0U;
        uint8_t task_start_requested = 0U;

        last_key_action = HAL_GetTick();
        if (calibration_state != CALIBRATION_IDLE)
        {
          StopAndDisableMotor(&motor_enabled);
          calibration_state = CALIBRATION_IDLE;
          motor_position_request_pending = 0U;
        }
        else if ((pressed_keys & 0x01U) != 0U)
        {
          if (closed_loop_enabled != 0U)
          {
            StopAndDisableMotor(&motor_enabled);
            closed_loop_enabled = 0U;
            previous_command_active = 0U;
            task_state = TASK_IDLE;
          }
          else
          {
            target_x_deci_cm = TASK_POSITIVE_TARGET_DECI_CM;
            start_requested = 1U;
            task_start_requested = 1U;
          }
        }
        else if ((pressed_keys & 0x02U) != 0U)
        {
          target_x_deci_cm = 0;
          start_requested = (closed_loop_enabled == 0U) ? 1U : 0U;
        }
        else if ((pressed_keys & 0x04U) != 0U)
        {
          target_x_deci_cm = TASK_POSITIVE_TARGET_DECI_CM;
          start_requested = (closed_loop_enabled == 0U) ? 1U : 0U;
        }
        else if ((pressed_keys & 0x08U) != 0U)
        {
          target_x_deci_cm = TASK_NEGATIVE_TARGET_DECI_CM;
          start_requested = (closed_loop_enabled == 0U) ? 1U : 0U;
        }

        if (start_requested != 0U)
        {
          if ((vision_data_valid == 0U) || (ball_state_valid == 0U))
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_NO_VISION);
          }
          else if ((now_ms - last_ball_sample_ms) > VISION_TIMEOUT_MS)
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_VISION_TIMEOUT);
          }
          else if (!Emm_V5_Read_Current_Position(MOTOR_ADDRESS,
                                                  &motor_position_counts))
          {
            (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_POSITION_READ);
          }
          else if ((task_start_requested != 0U)
                   && ((AbsInt32(ball_x_est_deci_cm)
                        > TASK_START_POSITION_TOLERANCE_DECI_CM)
                       || (AbsInt32(ball_velocity_deci_cm_per_s)
                           > TASK_SETTLED_VELOCITY_DECI_CM_S)))
          {
            (void)Debug_PrintTaskEvent(TASK_EVENT_START_POSITION, 0U);
          }
          else
          {
            /* Always use the calibrated mechanical zero, never the start position. */
            EnableMotor(&motor_enabled);
            motor_zero_counts = MOTOR_FIXED_ZERO_COUNTS;
            motor_pulse_est = 0;
            motor_tilt_target_pulse = 0;
            motor_static_bias_pulse = 0;
            last_static_bias_update = now_ms;
            motor_position_valid = 1U;
            last_motor_position_ms = now_ms;
            closed_loop_enabled = 1U;
            closed_loop_start_ms = now_ms;
            last_control_update = now_ms;
            last_position_query = now_ms;
            previous_command_active = 0U;
            motor_position_request_pending = 0U;
            (void)Debug_PrintClosedLoopEnabled();
            if (task_start_requested != 0U)
            {
              task_state = TASK_TO_POSITIVE;
              task_start_ms = now_ms;
              task_settled_start_ms = 0U;
              task_reverse_boost_until_ms = 0U;
              (void)Debug_PrintTaskEvent(TASK_EVENT_START, 0U);
            }
            else
            {
              task_state = TASK_IDLE;
            }
          }
        }
      }
    }
    if (calibration_state != CALIBRATION_IDLE)
    {
      if ((now_ms - last_ball_sample_ms) > VISION_TIMEOUT_MS)
      {
        StopAndDisableMotor(&motor_enabled);
        calibration_state = CALIBRATION_IDLE;
        motor_position_request_pending = 0U;
      }
      else if ((motor_position_valid == 0U)
               || ((now_ms - last_motor_position_ms) > MOTOR_POSITION_MAX_AGE_MS))
      {
        StopAndDisableMotor(&motor_enabled);
        calibration_state = CALIBRATION_IDLE;
        motor_position_request_pending = 0U;
      }
      else if (((now_ms - last_control_update) >= MOTOR_CONTROL_PERIOD_MS)
               && (motor_position_request_pending == 0U))
      {
        int32_t desired_pulse = calibration_target_pulse;
        int32_t step;

        last_control_update = now_ms;
        motor_pulse_est = MotorPositionToPulse(motor_position_counts,
                                                motor_zero_counts);
        if ((calibration_state == CALIBRATION_MOVE)
            && (AbsInt32(motor_pulse_est - calibration_target_pulse)
                <= CALIBRATION_PULSE_TOLERANCE))
        {
          calibration_state = CALIBRATION_HOLD;
          calibration_phase_start_ms = now_ms;
        }
        else if ((calibration_state == CALIBRATION_MOVE)
                 && ((now_ms - calibration_phase_start_ms)
                     > CALIBRATION_MOVE_TIMEOUT_MS))
        {
          StopAndDisableMotor(&motor_enabled);
          calibration_state = CALIBRATION_IDLE;
        }
        else if ((calibration_state == CALIBRATION_HOLD)
                 && ((now_ms - calibration_phase_start_ms)
                     >= CALIBRATION_HOLD_MS))
        {
          calibration_state = CALIBRATION_RETURN;
          calibration_target_pulse = 0;
          calibration_phase_start_ms = now_ms;
          desired_pulse = 0;
        }
        else if ((calibration_state == CALIBRATION_RETURN)
                 && (AbsInt32(motor_pulse_est) <= CALIBRATION_PULSE_TOLERANCE))
        {
          (void)Debug_PrintCalibrationState('D', 0, ball_x_est_deci_cm,
                                            ball_velocity_deci_cm_per_s,
                                            motor_pulse_est,
                                            now_ms - calibration_start_ms);
          StopAndDisableMotor(&motor_enabled);
          calibration_state = CALIBRATION_IDLE;
          previous_command_active = 0U;
        }

        if (calibration_state != CALIBRATION_IDLE)
        {
          if (calibration_state == CALIBRATION_RETURN)
          {
            desired_pulse = 0;
          }
          step = ClampInt32(desired_pulse - motor_pulse_est,
                            -MOTOR_SHORT_STEP_MAX_PULSES,
                            MOTOR_SHORT_STEP_MAX_PULSES);
          if ((step != 0)
              && (SendShortRelativePulse(motor_pulse_est, step, 0U) == 0U))
          {
            StopAndDisableMotor(&motor_enabled);
            calibration_state = CALIBRATION_IDLE;
          }
        }
      }
      if ((calibration_state != CALIBRATION_IDLE)
          && (motor_position_request_pending == 0U)
          && ((now_ms - last_position_query) >= MOTOR_POSITION_QUERY_PERIOD_MS))
      {
        last_position_query = now_ms;
        if (Emm_V5_Request_Current_Position(MOTOR_ADDRESS))
        {
          motor_position_request_pending = 1U;
        }
      }
    }
    else if (closed_loop_enabled != 0U)
    {
      if ((now_ms - last_ball_sample_ms) > VISION_TIMEOUT_MS)
      {
        StopAndDisableMotor(&motor_enabled);
        closed_loop_enabled = 0U;
        previous_command_active = 0U;
      }
      else if ((motor_position_valid == 0U)
               || ((now_ms - last_motor_position_ms) > MOTOR_POSITION_MAX_AGE_MS))
      {
        StopAndDisableMotor(&motor_enabled);
        closed_loop_enabled = 0U;
        previous_command_active = 0U;
      }
      else
      {
        if (((task_state == TASK_TO_POSITIVE)
             || (task_state == TASK_TO_NEGATIVE))
            && ((now_ms - task_start_ms) > TASK_MAX_DURATION_MS))
        {
          StopAndDisableMotor(&motor_enabled);
          closed_loop_enabled = 0U;
          previous_command_active = 0U;
          task_state = TASK_IDLE;
          (void)Debug_PrintTaskEvent(TASK_EVENT_TIMEOUT,
                                     now_ms - task_start_ms);
        }
        else if ((task_state == TASK_TO_POSITIVE)
                 && (ball_x_est_deci_cm >= TASK_POSITIVE_REVERSE_DECI_CM))
        {
          target_x_deci_cm = TASK_NEGATIVE_TARGET_DECI_CM;
          task_state = TASK_TO_NEGATIVE;
          task_settled_start_ms = 0U;
          task_reverse_boost_until_ms = now_ms + TASK_REVERSE_BOOST_MS;
          (void)Debug_PrintTaskEvent(TASK_EVENT_REVERSE,
                                     now_ms - task_start_ms);
        }
        else if (task_state == TASK_TO_NEGATIVE)
        {
          if ((AbsInt32((int32_t)ball_x_est_deci_cm
                        - TASK_NEGATIVE_TARGET_DECI_CM)
               <= TASK_SETTLED_POSITION_TOLERANCE_DECI_CM)
              && (AbsInt32(ball_velocity_deci_cm_per_s)
                  <= TASK_SETTLED_VELOCITY_DECI_CM_S))
          {
            if (task_settled_start_ms == 0U)
            {
              task_settled_start_ms = now_ms;
            }
            else if ((now_ms - task_settled_start_ms) >= TASK_SETTLED_HOLD_MS)
            {
              task_state = TASK_COMPLETE;
              (void)Debug_PrintTaskEvent(TASK_EVENT_COMPLETE,
                                         now_ms - task_start_ms);
            }
          }
          else
          {
            task_settled_start_ms = 0U;
          }
        }

        if (closed_loop_enabled != 0U)
        {
          motor_pulse_est = MotorPositionToPulse(motor_position_counts,
                                                  motor_zero_counts);
        if (motor_pulse_est >= MOTOR_SOFTWARE_LIMIT_PULSES)
        {
          StopAndDisableMotor(&motor_enabled);
          closed_loop_enabled = 0U;
          previous_command_active = 0U;
          (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_UPPER_LIMIT);
        }
        else if (motor_pulse_est <= -MOTOR_SOFTWARE_LIMIT_PULSES)
        {
          StopAndDisableMotor(&motor_enabled);
          closed_loop_enabled = 0U;
          previous_command_active = 0U;
          (void)Debug_PrintClosedLoopRejected(CLOSED_LOOP_REJECT_LOWER_LIMIT);
        }
        else if (((now_ms - last_control_update) >= MOTOR_CONTROL_PERIOD_MS)
                 && (motor_position_request_pending == 0U))
        {
          int32_t position_error = (int32_t)target_x_deci_cm
                                   - ball_x_est_deci_cm;
          int32_t control_error = position_error;
          int32_t desired_tilt_pulse;
          int32_t step;
          int32_t step_limit;
          uint8_t direction;
          uint8_t fast_braking;

          last_control_update = now_ms;

          if ((AbsInt32(position_error) <= BALL_POSITION_DEADBAND_DECI_CM)
              && (AbsInt32(ball_velocity_deci_cm_per_s)
                  <= BALL_SETTLED_VELOCITY_DECI_CM_S))
          {
            desired_tilt_pulse = 0;
            motor_static_bias_pulse = 0;
          }
          else
          {
            /* The rod target is bounded; short pulses only chase this target. */
            desired_tilt_pulse = -((BALL_POSITION_GAIN_NUMERATOR * control_error
                                    - BALL_VELOCITY_GAIN_NUMERATOR
                                      * ball_velocity_deci_cm_per_s)
                                   / BALL_TILT_GAIN_DIVISOR);
            desired_tilt_pulse = ClampInt32(desired_tilt_pulse,
                                             -MOTOR_TILT_TARGET_LIMIT_PULSES,
                                             MOTOR_TILT_TARGET_LIMIT_PULSES);
            /* Overcome static friction only while the ball is effectively still. */
            if ((AbsInt32(position_error) >= BALL_STATIC_TRIGGER_DECI_CM)
                && (AbsInt32(ball_velocity_deci_cm_per_s)
                 < BALL_MOTION_THRESHOLD_DECI_CM_S)
                && (AbsInt32(desired_tilt_pulse) < MOTOR_STATIC_TILT_PULSES))
            {
              desired_tilt_pulse = (position_error < 0)
                                 ? MOTOR_STATIC_TILT_PULSES
                                 : -MOTOR_STATIC_TILT_PULSES;
            }
            if ((AbsInt32(position_error) < BALL_STATIC_TRIGGER_DECI_CM)
                || (AbsInt32(ball_velocity_deci_cm_per_s)
                    >= BALL_MOTION_THRESHOLD_DECI_CM_S))
            {
              motor_static_bias_pulse = 0;
            }
            else if (((position_error < 0) && (motor_static_bias_pulse < 0))
                     || ((position_error > 0) && (motor_static_bias_pulse > 0)))
            {
              motor_static_bias_pulse = 0;
            }
            else if (((now_ms - last_static_bias_update)
                      >= MOTOR_STATIC_BIAS_PERIOD_MS)
                     && (AbsInt32(motor_pulse_est - (desired_tilt_pulse
                                                      + motor_static_bias_pulse))
                         <= MOTOR_SHORT_STEP_MAX_PULSES))
            {
              last_static_bias_update = now_ms;
              motor_static_bias_pulse += (position_error < 0)
                                       ? MOTOR_STATIC_BIAS_STEP_PULSES
                                       : -MOTOR_STATIC_BIAS_STEP_PULSES;
              motor_static_bias_pulse = ClampInt32(
                  motor_static_bias_pulse, -MOTOR_STATIC_BIAS_LIMIT_PULSES,
                  MOTOR_STATIC_BIAS_LIMIT_PULSES);
            }
            desired_tilt_pulse += motor_static_bias_pulse;
            desired_tilt_pulse = ClampInt32(desired_tilt_pulse,
                                             -MOTOR_TILT_TARGET_LIMIT_PULSES,
                                             MOTOR_TILT_TARGET_LIMIT_PULSES);
            if (AbsInt32(position_error) <= BALL_POSITION_DEADBAND_DECI_CM)
            {
              desired_tilt_pulse = ClampInt32(
                  desired_tilt_pulse, -MOTOR_TERMINAL_TILT_LIMIT_PULSES,
                  MOTOR_TERMINAL_TILT_LIMIT_PULSES);
            }
          }
          motor_tilt_target_pulse = desired_tilt_pulse;
          step = desired_tilt_pulse - motor_pulse_est;
          if (step == 0)
          {
            previous_command_active = 0U;
          }
          else
          {
            direction = (step > 0) ? 0U : 1U;
            fast_braking = ((task_state == TASK_TO_NEGATIVE)
                             && (now_ms < task_reverse_boost_until_ms))
                              ? 1U : 0U;
            if ((previous_command_active != 0U)
                && (direction != previous_command_direction))
            {
              Emm_V5_Stop_Now(MOTOR_ADDRESS, false);
              fast_braking = 1U;
            }
            step_limit = (fast_braking != 0U)
                           ? MOTOR_REVERSE_BRAKE_MAX_STEP_PULSES
                           : MOTOR_SHORT_STEP_MAX_PULSES;
            step = ClampInt32(step, -step_limit, step_limit);
            if (SendShortRelativePulse(motor_pulse_est, step,
                                       fast_braking) == 0U)
            {
              StopAndDisableMotor(&motor_enabled);
              closed_loop_enabled = 0U;
              previous_command_active = 0U;
            }
            else
            {
              previous_command_direction = direction;
              previous_command_active = 1U;
            }
          }
        }
      }

      if ((closed_loop_enabled != 0U)
          && (motor_position_request_pending == 0U)
          && ((now_ms - last_position_query) >= MOTOR_POSITION_QUERY_PERIOD_MS))
      {
        last_position_query = now_ms;
        if (Emm_V5_Request_Current_Position(MOTOR_ADDRESS))
        {
          motor_position_request_pending = 1U;
        }
      }
        }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
