#include "movement.h"

#include "tim.h"
#include <string.h>

#define MOVEMENT_PWM_DELTA_MAX  300

static int32_t Movement_Abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t Movement_Max4Abs(int32_t a, int32_t b,
                                int32_t c, int32_t d)
{
  int32_t maximum = Movement_Abs(a);

  if (Movement_Abs(b) > maximum)
  {
    maximum = Movement_Abs(b);
  }
  if (Movement_Abs(c) > maximum)
  {
    maximum = Movement_Abs(c);
  }
  if (Movement_Abs(d) > maximum)
  {
    maximum = Movement_Abs(d);
  }
  return maximum;
}

static uint16_t Movement_PermilleToPwm(int32_t permille)
{
  int32_t pwm;

  if (permille > 1000)
  {
    permille = 1000;
  }
  else if (permille < -1000)
  {
    permille = -1000;
  }

  pwm = (int32_t)MOVEMENT_PWM_NEUTRAL +
      ((permille * MOVEMENT_PWM_DELTA_MAX) / 1000);

  if (pwm > (int32_t)MOVEMENT_PWM_MAX)
  {
    pwm = MOVEMENT_PWM_MAX;
  }
  else if (pwm < (int32_t)MOVEMENT_PWM_MIN)
  {
    pwm = MOVEMENT_PWM_MIN;
  }
  return (uint16_t)pwm;
}

void Movement_SetNeutral(MotorOutputs_t *outputs)
{
  if (outputs == NULL)
  {
    return;
  }

  outputs->vertical_fl = MOVEMENT_PWM_NEUTRAL;
  outputs->vertical_fr = MOVEMENT_PWM_NEUTRAL;
  outputs->vertical_rl = MOVEMENT_PWM_NEUTRAL;
  outputs->vertical_rr = MOVEMENT_PWM_NEUTRAL;
  outputs->horizontal_fl = MOVEMENT_PWM_NEUTRAL;
  outputs->horizontal_fr = MOVEMENT_PWM_NEUTRAL;
  outputs->horizontal_rl = MOVEMENT_PWM_NEUTRAL;
  outputs->horizontal_rr = MOVEMENT_PWM_NEUTRAL;
}

bool Movement_SetSingleMotorTest(MotorOutputs_t *outputs,
                                 uint8_t motor_index,
                                 int8_t direction)
{
  uint16_t testPwm;

  if ((outputs == NULL) || (motor_index >= 8U) ||
      ((direction != 1) && (direction != -1)))
  {
    return false;
  }

  Movement_SetNeutral(outputs);
  testPwm = (direction > 0) ? 1550U : 1450U;

  switch (motor_index)
  {
    case 0U: outputs->vertical_fl = testPwm; break;
    case 1U: outputs->vertical_fr = testPwm; break;
    case 2U: outputs->vertical_rl = testPwm; break;
    case 3U: outputs->vertical_rr = testPwm; break;
    case 4U: outputs->horizontal_fl = testPwm; break;
    case 5U: outputs->horizontal_fr = testPwm; break;
    case 6U: outputs->horizontal_rl = testPwm; break;
    case 7U: outputs->horizontal_rr = testPwm; break;
    default: return false;
  }

  return true;
}

bool Movement_Compute(const ControlCommand_t *command,
                      MotorOutputs_t *outputs)
{
  int32_t horizontalFl;
  int32_t horizontalFr;
  int32_t horizontalRl;
  int32_t horizontalRr;
  int32_t maximum;

  if ((command == NULL) || (outputs == NULL))
  {
    return false;
  }

  Movement_SetNeutral(outputs);

  if (((command->control_flags & COMM_COMMAND_ARM_REQUEST) == 0U) ||
      ((command->control_flags & COMM_COMMAND_EMERGENCY_STOP) != 0U))
  {
    return true;
  }

  outputs->vertical_fl = Movement_PermilleToPwm(command->heave_permille);
  outputs->vertical_fr = Movement_PermilleToPwm(command->heave_permille);
  outputs->vertical_rl = Movement_PermilleToPwm(command->heave_permille);
  outputs->vertical_rr = Movement_PermilleToPwm(command->heave_permille);

  /* Standard four-thruster X mix. Physical directions are verified later. */
  horizontalFl = (int32_t)command->surge_permille +
                 (int32_t)command->sway_permille +
                 (int32_t)command->yaw_permille;
  horizontalFr = (int32_t)command->surge_permille -
                 (int32_t)command->sway_permille -
                 (int32_t)command->yaw_permille;
  horizontalRl = (int32_t)command->surge_permille -
                 (int32_t)command->sway_permille +
                 (int32_t)command->yaw_permille;
  horizontalRr = (int32_t)command->surge_permille +
                 (int32_t)command->sway_permille -
                 (int32_t)command->yaw_permille;

  /* Normalize mixed commands instead of destroying their ratio by clipping. */
  maximum = Movement_Max4Abs(horizontalFl, horizontalFr,
                             horizontalRl, horizontalRr);
  if (maximum > 1000)
  {
    horizontalFl = (horizontalFl * 1000) / maximum;
    horizontalFr = (horizontalFr * 1000) / maximum;
    horizontalRl = (horizontalRl * 1000) / maximum;
    horizontalRr = (horizontalRr * 1000) / maximum;
  }

  outputs->horizontal_fl = Movement_PermilleToPwm(horizontalFl);
  outputs->horizontal_fr = Movement_PermilleToPwm(horizontalFr);
  outputs->horizontal_rl = Movement_PermilleToPwm(horizontalRl);
  outputs->horizontal_rr = Movement_PermilleToPwm(horizontalRr);
  return true;
}

static uint16_t Movement_LimitOnePwm(uint16_t pwm,
                                     uint16_t maximumDeltaUs)
{
  uint16_t lowerLimit;
  uint16_t upperLimit;

  if (maximumDeltaUs > (MOVEMENT_PWM_MAX - MOVEMENT_PWM_NEUTRAL))
  {
    maximumDeltaUs = MOVEMENT_PWM_MAX - MOVEMENT_PWM_NEUTRAL;
  }

  lowerLimit = MOVEMENT_PWM_NEUTRAL - maximumDeltaUs;
  upperLimit = MOVEMENT_PWM_NEUTRAL + maximumDeltaUs;

  if (pwm < lowerLimit)
  {
    return lowerLimit;
  }
  if (pwm > upperLimit)
  {
    return upperLimit;
  }
  return pwm;
}

void Movement_LimitPwmDelta(MotorOutputs_t *outputs,
                            uint16_t maximumDeltaUs)
{
  if (outputs == NULL)
  {
    return;
  }

  outputs->vertical_fl =
      Movement_LimitOnePwm(outputs->vertical_fl, maximumDeltaUs);
  outputs->vertical_fr =
      Movement_LimitOnePwm(outputs->vertical_fr, maximumDeltaUs);
  outputs->vertical_rl =
      Movement_LimitOnePwm(outputs->vertical_rl, maximumDeltaUs);
  outputs->vertical_rr =
      Movement_LimitOnePwm(outputs->vertical_rr, maximumDeltaUs);
  outputs->horizontal_fl =
      Movement_LimitOnePwm(outputs->horizontal_fl, maximumDeltaUs);
  outputs->horizontal_fr =
      Movement_LimitOnePwm(outputs->horizontal_fr, maximumDeltaUs);
  outputs->horizontal_rl =
      Movement_LimitOnePwm(outputs->horizontal_rl, maximumDeltaUs);
  outputs->horizontal_rr =
      Movement_LimitOnePwm(outputs->horizontal_rr, maximumDeltaUs);
}

void Movement_Apply(const MotorOutputs_t *outputs)
{
  if (outputs == NULL)
  {
    return;
  }

  /* Vertical: FL=PB8, FR=PB9, RL=PA0, RR=PB1. */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, outputs->vertical_fl);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, outputs->vertical_fr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, outputs->vertical_rl);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, outputs->vertical_rr);

  /* Horizontal: FL=PA6, FR=PA1, RL=PA7, RR=PB0. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, outputs->horizontal_fl);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, outputs->horizontal_fr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, outputs->horizontal_rl);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, outputs->horizontal_rr);
}

bool Movement_StartPwmNeutral(void)
{
  MotorOutputs_t neutral;

  Movement_SetNeutral(&neutral);
  Movement_Apply(&neutral);

  return ((HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) == HAL_OK) &&
          (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) == HAL_OK));
}

static bool Movement_AllNeutral(const MotorOutputs_t *outputs)
{
  return ((outputs->vertical_fl == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->vertical_fr == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->vertical_rl == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->vertical_rr == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->horizontal_fl == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->horizontal_fr == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->horizontal_rl == MOVEMENT_PWM_NEUTRAL) &&
          (outputs->horizontal_rr == MOVEMENT_PWM_NEUTRAL));
}

uint8_t Movement_RunSelfTest(void)
{
  ControlCommand_t command;
  MotorOutputs_t outputs;

  (void)memset(&command, 0, sizeof(command));
  command.control_flags = COMM_COMMAND_ARM_REQUEST;
  if ((!Movement_Compute(&command, &outputs)) ||
      (!Movement_AllNeutral(&outputs)))
  {
    return 1U;
  }

  command.heave_permille = 1000;
  if ((!Movement_Compute(&command, &outputs)) ||
      (outputs.vertical_fl != MOVEMENT_PWM_MAX) ||
      (outputs.vertical_fr != MOVEMENT_PWM_MAX) ||
      (outputs.vertical_rl != MOVEMENT_PWM_MAX) ||
      (outputs.vertical_rr != MOVEMENT_PWM_MAX))
  {
    return 2U;
  }

  (void)memset(&command, 0, sizeof(command));
  command.control_flags = COMM_COMMAND_ARM_REQUEST;
  command.surge_permille = 1000;
  if ((!Movement_Compute(&command, &outputs)) ||
      (outputs.horizontal_fl != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_fr != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_rl != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_rr != MOVEMENT_PWM_MAX))
  {
    return 3U;
  }

  command.surge_permille = 0;
  command.sway_permille = 1000;
  if ((!Movement_Compute(&command, &outputs)) ||
      (outputs.horizontal_fl != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_fr != MOVEMENT_PWM_MIN) ||
      (outputs.horizontal_rl != MOVEMENT_PWM_MIN) ||
      (outputs.horizontal_rr != MOVEMENT_PWM_MAX))
  {
    return 4U;
  }

  command.sway_permille = 0;
  command.yaw_permille = 1000;
  if ((!Movement_Compute(&command, &outputs)) ||
      (outputs.horizontal_fl != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_fr != MOVEMENT_PWM_MIN) ||
      (outputs.horizontal_rl != MOVEMENT_PWM_MAX) ||
      (outputs.horizontal_rr != MOVEMENT_PWM_MIN))
  {
    return 5U;
  }

  command.control_flags = COMM_COMMAND_EMERGENCY_STOP;
  if ((!Movement_Compute(&command, &outputs)) ||
      (!Movement_AllNeutral(&outputs)))
  {
    return 6U;
  }

  command.control_flags = COMM_COMMAND_ARM_REQUEST;
  command.surge_permille = 1000;
  Movement_Compute(&command, &outputs);
  Movement_LimitPwmDelta(&outputs, 60U);
  if ((outputs.horizontal_fl != 1560U) ||
      (outputs.horizontal_fr != 1560U) ||
      (outputs.horizontal_rl != 1560U) ||
      (outputs.horizontal_rr != 1560U))
  {
    return 7U;
  }

  return 0U;
}

uint8_t Movement_RunPwmNeutralTest(void)
{
  uint32_t tim2Channels = TIM_CCER_CC1E | TIM_CCER_CC2E;
  uint32_t tim3Channels = TIM_CCER_CC1E | TIM_CCER_CC2E |
                          TIM_CCER_CC3E | TIM_CCER_CC4E;
  uint32_t tim4Channels = TIM_CCER_CC3E | TIM_CCER_CC4E;

  if ((TIM2->PSC != 99U) || (TIM3->PSC != 99U) || (TIM4->PSC != 99U) ||
      (TIM2->ARR != 19999U) || (TIM3->ARR != 19999U) ||
      (TIM4->ARR != 19999U))
  {
    return 1U;
  }

  if ((TIM2->CCR1 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM2->CCR2 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM3->CCR1 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM3->CCR2 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM3->CCR3 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM3->CCR4 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM4->CCR3 != MOVEMENT_PWM_NEUTRAL) ||
      (TIM4->CCR4 != MOVEMENT_PWM_NEUTRAL))
  {
    return 2U;
  }

  if (((TIM2->CR1 & TIM_CR1_CEN) == 0U) ||
      ((TIM3->CR1 & TIM_CR1_CEN) == 0U) ||
      ((TIM4->CR1 & TIM_CR1_CEN) == 0U))
  {
    return 3U;
  }

  if (((TIM2->CCER & tim2Channels) != tim2Channels) ||
      ((TIM3->CCER & tim3Channels) != tim3Channels) ||
      ((TIM4->CCER & tim4Channels) != tim4Channels))
  {
    return 4U;
  }

  return 0U;
}
