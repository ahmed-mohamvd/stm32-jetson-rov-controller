#ifndef MOVEMENT_H
#define MOVEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_protocol.h"
#include <stdbool.h>
#include <stdint.h>

#define MOVEMENT_PWM_NEUTRAL  1500U
#define MOVEMENT_PWM_MIN      1200U
#define MOVEMENT_PWM_MAX      1800U

typedef struct
{
  uint16_t vertical_fl;
  uint16_t vertical_fr;
  uint16_t vertical_rl;
  uint16_t vertical_rr;
  uint16_t horizontal_fl;
  uint16_t horizontal_fr;
  uint16_t horizontal_rl;
  uint16_t horizontal_rr;
} MotorOutputs_t;

void Movement_SetNeutral(MotorOutputs_t *outputs);
bool Movement_SetSingleMotorTest(MotorOutputs_t *outputs,
                                 uint8_t motor_index,
                                 int8_t direction);
bool Movement_Compute(const ControlCommand_t *command,
                      MotorOutputs_t *outputs);
void Movement_LimitPwmDelta(MotorOutputs_t *outputs,
                            uint16_t maximum_delta_us);
void Movement_Apply(const MotorOutputs_t *outputs);
bool Movement_StartPwmNeutral(void);
uint8_t Movement_RunSelfTest(void);
uint8_t Movement_RunPwmNeutralTest(void);

#ifdef __cplusplus
}
#endif

#endif /* MOVEMENT_H */
