#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* Call after osKernelInitialize() and before the communication tasks start. */
bool CommInterface_Init(void);

bool CommInterface_SetCommand(const ControlCommand_t *command,
                              uint32_t timeout_ticks);
bool CommInterface_GetCommand(ControlCommand_t *command,
                              uint32_t timeout_ticks);
bool CommInterface_UpdateCommandSafety(uint32_t now_ms,
                                       uint32_t link_timeout_ms,
                                       uint32_t timeout_ticks);
bool CommInterface_GetSafetyState(bool *link_ok,
                                  bool *failsafe_active,
                                  uint32_t timeout_ticks);
bool CommInterface_SetTelemetry(const TelemetrySnapshot_t *telemetry,
                                uint32_t timeout_ticks);
bool CommInterface_GetTelemetry(TelemetrySnapshot_t *telemetry,
                                uint32_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* COMM_INTERFACE_H */
