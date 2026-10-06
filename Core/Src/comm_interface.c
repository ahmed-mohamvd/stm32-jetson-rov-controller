#include "comm_interface.h"

#include "cmsis_os2.h"
#include <string.h>

static osMutexId_t commDataMutex;
static ControlCommand_t latestReceivedCommand;
static ControlCommand_t activeCommand;
static TelemetrySnapshot_t latestTelemetry;
static bool commandAvailable;
static bool commandLinkOk;
static bool commandFailsafeActive;

static void CommInterface_MakeCommandNeutral(ControlCommand_t *command)
{
  uint16_t emergencyStop;

  emergencyStop = command->control_flags & COMM_COMMAND_EMERGENCY_STOP;
  command->control_flags = emergencyStop;
  command->surge_permille = 0;
  command->sway_permille = 0;
  command->heave_permille = 0;
  command->yaw_permille = 0;
  command->depth_setpoint_mm = 0;
}

bool CommInterface_Init(void)
{
  static const osMutexAttr_t mutexAttributes = {
    .name = "commDataMutex"
  };

  (void)memset(&latestReceivedCommand, 0, sizeof(latestReceivedCommand));
  (void)memset(&activeCommand, 0, sizeof(activeCommand));
  (void)memset(&latestTelemetry, 0, sizeof(latestTelemetry));
  commandAvailable = false;
  commandLinkOk = false;
  commandFailsafeActive = true;

  commDataMutex = osMutexNew(&mutexAttributes);
  return (commDataMutex != NULL);
}

static bool CommInterface_Lock(uint32_t timeoutTicks)
{
  return ((commDataMutex != NULL) &&
          (osMutexAcquire(commDataMutex, timeoutTicks) == osOK));
}

static void CommInterface_Unlock(void)
{
  (void)osMutexRelease(commDataMutex);
}

bool CommInterface_SetCommand(const ControlCommand_t *command,
                              uint32_t timeoutTicks)
{
  if ((command == NULL) || !CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  latestReceivedCommand = *command;
  activeCommand = *command;
  commandAvailable = true;
  commandLinkOk = true;
  commandFailsafeActive =
      ((command->control_flags & COMM_COMMAND_EMERGENCY_STOP) != 0U);

  if (commandFailsafeActive)
  {
    CommInterface_MakeCommandNeutral(&activeCommand);
  }
  CommInterface_Unlock();
  return true;
}

bool CommInterface_GetCommand(ControlCommand_t *command,
                              uint32_t timeoutTicks)
{
  if ((command == NULL) || !CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  *command = activeCommand;
  CommInterface_Unlock();
  return true;
}

bool CommInterface_UpdateCommandSafety(uint32_t now_ms,
                                       uint32_t link_timeout_ms,
                                       uint32_t timeoutTicks)
{
  bool emergencyStop;

  if (!CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  commandLinkOk = commandAvailable &&
      ((now_ms - latestReceivedCommand.received_at_ms) <= link_timeout_ms);
  emergencyStop = commandAvailable &&
      ((latestReceivedCommand.control_flags &
        COMM_COMMAND_EMERGENCY_STOP) != 0U);
  commandFailsafeActive = (!commandLinkOk) || emergencyStop;

  if (commandAvailable)
  {
    activeCommand = latestReceivedCommand;
  }
  else
  {
    (void)memset(&activeCommand, 0, sizeof(activeCommand));
  }

  if (commandFailsafeActive)
  {
    CommInterface_MakeCommandNeutral(&activeCommand);
  }

  CommInterface_Unlock();
  return true;
}

bool CommInterface_GetSafetyState(bool *link_ok,
                                  bool *failsafe_active,
                                  uint32_t timeoutTicks)
{
  if ((link_ok == NULL) || (failsafe_active == NULL) ||
      !CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  *link_ok = commandLinkOk;
  *failsafe_active = commandFailsafeActive;
  CommInterface_Unlock();
  return true;
}

bool CommInterface_SetTelemetry(const TelemetrySnapshot_t *telemetry,
                                uint32_t timeoutTicks)
{
  if ((telemetry == NULL) || !CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  latestTelemetry = *telemetry;
  CommInterface_Unlock();
  return true;
}

bool CommInterface_GetTelemetry(TelemetrySnapshot_t *telemetry,
                                uint32_t timeoutTicks)
{
  if ((telemetry == NULL) || !CommInterface_Lock(timeoutTicks))
  {
    return false;
  }

  *telemetry = latestTelemetry;
  CommInterface_Unlock();
  return true;
}
