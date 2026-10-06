#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define COMM_SOF_1                    0xAAU
#define COMM_SOF_2                    0x55U
#define COMM_PROTOCOL_VERSION         0x01U
#define COMM_MAX_PAYLOAD_SIZE         64U
#define COMM_FRAME_OVERHEAD_SIZE      8U
#define COMM_COMMAND_PAYLOAD_SIZE     14U
#define COMM_TELEMETRY_PAYLOAD_SIZE   18U

typedef enum
{
  COMM_MESSAGE_COMMAND = 0x01U,
  COMM_MESSAGE_TELEMETRY = 0x02U
} CommMessageType_t;

typedef enum
{
  COMM_COMMAND_ARM_REQUEST = (1U << 0),
  COMM_COMMAND_DEPTH_HOLD_ENABLE = (1U << 1),
  COMM_COMMAND_EMERGENCY_STOP = (1U << 2)
} CommCommandFlags_t;

typedef enum
{
  COMM_STATUS_ARMED = (1U << 0),
  COMM_STATUS_COMMAND_LINK_OK = (1U << 1),
  COMM_STATUS_FAILSAFE_ACTIVE = (1U << 2),
  COMM_STATUS_DEPTH_VALID = (1U << 3),
  COMM_STATUS_ATTITUDE_VALID = (1U << 4),
  COMM_STATUS_SENSOR_FAULT = (1U << 5),
  COMM_STATUS_CONTROL_FAULT = (1U << 6),
  COMM_STATUS_RX_CRC_ERROR_SEEN = (1U << 7),
  COMM_STATUS_IMU_DETECTED = (1U << 8),
  COMM_STATUS_DEPTH_SENSOR_DETECTED = (1U << 9),
  COMM_STATUS_YAW_VALID = (1U << 10),
  COMM_STATUS_MAGNETOMETER_DETECTED = (1U << 11)
} CommStatusFlags_t;

/* In-memory control snapshot. Do not transmit this structure with memcpy. */
typedef struct
{
  uint16_t control_flags;
  int16_t surge_permille;
  int16_t sway_permille;
  int16_t heave_permille;
  int16_t yaw_permille;
  int32_t depth_setpoint_mm;
  uint32_t received_at_ms;
  uint8_t sequence;
} ControlCommand_t;

/* In-memory telemetry snapshot. Do not transmit this structure with memcpy. */
typedef struct
{
  uint32_t uptime_ms;
  int32_t depth_mm;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint16_t status_flags;
  uint8_t last_command_sequence;
} TelemetrySnapshot_t;

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */
