#include "comm_encoder_selftest.h"

#include "comm_encoder.h"

#include <stddef.h>
#include <string.h>

uint8_t CommEncoder_RunSelfTest(void)
{
  static const uint8_t expectedFrame[COMM_TELEMETRY_FRAME_SIZE] = {
    0xAAU, 0x55U, 0x01U, 0x02U, 0x12U, 0x9CU,
    0x40U, 0xE2U, 0x01U, 0x00U,
    0xAAU, 0x05U, 0x00U, 0x00U,
    0x85U, 0xFFU,
    0xD2U, 0x00U,
    0x0FU, 0x0EU,
    0x1AU, 0x00U,
    0x2AU, 0x00U,
    0x36U, 0xE1U
  };
  TelemetrySnapshot_t telemetry = {
    .uptime_ms = 123456U,
    .depth_mm = 1450,
    .roll_cdeg = -123,
    .pitch_cdeg = 210,
    .yaw_cdeg = 3599,
    .status_flags = 0x001AU,
    .last_command_sequence = 0x2AU
  };
  uint8_t frame[COMM_TELEMETRY_FRAME_SIZE];
  uint16_t frameLength;

  /* Test 1: the complete encoded frame must match the protocol example. */
  frameLength = CommEncoder_BuildTelemetry(&telemetry, 0x9CU, frame,
                                           sizeof(frame));
  if ((frameLength != COMM_TELEMETRY_FRAME_SIZE) ||
      (memcmp(frame, expectedFrame, sizeof(expectedFrame)) != 0))
  {
    return 1U;
  }

  /* Test 2: reject a destination buffer that is one byte too small. */
  if (CommEncoder_BuildTelemetry(&telemetry, 0x9CU, frame,
                                 sizeof(frame) - 1U) != 0U)
  {
    return 2U;
  }

  /* Test 3: reject NULL input pointers instead of dereferencing them. */
  if ((CommEncoder_BuildTelemetry(NULL, 0x9CU, frame, sizeof(frame)) != 0U) ||
      (CommEncoder_BuildTelemetry(&telemetry, 0x9CU, NULL,
                                  sizeof(frame)) != 0U))
  {
    return 3U;
  }

  return 0U;
}
