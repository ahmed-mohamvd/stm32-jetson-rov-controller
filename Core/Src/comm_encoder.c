#include "comm_encoder.h"

#include <stddef.h>

enum
{
  FRAME_VERSION_INDEX = 2,
  FRAME_TYPE_INDEX = 3,
  FRAME_LENGTH_INDEX = 4,
  FRAME_SEQUENCE_INDEX = 5,
  FRAME_PAYLOAD_INDEX = 6
};

static void CommEncoder_WriteU16Le(uint8_t *bytes, uint16_t value)
{
  bytes[0] = (uint8_t)(value & 0xFFU);
  bytes[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CommEncoder_WriteU32Le(uint8_t *bytes, uint32_t value)
{
  bytes[0] = (uint8_t)(value & 0xFFU);
  bytes[1] = (uint8_t)((value >> 8) & 0xFFU);
  bytes[2] = (uint8_t)((value >> 16) & 0xFFU);
  bytes[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint16_t CommEncoder_CrcUpdate(uint16_t crc, uint8_t byte)
{
  uint8_t bit;

  crc ^= ((uint16_t)byte << 8);

  for (bit = 0U; bit < 8U; ++bit)
  {
    if ((crc & 0x8000U) != 0U)
    {
      crc = (uint16_t)((crc << 1) ^ 0x1021U);
    }
    else
    {
      crc = (uint16_t)(crc << 1);
    }
  }

  return crc;
}

static uint16_t CommEncoder_CalculateCrc(const uint8_t *bytes,
                                         uint16_t byteCount)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;

  for (index = 0U; index < byteCount; ++index)
  {
    crc = CommEncoder_CrcUpdate(crc, bytes[index]);
  }

  return crc;
}

uint16_t CommEncoder_BuildTelemetry(const TelemetrySnapshot_t *telemetry,
                                    uint8_t sequence,
                                    uint8_t *frame,
                                    uint16_t frameCapacity)
{
  uint8_t *payload;
  uint16_t crc;

  if ((telemetry == NULL) || (frame == NULL) ||
      (frameCapacity < COMM_TELEMETRY_FRAME_SIZE))
  {
    return 0U;
  }

  frame[0] = COMM_SOF_1;
  frame[1] = COMM_SOF_2;
  frame[FRAME_VERSION_INDEX] = COMM_PROTOCOL_VERSION;
  frame[FRAME_TYPE_INDEX] = COMM_MESSAGE_TELEMETRY;
  frame[FRAME_LENGTH_INDEX] = COMM_TELEMETRY_PAYLOAD_SIZE;
  frame[FRAME_SEQUENCE_INDEX] = sequence;

  payload = &frame[FRAME_PAYLOAD_INDEX];
  CommEncoder_WriteU32Le(&payload[0], telemetry->uptime_ms);
  CommEncoder_WriteU32Le(&payload[4], (uint32_t)telemetry->depth_mm);
  CommEncoder_WriteU16Le(&payload[8], (uint16_t)telemetry->roll_cdeg);
  CommEncoder_WriteU16Le(&payload[10], (uint16_t)telemetry->pitch_cdeg);
  CommEncoder_WriteU16Le(&payload[12], (uint16_t)telemetry->yaw_cdeg);
  CommEncoder_WriteU16Le(&payload[14], telemetry->status_flags);
  payload[16] = telemetry->last_command_sequence;
  payload[17] = 0U;

  crc = CommEncoder_CalculateCrc(&frame[FRAME_VERSION_INDEX],
                                 4U + COMM_TELEMETRY_PAYLOAD_SIZE);
  CommEncoder_WriteU16Le(&frame[FRAME_PAYLOAD_INDEX +
                                  COMM_TELEMETRY_PAYLOAD_SIZE],
                         crc);

  return COMM_TELEMETRY_FRAME_SIZE;
}
