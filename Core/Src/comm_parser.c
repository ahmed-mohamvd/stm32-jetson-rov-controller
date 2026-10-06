#include "comm_parser.h"

#include <stddef.h>
#include <string.h>

enum
{
  HEADER_VERSION_INDEX = 0,
  HEADER_TYPE_INDEX = 1,
  HEADER_LENGTH_INDEX = 2,
  HEADER_SEQUENCE_INDEX = 3,
  HEADER_SIZE = 4
};

static void CommParser_ResetFrame(CommParser_t *parser)
{
  parser->state = COMM_PARSER_WAIT_SOF_1;
  parser->header_index = 0U;
  parser->payload_index = 0U;
  parser->received_crc_low = 0U;
}

static uint16_t CommParser_CrcUpdate(uint16_t crc, uint8_t byte)
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

static uint16_t CommParser_CalculateCrc(const CommParser_t *parser)
{
  uint16_t crc = 0xFFFFU;
  uint8_t index;

  for (index = 0U; index < HEADER_SIZE; ++index)
  {
    crc = CommParser_CrcUpdate(crc, parser->header[index]);
  }

  for (index = 0U; index < parser->header[HEADER_LENGTH_INDEX]; ++index)
  {
    crc = CommParser_CrcUpdate(crc, parser->payload[index]);
  }

  return crc;
}

static uint16_t CommParser_ReadU16Le(const uint8_t *bytes)
{
  return ((uint16_t)bytes[0]) |
         ((uint16_t)bytes[1] << 8);
}

static uint32_t CommParser_ReadU32Le(const uint8_t *bytes)
{
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) |
         ((uint32_t)bytes[3] << 24);
}

static void CommParser_DecodeCommand(const CommParser_t *parser,
                                     uint32_t receivedAtMs,
                                     ControlCommand_t *command)
{
  command->control_flags = CommParser_ReadU16Le(&parser->payload[0]);
  command->surge_permille = (int16_t)CommParser_ReadU16Le(&parser->payload[2]);
  command->sway_permille = (int16_t)CommParser_ReadU16Le(&parser->payload[4]);
  command->heave_permille = (int16_t)CommParser_ReadU16Le(&parser->payload[6]);
  command->yaw_permille = (int16_t)CommParser_ReadU16Le(&parser->payload[8]);
  command->depth_setpoint_mm =
      (int32_t)CommParser_ReadU32Le(&parser->payload[10]);
  command->received_at_ms = receivedAtMs;
  command->sequence = parser->header[HEADER_SEQUENCE_INDEX];
}

void CommParser_Init(CommParser_t *parser)
{
  if (parser == NULL)
  {
    return;
  }

  (void)memset(parser, 0, sizeof(*parser));
  CommParser_ResetFrame(parser);
}

CommParseResult_t CommParser_PushByte(CommParser_t *parser,
                                     uint8_t byte,
                                     uint32_t receivedAtMs,
                                     ControlCommand_t *command)
{
  uint16_t calculatedCrc;
  uint16_t receivedCrc;

  if (parser == NULL)
  {
    return COMM_PARSE_NONE;
  }

  if ((parser->state != COMM_PARSER_WAIT_SOF_1) &&
      ((receivedAtMs - parser->last_byte_at_ms) > COMM_RX_FRAME_TIMEOUT_MS))
  {
    ++parser->stats.frame_timeouts;
    CommParser_ResetFrame(parser);
  }

  parser->last_byte_at_ms = receivedAtMs;

  switch (parser->state)
  {
    case COMM_PARSER_WAIT_SOF_1:
      if (byte == COMM_SOF_1)
      {
        parser->state = COMM_PARSER_WAIT_SOF_2;
      }
      break;

    case COMM_PARSER_WAIT_SOF_2:
      if (byte == COMM_SOF_2)
      {
        parser->header_index = 0U;
        parser->state = COMM_PARSER_READ_HEADER;
      }
      else if (byte != COMM_SOF_1)
      {
        parser->state = COMM_PARSER_WAIT_SOF_1;
      }
      break;

    case COMM_PARSER_READ_HEADER:
      parser->header[parser->header_index++] = byte;

      if (parser->header_index == HEADER_SIZE)
      {
        if (parser->header[HEADER_VERSION_INDEX] != COMM_PROTOCOL_VERSION)
        {
          ++parser->stats.version_errors;
          CommParser_ResetFrame(parser);
          return COMM_PARSE_ERROR_VERSION;
        }

        if (parser->header[HEADER_TYPE_INDEX] != COMM_MESSAGE_COMMAND)
        {
          ++parser->stats.type_errors;
          CommParser_ResetFrame(parser);
          return COMM_PARSE_ERROR_TYPE;
        }

        if (parser->header[HEADER_LENGTH_INDEX] != COMM_COMMAND_PAYLOAD_SIZE)
        {
          ++parser->stats.length_errors;
          CommParser_ResetFrame(parser);
          return COMM_PARSE_ERROR_LENGTH;
        }

        parser->payload_index = 0U;
        parser->state = COMM_PARSER_READ_PAYLOAD;
      }
      break;

    case COMM_PARSER_READ_PAYLOAD:
      parser->payload[parser->payload_index++] = byte;

      if (parser->payload_index == parser->header[HEADER_LENGTH_INDEX])
      {
        parser->state = COMM_PARSER_READ_CRC_LOW;
      }
      break;

    case COMM_PARSER_READ_CRC_LOW:
      parser->received_crc_low = byte;
      parser->state = COMM_PARSER_READ_CRC_HIGH;
      break;

    case COMM_PARSER_READ_CRC_HIGH:
      receivedCrc = ((uint16_t)parser->received_crc_low) |
                    ((uint16_t)byte << 8);
      calculatedCrc = CommParser_CalculateCrc(parser);

      if (receivedCrc != calculatedCrc)
      {
        ++parser->stats.crc_errors;
        CommParser_ResetFrame(parser);
        return COMM_PARSE_ERROR_CRC;
      }

      if (command == NULL)
      {
        CommParser_ResetFrame(parser);
        return COMM_PARSE_NONE;
      }

      CommParser_DecodeCommand(parser, receivedAtMs, command);
      ++parser->stats.valid_commands;
      CommParser_ResetFrame(parser);
      return COMM_PARSE_COMMAND_READY;

    default:
      CommParser_ResetFrame(parser);
      break;
  }

  return COMM_PARSE_NONE;
}

const CommParserStats_t *CommParser_GetStats(const CommParser_t *parser)
{
  if (parser == NULL)
  {
    return NULL;
  }

  return &parser->stats;
}
