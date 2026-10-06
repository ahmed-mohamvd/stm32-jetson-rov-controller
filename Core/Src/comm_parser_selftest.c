#include "comm_parser_selftest.h"

#include "comm_parser.h"

#include <stdbool.h>
#include <string.h>

static const uint8_t validCommandFrame[] = {
  0xAAU, 0x55U, 0x01U, 0x01U, 0x0EU, 0x2AU,
  0x01U, 0x00U, 0xFAU, 0x00U, 0x9CU, 0xFFU, 0x00U, 0x00U,
  0x32U, 0x00U, 0xDCU, 0x05U, 0x00U, 0x00U, 0x10U, 0x78U
};

static CommParseResult_t FeedBytes(CommParser_t *parser,
                                   const uint8_t *bytes,
                                   uint32_t byteCount,
                                   uint32_t startTimeMs,
                                   ControlCommand_t *command)
{
  CommParseResult_t result = COMM_PARSE_NONE;
  uint32_t index;

  for (index = 0U; index < byteCount; ++index)
  {
    result = CommParser_PushByte(parser, bytes[index],
                                 startTimeMs + index, command);
  }

  return result;
}

static bool CommandMatchesExample(const ControlCommand_t *command)
{
  return ((command->control_flags == COMM_COMMAND_ARM_REQUEST) &&
          (command->surge_permille == 250) &&
          (command->sway_permille == -100) &&
          (command->heave_permille == 0) &&
          (command->yaw_permille == 50) &&
          (command->depth_setpoint_mm == 1500) &&
          (command->sequence == 0x2AU));
}

uint8_t CommParser_RunSelfTest(void)
{
  static const uint8_t noise[] = {0x00U, 0x12U, 0xAAU, 0x01U, 0x77U};
  static const uint8_t invalidLengthHeader[] = {
    0xAAU, 0x55U, 0x01U, 0x01U, 0x0FU, 0x01U
  };
  uint8_t badCrcFrame[sizeof(validCommandFrame)];
  CommParser_t parser;
  ControlCommand_t command;
  CommParseResult_t result;
  const CommParserStats_t *stats;

  /* Test 1: accept and decode a known-good command. */
  CommParser_Init(&parser);
  (void)memset(&command, 0, sizeof(command));
  result = FeedBytes(&parser, validCommandFrame, sizeof(validCommandFrame),
                     10U, &command);
  if ((result != COMM_PARSE_COMMAND_READY) || !CommandMatchesExample(&command))
  {
    return 1U;
  }

  /* Test 2: reject the same command when its CRC is corrupted. */
  CommParser_Init(&parser);
  (void)memcpy(badCrcFrame, validCommandFrame, sizeof(badCrcFrame));
  badCrcFrame[sizeof(badCrcFrame) - 1U] ^= 0x01U;
  result = FeedBytes(&parser, badCrcFrame, sizeof(badCrcFrame),
                     50U, &command);
  stats = CommParser_GetStats(&parser);
  if ((result != COMM_PARSE_ERROR_CRC) || (stats == NULL) ||
      (stats->crc_errors != 1U) || (stats->valid_commands != 0U))
  {
    return 2U;
  }

  /* Test 3: ignore unrelated bytes and then find the next SOF. */
  CommParser_Init(&parser);
  result = FeedBytes(&parser, noise, sizeof(noise), 100U, &command);
  if (result != COMM_PARSE_NONE)
  {
    return 3U;
  }
  result = FeedBytes(&parser, validCommandFrame, sizeof(validCommandFrame),
                     110U, &command);
  if ((result != COMM_PARSE_COMMAND_READY) || !CommandMatchesExample(&command))
  {
    return 3U;
  }

  /* Test 4: time out a partial frame, then accept the following full frame. */
  CommParser_Init(&parser);
  result = FeedBytes(&parser, validCommandFrame, 10U, 200U, &command);
  if (result != COMM_PARSE_NONE)
  {
    return 4U;
  }
  result = FeedBytes(&parser, validCommandFrame, sizeof(validCommandFrame),
                     400U, &command);
  stats = CommParser_GetStats(&parser);
  if ((result != COMM_PARSE_COMMAND_READY) || (stats == NULL) ||
      (stats->frame_timeouts != 1U))
  {
    return 4U;
  }

  /* Test 5: reject a bad length and recover for the next valid frame. */
  CommParser_Init(&parser);
  result = FeedBytes(&parser, invalidLengthHeader,
                     sizeof(invalidLengthHeader), 500U, &command);
  if (result != COMM_PARSE_ERROR_LENGTH)
  {
    return 5U;
  }
  result = FeedBytes(&parser, validCommandFrame, sizeof(validCommandFrame),
                     510U, &command);
  stats = CommParser_GetStats(&parser);
  if ((result != COMM_PARSE_COMMAND_READY) || (stats == NULL) ||
      (stats->length_errors != 1U))
  {
    return 5U;
  }

  return 0U;
}
