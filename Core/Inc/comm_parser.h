#ifndef COMM_PARSER_H
#define COMM_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_protocol.h"
#include <stdint.h>

#define COMM_RX_FRAME_TIMEOUT_MS  100U

typedef enum
{
  COMM_PARSER_WAIT_SOF_1 = 0,
  COMM_PARSER_WAIT_SOF_2,
  COMM_PARSER_READ_HEADER,
  COMM_PARSER_READ_PAYLOAD,
  COMM_PARSER_READ_CRC_LOW,
  COMM_PARSER_READ_CRC_HIGH
} CommParserState_t;

typedef enum
{
  COMM_PARSE_NONE = 0,
  COMM_PARSE_COMMAND_READY,
  COMM_PARSE_ERROR_VERSION,
  COMM_PARSE_ERROR_TYPE,
  COMM_PARSE_ERROR_LENGTH,
  COMM_PARSE_ERROR_CRC,
  COMM_PARSE_ERROR_TIMEOUT
} CommParseResult_t;

typedef struct
{
  uint32_t valid_commands;
  uint32_t version_errors;
  uint32_t type_errors;
  uint32_t length_errors;
  uint32_t crc_errors;
  uint32_t frame_timeouts;
} CommParserStats_t;

typedef struct
{
  CommParserState_t state;
  uint8_t header[4];
  uint8_t header_index;
  uint8_t payload[COMM_MAX_PAYLOAD_SIZE];
  uint8_t payload_index;
  uint8_t received_crc_low;
  uint32_t last_byte_at_ms;
  CommParserStats_t stats;
} CommParser_t;

void CommParser_Init(CommParser_t *parser);

CommParseResult_t CommParser_PushByte(CommParser_t *parser,
                                     uint8_t byte,
                                     uint32_t received_at_ms,
                                     ControlCommand_t *command);

const CommParserStats_t *CommParser_GetStats(const CommParser_t *parser);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PARSER_H */
