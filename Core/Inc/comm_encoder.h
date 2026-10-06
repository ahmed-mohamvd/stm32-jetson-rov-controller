#ifndef COMM_ENCODER_H
#define COMM_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_protocol.h"
#include <stdint.h>

#define COMM_TELEMETRY_FRAME_SIZE \
  (COMM_FRAME_OVERHEAD_SIZE + COMM_TELEMETRY_PAYLOAD_SIZE)

/*
 * Serializes one telemetry snapshot into a complete wire frame.
 * Returns COMM_TELEMETRY_FRAME_SIZE on success, or zero on invalid input.
 */
uint16_t CommEncoder_BuildTelemetry(const TelemetrySnapshot_t *telemetry,
                                    uint8_t sequence,
                                    uint8_t *frame,
                                    uint16_t frame_capacity);

#ifdef __cplusplus
}
#endif

#endif /* COMM_ENCODER_H */
