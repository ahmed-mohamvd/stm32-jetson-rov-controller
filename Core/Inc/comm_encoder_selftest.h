#ifndef COMM_ENCODER_SELFTEST_H
#define COMM_ENCODER_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Returns zero on success, otherwise the number of the failed test. */
uint8_t CommEncoder_RunSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_ENCODER_SELFTEST_H */
