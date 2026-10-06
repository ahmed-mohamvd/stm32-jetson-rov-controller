#ifndef COMM_PARSER_SELFTEST_H
#define COMM_PARSER_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Returns zero on success, otherwise the number of the failed test. */
uint8_t CommParser_RunSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PARSER_SELFTEST_H */
