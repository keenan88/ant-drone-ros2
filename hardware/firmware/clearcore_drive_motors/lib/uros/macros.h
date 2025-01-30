#include <Arduino.h>
#include "ClearCore.h"

#define RC_CHECK(fn)                                                    \
{                                                                     \
  rcl_ret_t temp_rc = fn;                                             \
  char buffer[150];                                                   \
  if (temp_rc == RCL_RET_INVALID_ARGUMENT) {                          \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_CHECK failed: null ptr arg in file: %s at line: %d. REBOOTING!", \
              __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
    delay(1000);                                                      \
    NVIC_SystemReset();                                               \
  }                                                                   \
  if (temp_rc == RCL_RET_ERROR) {                                     \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_CHECK failed in file: %s at line: %d. REBOOTING!",               \
              __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
    delay(1000);                                                      \
    NVIC_SystemReset();                                               \
  }                                                                   \
}

#define RC_SOFT_CHECK(fn)                                             \
{                                                                     \
  rcl_ret_t temp_rc = fn;                                             \
  char buffer[150];                                                   \
  if (temp_rc == RCL_RET_INVALID_ARGUMENT) {                          \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_SOFT_CHECK failed: null ptr arg in file: %s at line: %d. Continuing program.", \
              __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
    delay(1000);                                                      \
  }                                                                   \
  if (temp_rc == RCL_RET_ERROR) {                                     \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_SOFT_CHECK failed in file: %s at line: %d. Continuing program.",               \
              __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
    delay(1000);                                                      \
  }                                                                   \
}


#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)
