#include <Arduino.h>
#include "ClearCore.h"

#define RC_CHECK(fn)                                                    \
{                                                                     \
  rcl_ret_t temp_rc = fn;                                             \
  char buffer[150];                                                   \
  if (temp_rc != RCL_RET_OK) {                          \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_CHECK failed (code %d): in file: %s at line: %ld. REBOOTING!", \
              temp_rc, __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
    delay(1000);                                                      \
    NVIC_SystemReset();                                               \
  }                                                                   \
}

#define RC_SOFT_CHECK(fn)                                             \
{                                                                     \
  rcl_ret_t temp_rc = fn;                                             \
  char buffer[150];                                                   \
  if (temp_rc != RCL_RET_OK) {                          \
    snprintf(buffer, sizeof(buffer),                                  \
              "RC_SOFT_CHECK failed (code %d): in file: %s at line: %ld. Continuing!", \
              temp_rc, __FILE__, __LINE__);                                     \
    ConnectorUsb.SendLine(buffer);                                        \
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
