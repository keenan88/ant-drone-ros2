#ifndef MACROS_H_
#define MACROS_H_

#include <Arduino.h>
#include "micro_ros.h"


#define RC_CHECK(fn)                           \
  {                                            \
    rcl_ret_t temp_rc = fn;                    \
    if ((temp_rc != RCL_RET_OK)) {             \
      esp_restart();                           \
    }                                          \
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

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
  } \
}

#endif /* MACROS_H_ */