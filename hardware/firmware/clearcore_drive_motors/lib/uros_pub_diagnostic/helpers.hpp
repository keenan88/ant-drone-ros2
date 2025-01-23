#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "uros_config.h"

class StringMessage {
 private:
  static const size_t MAX_SIZE = kStringLength;
  char buffer[MAX_SIZE];

 public:
  StringMessage() { buffer[0] = '\0'; }

  StringMessage(const char *str) { assign(str); }

  void assign(const char *str) {
    strncpy(buffer, str, MAX_SIZE - 1);
    buffer[MAX_SIZE - 1] = '\0';  // Ensure null-terminated
  }

  void format(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, MAX_SIZE, fmt, args);
    va_end(args);
    buffer[MAX_SIZE - 1] = '\0';  // Ensure null-terminated
  }

  const char *c_str() const { return buffer; }

  // Overload the assignment operator for convenience
  StringMessage &operator=(const char *str) {
    assign(str);
    return *this;
  }
};

#endif  // HELPERS_HPP