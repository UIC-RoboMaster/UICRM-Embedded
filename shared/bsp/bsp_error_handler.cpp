#include "bsp_error_handler.h"

void bsp_error_handler(const char* func, int line, const char* msg) {
  print("[ERROR at ");
  print("%s:", func);
  print("%d] ", line);
  print("%s\r\n", msg);
  return;
}
