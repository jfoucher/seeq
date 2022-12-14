
#ifndef _KEYPAD_H
#define _KEYPAD_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"

void init_matrix(uint8_t rows, uint8_t cols, uint8_t row_pins[], uint8_t col_pins[]);
int scan_matrix(void);

#ifdef __cplusplus
 }
#endif

#endif
