
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "pico/stdlib.h"



uint8_t * rows;
uint8_t * cols;

int nr = 0;
int nc = 0;
int mask = 0;

void init_matrix(uint8_t nrows, uint8_t ncols, uint8_t row_pins[], uint8_t col_pins[]) {
    
    rows = (uint8_t *)malloc( sizeof(uint8_t) * nrows );
    cols = (uint8_t *)malloc( sizeof(uint8_t) * ncols );
    nr = nrows;
    nc = ncols;
    // Set cols as outputs
    for (int i = 0; i < ncols; i++) {
        gpio_init(col_pins[i]);
        gpio_set_dir(col_pins[i], GPIO_IN);
        cols[i] = col_pins[i];
    }

    // Set rows as inputs
    for (int i = 0; i < nrows; i++) {
        gpio_init(row_pins[i]);
        gpio_set_dir(row_pins[i], GPIO_IN);
        gpio_pull_up(row_pins[i]);
        rows[i] = row_pins[i];
        mask |= 1 << row_pins[i];
    }
}

int scan_matrix(void) {
    int pressed = -1;
    for (int i = 0; i < nc; i++) {
        // Check if any attached row pin is low
        gpio_set_dir(cols[i], GPIO_OUT);
        gpio_put(cols[i], 0);

        for (int j = 0; j < nr; j++) {
            if (gpio_get(rows[j]) == 0) {
                pressed = i*nr + j; 
            }
        }
        gpio_set_dir(cols[i], GPIO_IN);
        if (pressed > -1) {
            return pressed;
        }
    }
    

    return pressed;
}