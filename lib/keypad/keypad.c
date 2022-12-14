
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
    // Set cols as inputs
    for (int i = 0; i < ncols; i++) {
        gpio_init(col_pins[i]);
        gpio_pull_up(col_pins[i]);
        gpio_set_dir(row_pins[i], GPIO_IN);
        cols[i] = col_pins[i];
        mask |= 1 << col_pins[i];
    }

    // Set rows as outputs
    for (int i = 0; i < nrows; i++) {
        gpio_init(row_pins[i]);
        gpio_set_dir(row_pins[i], GPIO_OUT);
        gpio_put(row_pins[i], 1);
        rows[i] = row_pins[i];
    }
}

int scan_matrix(void) {
    for (int i = 0; i < nr; i++) {
        // Set all row pins high except for the one we are currently reading
        for (int n = 0; n < nr; n++) {
            gpio_put(rows[n], n==i ? 0 : 1);
        }

        // Check if any attached row pin is low

        if (gpio_get_all() && mask) {
            // We have at least one low pin
            // Check all row pins to see which one is low
            for (int j = 0; j < nc; j++) {
                if (gpio_get(cols[j])) {
                    return i*nc + j; 
                }
            }
        }
    }

    return -1;
}