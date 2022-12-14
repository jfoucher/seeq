#include <stdlib.h>
#include <stdint.h>

#include "lib/midi_uart_lib/midi_uart_lib.h"

#ifndef MIDI_H
#define MIDI_H


void note_on(uint8_t);
void note_off(uint8_t);
uint8_t poll_midi_uart_rx(void *, uint8_t *);
#endif