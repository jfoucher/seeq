#include <stdlib.h>
#include <stdint.h>

#include "lib/midi_uart_lib/midi_uart_lib.h"

#ifndef MIDI_H
#define MIDI_H


void note_on(uint8_t midi_note, uint8_t velocity);
void note_off(uint8_t midi_note);
uint8_t poll_midi_uart_rx(void * midi_uart_instance, uint8_t * rx, int bsize);
#endif