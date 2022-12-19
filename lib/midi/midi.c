#include <stdlib.h>
#include <stdint.h>
#include "midi.h"


#define CHANNEL 0
#define CABLE_NUM 0

void note_on(uint8_t midi_note, uint8_t velocity) {
  // Send Note On for current position at full velocity (127) on channel 1.
  uint8_t note_on[3] = { 0x90 | CHANNEL, midi_note, velocity };
  tud_midi_stream_write(CABLE_NUM, note_on, 3);
}

void note_off(uint8_t midi_note) {
  // Send Note On for current position at full velocity (127) on channel 1.
  uint8_t note_off[3] = { 0x80 | CHANNEL, midi_note, 0};
  tud_midi_stream_write(CABLE_NUM, note_off, 3);
}

uint8_t poll_midi_uart_rx(void * midi_uart_instance, uint8_t * rx)
{
    // Pull any bytes received on the MIDI UART out of the receive buffer and
    // send them out via USB MIDI on virtual cable 0
    uint8_t nread = midi_uart_poll_rx_buffer(midi_uart_instance, rx, sizeof(rx));
    return nread;

}