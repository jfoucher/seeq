/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jonathan Foucher - Six Pixels
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/sync.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "lib/encoder/encoder.hpp"
#include "lib/OLED/OLED.h"
#include "lib/OLED/font/Cherry_Cream_Soda_Regular_16.h"
#include "lib/OLED/font/Dialog_bold_16.h"

#include "pico/binary_info.h"


#include "bsp/board.h"
#include "tusb.h"

#include "lib/midi/midi.h"
// #include "lib/keypad/keypad.h"

#include "lib/midi/midi.c"
#include "lib/encoder_button/encoder_button.h"
#include "lib/encoder_button/encoder_button.c"
#include "lib/midi_uart_lib/midi_uart_lib.h"

#define MIDI_UART_NUM 1
const uint MIDI_UART_TX_GPIO = 16;
const uint MIDI_UART_RX_GPIO = 17;

static void *midi_uart_instance;



#define CABLE_NUM 0 // MIDI jack associated with USB endpoint
#define CHANNEL 0 // 0 for channel 1

void core1_func();
static semaphore_t video_initted;

queue_t queue;

uint8_t sequence[16] = {74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85};
uint8_t active[16];
uint8_t first_step = 0;
uint8_t last_step = 15;
uint8_t current_led = 0;
uint8_t previous_led = 20;
uint8_t play_step = 0;
uint8_t led_row_pins[4] = {1, 0, 3, 2};
uint8_t led_col_pins[4] = {7, 6, 5, 4};

int tempo = 120;


absolute_time_t t;
absolute_time_t last_play_time;
absolute_time_t last_off_time;
bool playing = true;
bool recording = false;

// Keypad pins

uint8_t row_pins[] = {9, 8, 11, 10};
uint8_t col_pins[] = {15, 14, 13, 12};



void init_leds() {
    for(int i = 0; i<4;i++) {
        uint8_t p = led_row_pins[i];

        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, 0);
    }
    for(int i = 0; i<4;i++) {
        uint8_t p = led_col_pins[i];

        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, 1);
    }
}

void led_display(uint8_t led) {
    // if (led == previous_led) {
    //     return ;
    // }
    // previous_led = led;
    for(int i = 0; i<4;i++) {
        uint8_t p = led_row_pins[i];
        gpio_put(p, 0);
    }
    for(int i = 0; i<4;i++) {
        uint8_t p = led_col_pins[i];
        gpio_put(p, 1);
    }
    if (led < 16 && led >= 0) {
        uint8_t row = led / 4;
        uint8_t col = led % 4;
        gpio_put(led_row_pins[row], 1);
        gpio_put(led_col_pins[col], 0);
    }
}

int old_tempo = 0;

void core1_func() {
    // sleep and release on core 1
    
        // SCL, SDA, Width, Height, Frequency, I2C Port
    OLED oled(21, 20, 128, 32, 100000, true, i2c0);

    // Draw two circles
    oled.drawFilledCircle(100, 4, 4);

    // Print custom font string
    oled.setFont(&Dialog_bold_16);
    uint8_t string2[] = "SEEQ";
    oled.print(60, 5, string2);
    // Draw a line
    oled.drawFastHLine(0, 31, 128);
    oled.drawFastHLine(0, 0, 128);
    oled.show();
    sleep_ms(3000);


    sem_release(&video_initted);
    // Turn scroll ON
    // oled.setScrollDir(true);
    // oled.isScroll(true);
    while (true) {
        tight_loop_contents();
        int new_tempo;
        while (queue_try_remove(&queue, &new_tempo)){}

        if (old_tempo != new_tempo) {
            // uint8_t string2[] = "Tempo";
            // oled.print(0, 0, string2);
            char intStr[4];
            sprintf(intStr, "%d",  new_tempo);
            oled.clear();

            oled.print(0, 0, (uint8_t *)intStr);
            // printf("got tempo from queue %d\n", new_tempo);
            old_tempo = new_tempo;
            oled.show();
        }
    }
}


void println(const char * format, ... ) {
    char text[12];
    va_list arguments;
    va_start(arguments, format);
    // https://mylifeforthecode.github.io/creating-a-custom-printf-function-in-c/
    vsprintf(text, format, arguments);
    va_end(arguments);
    strcat(text, "\n");
    tud_cdc_write(text, strlen(text));
    tud_cdc_write_flush();
}


void rotaryChangedCallback(encoder::Encoder * enc)
{
    int x = enc->count();
    if (x <= 1) {
        x = 1;
        enc->set_count(x);
    }
    if (x > 1000) {
        x = 1000;
        enc->set_count(x);
    }
    tempo = x * 5;
    queue_try_add(&queue, &tempo);
}


void midi_task(void);
void cdc_task(void);
void keypad_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  #ifdef OVERCLOCK
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    set_sys_clock_khz(260000, true);
  #endif
  stdio_init_all();


  board_init();

  tusb_init();


  init_leds();

  // init_matrix(4, 4, row_pins, col_pins);

  midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);


  pressed = true;
  encoder_init(22);



  const uint PIN_A = 26;  // The A channel pin
  const uint PIN_B = 27;  // The B channel pin
  encoder::Encoder enc(pio0, 0, {PIN_A, PIN_B});

  queue_init(&queue, sizeof(tempo), 32);
  queue_try_add(&queue, &tempo);

  enc.init();
  enc.set_count(tempo/5);
  enc.setCallback(rotaryChangedCallback);
  playing = true;

  printf("starting second core\n");
  // create a semaphore to be posted when video init is complete
  sem_init(&video_initted, 0, 1);

  // launch all the video on core 1, so it isn't affected by USB handling on core 0
  multicore_launch_core1(core1_func);
  // wait for initialization of video to be complete
  sem_acquire_blocking(&video_initted);

  printf("second core started\n");

  last_play_time = get_absolute_time();
  last_off_time = get_absolute_time();

  while (1)
  {
    tud_task(); // tinyusb device task
    keypad_task();
    midi_task();
    cdc_task();
  }


  return 0;
}


//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

void midi_task(void)
{
  t = get_absolute_time();

  int64_t delta = absolute_time_diff_us(last_play_time, t);
  int64_t delta_off = absolute_time_diff_us(last_off_time, t);

      // elif(playing and tempo > 0 and delta > 30/tempo and delta_off > 60/tempo):
      //   # send note off halfway to the next note
      //   last_note_off_time = t
        
      //   ps = play_step - 1
      //   if ps < 0:
      //       ps = last_step
            
      //   if (active[ps]):
      //       note_off(sequence[ps])

  if (playing && delta > 60000000 / tempo) {
    last_play_time = t;
    led_display(play_step);
    note_on(sequence[play_step]);
    println("playing %d", sequence[play_step]);

    play_step++;
    if (play_step > last_step) {
        play_step = first_step;
    }
  } else if (playing && delta > 30000000 / tempo and delta_off > 60000000 / tempo) {
    last_off_time = t;
    uint8_t ps = play_step - 1;
    if (ps == 0xFF) {
      ps = last_step;
    }
    note_off(sequence[ps]);
  }

  // Change state if button is taped

  if (!pressed 
    && to_ms_since_boot(released_at) > to_ms_since_boot(pressed_at) 
    && absolute_time_diff_us(pressed_at, released_at) < 500000
  ) {
    pressed_at = t;
    println("change state");
    playing = !playing;
  }

  uint8_t rx[48];

  poll_midi_uart_rx(midi_uart_instance, rx);

    // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);



}

void keypad_task() {
  int result = -1;//scan_matrix();

  // println("scan result: %d", result);
  if (result >= 0) {
    play_step = result;
  }
  
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{

}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;

}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{

}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      // read data
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      (void) count;

      // Echo back
      // Note: Skip echo by commenting out write() and write_flush()
      // for throughput test e.g
      //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}