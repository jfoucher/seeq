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
#include "hardware/flash.h"
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
#include "lib/keypad/keypad.h"

#include "lib/midi/midi.c"
#include "lib/encoder_button/encoder_button.h"
#include "lib/encoder_button/encoder_button.c"
#include "lib/midi_uart_lib/midi_uart_lib.h"

#define MIDI_UART_NUM 1
#define CABLE_NUM 0 // MIDI jack associated with USB endpoint
#define CHANNEL 0 // 0 for channel 1
#define ENCODER_PIN_A 26
#define ENCODER_PIN_B 27

#define FLASH_TARGET_OFFSET (512 * 1024)

 
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

const uint MIDI_UART_TX_GPIO = 16;
const uint MIDI_UART_RX_GPIO = 17;

static void *midi_uart_instance;

void println(const char * format, ... );
void print(const char * format, ... );

void __time_critical_func(core1_func)();
void save_data_task(void);
static semaphore_t video_initted;

queue_t queue;
queue_t note_queue;

uint8_t sequence[16] = {74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85};
uint8_t active[16] = {true, true, true, true,true, true, true, true,true, true, true, true,true, true, true, true};
int8_t pressed_key = -1;
uint8_t first_step = 0;
uint8_t last_step = 15;
uint8_t new_last_step = 15;
uint8_t current_led = 0;
uint8_t previous_led = 20;
uint8_t play_step = 0;
uint8_t led_row_pins[4] = {1, 0, 3, 2};
uint8_t led_col_pins[4] = {7, 6, 5, 4};

uint16_t tempo = 120;
uint16_t old_tempo = 0;


int note_display = -1;
int old_note_display = 0;

absolute_time_t t;
absolute_time_t last_play_time;
absolute_time_t last_off_time;
absolute_time_t last_keypad_time;
absolute_time_t last_save_time;
absolute_time_t last_manual_save_time;
bool playing = true;
bool recording = false;

bool changed_enc_count_on_press = false;

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


struct SeeqData {
  uint16_t header;
  uint16_t tempo;
  uint8_t first_step;
  uint8_t last_step;
  uint8_t sequence[16];
  uint8_t active[16];
  
};


SeeqData seeqData;

bool saveData() {
    seeqData.tempo = tempo;
    seeqData.header = 0x55AA;
    for (int i = 0; i< 16; i++) {
      seeqData.sequence[i] = sequence[i];
      seeqData.active[i] = active[i];
    }
    seeqData.last_step = last_step;
    seeqData.first_step = first_step;

    uint8_t* dataAsBytes = (uint8_t*) &seeqData;
    int dataSize = sizeof(seeqData);

    println("save tempo %i", seeqData.tempo);
    println("save last_step %i", seeqData.last_step);
    
    int writeSize = (dataSize / FLASH_PAGE_SIZE) + 1; // how many flash pages we're gonna need to write
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1; // how many flash sectors we're gonna need to erase
    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, dataAsBytes, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);

    for (int i = 0; i < dataSize; ++i) {
        if (dataAsBytes[i] != flash_target_contents[i])
            return false;
    }
    println("ls from flash %02X", flash_target_contents[5]);
    
    return true;
}

void readData() {
    memcpy(&seeqData, flash_target_contents, sizeof(seeqData));
    if (seeqData.header != 0x55AA) {
      return;
    }
    if (seeqData.tempo > 0 && seeqData.tempo <= 5000) {
      tempo = seeqData.tempo;
    }
    if (seeqData.last_step >= 0 && seeqData.last_step <= 15) {
      last_step = (uint8_t)seeqData.last_step;
      new_last_step = last_step;
    }

    if (seeqData.first_step >= 0 && seeqData.first_step <= 15) {
      first_step = (uint8_t)seeqData.first_step;
    }

    for (int i=0; i < 16; i++) {
      if (seeqData.sequence[i] <= 0x7F) {
        sequence[i] = seeqData.sequence[i];
      }
      if (seeqData.active[i] <= 0x7F) {
        active[i] = (bool)seeqData.active[i];
      }
    }
}

void led_display(uint8_t led) {
    if (led == previous_led) {
        return ;
    }
    previous_led = led;
    //turn off all leds
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

const char * notes[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};


void note_text(char * note_str, uint8_t midi_note) {
  int n = ((int)midi_note) - 12;
  int octave = n / 12;
  if (n < 0) {
    octave = -1;
  }
  const char * note = notes[midi_note%12];

  sprintf(note_str, "%s%i", note, octave);
}

void __time_critical_func(core1_func)() {
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
        uint16_t new_tempo = old_tempo;
        while (queue_try_remove(&queue, &new_tempo)){}
        int new_note_display = old_note_display;
        while (queue_try_remove(&note_queue, &new_note_display)){}

        bool clear = old_note_display != new_note_display || old_tempo != new_tempo;

        if (clear) {
          oled.clear();

          char intStr[4];
          sprintf(intStr, "%d",  new_tempo);
          oled.print(0, 0, (uint8_t *)intStr);
          old_tempo = new_tempo;

          char str[4];

          if (new_note_display >= 0 && new_note_display <= 0x7F) {
            note_text(str, new_note_display);
            //println("note text %s", str);
          } else {
            sprintf(str, "%s",  "  - ");
          }

          oled.print(64, 0, (uint8_t *)str);
          old_note_display = new_note_display;
        
          oled.show();
        }
    }
}

void print(const char * format, ... ) {
    char text[64];
    va_list arguments;
    va_start(arguments, format);
    // https://mylifeforthecode.github.io/creating-a-custom-printf-function-in-c/
    vsprintf(text, format, arguments);
    va_end(arguments);
    tud_cdc_write(text, strlen(text));
    tud_cdc_write_flush();
}

void println(const char * format, ... ) {
    char text[64];
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
  if (pressed_key >= 0) {
    // A key is pressed, so change corresponding note
    if (x < 0) {
        x = 0;
        enc->set_count(x);
    }
    if (x > 127) {
        x = 127;
        enc->set_count(x);
    }

    sequence[pressed_key] = x;
    if (!playing) {
      note_on(sequence[pressed_key]);
      //sleep_ms(100);
      note_off(sequence[pressed_key]);
    }
  } else if (pressed) {
    // Encoder is pressed, change last step
    if (x < 0) {
        x = 0;
        enc->set_count(x);
    }
    if (x > 15) {
        x = 15;
        enc->set_count(x);
    }
    new_last_step = x;
  } else {
    if (x < 1) {
        x = 1;
        enc->set_count(x);
    }
    if (x > 5000) {
        x = 5000;
        enc->set_count(x);
    }
    tempo = (uint16_t)x;

    queue_try_add(&queue, &tempo);
  }
}


void midi_task(void);
void cdc_task(void);
void keypad_task(void);


encoder::Encoder enc(pio0, 0, {ENCODER_PIN_A, ENCODER_PIN_B});


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

  init_matrix(4, 4, row_pins, col_pins);

  midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);

  pressed = false;

  //Init encoder button
  encoder_init(22);



  enc.init();
  enc.set_count(tempo);
  enc.setCallback(rotaryChangedCallback);
  playing = true;



  // create a semaphore to be posted when video init is complete
  sem_init(&video_initted, 0, 1);

  // launch all the video on core 1, so it isn't affected by USB handling on core 0
  multicore_launch_core1(core1_func);
  // wait for initialization of video to be complete
  sem_acquire_blocking(&video_initted);
  // restore saved data
  readData();

  queue_init(&queue, sizeof(tempo), 32);
  queue_try_add(&queue, &tempo);

  queue_init(&note_queue, sizeof(uint8_t), 32);

  queue_try_add(&note_queue, &note_display);

  last_play_time = get_absolute_time();
  last_off_time = get_absolute_time();
  last_keypad_time = get_absolute_time();
  last_save_time = get_absolute_time();
  last_manual_save_time = get_absolute_time();
  


  while (1)
  {
    tud_task(); // tinyusb device task
    keypad_task();
    midi_task();
    cdc_task();
    save_data_task();
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

  led_display(current_led);

  if (playing && delta > 60000000 / tempo) {
    last_play_time = t;

    if (active[play_step]) {
      note_on(sequence[play_step]);
      current_led = play_step;
      if (pressed_key < 0) {
        note_display = sequence[play_step];
        //println("showing note %i", note);
        queue_try_add(&note_queue, &note_display);
      }
    } else {
      current_led = -1;
      if (pressed_key < 0) {
        note_display = -1;
        //println("showing note %i", note);
        queue_try_add(&note_queue, &note_display);
      }
    }

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

  

  if (pressed && !changed_enc_count_on_press && pressed_key < 0) {
    println("preprare to change new last step");
    new_last_step = last_step;
    enc.set_count(last_step);
    changed_enc_count_on_press = true;
  } else if (!pressed && changed_enc_count_on_press) {
    println("done changing  new last step");
    enc.set_count(tempo);
    changed_enc_count_on_press = false;
  }

  if (!pressed && new_last_step != last_step && play_step == 0) {
    println("setting last step to new last step %d %d", last_step, new_last_step);
    note_off(sequence[last_step]);
    last_step = new_last_step;
  }

  if (pressed && pressed_key < 0) {
    led_display(new_last_step);
  }

  if (pressed && absolute_time_diff_us(pressed_at, get_absolute_time()) > 2000000 && last_step == new_last_step && absolute_time_diff_us(last_manual_save_time, get_absolute_time()) > 1000000) {
    // Long press without rotation, save data
    last_manual_save_time = get_absolute_time();
    saveData();
  }

  // Button tapped event
  if (!pressed 
    && to_ms_since_boot(released_at) > to_ms_since_boot(pressed_at) 
    && absolute_time_diff_us(pressed_at, released_at) < 500000
  ) {
    pressed_at = t;
    if (pressed_key < 0) {
      // If no key is pressed change state
      
      println("change state");
      playing = !playing;

      if (!playing) {
        int note = sequence[play_step];
        queue_try_add(&note_queue, &note);
      }
    } else {
      // If a key is pressed, toggle note
      active[pressed_key] = !active[pressed_key];
    }
  }

  uint8_t rx[48];

  poll_midi_uart_rx(midi_uart_instance, rx);

    // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);
}

bool removed_note = true;


void keypad_task() {
  if (pressed_key >= 0) {
    led_display((uint8_t)pressed_key);
  }
  int64_t delta = absolute_time_diff_us(last_play_time, get_absolute_time());
  if (delta > 10000) {
    int result = scan_matrix();
    if (result >= 0) {
      pressed_key = (int8_t)result;
      enc.set_count(sequence[pressed_key]);
      int note = (int)sequence[pressed_key];
      if (note != note_display) {
        queue_try_add(&note_queue, &note);
        note_display = note;
        removed_note = false;
      }
      
    } else {
      pressed_key = -1;
      int pk = pressed_key;
      if (pk != note_display && !removed_note) {
        queue_try_add(&note_queue, &pk);
        note_display = pk;
        removed_note = true;
      }
      uint8_t ls = last_step;
      if (new_last_step != ls) {
        ls = new_last_step;
      }
      enc.set_count(pressed ? ls : tempo);
    }
  }
}

void save_data_task() {
  int64_t delta = absolute_time_diff_us(last_save_time, get_absolute_time());

  // Autosave every minute

  if (delta > 60000000) {
    if (saveData()) {
      println("saved data");
    } else {
      println("save data fail");
    }
    last_save_time = get_absolute_time();
    
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