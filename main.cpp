/**
 * Copyright (c) 2021 Jonathan Foucher
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>



#define PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT 64


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



#define CHAR_BUFFER_ADDRESS 0x277
#define CHAR_BUFFER_NUM_ADDRESS 0xC6
// If this is active, then an overclock will be applied
// #define OVERCLOCK

// Delay startup by so many seconds
// #define START_DELAY 3



absolute_time_t start;

void core1_func();
static semaphore_t video_initted;

queue_t queue;

uint8_t sequence[16];
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
bool playing = true;
bool recording = false;

// uint8_t __time_critical_func(read6502)(uint16_t address) {
//     if (address == VIA2_PORTA2) {
//         address = VIA2_PORTA;
//     }
//     if (address == VIA2_T1CL && mpu_memory[VIA2_IFR]) {
//       mpu_memory[VIA2_IFR] = 0x00;
//     //   printf("timer int cleared by T1CL read\n");
//     }
//     // if (address == VIA2_PORTB || address == VIA2_PORTA) {
//     //     return 0;
//     // }
//     return mpu_memory[address];
// }

// def led_display(led):
//     # scan the keyboard matrix and turn on the correct LED
//     global previous_led
//     # exit if no change
//     if (previous_led == led):
//         return
    
//     #turn everything off
//     for pin in led_row_pins:
//         pin.value = False
//     for pin in led_col_pins:
//         pin.value = True
//     # turn our led on
//     if (led < 16 and led >= 0):
//         # turn on the led that should currently be on according to current_led
//         row = led // 4
//         col = led % 4
//         led_row_pins[row].value = True
//         led_col_pins[col].value = False
//     previous_led = led

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


int main() {
#ifdef OVERCLOCK
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    set_sys_clock_khz(260000, true);
#endif
stdio_init_all();

#ifdef START_DELAY
    for(uint8_t i = START_DELAY; i > 0; i--) {
        printf("Starting in %d \n", i);
        sleep_ms(1000);
    }
#endif
    printf("Initializing LEDs\n");
    init_leds();

    const uint PIN_A = 26;  // The A channel pin
    const uint PIN_B = 27;  // The B channel pin
    encoder::Encoder enc(pio0, 0, {PIN_A, PIN_B});

    queue_init(&queue, sizeof(tempo), 32);
    queue_try_add(&queue, &tempo);

    enc.init();
    enc.set_count(tempo/5);
    enc.setCallback(rotaryChangedCallback);

    // rotary_encoder_t *encoder = create_encoder(26, on_rotary_change);
    // encoder->rotation = tempo;
    // printf("Position: %d\n", encoder->rotation);

    playing = true;

    printf("starting second core\n");
    // create a semaphore to be posted when video init is complete
    sem_init(&video_initted, 0, 1);

    // launch all the video on core 1, so it isn't affected by USB handling on core 0
    multicore_launch_core1(core1_func);
    // wait for initialization of video to be complete
    sem_acquire_blocking(&video_initted);

    //hookexternal(callback);

    printf("second core started\n");

    last_play_time = get_absolute_time();

    while(1) {
        t = get_absolute_time();
        int64_t delta = absolute_time_diff_us(last_play_time, t);
        if (playing && delta > 60000000 / tempo) {
            last_play_time = t;
            
            led_display(play_step);
            play_step++;
            if (play_step > last_step) {
                play_step = first_step;
            }
        }

    }


    return 0;
}



