#include "encoder_button.h"

void encoder_init(int p) {
  // Init encoder button
    pin = p;
    gpio_init(pin);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &encoder_changed);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_put(25, false);
}

void encoder_changed(uint gpio, uint32_t events) {
    if (absolute_time_diff_us(changed_at, get_absolute_time()) > DELAY_TIME) {
        pressed = !gpio_get(pin);
        gpio_put(25, pressed);
        changed_at = get_absolute_time();

        if (pressed) {
            pressed_at = changed_at;
        } else {
            released_at = changed_at;
        }
    }
}