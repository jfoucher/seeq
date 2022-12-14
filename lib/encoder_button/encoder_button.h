
#ifndef ENCODER_BUTTON_H
#define ENCODER_BUTTON_H
#define DELAY_TIME 10000

absolute_time_t changed_at;
absolute_time_t pressed_at;
absolute_time_t released_at;
int pin;
bool pressed;
void encoder_init(int pin);
void encoder_changed(uint gpio, uint32_t events);

#endif