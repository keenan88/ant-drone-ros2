
#ifndef JACK_H
#define JACK_H

#include <stdint.h>

void tick_jack();

void init_jack();

int32_t get_jack_pos();

void go_up();

void go_to_height_ticks(int32_t desired_pos);

void go_to_height_mm(int32_t desired_pos_mm);

#endif