#include "jack.hpp"
#include "uros_sub_jack_pos.h"
#include <Preferences.h>
#include <Encoder.h>

#define ENC_P1 GPIO_NUM_23
#define ENC_P2 GPIO_NUM_18

int32_t jack_start_pos_ticks;
int32_t jack_pos_ticks;
int32_t tolerance_ticks = 250;
bool jack_at_target = false;


Encoder* encoder = nullptr;
Preferences* preferences = nullptr;

void tick_jack() {
    int32_t desired_pos = get_jack_pos_setpoint();
    int32_t curr_pos_rel_to_start = encoder -> read();
    
    jack_pos_ticks = jack_start_pos_ticks + curr_pos_rel_to_start;

    if(desired_pos < jack_pos_ticks - tolerance_ticks)
    {
        // TODO - write positive 24V to motor controller
        jack_at_target = false;
    }
    else if(desired_pos > jack_pos_ticks + tolerance_ticks)
    {
        // TODO - negative 24V to motor controller
        jack_at_target = false;
    }
    else
    {
        // TODO - 0V to motor controller

        if(!jack_at_target)
        { // Write jack position once, at the end of every movement
            preferences -> putInt("last_jack_pos", jack_pos_ticks);
        }
        jack_at_target = true;
    }

}

void init_jack(){
    encoder = new Encoder(ENC_P1, ENC_P2);

    preferences = new Preferences();
    preferences -> begin("storage", false);  // "storage" is the namespace, false means read/write mode'
    jack_start_pos_ticks = preferences -> getInt("last_jack_pos", 0);
}

int32_t get_jack_pos()
{
    return jack_pos_ticks;
}