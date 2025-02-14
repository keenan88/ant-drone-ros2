#include "jack.hpp"
#include "uros_sub_jack_pos.h"
#include <Encoder.h>

// Motor driver pins
#define R_EN GPIO_NUM_17
#define L_EN GPIO_NUM_13
#define DOWN_PIN GPIO_NUM_19
#define UP_PIN GPIO_NUM_22

// Jack hall effect feedback
#define ENC_P1 GPIO_NUM_35
#define ENC_P2 GPIO_NUM_39

#define CURRENT_SENSE GPIO_NUM_15


int32_t jack_pos_ticks;
int32_t tolerance_ticks = 25;

Encoder* encoder = nullptr;

void go_up()
{
    digitalWrite(DOWN_PIN, LOW); // LOW impedance state drains line
    digitalWrite(UP_PIN, HIGH); // TODO - make up pin an analogue write to control upwards velocity, if needed.
}

void go_down()
{
    digitalWrite(UP_PIN, LOW); 
    digitalWrite(DOWN_PIN, HIGH); // High impedance state enables pull-up resistor. If mag sensor is engaged, line will be drained and no movement occurs.
}

void stop_movement()
{
    digitalWrite(UP_PIN, LOW); 
    digitalWrite(DOWN_PIN, LOW);
}

void go_to_height_ticks(int32_t desired_pos)
{
    int32_t curr_ticks = encoder -> read();

    if(desired_pos < curr_ticks - tolerance_ticks)
    {
        Serial.println("Going Down");
        go_down();
        
    }
    else if(desired_pos > curr_ticks + tolerance_ticks)
    {
        Serial.println("Going up");
        go_up();
    }
    else
    {
        Serial.println("Staying put");
        stop_movement();
    }

    Serial.println(curr_ticks);
}

void go_to_height_mm(int32_t desired_pos_mm)
{
    int32_t desired_pos = 33.5 * desired_pos_mm;
    int32_t curr_ticks = encoder -> read();

    if(desired_pos < curr_ticks - tolerance_ticks)
    {
        Serial.println("Going Down");
        go_down();
        
    }
    else if(desired_pos > curr_ticks + tolerance_ticks)
    {
        Serial.println("Going up");
        go_up();
    }
    else
    {
        Serial.println("Staying put");
        stop_movement();
    }

    Serial.println(curr_ticks);
}

void tick_jack() {
    int32_t desired_pos = get_jack_pos_setpoint();
    int32_t curr_pos_rel_to_start = encoder -> read();
    
    // jack_pos_ticks = jack_start_pos_ticks + curr_pos_rel_to_start;

    if(desired_pos < jack_pos_ticks - tolerance_ticks)
    {
        // TODO - write positive 24V to motor controller
    }
    else if(desired_pos > jack_pos_ticks + tolerance_ticks)
    {
        // TODO - negative 24V to motor controller
    }
    else
    {
        // TODO - 0V to motor controller

    }
}

void home_jack()
{
    Serial.println("Homing...");
    go_down();

    uint8_t mag_sensor_engaged = false;

    while(!mag_sensor_engaged)
    {
        mag_sensor_engaged = digitalRead(DOWN_PIN) == 0; // Line will be drained once magnetic sensor engages
        delay(10); // DOWN_PIN always returns 0 without delay.. not sure if reading drains line voltage a bit? 10ms delay is fine.
    }

    stop_movement();
    
    // Reset encoder so that home position is zeroe'd,
    if (encoder != nullptr) {
        delete encoder;
    }
    encoder = new Encoder(ENC_P1, ENC_P2); 

    Serial.print("Homing complete. Start position: ");
    Serial.println(encoder -> read());
    delay(2000);
}


void init_jack(){
    encoder = new Encoder(ENC_P1, ENC_P2);

    // Both enable pins must be high for motor to move in either direction. Disabling an enable pin disables movement in both directions. 
    pinMode(L_EN, OUTPUT); 
    pinMode(R_EN, OUTPUT);
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    pinMode(CURRENT_SENSE, INPUT);

    pinMode(DOWN_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(UP_PIN, OUTPUT);

    home_jack();

}

int32_t get_jack_pos()
{
    return jack_pos_ticks;
}