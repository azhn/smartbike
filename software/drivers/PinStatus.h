#ifndef PINSTATUS_H_
#define PINSTATUS_H_

#include <stdbool.h>
#include <stdint.h>

static uint32_t _pin_status = 0x0000;

static void initializePinStatus() {
    _pin_status = 0x0000;    
}

static void setPinStatus(uint8_t pin_num, bool value){
    if(value) { // set value to 1
        _pin_status |= (1<<pin_num);
    } else { // set value to 0
        _pin_status &= ( ~(1<<pin_num) );
    }
}

static bool getPinStatus(uint8_t pin_num){
    return ( (_pin_status & 1<<pin_num) );
}

static bool getPinStatusClear(uint8_t pin_num){
    bool ret = ( (_pin_status & 1<<pin_num) );
    _pin_status &= ( ~(1<<pin_num) );
    return ret;
}

// TODO: Do we need to check if our flag handlers haven't been called?
static void state_update_flags(State* state) {
    uint8_t i;
    for (i=0; i<_NUM_FLAGS; ++i) {
        state->flags[i] |= getPinStatusClear(state->pin_mappings[i]);
    }
}


#endif // PINSTATUS_H_
