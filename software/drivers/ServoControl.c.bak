#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"
#include "LightControl.h"
#include "ServoControl.h"

void initializeGearControl() {
    int i;
    for (i=0; i<_NUM_SERVOS; ++i) {
        _servo[i]._pwm = _servo_addresses[i];
    }
    _retrieve_gear_states();

    setLEDLightState(_curr_gear, LIGHT_STATE_ON);
}

void shift_up(uint8_t next_gear) {
    _servo[REAR]._pwm = REAR_GEAR_UP[next_gear];
    _servo[FRONT]._pwm = FRONT_GEAR_UP[next_gear];
    pca9685_setPWM(_servo[REAR]._address, 0, _servo[REAR]._pwm);
    pca9685_setPWM(_servo[FRONT]._address, 0, _servo[FRONT]._pwm);
}

void shift_down(uint8_t next_gear) {
    _servo[REAR]._pwm = REAR_GEAR_DOWN[next_gear];
    _servo[FRONT]._pwm = FRONT_GEAR_DOWN[next_gear];
    pca9685_setPWM(_servo[REAR]._address, 0, _servo[REAR]._pwm);
    pca9685_setPWM(_servo[FRONT]._address, 0, _servo[FRONT]._pwm);
}

void updateGears(uint8_t next_gear)
{
    assert(next_gear < NUM_GEARS);
    if(next_gear > _curr_gear)
    {
        shift_up(next_gear);
    }
    else
    {
        shift_down(next_gear);
    }

    // Turn off prev LED gear state
    setLEDLightState(_curr_gear, LIGHT_STATE_OFF);
    _curr_gear = next_gear;
    setLEDLightState(_curr_gear, LIGHT_STATE_ON);

    _save_gear_states();
}

void _save_gear_states() {
    /* TODO:
     * 1) Serialize pwm for front and rear servos
     * 2) Serialize _curr_gear
     * 3) Save in FRAM
     */
}

void _retrieve_gear_states() {
    /* TODO:
     * 1) Deserialize pwm for front and rear servos
     * 2) Deserialize _curr_gear
     * 3) Save into _servo[x]._pwm and _curr_gear
     */
}
