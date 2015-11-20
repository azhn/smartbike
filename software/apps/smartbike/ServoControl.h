#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"

#define NUM_GEARS 7
/****************************************************************************
                               Types  
*****************************************************************************/

typedef enum {REAR=0, FRONT=1, _NUM_SERVOS=2} ServoPosition;
typedef struct {
    pwm_address_t _address;
    pwm_t _pwm;
} Servo;

Servo _servo[_NUM_SERVOS];
/****************************************************************************
                                 Globals
*****************************************************************************/
const static pwm_address_t _servo_addresses[_NUM_SERVOS] = {0/*INSERT ADDR*/,1/*INSERT ADDR*/};

static uint8_t _curr_gear;


const static uint16_t REAR_GEAR_UP[NUM_GEARS] = {310, 340, 340, 352, 365, 372, 372};
const static uint16_t REAR_GEAR_DOWN[NUM_GEARS] = {304, 340, 315, 327, 340, 372, 372};
//uint16_t REAR_GEAR_DOWN[NUM_GEARS] = {304, 315, 327, 343, 372, 342, 372};
const static uint16_t FRONT_GEAR_UP[NUM_GEARS] = {400, 400, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/};
const static uint16_t FRONT_GEAR_DOWN[NUM_GEARS] = {400, 440, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/, 0/*INSERT PWM*/};

/****************************************************************************
                           Initialization
*****************************************************************************/

void initializeServos();

/*****************************************************************************
                           Servo shifting
*****************************************************************************/

// Save and retrieve from FRAM

void _save_servo_states();
void _retrieve_servo_states();

/*****************************************************************************
                           Servo shifting
*****************************************************************************/
void shift_up(uint8_t next_gear);
void shift_down(uint8_t next_gear);
void updateGears(uint8_t next_gear);

#endif // SERVOCONTROL_H_
