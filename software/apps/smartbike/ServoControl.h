#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"
#include "BikeState.h"

#define REAR_PWM 0
#define FRONT_PWM 1

const uint16_t REAR_GEAR_UP[6] = {310, 340, 340, 352, 365, 372};
const uint16_t REAR_GEAR_DOWN[6] = {308, 340, 315, 327, 340, 372};
const uint16_t FRONT_GEAR_UP[6] = {400, 400, 313, 313, 313, 313};
const uint16_t FRONT_GEAR_DOWN[6] = {400, 440, 313, 313, 313, 313};



void update_servos(State* state);

#endif // SERVOCONTROL_H_
