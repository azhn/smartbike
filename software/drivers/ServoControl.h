#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"
#include "BikeState.h"

#define REAR_PWM 0
#define FRONT_PWM 1

void update_servos(State* state);

#endif // SERVOCONTROL_H_
