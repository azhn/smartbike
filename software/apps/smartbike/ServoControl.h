#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
                               Types  
*****************************************************************************/

typedef enum {BACK=0, FRONT=1, _NUM_SERVOS=2} ServoPosition;

typedef struct {
    ServoPosition _pos;
    int _curr_angle;
    int _max_angle;
    int _min_angle;
} Servo;

Servo* servos[_NUM_SERVOS];

/****************************************************************************
                           Initialization
*****************************************************************************/

/* NOTE: need to remember to initialize I2C to PWM driver and
         set the correct pins.
*/
/* initialize the specified servo */
void initializeServo(ServoPosition pos);




/*****************************************************************************
                           Servo Read/Write
*****************************************************************************/

/* rotate to the specified angle */
void rotateServo(ServoPosition pos, int angle);

#endif // SERVOCONTROL_H_
