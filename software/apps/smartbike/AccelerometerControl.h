#ifndef ACCELEROMETERCONTROL_H_
#define ACCELEROMETERCONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* memset */
#include "spi_driver.h"

typedef struct {
    int _timestamp;
    int16_t _axis_x;
    int16_t _axis_y;
    int16_t _axis_z;
} AccelerometerState;

typedef struct {
    AccelerometerState _sample1;
    AccelerometerState _sample2;
    AccelerometerState _sample3;
} AccelerometerRobustData;

/*****************************************************************************
  Initialization
 *****************************************************************************/

// Note: Remeber to use SPI with accelerometer and the correct pins for those

/* configure accelerometer registers */
void config_accelerometer();

/* initialize the accelerometer */
void initializeAccelerometer( );


/*****************************************************************************
  Read Accelerometer Data
 *****************************************************************************/
/* read accelerometer x axis */
int16_t readAxisX( );

/* read accelerometer y axis */
int16_t readAxisY( );

/* read accelerometer z axis */
int16_t readAxisZ( );

/* update all data */
void updateAccelerometerState( );

#endif // ACCELEROMETERCONTROL_H_
