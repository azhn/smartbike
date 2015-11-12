#ifndef SHIFTINGCONTROL_H_
#define SHIFTINGCONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* memset */
#include "spi_driver.h"

/*
Hex value for encoding gear setting
0x X X
   | |
   | --> rear gear value: 1 - largest gear, 5 - smallest gear
   |
    --> front gear value: 1 - largest gear, 2 - smallest gear
*/
typedef enum Gear { 
	A = 0x11, 
	B = 0x12,  
	C = 0x13,
	D = 0x14,
	E = 0x15,
	F = 0x21,
	G = 0x22,
	H = 0x23,
	I = 0x24,
	J = 0x25,
};

typedef struct {
    GearPWM 	_gear;
    uint16_t 	_speed_max;
    uint16_t 	_speed_min;
} GearSpeedState;



// typedef struct {
//     GearSpeedState	_speed_range_1;
//     GearSpeedState	_speed_range_2;
//     GearSpeedState	_speed_range_3;
//     GearSpeedState	_speed_range_4;
//     GearSpeedState	_speed_range_5;
// } GearSpeedMapping;

/*****************************************************************************
  Initialization
 *****************************************************************************/

/* initialize the speed-gear mapping */
void initializeShiftMapping( GearSpeedMapping * cfg );


/*****************************************************************************
  Get gear/speed information
 *****************************************************************************/

/* get front gear status */
uint16_t getCurrentFrontGear( );

/* get front gear status */
uint16_t getCurrentFrontGear( );

/*****************************************************************************
  Set gears
 *****************************************************************************/

/* set front gear */
void setCurrentFrontGear( Gear newFront );

/* set rear gear  */
void setCurrentRearGear( Gear newRear );

#endif // SHIFTINGCONTROL_H_
