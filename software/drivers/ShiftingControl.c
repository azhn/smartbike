#include "ShiftingControl.h"
#include "pwm.h"

static GearSpeedMapping * _cfg = NULL;

// store current gear status
static Gear _currGear = 0;

// gear pwm mappings
static uint8_t  _rearGearCount = 0;
static uint8_t  _frontGearCount = 0;

static uint16_t _frontPWMShiftUp[] = NULL;
static uint16_t _rearPWMShiftUp[] = NULL;

static uint16_t _frontPWMShiftDown[] = NULL;
static uint16_t _rearPWMShiftDown[] = NULL;

/*****************************************************************************
  Initialization
 *****************************************************************************/

/* initialize the shifting algorithm */
void initializeShiftMapping( GearSpeedMapping * cfg, 
                            uint16_t * rearPWM, uint8_t rearCount, 
                            uint16_t * frontPWM, uint8_t frontCount ) {
  // store the cofiguration
  _cfg = cfg; 

  // set gear count
  _rearGearCount  = rearCount;
  _frontGearCount = frontCount;

  // store gear-pwm mappings
  _rearPWM  = rearPWM;  
  _frontPWM = frontPWM;

  // initialize PWM
  pca9685_init(&twi_instance);
  pca9685_setPWMFreq(50);

  // intialize servos to previous
  //pca9685_setPin(1, _rearPWM[0], 0);
  //pca9685_setPin(1, _rearPWM[0], 0);
}


/*****************************************************************************
  Get gear information
 *****************************************************************************/

/* get current gear status */
Gear getCurrentGear(){
  return _currGear;
}

/* get front gear status (0 means invalid) */
uint8_t getCurrentFrontGearNum(){
  // first 4 bits represent Front 
  return ( (_currGear & 0xF0) >> 4 );
}

/* get rear gear status (0 means invalid) */
uint8_t getCurrentRearGear(){
  // _currRearGear stores what gear the rear is on currently
  return ( (_currGear & 0x0F) );
}

/*****************************************************************************
  Set gears
 *****************************************************************************/

/* set front gear */
void setCurrentFrontGear( Gear newFront ){
  // store the new value that we set
  _currFrontGear = newFront;
  
  // 
}

/* set rear gear  */
void setCurrentRearGear( Gear newRear ){
  // store the new value that we set
  _currRearGear = newRear;

  // 
}
