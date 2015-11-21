#include "AccelerometerControl.h"
#include "adxl362.h"


/*
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
*/
#define SOFT_RESET 0x1F
#define RESET_CODE 0x52 // for soft_reset reg

#define MEASUREMENT_MODE 0x02

#define POWER_CTL 0x2D
#define WAKEUP_MODE_EN 0x08

#define XDATA_L 0x0E
#define XDATA_H 0x0F
#define YDATA_L 0x10
#define YDATA_H 0x11
#define ZDATA_L 0x12
#define ZDATA_H 0x13

#define XDATA 0x08
#define YDATA 0x09
#define ZDATA 0x0A


/*****************************************************************************
  Initialization
 *****************************************************************************/

// Note: Remember to use SPI with accelerometer and the correct pins for those

/* config registers for accelerometer */
void config_accelerometer(){
  /* 0x20 - Activity Threshold Register (L) */
  /* 0x21 - Activity Threshold Register (H) */
  //adxl362_set_activity_threshold(uint16_t act_threshold);

  /* 0x22 - Activity Time Register (TIME_ACT) */
  adxl362_set_activity_time(0x00);

  /* 0x23 - Inactivity Threshold Registers (L) */
  /* 0x24 - Inactivity Threshold Registers (H) */
  //adxl362_set_inactivity_threshold(uint16_t inact_threshold);

  /* 0x25 - Inactivity Time Registers (L) */
  /* 0x26 - Inactivity Time Registers (H) */
  adxl362_set_inactivity_time(0x00);

  /* 0x27 - Activity/Inactivity Control Registers */
  //  ALREADY DONE in adxl362_config_interrupt_mode()

  /* 0x28 - FIFO Control Register */
  /* 0x29 - FIFO Samples Register */
  //  ALREADY DONE in adxl362_config_FIFO()

  /* 0x2A - INTMAP1 */
  /* 0x2B - INTMAP2 */
  //  ALREADY DONE in adxl362_config_INTMAP()
  
  /* 0x2C - Filter Control Register */
  //  ALREADY DONE in adxl362_config_measurement_range()

  /* 0x2D - Power Control Register */
  //  ALREADY DONE in adxl362_accelerometer_init
}

/* initialize the accelerometer */
void initializeAccelerometer( ) {

  // initialize SPI
  spi_init();

  // send a soft reset to the accelerometer
  uint8_t data[1] = {RESET_CODE};
  spi_write_reg(SOFT_RESET, data, 1);

  //wait for device to be reset
  for (int i = 0; i < 100; i++);

  // the accel_init function only configs the 0x2D reg, no others
  // SO, configure the rest of the accelerometer settings before
  //    doing the power control register (which turns measurment on)
  //  (0x20 - 0x26 regs)
  config_accelerometer();

  // configure interrupts to be off, default mode (for now) - (0x27 reg)
  adxl362_config_interrupt_mode( adxl362_INTERRUPT_DEFAULT, 0, 0 );

  // configure FIFO mode to be off so we sample in real-time (0x28, 0x29 regs)
  adxl362_config_FIFO( adxl362_DISABLE_FIFO, 0, 0 );

  // setup data ready interrupt to int pin 1 (maps data ready status to int1 pin)
  //    (0x2A, 0x2B registers)
  adxl362_interrupt_map_t dataReadyIntMap;
  memset(&dataReadyIntMap, 0, sizeof dataReadyIntMap);
  dataReadyIntMap.DATA_READY = true;
  adxl362_config_INTMAP(&dataReadyIntMap, true);

  // set measurement range to 2G (0x2C reg)
  adxl362_config_measurement_range( adxl362_MEAS_RANGE_2G );

  // initialize accel to measurement mode (with normal noise levels)
  //adxl362_accelerometer_init( adxl362_NOISE_NORMAL, 1, 0, 0 );
  //  DOING IT MANUALLY SO SOFTRESET NOT DONE AT END:
  data[0] = MEASUREMENT_MODE;
  data[0] = data[0] | (adxl362_NOISE_NORMAL << 4);
  spi_write_reg(POWER_CTL, data, 1);



  // RANDOM STUFF:
  //adxl362_activity_inactivity_interrupt_enable();
  // read X axis to clear data ready interrupt
  // readAxisX();

}


/*****************************************************************************
  Read Accelerometer Data
 *****************************************************************************/

/* combine two 8-bit values to form a 16-bit */
uint16_t convertTo16Bit(uint8_t * msb, uint8_t * lsb) {
  return ( (*msb << 8) | *lsb );
}

// Do we want these functions to return the values or pass in a
//   pointer to a state struct and update the value in there...? 

/* read accelerometer x axis */
int16_t readAxisX( ) {

  // create temp variable to return
  uint8_t x_data[2];

  // read the data
  spi_read_reg(XDATA_L, x_data, 2);

  // return the data
  //   NOTE: second 8-bit value read should be MSB
  return( convertTo16Bit(&x_data[1], &x_data[0]) );

}

/* read accelerometer y axis */
int16_t readAxisY( ) {

  // create temp variable to return
  uint8_t y_data[2];

  // read the data
  spi_read_reg(YDATA_L, y_data, 2);

  // return the data
  //  NOTE: second 8-bit value read should be MSB
  return( convertTo16Bit(&y_data[1], &y_data[0]) );

}

/* read accelerometer z axis */
int16_t readAxisZ( ) {

  // create temp variable to return
  uint8_t z_data[2];

  // read the data
  spi_read_reg(ZDATA_L, z_data, 2);

  // return the data
  //  NOTE: second 8-bit value read should be MSB
  return( convertTo16Bit(&z_data[1], &z_data[0]) );

}

/* update all data */
void updateAccelerometerState( AccelerometerState * state ) {
  // read data from each axis
  /*adxl362_sample_accel_word( state->_axis_x,
			     state->_axis_y,
			     state->_axis_z
			    );
  */
  // update the timestamp
  //state->timestamp = 
  //	This has to be updated using an internal timer...(?)
}

/* read accelerometer state */
uint8_t readAccelStatus( ){
  uint8_t data[1];
  spi_read_reg(0x0B, data, 1);

  return data[0];
}

