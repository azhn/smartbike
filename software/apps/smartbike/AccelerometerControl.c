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


/*****************************************************************************
  Initialization
 *****************************************************************************/

// Note: Remember to use SPI with accelerometer and the correct pins for those

/* initialize the accelerometer */
void initializeAccelerometer( ) {

  // initialize accel to measurement mode (with normal noise levels)
  adxl362_accelerometer_init( adxl_362_NOISE_NORMAL, 1, 0, 0 );

  // set measurement range to 2G
  adxl362_config_measurement_range( adxl362_MEAS_RANGE_2G );

  // configure interrupts to be off, default mode (for now)
  adxl362_config_interrupt_mode( adxl362_INTERRUPT_DEFAULT, 0, 0 );

  // configure FIFO mode to be off so we sample in real-time
  adxl362_config_FIFO( adxl362_DISABLE_FIFO, 0, 0 );

}


/*****************************************************************************
  Read Accelerometer Data
 *****************************************************************************/

// Do we want these functions to return the values or pass in a
//   pointer to a state struct and update the value in there...? 

/* read accelerometer x axis */
int16_t readAxisX( ) {

  // create temp variable to return
  uint8_t x_data[2];

  // read the data
  adxl362_sample_accel_word_x( x_data );

  // return the data
  //   NOTE: second 8-bit value read should be MSB
  return( (x_data[1] << 8) | x_data[0] );

}

/* read accelerometer y axis */
int16_t readAxisY( ) {

  // create temp variable to return
  uint8_t y_data[2];

  // read the data
  adxl362_sample_accel_word_y( y_data );

  // return the data
  //  NOTE: second 8-bit value read should be MSB
  return( (y_data[1] << 8) | y_data[0] );

}

/* read accelerometer z axis */
int16_t readAxisZ( ) {

  // create temp variable to return
  uint8_t z_data[2];

  // read the data
  adxl362_sample_accel_word_z( z_data );

  // return the data
  //  NOTE: second 8-bit value read should be MSB
  return( (z_data[1] << 8) | z_data[0] );

}

/* update all data */
void updateAccelerometerState( AccelerometerState * state ) {
  // read data from each axis
  adxl362_sample_accel_word( state->axis_x,
			     state->axis_y,
			     state->axis_z
			    );
  // update the timestamp
  //state->timestamp = 
  //	This has to be updated using an internal timer...(?)
}

