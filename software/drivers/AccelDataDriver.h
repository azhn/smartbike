#ifndef ACCELDATADRIVER_H_
#define ACCELDATADRIVER_H_

#include <stdbool.h>

#define DATA_BITSHIFT_DIVISOR 2 // x >> 2 = x/4
#define MAX_DATA_COUNT (1<<DATA_BITSHIFT_DIVISOR)

// 1<<15 arbitrarily large negative number accelerometer can't produce
#define ACCEL_DATA_ERROR (1<<(sizeof(int16_t)*8-1)) 

typedef enum AccelDataType {
    DATA_X,
    DATA_Y,
    DATA_Z,
     _NUM_ACCEL_DATA
} AccelDataType;

typedef struct AccelData {
    int16_t _accel_data[_NUM_ACCEL_DATA];
    bool _accel_poll[_NUM_ACCEL_DATA];
    uint8_t _internal_count[_NUM_ACCEL_DATA];
} AccelData;

AccelData _AccelDataBank;

int16_t readData(AccelDataType type);



/******************************************************************************
* USER FUNCTIONS
******************************************************************************/
// Will populate internal data bank. Should be called whenever Accelerometer
// has available data in the main loop.
void populateAccelDataBank();

// Will poll accelerometer and save necessary values for this data type
void setPollAccelData(AccelDataType type);


// Will disable polling of accelerometer for this data type
void unsetPollAccelData(AccelDataType type);

// Will only populate (out) if data is available. As an indicator for completion
// one can pass the pointer to a counter (ppos), and it will be incremented only
// upon a successful data grab.
bool grabAccelData(AccelDataType type, int16_t* out, uint16_t *ppos);


#endif // DATADRIVER_H_
