#include <stdbool.h>
//#include <assert.h>
#include "AccelerometerControl.h"
#include "AccelDataDriver.h"


void initializeDataBank(bool readX, bool readY, bool readZ) {
    uint8_t i; 
    for (i=0; i<_NUM_ACCEL_DATA; ++i) {
        _AccelDataBank._accel_poll[i] = false;
    }
    if (readX) {
        _AccelDataBank._accel_poll[DATA_X] = true;
    }

    if (readY) {
        _AccelDataBank._accel_poll[DATA_Y] = true;
    }

    if (readZ) {
        _AccelDataBank._accel_poll[DATA_Z] = true;
    }
}
int16_t readData(AccelDataType type) {
    //assert (type < _NUM_ACCEL_DATA);
    if (type == DATA_X) {
        return readAxisX();
    } else if (type == DATA_Y) {
        return readAxisY();
    } else if (type == DATA_Z) {
        return readAxisZ();
    }
    return (int16_t)ACCEL_DATA_ERROR;
}

void populateAccelDataBank() {
    int8_t i;
    for (i=0; i < _NUM_ACCEL_DATA; ++i) {
        if (_AccelDataBank._accel_poll[i]) {
            // Increment counter in the beginning
            _AccelDataBank._internal_count[i]++;
            // greater than ensures that we don't throw out our n-sampled value
            if (_AccelDataBank._internal_count[i] > MAX_DATA_COUNT) {
                _AccelDataBank._internal_count[i] = 1;

                _AccelDataBank._accel_data[i] = readData(i);

            } else {
                _AccelDataBank._accel_data[i] += readData(i);
            }
        }
    }
}

void setPollAccelData(AccelDataType type) {
   // assert(type < _NUM_ACCEL_DATA);
    _AccelDataBank._accel_poll[type] = true;
}

void unsetPollAccelData(AccelDataType type) {
   // assert(type < _NUM_ACCEL_DATA);
    _AccelDataBank._accel_poll[type] = false;
}


bool grabAccelData(AccelDataType type, int16_t* out, uint16_t *ppos) {
   // assert(out != NULL);
    if (_AccelDataBank._accel_poll[type] &&
        _AccelDataBank._internal_count[type] == MAX_DATA_COUNT) {
        *out = _AccelDataBank._accel_data[type] >> DATA_BITSHIFT_DIVISOR;
        if (ppos != NULL) {
            (*ppos)++;
        }
        return true;
    }
    return false;
}
