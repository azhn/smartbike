#include <stdbool.h>
#include <assert.h>
#include "AccelerometerControl.h"
#include "AccelDataDriver.h"


int16_t readData(AccelDataType type) {
    assert (type < _NUM_ACCEL_DATA);
    if (type == DATA_X) {
        return readAxisX();
    } else if (type == DATA_Y) {
        return readAxisY();
    } else if (type == DATA_Z) {
        return readAxisZ();
    }
    return ACCEL_DATA_ERROR;
}
void populateAccelDataBank() {
    int8_t i;
    for (i=0; i < _NUM_ACCEL_DATA; ++i) {
        if (_accel_poll[i]) {
            // Increment counter in the beginning
            _AccelDataBank._internal_count[i]++;
            // greater than ensures that we don't throw out our n-sampled value
            if (_AccelDataBank._interal_count[i] > MAX_DATA_COUNT) {
                _AccelDataBank._interal_count = 0;
                _AccelDataBank._accel_data[i] = readData((Acceldata)i);
            } else {
                _AccelDataBank._accel_data[i] += readData((Acceldata)i);
            }
        }
    }
}

void grabAccelData(AccelDataType type, uint16_t* out, uint16_t *ppos) {
    assert(out != NULL);
    if (_AccelDataBank._accel_poll[type] &&
        _AccelDataBank._interal_count[type] == MAX_DATA_COUNT) {
        *out = _AccelDataBank._accel_data[type] >> DATA_BITSHIFT_DIVISOR;
        if (ppos != NULL) {
            ppos++;
        }
    }
}
