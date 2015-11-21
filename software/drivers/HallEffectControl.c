#include "HallEffectControl.h"

void wheel_speed_handler(uint32_t* out) {
    *out = 0; /* TODO: Assignment of millisecond value */
}

void pedalling_speed_hander(uint32_t* out) {
    *out = 0; /* TODO: Assignment of millisecond value */
}


uint32_t get_bike_speed(const uint32_t* prev_count, const uint32_t* curr_count) {
    // TODO: check if prev_count > curr_count, fix the counts
    return MM_PER_INT/(curr_count - prev_count);
}

float get_pedalling_speed(const uint32_t* prev_count, const uint32_t* curr_count) {
    // TODO: check if prev_count > curr_count, fix the counts
    return 1.0/(curr_count - prev_count);
}
