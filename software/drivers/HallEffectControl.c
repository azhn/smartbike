#include "HallEffectControl.h"

void wheel_interrupt_handler(State* bike) {
    bike->last_milli = bike->curr_milli;
	bike->curr_milli = get_millis();
	bike->flags[WHEEL_FLAG] = true;
}

void pedalling_interrupt_handler(State* bike) {
    bike->flags[PEDAL_FLAG] = true;
}


void update_target_state(State* bike) {
	int mm_per_ms;

        bike->last_delta = bike->curr_delta;
        bike->curr_delta = bike->curr_milli - bike->last_milli;

        mm_per_ms = MM_PER_INT / bike->curr_delta;

        if(bike->manual_mode)
        {
            if(bike->flags[SHIFT_UP_FLAG])
            {
                ++bike->target_gear;
            }
            if(bike->flags[SHIFT_DOWN_FLAG] && bike->target_gear > 0)
            {
                --bike->target_gear;
            }
        }
        else 
        {
            bike->target_gear = mm_per_ms >> 1;
        }

        if(bike->target_gear > 5) bike->target_gear = 5;
        if(bike->target_gear < 0) bike->target_gear = 0;
	bike->flags[WHEEL_FLAG] = false;
        bike->flags[SHIFT_DOWN_FLAG] = false;
        bike->flags[SHIFT_UP_FLAG] = false;
}

float get_pedalling_speed(const uint32_t* prev_count, const uint32_t* curr_count) {
    // TODO: check if prev_count > curr_count, fix the counts
    return 1.0/(curr_count - prev_count);
}
