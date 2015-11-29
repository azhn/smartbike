#ifndef ACCELTURNCONTROL_H_
#define ACCELTURNCONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* memset */
#include <math.h>

#include "spi_driver.h"
//#include "LightControl.h"
#include "LightAction.h"



/*****************************************************************************/
#define LEFT_BUTTON 0
#define RIGHT_BUTTON 1
#define NUM_TURN_BUTTONS 2
#define NUM_TURN_LIGHTS 2

#define ACCEL_OUT_THRESH 200
#define ACCEL_IN_THRESH 150
#define ACCEL_TILT_THRESH 100
/*****************************************************************************/

// state enums
typedef enum{
    OFF,        // OFF
    SIGNAL_R,   // Signal Right - check for entering right turn
    SIGNAL_L,   // Signal Left  - check for entering left turn
    RETURN_R,   // Return Right - check for returning from right turn
    RETURN_L,   // Return Left  - check for returning  from left turn
    _NUM_STATES
} TS_STATE;

// [0] - left, [1] - right light
LightAction light_output;

static TS_STATE BTN_STATES[_NUM_STATES][NUM_TURN_BUTTONS] =
{
	//    LB        RB          Curr State
	{  SIGNAL_L, SIGNAL_R  }, // Off
	{  SIGNAL_L, OFF       }, // SIGNAL_R
	{  OFF,      SIGNAL_R  }, // SIGNAL_L
	{  SIGNAL_L, OFF       }, // RETURN_R
	{  OFF,      SIGNAL_R  }  // RETURN_L
};

static bool LIGHT_STATES[_NUM_STATES][NUM_TURN_LIGHTS] =
{
	//   LL     RL        Curr State
	{  false, false  }, // Off
	{  false, true   }, // SIGNAL_R
	{  true,  false  }, // SIGNAL_L
	{  false, true   }, // RETURN_R
	{  true,  false  }  // RETURN_L
};

static TS_STATE curr_state = OFF;


// check_in/out_thresh variables
static const int16_t sampled_accel_x = 0;
static uint16_t thresh_out_count = 0;
static uint16_t thresh_in_count = 0;


/*****************************************************************************/
void reset_state();
bool check_in_thresh( bool right_btn_pressed, const int16_t * curr_x_val );
bool check_out_thresh( bool right_btn_pressed, const int16_t * curr_x_val );

void set_light_output();

// Check for any state transitions needed based on button presses
void btn_state_change( bool left_btn_pressed, bool right_btn_pressed );
void btn_state_change_alt( State* state );

// Perform threshold checking based on current state
LightAction do_state_action( int16_t accel_x_val );

#endif // ACCELTURNCONTROL_H_
