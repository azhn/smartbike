#include <assert.h>
#include "AccelTurnControl.h"

void reset_state() {
  thresh_out_count    = 0;
  thresh_in_count     = 0;
}

bool check_in_thresh( bool right_btn_pressed, const int16_t * curr_x_val ){
  // beginning of turn - check if tilt is outisde zero-thresh range
  if((abs( *curr_x_val - sampled_accel_x ) > ACCEL_OUT_THRESH) ){
      if(
        (!right_btn_pressed && (*curr_x_val < sampled_accel_x)) ||
        ( right_btn_pressed && (*curr_x_val > sampled_accel_x)) ){
          thresh_out_count++;  
      }

      if(thresh_out_count > ACCEL_TILT_THRESH){
          return true;
      }
  }

  return false;
}

bool check_out_thresh( bool right_btn_pressed, const int16_t * curr_x_val){
  //check for return into zero-thresh
  if((abs( *curr_x_val - sampled_accel_x ) <= ACCEL_IN_THRESH) ){
      thresh_in_count++;

      if( thresh_in_count > ACCEL_TILT_THRESH){
          return true;
      }
  }

  return false;
}

void set_light_output() {
  // left lights
  light_output[0].pos = LEFT_TURN;
  if(LIGHT_STATES[curr_state][0]){
    light_output[0].state = LIGHT_STATE_BLINKING;  
  } else {
    light_output[0].state = LIGHT_STATE_OFF;
  }
  
  // right lights
  light_output[1].pos = RIGHT_TURN;
  if(LIGHT_STATES[curr_state][1]){
    light_output[1].state = LIGHT_STATE_BLINKING;  
  } else {
    light_output[1].state = LIGHT_STATE_OFF;
  }
}

// Check for any state transitions needed based on button presses
void btn_state_change( bool left_btn_pressed, bool right_btn_pressed ) {
  // must be called when a button is pressed
  //assert(!left_btn_pressed && !right_btn_pressed);
  if(!left_btn_pressed && !right_btn_pressed){
    return;
  }

  // handle simultaneous press by resetting to OFF
  if(left_btn_pressed && right_btn_pressed){
    curr_state = OFF;
    return;
  }

  // get next state from our BTN_STATES array
  if(left_btn_pressed && !right_btn_pressed){
    curr_state = BTN_STATES[ curr_state ][0];  
  } else if(!left_btn_pressed && right_btn_pressed){
    curr_state = BTN_STATES[ curr_state ][1];
  } else {
    assert(false);
  }
}

// Perform threshold checking based on current state
LightAction * do_state_action( int16_t accel_x_val ) {
  switch( curr_state ){
    case OFF: // All lights off
      //do nothing: btn_state_change controls state transition from
      //  OFF to something else based on button press
      break;
    case SIGNAL_R: // Right btn pressed, check for entering turn before going to next state
      if(check_in_thresh( true, accel_x_val )){
        curr_state = RETURN_R;
      }
      break;
    case SIGNAL_L: // Left btn pressed, check for entering turn before going to next state
      if(check_in_thresh( false, accel_x_val )){
          curr_state = RETURN_L;
      }
      break;
    case RETURN_R: // Entered right turn, check for return
      if(check_out_thresh( true, accel_x_val )){
        curr_state = OFF;
        reset_state();
      }
      break;
    case RETURN_L:
      if(check_out_thresh( false, accel_x_val )){
        curr_state = OFF;
        reset_state();
      }
      break;
    default:
      assert(false);
      break;
  }

  // set light output based on which state we're in
  set_light_output();

  // return the light actions array
  return light_output;
}