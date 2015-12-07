#include <assert.h>
#include "AccelTurnControl.h"
#include "led.h"


void update_handle_turn_status(State * bike, bool right_turn) {
  if(!right_turn) { // left turn
    if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_LEFT_TURN_FLAG])) {
        bike->handle_left_turn = true;
    } else {
        bike->handle_left_turn = false;
    }
  } else { // right turn
    if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG])) {
        bike->handle_right_turn = true;
    } else {
        bike->handle_right_turn = false;
    }
  }

}

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
  // left and right lights
  if(LIGHT_STATES[curr_state][0] && LIGHT_STATES[curr_state][1]){
    light_output = LIGHT_ACTION_NONE;
  } 
  // left light only
  else if(LIGHT_STATES[curr_state][0] && !LIGHT_STATES[curr_state][1]){
    light_output = LIGHT_ACTION_LEFT_TURN;
  }
  // right light only
  else if(!LIGHT_STATES[curr_state][0] && LIGHT_STATES[curr_state][1]){
    light_output = LIGHT_ACTION_RIGHT_TURN;  
  } 
  // no lights
  else {
    light_output = LIGHT_ACTION_NONE;
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
    // assert(false);
  }
}

void btn_state_change_alt( State* state ) {
    // must be called when a button is pressed
    //assert(!left_btn_pressed && !right_btn_pressed);
    bool *left_btn_pressed = &(state->flags[LEFT_TURN_FLAG]);
    bool *right_btn_pressed = &(state->flags[RIGHT_TURN_FLAG]);
    
    // TODO: Change the flag name
    // bool *handle_is_turned = &(state->flags[MANUAL_MODE_SWITCH_FLAG]);
    bool *handle_left_is_turned = &(state->flags[HANDLE_LEFT_TURN_FLAG]);
    bool *handle_right_is_turned = &(state->flags[HANDLE_RIGHT_TURN_FLAG]);
    bool handle_is_turned = (*handle_left_is_turned) || (*handle_right_is_turned);

    if((!*left_btn_pressed && !*right_btn_pressed) && !handle_is_turned){
        return;
    }

    // handle simultaneous press by resetting to OFF
    if(*left_btn_pressed && *right_btn_pressed){
        curr_state = OFF;
        return;
    }

    // get next state from our BTN_STATES array
    if(*left_btn_pressed && !*right_btn_pressed){
        curr_state = BTN_STATES[ curr_state ][LEFT_BUTTON];
    } else if(!*left_btn_pressed && *right_btn_pressed){
        curr_state = BTN_STATES[ curr_state ][RIGHT_BUTTON];
    } else {
        // assert(false);
    }

    if (handle_is_turned) {
        if (state->handle_right_turn  && !state->handle_left_turn/* && !bike->handle_turn_left*/) {
            // led_on(LED_0);
            _handle_turn = HANDLE_RIGHT_TURN;
        } else if(!state->handle_right_turn && state->handle_left_turn) {
            // led_on(LED_0);
            _handle_turn = HANDLE_LEFT_TURN;
        } else {
            _handle_turn = HANDLE_NO_TURN;
        }  
    }

    *left_btn_pressed = false;
    *right_btn_pressed = false;
    *handle_left_is_turned = false;
    *handle_right_is_turned = false;
}

// Perform threshold checking based on current state
LightAction do_state_action( int16_t accel_x_val ) {
  switch( curr_state ){
    case OFF: // All lights off
      //do nothing: btn_state_change controls state transition from
      //  OFF to something else based on button press
      reset_state();
      break;
    case SIGNAL_R: // Right btn pressed, check for entering turn before going to next state
      if(check_in_thresh( true, &accel_x_val )){
        curr_state = RETURN_R_ACCEL;
      }else if (_handle_turn == HANDLE_RIGHT_TURN){
        // led_on(LED_0);
        curr_state = RETURN_HANDLE_R;
      }
      break;
    case SIGNAL_L: // Left btn pressed, check for entering turn before going to next state
      if(check_in_thresh( false, &accel_x_val )){
          curr_state = RETURN_L_ACCEL;
      } else if(_handle_turn == HANDLE_LEFT_TURN){
          // led_on(LED_0);
          curr_state = RETURN_HANDLE_L;
      }
      break;
    case RETURN_R_ACCEL: // Entered right turn, check for return
      if( (check_out_thresh( true, &accel_x_val ))){ // ADD check for speed ( && speed > TEMP )
        curr_state = OFF;
        reset_state();
      }
      break;
    case RETURN_L_ACCEL:
      if( (check_out_thresh( false, &accel_x_val ) ) ) { // ADD check for speed ( && speed > TEMP )
        curr_state = OFF;
        reset_state();
      }
      break;
    case RETURN_HANDLE_R:
      if( ((_handle_turn == HANDLE_NO_TURN) )){ // ADD check for speed ( && speed > TEMP )
        curr_state = OFF;
        reset_state();
        // led_off(LED_0);
      }
      break;
    case RETURN_HANDLE_L:
      if( ((_handle_turn == HANDLE_NO_TURN) )){ // ADD check for speed ( && speed > TEMP )
        curr_state = OFF;
        reset_state();
        // led_off(LED_0);
      }
      break;
    default:
      // assert(false);
      break;
  }

  // set light output based on which state we're in
  set_light_output();

  // return the light actions array
  return light_output;
}
