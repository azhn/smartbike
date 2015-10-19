#include "ButtonControl.h"
#include "app_button.h"

/*
typedef enum {SHIFT, SIGNAL, _BUTTON_TYPE_SIZE} ButtonType;
typedef enum {LEFT, RIGHT, _BUTTON_POS_SIZE} ButtonPosition;
typedef struct {
    bool toggled_on;
    bool _has_led;
} Button;
*/

/* Global Variables */
Button* leftShift;
Button* rightShift;
Button* leftSignal;
Button* rightSignal;

/************************************************************************
			    Initialize Button
************************************************************************/
/* initialize a specific button */
void initButton(ButtonType type, ButtonPosition pos, Button * state, uint8_t pinNum) {
  // create button config struct
  app_button_cfg_t btn_cfg;
  btn_cfg.pin_no = pinNum;
  btn_cfg.active_state = APP_BUTTON_ACTIVE_LOW; //Is this correct...?
  //btn_cfg.pull_cfg =  --> what is this suposed to be?
  btn_cfg.button_handler =

  // based on the button, initialize the specific Button pointer
  switch (type) {
    case SHIFT:
	if(pos == LEFT) {
	  // leftShift = state;

	  //initialize the corresponding GPIO pins
	}
 	else {
	  // rightShift = state;

	  //initialize the corresponding GPIO pins
	}
	break;
    case SIGNAL:
	if(pos ==  LEFT) {
	  // leftSignal = state;

	  //initialize the corresponding GPIO pins
	}
	else {
	  // rightSignal = state;

	  //initialize the corresponding GPIO pins
	}
	break;
    default:
	break;
  }
}

/* initialize all buttons to default/preset values */
void initAllButtons() {

}

/*****************************************************************************
                            Check Button Status                         
*****************************************************************************/

/* check if the button was toggled on */
bool button(ButtonType type, ButtonPosition pos) {
  return FALSE;
}

