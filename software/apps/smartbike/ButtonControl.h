#include <stdbool.h>
#include <stdint.h>

typedef enum {SHIFT, SIGNAL, _BUTTON_TYPE_SIZE} ButtonType;
typedef enum {LEFT, RIGHT, _BUTTON_POS_SIZE} ButtonPosition;
typedef struct {
    bool toggled_on;
    bool _has_led;
} Button;

/************************************************************************
			    Initialize Button
************************************************************************/
/* initialize a specific button */
void initButton(ButtonType type, ButtonPosition pos);

/* initialize all buttons to default/preset values */
void initAllButtons();

/*****************************************************************************
                            Check Button Status                         
*****************************************************************************/

/* check if the button was toggled on */
bool button(ButtonType type, ButtonPosition pos);

