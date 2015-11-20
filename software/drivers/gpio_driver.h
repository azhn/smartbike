#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

// Nordic Libraries
#include "nordic_common.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#include "nrf_gpio.h"

#define GPIO_ACTIVE_HIGH NRF_GPIOTE_POLARITY_LOTOHI 
#define GPIO_ACTIVE_LOW NRF_GPIOTE_POLARITY_HITOLO
#define GPIO_ACTIVE_TOGGLE NRF_GPIOTE_POLARITY_TOGGLE 

#define NUM_GPIO_PINS 32
typedef enum PinDirection {
    PIN_UNDEFINED = 0,
    PIN_GPIOTE_IN,
    PIN_PORT_IN,
    PIN_OUT 
} PinDirection;

typedef struct
{
    uint8_t                      pin_no;
    nrf_gpiote_polarity_t        polarity;
    nrf_gpio_pin_pull_t          pull_cfg;     /**< Pull-up or -down configuration. */
    nrf_drv_gpiote_evt_handler_t gpio_handler; /**< Handler to be called when gpiote is triggered. */
    PinDirection                 pin_direction;/**< True: GPIOTE, False: PORT> */ 
} gpio_cfg_t;


PinDirection _pin_direction[NUM_GPIO_PINS];
static void _pin_direction_reset();
/* @param[in]  gpio_cfgs           Array of buttons to be used (NOTE: Must be static!).
 * @param[in]  button_count        Number of pins.
 * 
 * @return   NRF_SUCCESS on success, otherwise an error code.
 */


uint32_t gpio_init(gpio_cfg_t *gpio_cfgs,
                   uint8_t gpio_count);


void gpio_input_enable_all(void);
void gpio_input_disable_all(void);

void gpio_output_set(uint8_t pin_no, uint8_t value);
void gpio_output_toggle(uint8_t pin_no);
/*
 * @param[in] gpio_pin
 * @param[out] p_is_pushed
 */
//uint32_t gpio_is_active(uint8_t gpio_pin, bool * p_is_pushed);



#endif // GPIO_DRIVER_H
