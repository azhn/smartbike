#include <stdint.h>
#include <stdbool.h>

// Nordic Libraries
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_config.h"
#include "blees.h"
#include "led.h"

#include "gpio_driver.h"

static gpio_input_cfg_t *             _gpio_input_cfgs = NULL;
static uint8_t                        _gpio_input_count; /**< Number of configured buttons. */


static uint32_t _pin_state;
static uint32_t _pin_transition;

uint32_t gpio_input_init(gpio_input_cfg_t *gpio_cfgs,
                         uint8_t gpio_input_count) {
    uint32_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    // Save configuration.
    _gpio_input_cfgs = gpio_cfgs;
    _gpio_input_count = gpio_input_count;

    _pin_state      = 0;
    _pin_transition = 0;

    // TODO: WHAT IS CHANNEL??
    while (gpio_input_count--) {
        gpio_input_cfg_t* curr_input = &_gpio_input_cfgs[gpio_input_count];

        nrf_gpio_cfg_input(curr_input->pin_no, curr_input->pull_cfg);

        nrf_gpiote_event_configure(GPIOTE_CHANNEL_0,
                                   curr_input->pin_no,
                                   curr_input->polarity); 
    }

    // TODO: WHAT IS INTENSET DO WITH VALUES??
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0

    NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts
    
    return NRF_SUCCESS;
}

void gpio_input_enable_all(void) {
    int8_t i;
    for (i=0; i<_gpio_input_count; ++i) {
        nrf_drv_gpiote_in_event_enable(_gpio_input_cfgs[i].pin_no, true);
    }
}

void gpio_input_disable_all(void) {
    uint8_t i;
    for (i=0; i<_gpio_input_count; ++i) {
        nrf_drv_gpiote_in_event_disable(_gpio_input_cfgs[i].pin_no);
    }
}

void gpio_output_init(uint8_t* pin_no, uint8_t gpio_output_count) {
    int8_t i;
    for (i=0; i<gpio_output_count; ++i) {
         nrf_gpio_cfg_output(pin_no[i]);
    }
}

void gpio_output_set(uint8_t pin_no, uint8_t value) {
    nrf_gpio_pin_write(pin_no, value);
}

void gpio_output_toggle(uint8_t pin_no) {
    nrf_gpio_pin_toggle(pin_no);
}

