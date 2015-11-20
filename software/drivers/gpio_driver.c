#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Nordic Libraries
#include "nordic_common.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_config.h"
//#include "smartbike.h"
#include "led.h"

#include "gpio_driver.h"

static gpio_cfg_t *             _gpio_cfgs = NULL;
static uint8_t                        _gpio_count; /**< Number of configured buttons. */



void _pin_direction_reset() {
    int i;
    for (i=0; i<NUM_GPIO_PINS; ++i) {
        _pin_direction[i] = PIN_UNDEFINED;
    }
}
uint32_t gpio_init(gpio_cfg_t *gpio_cfgs,
                   uint8_t gpio_count) {
    assert(gpio_count < NUM_GPIO_PINS);
    uint32_t err_code;

    // TODO: WE MIGHT OR MIGHT NOT NEED THIS!! probably do tho
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        _pin_direction_reset();
    }

    // Save configuration.
    _gpio_cfgs = gpio_cfgs;
    _gpio_count = gpio_count;
    
    while (gpio_count--) {
        assert(_pin_direction[gpio_count] == PIN_UNDEFINED);

        gpio_cfg_t* curr_input = &_gpio_cfgs[gpio_count];
        assert(curr_input->pin_direction != PIN_UNDEFINED);
        assert(curr_input->pin_no < NUM_GPIO_PINS);

        if (curr_input->pin_direction == PIN_OUT) {
            nrf_gpio_cfg_output(curr_input->pin_no);
        } else { // (PIN_GPIOTE_IN || PIN_PORT_IN)
            nrf_drv_gpiote_in_config_t p_config =
                {curr_input->polarity, curr_input->pull_cfg,
                 false, (curr_input->pin_direction == PIN_GPIOTE_IN) ? true : false};

            err_code = nrf_drv_gpiote_in_init(curr_input->pin_no, 
                                              &p_config,
                                              curr_input->gpio_handler);

            if (curr_input->pin_direction == PIN_PORT_IN) {
                nrf_gpio_cfg_sense_input(curr_input->pin_no, curr_input->pull_cfg,
                                         curr_input->polarity);
            }
        }

        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        _pin_direction[curr_input->pin_no] = curr_input->pin_direction;

    }

    return NRF_SUCCESS;
}

void gpio_input_enable_all(void) {
    int8_t i;
    for (i=0; i<_gpio_count; ++i) {
        uint8_t pin_no = _gpio_cfgs[i].pin_no;
        if (_pin_direction[pin_no] == PIN_GPIOTE_IN ||
            _pin_direction[pin_no] == PIN_PORT_IN) {

            nrf_drv_gpiote_in_event_enable(pin_no, true);
        }
    }
}

void gpio_input_disable_all(void) {
    uint8_t i;
    for (i=0; i<_gpio_count; ++i) {
        uint8_t pin_no = _gpio_cfgs[i].pin_no;
        if (_pin_direction[pin_no] == PIN_GPIOTE_IN ||
            _pin_direction[pin_no] == PIN_PORT_IN) {

            nrf_drv_gpiote_in_event_disable(pin_no);
        }
    }
}

void gpio_output_set(uint8_t pin_no, uint8_t value) {
    assert(pin_no < NUM_GPIO_PINS && _pin_direction[pin_no] == PIN_OUT);
    nrf_gpio_pin_write(pin_no, value);
}

void gpio_output_toggle(uint8_t pin_no) {
    assert(pin_no < NUM_GPIO_PINS && _pin_direction[pin_no] == PIN_OUT);
    nrf_gpio_pin_toggle(pin_no);
}

