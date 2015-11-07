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

static gpio_input_cfg_t *             _gpio_input_cfgs = NULL;
static uint8_t                        _gpio_input_count; /**< Number of configured buttons. */


static uint32_t _pin_state;
static uint32_t _pin_transition;

uint32_t gpio_input_init(gpio_input_cfg_t *gpio_cfgs,
                         uint8_t gpio_input_count) {
    uint32_t err_code;

    // TODO: WE MIGHT OR MIGHT NOT NEED THIS!! probably do tho
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

    while (gpio_input_count--) {
        gpio_input_cfg_t* curr_input = &_gpio_input_cfgs[gpio_input_count];

        /*nrf_gpio_cfg_input(curr_input->pin_no, curr_input->pull_cfg);

        
        nrf_gpiote_event_configure(GPIOTE_CHANNEL_0,
                                   curr_input->pin_no,
                                   curr_input->polarity); */

        //Next line takes care of above 2 in nrf_drv_gpiote.c :330
       
        // set the last arg to true to have high accuracy
        //      (Needed so pin gpiote registration finds channels)
        nrf_drv_gpiote_in_config_t p_config =
            {curr_input->polarity, curr_input->pull_cfg, false,true};

        err_code = nrf_drv_gpiote_in_init(curr_input->pin_no, 
                                          &p_config,
                                          curr_input->gpio_handler);

        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

    }

    //NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0

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

