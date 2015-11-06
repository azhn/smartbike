/*
 * LED Blink App
 */

// Standard Libraries
#include <stdint.h>
#include <stdbool.h>

// Nordic Libraries
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
//#include "boards.h"
 #include "nrf_gpiote.h"
 #include "nrf_drv_gpiote.h"

// Platform, Peripherals, Devices, Services
#include "smartbike.h"
#include "led.h"
#include "gpio_driver.h"


/*******************************************************************************
 *   DEFINES
 ******************************************************************************/
#include "nrf_drv_config.h"

#define BLINK_TIMER_PRESCALER       0   // Value of RTC1 PRESCALER register
#define BLINK_TIMER_MAX_TIMERS      4   // Maximum number of simultaneous timers
#define BLINK_TIMER_OP_QUEUE_SIZE   4   // Size of timer operation queues
#define BLINK_RATE  APP_TIMER_TICKS(500, BLINK_TIMER_PRESCALER) // Blink every 0.5 seconds


/*******************************************************************************
 *   STATIC AND GLOBAL VARIABLES
 ******************************************************************************/

static app_timer_id_t test_timer;


/*******************************************************************************
 *   HANDLERS AND CALLBACKS
 ******************************************************************************/

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
    // APPL_LOG("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);
    // nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

/*@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

//service error callback
static void service_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
    // pstorage_sys_event_handler(sys_evt);
    // on_sys_evt(sys_evt);
}

// Timer fired handler
static void timer_handler (void* p_context) {
    led_toggle(LED_0);
}


/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

static void timers_init(void) {
    uint32_t err_code;

    APP_TIMER_INIT(BLINK_TIMER_PRESCALER, BLINK_TIMER_MAX_TIMERS,
            BLINK_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&test_timer, APP_TIMER_MODE_REPEATED,
            timer_handler);
    APP_ERROR_CHECK(err_code);
}


/*******************************************************************************
 *   HELPER FUNCTIONS
 ******************************************************************************/

/** @brief Function for the Power manager.
 */
static void power_manage (void) {
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// Start the timers
static void timers_start(void) {
    uint32_t err_code = app_timer_start(test_timer, BLINK_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}


/*******************************************************************************
 *   MAIN LOOP
 ******************************************************************************/

#define BUTTON_PIN 21

#define OUTPUT_PIN 22

#define PIN1 8 
#define PIN2 9
#define PIN3 10
static volatile uint32_t i = 0;
// Interrupt handler
/*void GPIOTE_IRQHandler(){
    //led_toggle(LED_0);
    nrf_gpio_pin_toggle(LED_0);
    //nrf_gpio_pin_toggle(OUTPUT_PIN);
    NRF_GPIOTE->EVENTS_IN[0] = 0;
}*/


volatile bool b8, b9, b10, b21, b22;

void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    if (pin == 8) {
        b8 = true;
    } else if (pin == 9) {
        b9 = true;
    } else if (pin == 10) {
        b10 = true;
    } else if (pin == 21) {
        b21 = true;
    } else if (pin == 22) {
        b22 = true;
    }
}

int main(void) {
    uint32_t err_code;
    uint8_t gpio_input_count;
    // // Initialization
    uint32_t i;
    i=0;
    b8=false; b9=false; b10=false; b21=false; b22=false;
    led_init(LED_0);
    led_init(LED_1);
    led_init(LED_2);
    for (i=0;i<1000;++i) {
        if (i%100 == 0) {
            led_toggle(LED_0); 
        }
    }
    led_off(LED_0);

    static gpio_input_cfg_t cfgs[] = {  {BUTTON_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {OUTPUT_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {PIN1, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {PIN2, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {PIN3, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler}};
    gpio_input_count = 5;

    /* SET OUTPUT WITH DRIVER */
    /* uint8_t output_pins[] = {PIN1,PIN2,PIN3};
     * gpio_output_init(output_pins, 3);
     */

    /* SET OUTPUT OLD WAY */
    /* nrf_gpio_cfg_output(OUTPUT_PIN);
     */

    
    /* SET INPUT WITH DRIVER */
    err_code = gpio_input_init(cfgs, gpio_input_count);
    if (err_code) {
        led_on(LED_1);
    }
    gpio_input_enable_all();

    /* SET INPUT OLD WAY */
    /* nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_NOPULL); //Configure pin 21 0 as input
     * nrf_gpiote_event_configure(GPIOTE_CHANNEL_0, BUTTON_PIN, NRF_GPIOTE_POLARITY_LOTOHI); 
     * nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
     */ 

    //NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0
    NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts

    while (true) {
        if (b21 == true) {
            b21 = false;
            led_toggle(LED_0);
        }
        if (b22 == true) {
            b22 = false;
            led_toggle(LED_1);
        }

        if (b8 == true) {
            b8 = false;
            led_toggle(LED_2);
        }
        if (b9 == true) {
            b9 = false;
            for (i=0;i<1000;++i) {
                if (i%100 == 0) {
                    led_toggle(LED_0); 
                }
            }
            led_off(LED_0); 
        }
        if (b10 == true) {
            b9 = false;
            for (i=0;i<1000;++i) {
                if (i%100 == 0) {
                    led_toggle(LED_1); 
                }
            }
            led_off(LED_1); 
        }

    }
}


