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
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_config.h"
// Platform, Peripherals, Devices, Services
#include "smartbike.h"
#include "led.h"


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
extern volatile uint32_t pin_test;

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

/**@brief Function for asserts in the SoftDevice.
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
    //led_toggle(BLEES_LED_PIN);
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

void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	if(pin == 21){
        led_toggle(LED_0);    
    }
    else if(pin == 22){
        led_toggle(LED_1);    
    }
}

int main(void) {
    uint32_t err_code;
nrf_drv_gpiote_in_config_t temp_t = {NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIO_PIN_NOPULL, false, false};


    // Initialization
    led_init(LED_0);
    //led_on(LED_0);
    led_init(LED_1);
    led_init(LED_2);
    //led_on(LED_1);

    // Setup clock
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);
    
    //nrf_gpio_cfg_input(22, NRF_GPIO_PIN_NOPULL);
    //nrf_gpio_cfg_input(21, NRF_GPIO_PIN_NOPULL);
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    nrf_drv_gpiote_in_init(21, &temp_t, &pin_handler);
    nrf_gpio_cfg_sense_input(21, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    

    nrf_drv_gpiote_in_init(22, &temp_t, &pin_handler);    
    nrf_gpio_cfg_sense_input(22, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

    // nrf_drv_gpiote_in_init(0, &temp_t, &pin_handler);

    NRF_GPIOTE->INTENSET = 0x8000000F;
    NVIC_EnableIRQ(GPIOTE_IRQn);

    // Setup and start timer
    //timers_init();
    //timers_start();

    while (1) {
        power_manage();
    
    }
}

