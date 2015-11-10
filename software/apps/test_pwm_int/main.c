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
#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

// Platform, Peripherals, Devices, Services
#include "smartbike.h"
#include "led.h"
#include "pwm.h"
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
static nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);
static uint8_t toggle = 1;

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
    // led_toggle(LED_2);


 //    if (toggle) {
	// pca9685_setPin(1, 200, 0);
 //    toggle = 0;
 //    led_on(LED_2);
 //    } else {
	// pca9685_setPin(1, 400, 0);
 //    led_off(LED_2);
 //    toggle = 1;
 //    }
    
    //toggle = !(toggle);
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

static void i2c_init() {
    nrf_drv_twi_config_t twi_config;

    // Initialize the I2C module
    twi_config.sda                = I2C_SDA_PIN;
    twi_config.scl                = I2C_SCL_PIN;
    twi_config.frequency          = NRF_TWI_FREQ_400K;
    twi_config.interrupt_priority = APP_IRQ_PRIORITY_HIGH;

    nrf_drv_twi_init(&twi_instance, &twi_config, NULL);
    nrf_drv_twi_enable(&twi_instance);
}


/*******************************************************************************
 *   MAIN LOOP
 ******************************************************************************/

#define BUTTON_PIN 21

#define OUTPUT_PIN 22

static volatile uint32_t i = 0;

volatile bool b21, b22;
volatile uint16_t pwm_count = 300;

void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    /*if (pin == 21) { // increase duty cycle
        b21 = true;
    } else if (pin == 22) { //decrease duty cycle
        b22 = true;
    }*/

        if (pin == 21) { //increase duty cycle
            pwm_count += 40;
            if(pwm_count >= 400){
                pwm_count = 400;
            }
            pca9685_setPin(1, pwm_count, 0);
            
            //b21 = false;
            led_toggle(LED_0);
        }
        if (pin == 22) {  //decrease duty cycle
            pwm_count -= 40;
            if(pwm_count <= 200){
                pwm_count = 200;
            }
            pca9685_setPin(1, pwm_count, 0);
         
            //b22 = false;
            led_toggle(LED_1);
        }

    //led_toggle(LED_0);
}

int main(void) {
    uint32_t err_code;
    uint8_t gpio_input_count;

    // Initialization
    // led_init(LED_2);
    // led_on(LED_2);
    uint32_t i;
    i=0;
    b21=false; b22=false;
    led_init(LED_0);
    led_init(LED_1);
    led_init(LED_2);
    //led_on(LED_0);

    // Setup clock
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    i2c_init();

    // since active high, pins need to be set to have a pull-down resistor,
    //      otherwise they will be floating
    static gpio_input_cfg_t cfgs[] = {  {BUTTON_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {OUTPUT_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler}};

    // It seems only 4 pins can be registered per channel
    gpio_input_count = 2;


    /* SET INPUT WITH DRIVER */
    err_code = gpio_input_init(cfgs, gpio_input_count);
    if (err_code) {
        led_on(LED_1);
    }
    gpio_input_enable_all();
    NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts


    // Setup clock
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Setup and start timer
    //timers_init();
    //timers_start();

    pca9685_init(&twi_instance);
    pca9685_setPWMFreq(50);
    //pca9685_setPWM(0x01, 0x0199, 0x04CC);
    pca9685_setPin(1, pwm_count, 0);
    while (1) {

        //power_manage();
    }
}

