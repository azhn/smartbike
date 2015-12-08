/* Copy/ight (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * This file contains the template code meant to be used in the tutorial 
 * "A beginner's tutorial: Advertising" found on http://devzone.nordicsemi.com/tutorials/
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer_appsh.h"
#include "led.h"

#include "our_service.h"
#include "ble_cscs.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                     "SMARTBIKE"                                /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                160                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_os_t                         m_our_service;
static ble_cscs_t                       m_cscs;
static ble_cscs_init_t                  m_cscs_init;
static ble_cscs_meas_t                  m_cscs_meas;

static ble_gap_adv_params_t             m_adv_params;

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

/**@brief Function for the Timer initialization.
 * This function sets up software timers. The clock source for the timers
is the nRF51s Real Time Clock 1 (RTC1). In this example we initialize 2 timers 
(see the #define APP_TIMER_MAX_TIMERS). These are used to time the bluetooth connection intervals
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
	
	// Declearing parameter structs. Try to go to the struct definitions to get
	// more information about what parameters they contain
    ble_gap_conn_params_t   gap_conn_params; 	// Struct to store GAP connection parameters like max min connection interval etc
    ble_gap_conn_sec_mode_t sec_mode;			// Struct to store security parameters 

	// A simple macro that sets the Security Mode and Level bits in sec_mode
	// to require no protection (open link)
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	// Store the device name and security mode in the SoftDevice. Our name is defined to "HelloWorld" in the beginning of this file
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);// Check for errors

	// Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	// Populate the GAP connection parameter struct
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	// Set GAP Peripheral Preferred Connection Parameters
    // The device use these prefered values when negotiating connection terms with another device
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);// Check for errors
											
	// Set appearence										  
	//sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
	sd_ble_gap_appearance_set(0);
    APP_ERROR_CHECK(err_code);// Check for errors
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t            err_code;
    ble_advdata_t       advdata;    // Struct containing advertising parameters
    uint8_t             flags       = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // Defines how your device is seen (or not seen) by other devices
	ble_uuid_t          adv_uuids[] = {{0x1816, BLE_UUID_TYPE_BLE}};    // Random example User Unique Identifier
  

    /* commented out so can fit full device name in advertisement
       manufacturer data set in scan response instead
    ble_advdata_manuf_data_t        manuf_data;
    uint8_t data[]                  = "SomeData!";
    manuf_data.company_identifier   = 0x0059;
    manuf_data.data.p_data          = data;
    manuf_data.data.size            = sizeof(data);   
    */
 
    // Populate the advertisement packet
    // Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    //advdata.short_name_len          = 6;                                             
    advdata.flags                   = flags;                                     // Must be included, but not discussed in this tutorial
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]); // Must be included, but not discussed in this tutorial
    advdata.uuids_complete.p_uuids  = adv_uuids;                                // Must be included, but not discussed in this tutorial
    
    // commented out so can fit full device name in advertisement
    // manufacturer data is set in scan response instead
    //advdata.p_manuf_specific_data  = &manuf_data;
   
    
    /*ble_advdata_manuf_data_t                manuf_data_response;
    uint8_t dataresponse[]                  = "LotsOfWords";
    manuf_data_response.company_identifier  = 0x0059;
    manuf_data_response.data.p_data         = dataresponse;
    manuf_data_response.data.size           = sizeof(dataresponse);
    */ 

    ble_uuid_t  m_adv_uuids[] = {{BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};

    ble_advdata_t                           advdata_response;
    
    // always initialize all fields in structs to zero or else might get unexpected behavior
    memset(&advdata_response, 0, sizeof(advdata_response));
    // populate scan response packet
    //advdata_response.name_type              = BLE_ADVDATA_NO_NAME;
    //advdata_response.p_manuf_specific_data  = &manuf_data_response;
    
    advdata_response.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata_response.uuids_complete.p_uuids  = m_adv_uuids;

    // Set the advertisement and scan response packet. 
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);// Check for errors
}



/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


static void services_init(void)
{
    
    //our_service_init(&m_our_service);

    memset(&m_cscs, 0, sizeof(m_cscs));
    memset(&m_cscs_init, 0, sizeof(m_cscs_init));
   
    m_cscs_init.evt_handler                     = NULL; // for now
    m_cscs_init.csc_meas_attr_md                = (ble_srv_cccd_security_mode_t){ .cccd_write_perm = {1, 1}, 
                                                    .read_perm       = {1, 1},
                                                    .write_perm      = {1, 1} 
                                                  };
    m_cscs_init.csc_ctrlpt_attr_md              = (ble_srv_cccd_security_mode_t){ .cccd_write_perm = {1, 1}, 
                                                    .read_perm       = {1, 1},
                                                    .write_perm      = {1, 1}
                                                  };
    m_cscs_init.csc_feature_attr_md             = (ble_srv_security_mode_t){ .read_perm       = {1, 1}, 
                                                    .write_perm      = {1, 1} 
                                                  };
    m_cscs_init.feature                         = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT;
    m_cscs_init.ctrplt_supported_functions      = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED; // required if having WHEEL_REV data
    m_cscs_init.ctrlpt_evt_handler              = NULL; // for now
    m_cscs_init.list_supported_locations        = NULL; // no location data needed
    m_cscs_init.size_list_supported_locations   = 0; // no location data so no size
    m_cscs_init.error_handler                   = NULL; // for now
    m_cscs_init.sensor_location                 = NULL; // no location data so no pointer needed

    ble_cscs_init(&m_cscs, &m_cscs_init);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code; // Variable used to check for any errors returned by functions

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);// Check for errors
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
	// Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    // Check for errors
	APP_ERROR_CHECK(err_code);// Check for errors
}


/**@brief Function for starting advertising.
 */
static void advertising_param_init(void) {
    //uint32_t             err_code;
    //ble_gap_adv_params_t adv_params;

    // Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

}
static void advertising_start(void)
{
    uint32_t             err_code;
    //ble_gap_adv_params_t adv_params;

    // Always initialize all fields in structs to zero or you might get unexpected behaviour
    /*memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
    */
    //m_adv_params = adv_params;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);// Check for errors
}

static void advertising_stop(void) {
    uint32_t err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    bool                             master_id_matches;
    ble_gap_sec_kdist_t *            p_distributed_keys;
    ble_gap_enc_info_t *             p_enc_info;
    ble_gap_irk_t *                  p_id_info;
    ble_gap_sign_info_t *            p_sign_info;

    static ble_gap_enc_key_t         m_enc_key;           /**< Encryption Key (Encryption Info and Master ID). */
    static ble_gap_id_key_t          m_id_key;            /**< Identity Key (IRK and address). */
    static ble_gap_sign_info_t       m_sign_key;          /**< Signing Key (Connection Signature Resolving Key). */
    static ble_gap_sec_keyset_t      m_keys = {.keys_periph = {&m_enc_key, &m_id_key, &m_sign_key}};

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // Resume advertising, but non-connectably
            m_adv_params.type = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
            advertising_start();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;         
            // Go back to advertising connectably
            advertising_stop();
            m_adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,
                                                   &m_keys);
            
            //err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &m_sec_params, NULL);
            APP_ERROR_CHECK(err_code);// Check for errors
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                 NULL,
                                                 0,
                                                 BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            
            //err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);// Check for errors
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            //     
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            master_id_matches  = memcmp(&p_ble_evt->evt.gap_evt.params.sec_info_request.master_id,
                                        &m_enc_key.master_id,
                                        sizeof(ble_gap_master_id_t)) == 0;
            p_distributed_keys = &m_auth_status.kdist_periph;

            p_enc_info  = (p_distributed_keys->enc  && master_id_matches) ? &m_enc_key.enc_info : NULL;
            p_id_info   = (p_distributed_keys->id   && master_id_matches) ? &m_id_key.id_info   : NULL;
            p_sign_info = (p_distributed_keys->sign && master_id_matches) ? &m_sign_key         : NULL;

            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, p_id_info, p_sign_info);
            APP_ERROR_CHECK(err_code);// Check for errors
            
            //err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);// Check for errors
            }
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_cscs_on_ble_evt(&m_cscs, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code; // Variable used to check for any errors returned by functions

    // Initialize the SoftDevice handler module using the low frequency crystal oscillator with 20 ppm accuracy
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Enable BLE stack 
	// Declaration of a simple struct containing ble parameters
    ble_enable_params_t ble_enable_params;
	
	// Always initialize all fields in structs to zero or you might get unexpected behaviour
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	
	// This parameter defines whether or not our device might reconfigure its services during its lifetime. 
	// In our case we set the value to 0 meaning that no service or attributes will ever change.
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
	
	// This call initializes the bluetooth stack, no other BLE related call can be called before this one has been executed.
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);// Check for errors
    
    // Tell the softdevice which function is to be called i case of a BLE event. In this case: ble_evt_dispatch()
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);// Check for errors
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}




/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);// Check for errors
}

static uint32_t cum_wheel_revs, cum_crank_revs = 0;
static uint16_t  last_wheel_time, last_crank_time = 0;
static app_timer_id_t test_timer1;
static void timer_handler1(void* p_context) {
    led_toggle(19);

    memset(&m_cscs_meas, 0, sizeof(m_cscs_meas));

    m_cscs_meas.is_wheel_rev_data_present       = true;
    m_cscs_meas.is_crank_rev_data_present       = true;
    m_cscs_meas.cumulative_wheel_revs           = cum_wheel_revs;
    m_cscs_meas.last_wheel_event_time           = last_wheel_time;
    m_cscs_meas.cumulative_crank_revs           = cum_crank_revs;
    m_cscs_meas.last_crank_event_time           = last_crank_time;

    cum_wheel_revs += 2 + (rand() & 0x1); // 3 wheel revs
    last_wheel_time += 2048 + (rand() & 0x1FF); // 2 second
    cum_crank_revs += 1; // 1 crank
    last_crank_time += 2048; // 2 second;

    ble_cscs_measurement_send(&m_cscs, &m_cscs_meas);
    
}

static void timers_create(void) {
    uint32_t err_code;

    err_code = app_timer_create(&test_timer1, APP_TIMER_MODE_REPEATED, timer_handler1);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(test_timer1, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) , NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    led_init(19);

    // Initialize
    timers_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    // Start execution
    advertising_param_init();
    advertising_start();

    timers_create(); 

    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */