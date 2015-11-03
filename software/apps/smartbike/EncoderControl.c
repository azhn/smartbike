#include <stdbool.h>
#include <stdint.h>


#include "config/nrf_drv_config.h"
#include "qdec/nrf_drv_qdec.h"
#include "app_util_platform.h"

#include "common.h"
#include "EncoderControl.h"


/*
  typedef struct
{
    nrf_qdec_reportper_t   reportper;          //< Report period in samples. 
    nrf_qdec_sampleper_t   sampleper;          //< Sampling period in microseconds.
    uint32_t               psela;              //< Pin number for A input. 
    uint32_t               pselb;              //< Pin number for B input. 
    uint32_t               pselled;            //< Pin number for LED output.
    uint32_t               ledpre;             //< Time (in microseconds) how long LED is switched on before sampling.
    nrf_qdec_ledpol_t      ledpol;             //< Active LED polarity. 
    bool                   dbfen;              //< State of debouncing filter.
    bool                   sample_inten;       //< Enabling sample ready interrupt. 
    uint8_t                interrupt_priority; //< QDEC interrupt priority.
} nrf_drv_qdec_config_t;
*/

// QDEC configuration parameter
nrf_drv_qdec_config_t qdec_config;


/****************************************************************************
       Handlers
****************************************************************************/
void qdec_handler(nrf_drv_qdec_event_t event) {
  //nothing for now
  return;
}

/****************************************************************************
       Initialization
****************************************************************************/
/* initialize the magnetic encoder */
bool initializeQuadDecoder( uint32_t pinA, uint32_t pinB ) {
  // we want pinA = 3, pinB = 4
  qdec_config.sampleper           = NRF_QDEC_SAMPLEPER_128us;
  qdec_config.reportper           = NRF_QDEC_REPORTPER_160;
  qdec_config.psela               = pinA; // GPIO pin #
  qdec_config.pselb               = pinB; // GPIO pin #
  qdec_config.pselled             = 0xFFFFFFFF; // uninitialized
  qdec_config.ledpol              = NRF_QDEC_LEPOL_ACTIVE_LOW;
  qdec_config.ledpre              = 0; // 0 us
  qdec_config.dbfen               = NRF_QDEC_DBFEN_ENABLE;
  qdec_config.sample_inten        = NRF_QDEC_INT_REPORTRDY_MASK; // report ready interrupt on
  qdec_config.interrupt_priority  = APP_IRQ_PRIORITY_LOW;//     = 3//QDEC_CONFIG_IRQ_PRIORITY;

  if(nrf_drv_qdec_init(&qdec_config, &qdec_handler) != NRF_SUCCESS) {
    return false;
  }

  // enable quadrature decoding
  nrf_drv_qdec_enable();
  return true;
}

/*uninitializeQuadDecoder(){

}*/

/****************************************************************************
       Retrieve Data
****************************************************************************/
/* read quadrature decoder count and timestamp, and store in history */
DataPair * readDecoderCount( ) {
  // nothing for now
  return NULL;

}


int16_t readCurrentCounter(){
  int16_t counterVal, counterDoubleVal;
  nrf_drv_qdec_accumulators_read(&counterVal, &counterDoubleVal);
  return counterVal;
}

