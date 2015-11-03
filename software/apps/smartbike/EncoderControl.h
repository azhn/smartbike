#ifndef ENCODERCONTROL_H_
#define ENCODERCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "qdec/nrf_drv_qdec.h"
#include "config/nrf_drv_config.h"
#include "common.h"



/****************************************************************************
			 Handlers
****************************************************************************/
void qdec_handler(nrf_drv_qdec_event_t event);

/****************************************************************************
			 Initialization
****************************************************************************/
/* initialize the magnetic encoder */
bool initializeQuadDecoder( uint32_t pinA, uint32_t pinB );

/****************************************************************************
			 Retrieve Data
****************************************************************************/
/* read quadrature decoder count and timestamp, and store in history */
DataPair * readDecoderCount( );
int16_t readCurrentCounter();

#endif // ENCODERCONTROL_H_
