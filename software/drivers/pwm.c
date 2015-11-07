#include "pwm.h"


static nrf_drv_twi_t * i2c_instance;

void pca9685_init(nrf_drv_twi_t * i2c_instance_param) {
	i2c_instance = i2c_instance_param;

	pca9685_reset();	
	
}


void pca9685_reset(void) {
	uint8_t data[2] = {PCA9685_MODE1, 0x0};

	nrf_drv_twi_enable(i2c_instance);

	nrf_drv_twi_tx(i2c_instance,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(i2c_instance);
}


void pca9685_setPWMFreq(float freq) {
  	float prescaleval = 25000000;
	uint8_t prescale = 0;
	uint8_t oldmode = 0;
	uint8_t newmode = 0;
	volatile int i = 0;

 	freq *= 0.9;  
  	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
  
  	prescale = (uint8_t)(prescaleval + 0.5);
  
  	oldmode = pca9685_readByte(PCA9685_MODE1);
  	newmode = (oldmode&0x7F) | 0x10; // sleep
  	pca9685_writeByte(PCA9685_MODE1, newmode); // go to sleep
  	pca9685_writeByte(PCA9685_PRESCALE, prescale); // set the prescaler
  	pca9685_writeByte(PCA9685_MODE1, oldmode);
  	
	//nrf_delay_us(5);
	for(; i < 50000; i++) {}

	pca9685_writeByte(PCA9685_MODE1, oldmode | 0xa1);  
                                          
}



void pca9685_setPWM(uint8_t num, uint16_t on, uint16_t off) {
	uint8_t data[5] = {(LED0_ON_L + 4 * num),
						((uint8_t)on),
						((uint8_t)(on >> 8)),
						((uint8_t)off),
						((uint8_t)(off >> 8))
					  };

	nrf_drv_twi_enable(i2c_instance);

	nrf_drv_twi_tx(i2c_instance,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(i2c_instance);

}



// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void pca9685_setPin(uint8_t num, uint16_t val, uint8_t invert)
{
  // Clamp value between 0 and 4095 inclusive.
  //val = min(val, 4095);
  if (val > 4095) {
	val = 4095;
  }
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      pca9685_setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      pca9685_setPWM(num, 0, 4096);
    }
    else {
      pca9685_setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      pca9685_setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      pca9685_setPWM(num, 0, 4096);
    }
    else {
      pca9685_setPWM(num, 0, val);
    }
  }
}


uint8_t pca9685_readByte(uint8_t reg) {
  	ret_code_t ret;
	uint8_t reg_addr[1] = {reg};
	uint8_t value[1];

	nrf_drv_twi_enable(i2c_instance);
	
	nrf_drv_twi_tx(
		i2c_instance,
		ADDR,
		reg_addr,
		sizeof(reg_addr),
		true
	);


//	nrf_drv_twi_disable(&i2c);
//	nrf_drv_twi_enable(&i2c);
	
	do 
	{
		ret = 
		nrf_drv_twi_rx(
			i2c_instance,
			ADDR,
			value,
			sizeof(value),
			false
		);

	}while(ret != NRF_SUCCESS);

	nrf_drv_twi_disable(i2c_instance);

	return *value;
  
}

void pca9685_writeByte(uint8_t reg, uint8_t in_data) {
	uint8_t data[2] = {reg, in_data};

	nrf_drv_twi_enable(i2c_instance);

	nrf_drv_twi_tx(i2c_instance,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(i2c_instance);
}


