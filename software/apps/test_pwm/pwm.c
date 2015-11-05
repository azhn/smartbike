#include "pwm.h"


static nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(1);


void init(void) {
	nrf_drv_twi_config_t i2c_conf;

	i2c_conf.sda = I2C_SDA_PIN;
	i2c_conf.scl = I2C_SCL_PIN;
	i2c_conf.frequency = NRF_TWI_FREQ_400K;
	i2c_conf.interrupt_priority = 1;

	nrf_drv_twi_init(&i2c, &i2c_conf, NULL);
	
	reset();
}


void reset(void) {
	uint8_t data[2] = {PCA9685_MODE1, 0x0};

	nrf_drv_twi_enable(&i2c);

	nrf_drv_twi_tx(&i2c,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(&i2c);
}


void setPWMFreq(float freq) {
  	float prescaleval = 25000000;
	uint8_t prescale = 0;
	uint8_t oldmode = 0;
	uint8_t newmode = 0;
	int i = 0;

 	freq *= 0.9;  
  	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
  
  	prescale = (uint8_t)(prescaleval + 0.5);
  
  	oldmode = read8(PCA9685_MODE1);
  	newmode = (oldmode&0x7F) | 0x10; // sleep
  	write8(PCA9685_MODE1, newmode); // go to sleep
  	write8(PCA9685_PRESCALE, prescale); // set the prescaler
  	write8(PCA9685_MODE1, oldmode);
  	
	while(i++ < 50000){} //delay a bit

	write8(PCA9685_MODE1, oldmode | 0xa1);  
                                          
}



void setPWM(uint8_t num, uint16_t on, uint16_t off) {
	uint8_t data[5] = {(LED0_ON_L + 4 * num),
						((uint8_t)on),
						((uint8_t)(on >> 8)),
						((uint8_t)off),
						((uint8_t)(off >> 8))
					  };

	nrf_drv_twi_enable(&i2c);

	nrf_drv_twi_tx(&i2c,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(&i2c);

}


/*
// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

*/
uint8_t read8(uint8_t reg) {
  	ret_code_t ret;
	uint8_t value;

	nrf_drv_twi_enable(&i2c);
	
	nrf_drv_twi_tx(
		&i2c,
		ADDR,
		reg,
		sizeof(reg),
		true
	);


//	nrf_drv_twi_disable(&i2c);
//	nrf_drv_twi_enable(&i2c);
	
	do 
	{
		ret = 
		nrf_drv_twi_rx(
			&i2c,
			ADDR,
			value,
			sizeof(value),
			false
		);

	}while(ret != NRF_SUCCESS);

	nrf_drv_twi_disable(&i2c);

	return value;
  
}

void write8(uint8_t reg, uint8_t in_data) {
	uint8_t data[2] = {reg, in_data};

	nrf_drv_twi_enable(&i2c);

	nrf_drv_twi_tx(&i2c,
				   ADDR,
				   data,
				   sizeof(data),
				   true);

	nrf_drv_twi_disable(&i2c);
}


