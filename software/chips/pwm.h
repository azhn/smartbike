#ifndef PWM_H
#define PWM_H

#include "nrf_drv_config.h"
#include "nrf_drv_twi.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "smartbike.h"

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define ADDR 0x40


void pca9685_init(nrf_drv_twi_t * i2c_instance_param);
void pca9685_reset(void);
void pca9685_setPWMFreq(float freq);
void pca9685_setPWM(uint8_t num, uint16_t on, uint16_t off);
void pca9685_setPin(uint8_t num, uint16_t val, uint8_t invert);
void pca9685_writeByte(uint8_t reg, uint8_t in_data);
uint8_t pca9685_readByte(uint8_t reg);

#endif
