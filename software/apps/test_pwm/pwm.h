#ifndef PWM_H
#define PWM_H

#include "nrf_drv_config.h"
#include "nrf_drv_twi.h"
#include "nrf_soc.h"
#include "blees.h"

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


void init(void);
void reset(void);
void setPWMFreq(float freq);
void setPWM(uint8_t num, uint16_t on, uint16_t off);
void write8(uint8_t reg, uint8_t in_data);
uint8_t read8(uint8_t reg);

#endif
