PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += smartbike.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += app_timer.c
#APPLICATION_SRCS += app_button.c
APPLICATION_SRCS += app_util_platform.c
APPLICATION_SRCS += nrf_delay.c
APPLICATION_SRCS += nrf_drv_common.c
APPLICATION_SRCS += nrf_drv_gpiote.c
#APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += nrf_drv_twi.c
APPLICATION_SRCS += spi_driver.c
APPLICATION_SRCS += led.c
APPLICATION_SRCS += app_scheduler.c
APPLICATION_SRCS += adxl362.c
APPLICATION_SRCS += gpio_driver.c
APPLICATION_SRCS += AccelerometerControl.c
APPLICATION_SRCS += AccelDataDriver.c
APPLICATION_SRCS += AccelTurnControl.c
APPLICATION_SRCS += LightControl.c
APPLICATION_SRCS += LightAction.c
APPLICATION_SRCS += pwm.c
APPLICATION_SRCS += BikeState.c
APPLICATION_SRCS += BikeTimers.c

ifndef TARGET
	TARGET = SMARTBIKE
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LOWER = $(shell echo $(TARGET) | tr A-Z a-z)
BOARD = BOARD_$(TARGET_UPPER)

APPLICATION_SRCS += $(TARGET_LOWER).c

LIBRARY_PATHS += ./ ../../platform/
LIBRARY_PATHS += ../../drivers/
SOURCE_PATHS += ../../platform/
SOURCE_PATHS += ../../drivers/

NRF_BASE_PATH ?= ../../nrf5x-base/
TEMPLATE_PATH ?= $(NRF_BASE_PATH)/nrf51-pure-gcc-setup/template/

DEVICE = NRF51
USE_SOFTDEVICE = s110
SOFTDEVICE ?= $(NRF_BASE_PATH)/softdevice/s110_nrf51_8.0.0_softdevice.hex

SDK_VERSION = 9
SDK_PATH ?= $(NRF_BASE_PATH)/sdk/nrf51_sdk_9.0.0/

CFLAGS = -Os
GDB_PORT_NUMBER = 2337

include $(NRF_BASE_PATH)/Makefile.include
include $(TEMPLATE_PATH)/Makefile
