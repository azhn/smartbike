PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_util_platform.c
APPLICATION_SRCS += nrf_delay.c
#APPLICATION_SRCS += nrf_drv_config.c
APPLICATION_SRCS += nrf_drv_common.c
APPLICATION_SRCS += nrf_drv_gpiote.c
APPLICATION_SRCS += nrf_drv_twi.c
#APPLICATION_SRCS += nrf_drv_qdec.c
APPLICATION_SRCS += led.c
APPLICATION_SRCS += app_scheduler.c
APPLICATION_SRCS += adxl362.c
APPLICATION_SRCS += spi_driver.c
APPLICATION_SRCS += app_error.c
#APPLICATION_SRCS += nrf_gpio.c

ifndef TARGET
	TARGET = SMARTBIKE
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LOWER = $(shell echo $(TARGET) | tr A-Z a-z)
BOARD = BOARD_$(TARGET_UPPER)

APPLICATION_SRCS += $(TARGET_LOWER).c

LIBRARY_PATHS += ./ ../../platform/
LIBRARY_PATHS += ../../chips/
LIBRARY_PATHS += ../../nrf5x-base/
LIBRARY_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/
LIBRARY_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/config/
#LIBRARY_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/qdec/
LIBRARY_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/libraries/util/
LIBRARY_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/libraries/util/

SOURCE_PATHS += ../../platform/
SOURCE_PATHS += ../../chips/
SOURCE_PATHS += ../../nrf5x-base/
SOURCE_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/
SOURCE_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/libraries/util/
SOURCE_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/config/
SOURCE_PATHS += ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/components/drivers_nrf/qdec/

NRF_BASE_PATH ?= ../../nrf5x-base/
TEMPLATE_PATH ?= $(NRF_BASE_PATH)/nrf51-pure-gcc-setup/template/

DEVICE = NRF51
USE_SOFTDEVICE = s110
SOFTDEVICE ?= $(NRF_BASE_PATH)/softdevice/s110_nrf51_8.0.0_softdevice.hex

SDK_VERSION = 9
SDK_PATH ?= $(NRF_BASE_PATH)/sdk/nrf51_sdk_9.0.0/

CFLAGS = -Os
GDB_PORT_NUMBER = 2331

include $(NRF_BASE_PATH)/Makefile.include
include $(TEMPLATE_PATH)/Makefile
