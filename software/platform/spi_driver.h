#ifndef SPI_DRIVER_H

#define SPI_DRIVER_H

// Pin selections
#define LED_PIN 25
#define SPI_SCK_PIN 0
#define SPI_MISO_PIN 29
#define SPI_MOSI_PIN 30
#define SPI_SS_PIN 22
#define NRF_SPI NRF_SPI0

//Initializes and enables spi
void spi_init();

void spi_write(uint8_t buf);

void spi_read(uint8_t* buf);

void spi_write_reg(uint8_t reg_addr, uint8_t * data, uint8_t num_bytes);

void spi_read_reg(uint8_t reg_addr, uint8_t * data, uint8_t num_bytes);

void spi_disable();

void spi_enable();

#endif
