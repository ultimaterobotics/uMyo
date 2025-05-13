#include <stdint.h>

void spi_init(uint8_t pin_SCK, uint8_t pin_COPI, uint8_t pin_CIPO);
uint8_t spi_is_busy();

void spi_write_reg8(uint8_t reg, uint8_t value, uint32_t cs_mask);
uint8_t spi_read_reg8(uint8_t reg, uint32_t cs_mask);
void spi_write_reg16(uint8_t reg, uint16_t value, uint32_t cs_mask);
uint16_t spi_read_reg16(uint8_t reg, uint32_t cs_mask);
void spi_write_reg24(uint8_t reg, uint32_t value, uint32_t cs_mask);
uint32_t spi_read_reg24(uint8_t reg, uint32_t cs_mask);
void spi_write_reg32(uint8_t reg, uint32_t value, uint32_t cs_mask);
uint32_t spi_read_reg32(uint8_t reg, uint32_t cs_mask);
void spi_write_buf(uint8_t reg, uint8_t *buf, int buf_len, uint32_t cs_mask);
void spi_read_buf(uint8_t reg, int buf_len, void (*rx_cplt)(), uint8_t *read_res_buf, uint32_t cs_mask);
void spi_read_buf_blocking(uint8_t reg, int buf_len, uint8_t *buf, uint32_t cs_mask);
