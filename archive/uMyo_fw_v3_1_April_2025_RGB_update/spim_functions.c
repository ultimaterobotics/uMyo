#include "spim_functions.h"
#include "nrf.h"

volatile uint8_t spi_rx_buf[32];
volatile uint8_t spi_tx_buf[32];
volatile uint8_t spi_busy = 0;
volatile uint8_t spi_need_callback = 0;
uint32_t spi_cs_mask = 0;

void (*spi_rx_cplt)() = 0;

void spi_init(uint8_t pin_SCK, uint8_t pin_COPI, uint8_t pin_CIPO)
{
	//pins i/o already configured in board configuration, so only need to assign them here
	NRF_SPIM0->PSEL.SCK = pin_SCK;
	NRF_SPIM0->PSEL.MOSI = pin_COPI;
	NRF_SPIM0->PSEL.MISO = pin_CIPO;
	
	NRF_SPIM0->FREQUENCY = 0x80000000; 
//	NRF_SPI0->FREQUENCY = 0x02000000; 
	/* 0x02000000 125 kbps
	 * 0x04000000 250 kbps
	 * 0x08000000 500 kbps
	 * 0x10000000 1 Mbps
	 * 0x20000000 2 Mbps
	 * 0x40000000 4 Mbps
	 * 0x80000000 8 Mbps */
	
	NRF_SPIM0->RXD.LIST = 0;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->RXD.MAXCNT = 16;
	
	NRF_SPIM0->TXD.LIST = 0;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	NRF_SPIM0->TXD.MAXCNT = 16;
	
	NRF_SPIM0->SHORTS = 0;
	NRF_SPIM0->INTENCLR = 0xFFFFFFFF;
	NRF_SPIM0->INTENSET = 1<<6; //END event
	
	NRF_SPIM0->CONFIG = 0b000; // 0bxx0 - SPI mode 0b00x - polarity, 0 - MSB first
	NRF_SPIM0->ENABLE = 7;
//	NVIC_EnableIRQ(SPI0_TWI0_IRQn);
	NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 8); //transaction completed, no need to rush
	NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
//	NVIC_SetPriority(3, 8); //transaction completed, no need to rush
//	NVIC_EnableIRQ(3);
//	SPIM0_SPIS0_IRQHandler();
}

uint8_t spi_is_busy()
{
	return spi_busy;
}

//void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler()
//void SPIM0_SPIS0_IRQHandler()
//void TWIM0_TWIS0_IRQHandler() //bug in 52810 defines, this handler goes for SPIM0 
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler() //but v3 is using 52832
{
	if(NRF_SPIM0->EVENTS_END)
	{
		if(spi_need_callback)
		{
			spi_need_callback = 0;
			spi_rx_cplt();
		}
	}
	NRF_GPIO->OUTSET = spi_cs_mask;
	NRF_SPIM0->EVENTS_END = 0;
	spi_busy = 0;
}

void spi_write_reg8(uint8_t reg, uint8_t value, uint32_t cs_mask)
{
	while(spi_busy) ;
	spi_cs_mask = cs_mask;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = value;
	NRF_SPIM0->TXD.MAXCNT = 2;
	NRF_SPIM0->TXD.LIST = 0;
	NRF_SPIM0->RXD.MAXCNT = 1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
}

uint8_t spi_read_reg8(uint8_t reg, uint32_t cs_mask)
{
	while(spi_busy) ;
	spi_cs_mask = cs_mask;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = 2;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	while(spi_busy);
	
	return spi_rx_buf[1];
}
void spi_write_reg16(uint8_t reg, uint16_t value, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = value>>8;
	spi_tx_buf[2] = value;
	NRF_SPIM0->TXD.MAXCNT = 3;
	NRF_SPIM0->RXD.MAXCNT = 1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
}
uint16_t spi_read_reg16(uint8_t reg, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = 3;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	while(spi_busy);
	
	return (spi_rx_buf[1]<<8) | spi_rx_buf[2];
}
void spi_write_reg24(uint8_t reg, uint32_t value, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = value>>16;
	spi_tx_buf[2] = value>>8;
	spi_tx_buf[3] = value;
	NRF_SPIM0->TXD.MAXCNT = 4;
	NRF_SPIM0->RXD.MAXCNT = 1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
}
uint32_t spi_read_reg24(uint8_t reg, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = 4;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	while(spi_busy);
	
	return (spi_rx_buf[1]<<16) | (spi_rx_buf[2]<<8) | spi_rx_buf[3];
}
void spi_write_reg32(uint8_t reg, uint32_t value, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = value>>24;
	spi_tx_buf[2] = value>>16;
	spi_tx_buf[3] = value>>8;
	spi_tx_buf[4] = value;
	NRF_SPIM0->TXD.MAXCNT = 5;
	NRF_SPIM0->RXD.MAXCNT = 1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
}
uint32_t spi_read_reg32(uint8_t reg, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;
	
	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = 5;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	while(spi_busy);
	
	return (spi_rx_buf[1]<<24) | (spi_rx_buf[2]<<16) | (spi_rx_buf[3]<<8) | spi_rx_buf[4];
}
void spi_write_buf(uint8_t reg, uint8_t *buf, int buf_len, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;

	spi_tx_buf[0] = reg;
	for(int x = 0; x < buf_len; x++)
		spi_tx_buf[1+x] = buf[x];
	NRF_SPIM0->TXD.MAXCNT = buf_len+1;
	NRF_SPIM0->RXD.MAXCNT = 1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
}
void spi_read_buf(uint8_t reg, int buf_len, void (*rx_cplt)(), uint8_t *read_res_buf, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = read_res_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;
	
	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = buf_len+1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	spi_rx_cplt = *rx_cplt;
	spi_need_callback = 1;
}
void spi_read_buf_blocking(uint8_t reg, int buf_len, uint8_t *buf, uint32_t cs_mask)
{
	while(spi_busy) ;
	NRF_SPIM0->RXD.PTR = spi_rx_buf;
	NRF_SPIM0->TXD.PTR = spi_tx_buf;
	spi_cs_mask = cs_mask;
	NRF_GPIO->OUTCLR = spi_cs_mask;
	
	spi_tx_buf[0] = reg;
	spi_tx_buf[1] = 0;
	NRF_SPIM0->TXD.MAXCNT = 1;
	NRF_SPIM0->RXD.MAXCNT = buf_len+1;
	spi_busy = 1;
	NRF_SPIM0->TASKS_START = 1;
	while(spi_busy);
	for(int x = 0; x < buf_len; x++)
		buf[x] = spi_rx_buf[1+x];
	
	return;
}
