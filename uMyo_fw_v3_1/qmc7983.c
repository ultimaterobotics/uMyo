#include "qmc7983.h"


const uint8_t qmc7983_addr = 0x2C;
const uint8_t qmc5883_addr = 0x0D;
uint8_t qmc_addr = qmc7983_addr;

typedef struct {
    union {
        uint32_t reg;
        struct {
			unsigned : 7;
            unsigned LASTTX_STARTRX : 1;
            unsigned LASTTX_SUSPEND : 1;
            unsigned LASTTX_STOP : 1;
            unsigned LASTRX_STARTTX : 1;
			unsigned : 1;
			unsigned LASTRX_STOP : 1;
			unsigned : 19;
        }f;
    };
} sTWIM_shorts;

typedef struct {
    union {
        uint32_t reg;
        struct {
			unsigned : 1;
            unsigned STOPPED : 1;
            unsigned : 7;
            unsigned ERROR : 1;
            unsigned : 8;
			unsigned SUSPENDED : 1;
			unsigned RXSTARTED : 1;
			unsigned TXSTARTED : 1;
            unsigned : 2;
            unsigned LASTRX : 1;
            unsigned LASTTX : 1;
            unsigned : 7;
        }f;
    };
} sTWIM_interrupts;

#define TWIM_TX_BUF_SIZE 64
#define TWIM_RX_BUF_SIZE 64
#define TWIM_TIMEOUT 50

uint8_t twim_buf_tx[TWIM_TX_BUF_SIZE];
uint8_t twim_buf_rx[TWIM_RX_BUF_SIZE];
volatile uint8_t twim_busy = 0;
volatile uint8_t twim_needs_callback = 0;

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler()
{
	if(NRF_TWIM1->EVENTS_STOPPED)
	{
		if(twim_needs_callback)
		{
			twim_needs_callback = 0;
//			twim_rx_cplt();
		}
	}
	twim_busy = 0;
}


void twim_init(uint8_t pin_SCL, uint8_t pin_SDA)
{
	NRF_TWIM1->PSEL.SCL = pin_SCL;
	NRF_TWIM1->PSEL.SDA = pin_SDA;
	//NRF_TWIM1->FREQUENCY = 0x01980000; //100 kbps
	//NRF_TWIM1->FREQUENCY = 0x04000000;// 250 kbps
	NRF_TWIM1->FREQUENCY = 0x06400000;// 400 kbps
	
	sTWIM_interrupts ints;
	ints.reg = 0;
	ints.f.STOPPED = 1;
	ints.f.ERROR = 1;
	NRF_TWIM1->ENABLE = 6; //6 - twim mode
}

void twim_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
	if(twim_busy)
	{
		uint32_t start_ms = millis();
		while(twim_busy && millis() - start_ms < TWIM_TIMEOUT);
		if(twim_busy) return;
	}
	twim_buf_tx[0] = reg;
	twim_buf_tx[1] = val;
	NRF_TWIM1->ADDRESS = addr;
	NRF_TWIM1->TXD.MAXCNT = 2;
	NRF_TWIM1->TXD.PTR = twim_buf_tx;
	NRF_TWIM1->TXD.LIST = 0;
	NRF_TWIM1->RXD.MAXCNT = 0;
	NRF_TWIM1->RXD.PTR = twim_buf_rx;
	NRF_TWIM1->RXD.LIST = 0;
	sTWIM_shorts shrt;
	shrt.reg = 0;
	shrt.f.LASTTX_STOP = 1;
	NRF_TWIM1->SHORTS = shrt.reg;
	NRF_TWIM1->EVENTS_STOPPED = 0;
	NRF_TWIM1->EVENTS_ERROR = 0;
	NRF_TWIM1->TASKS_STARTTX = 1;
	while(!(NRF_TWIM1->EVENTS_STOPPED || NRF_TWIM1->EVENTS_ERROR));
}

void twim_write_data(uint8_t addr, uint8_t *buf, uint8_t len)
{
	if(twim_busy)
	{
		uint32_t start_ms = millis();
		while(twim_busy && millis() - start_ms < TWIM_TIMEOUT);
		if(twim_busy) return;
	}
	if(len > TWIM_TX_BUF_SIZE) 
		len = TWIM_TX_BUF_SIZE; //lol just pretend we did all right TODO: maybe fix some day
	for(int x = 0; x < len; x++)
		twim_buf_tx[x] = buf[x];
	
	NRF_TWIM1->ADDRESS = addr;
	NRF_TWIM1->TXD.MAXCNT = len;
	NRF_TWIM1->TXD.PTR = twim_buf_tx;
	NRF_TWIM1->TXD.LIST = 0;
	NRF_TWIM1->RXD.MAXCNT = 0;
	NRF_TWIM1->RXD.PTR = twim_buf_rx;
	NRF_TWIM1->RXD.LIST = 0;
	sTWIM_shorts shrt;
	shrt.reg = 0;
	shrt.f.LASTTX_STOP = 1;
	NRF_TWIM1->SHORTS = shrt.reg;
	NRF_TWIM1->EVENTS_STOPPED = 0;
	NRF_TWIM1->EVENTS_ERROR = 0;
	NRF_TWIM1->TASKS_STARTTX = 1;
	twim_busy = 1;
}

uint8_t twim_read_reg(uint8_t addr, uint8_t reg)
{
	if(twim_busy)
	{
		uint32_t start_ms = millis();
		while(twim_busy && millis() - start_ms < TWIM_TIMEOUT);
		if(twim_busy) return;
	}
	twim_buf_tx[0] = reg;
	NRF_TWIM1->ADDRESS = addr;
	NRF_TWIM1->TXD.MAXCNT = 1;
	NRF_TWIM1->TXD.PTR = twim_buf_tx;
	NRF_TWIM1->TXD.LIST = 0;
	NRF_TWIM1->RXD.MAXCNT = 1;
	NRF_TWIM1->RXD.PTR = twim_buf_rx;
	NRF_TWIM1->RXD.LIST = 0;
	sTWIM_shorts shrt;
	shrt.reg = 0;
	shrt.f.LASTTX_STARTRX = 1;
	shrt.f.LASTRX_STOP = 1;
	NRF_TWIM1->SHORTS = shrt.reg;
	NRF_TWIM1->EVENTS_STOPPED = 0;
	NRF_TWIM1->EVENTS_ERROR = 0;
	NRF_TWIM1->TASKS_STARTTX = 1;
	while(!(NRF_TWIM1->EVENTS_STOPPED || NRF_TWIM1->EVENTS_ERROR));
	return twim_buf_rx[0];
}

uint8_t twim_read_buf(uint8_t addr, uint8_t reg, uint8_t len)
{
	if(twim_busy)
	{
		uint32_t start_ms = millis();
		while(twim_busy && millis() - start_ms < TWIM_TIMEOUT);
		if(twim_busy) return;
	}
	if(len > TWIM_RX_BUF_SIZE) 
		len = TWIM_RX_BUF_SIZE; //lol just pretend we did all right TODO: maybe fix some day
	
	twim_buf_tx[0] = reg;
	NRF_TWIM1->ADDRESS = addr;
	NRF_TWIM1->TXD.MAXCNT = 1;
	NRF_TWIM1->TXD.PTR = twim_buf_tx;
	NRF_TWIM1->TXD.LIST = 0;
	NRF_TWIM1->RXD.MAXCNT = len;
	NRF_TWIM1->RXD.PTR = twim_buf_rx;
	NRF_TWIM1->RXD.LIST = 0;
	sTWIM_shorts shrt;
	shrt.reg = 0;
	shrt.f.LASTTX_STARTRX = 1;
	shrt.f.LASTRX_STOP = 1;
	NRF_TWIM1->SHORTS = shrt.reg;
	NRF_TWIM1->EVENTS_STOPPED = 0;
	NRF_TWIM1->EVENTS_ERROR = 0;
	NRF_TWIM1->TASKS_STARTTX = 1;
}

void qmc_init()
{
	twim_init(2, 3);
	QMC7983_CFG cfg;
	delay_ms(1);
	uint8_t q79_id = twim_read_reg(qmc7983_addr, QMC7983_CHIPID_A);
	uint8_t q58_id = twim_read_reg(qmc5883_addr, QMC7983_CHIPID_A);
	if(q79_id == 0xFF || q79_id == 0) qmc_addr = qmc5883_addr;
	else qmc_addr = qmc7983_addr;
	cfg.reg = 0;
	cfg.MODE = 0b01; // continuous
	cfg.RNG = 0b00; //2G
	cfg.OSR = 0b10; //128x
	cfg.ODR = 0b10; //100 Hz
	twim_write_reg(qmc_addr, QMC7983_CFG_A, cfg.reg);
	twim_write_reg(qmc_addr, QMC7983_SETRES_A, 0xFF); //datasheet recommends
	twim_read_buf(qmc_addr, QMC7983_DATAX_A, 6);
}

uint32_t qmc_last_data_time = 0;
int16_t mag_x = 0, mag_y = 0, mag_z = 0;
int16_t mx_min = 32760, mx_max = -32760;
int16_t my_min = 32760, my_max = -32760;
int16_t mz_min = 32760, mz_max = -32760;
float north_heading = 0;
uint8_t in_mag_calibration = 0;

void qmc_read()
{
	uint32_t ms = millis();
	if(ms - qmc_last_data_time < 10) return; //100 Hz update expected
	qmc_last_data_time = ms;
	uint8_t dpos = 0;
	mag_x = (twim_buf_rx[dpos+1]<<8) | twim_buf_rx[dpos]; dpos += 2;
	mag_y = (twim_buf_rx[dpos+1]<<8) | twim_buf_rx[dpos]; dpos += 2;
	mag_z = (twim_buf_rx[dpos+1]<<8) | twim_buf_rx[dpos]; dpos += 2;
	if(qmc_addr == qmc5883_addr) //different axis position vs qmc7983, remap
	{
		int tt = mag_x;
		mag_x = mag_y;
		mag_y = -tt;
	}
	if(in_mag_calibration)
	{
		if(mag_x > mx_max) mx_max = mag_x+1;
		if(mag_x < mx_min) mx_min = mag_x-1;
		if(mag_y > my_max) my_max = mag_y+1;
		if(mag_y < my_min) my_min = mag_y-1;
		if(mag_z > mz_max) mz_max = mag_z+1;
		if(mag_z < mz_min) mz_min = mag_z-1;
	}
	
	int m1 = mag_x - mx_min;
	int m2 = mx_max - mx_min;
	mag_x = 10000*m1 / m2 - 5000;
	m1 = mag_y - my_min;
	m2 = my_max - my_min;
	mag_y = 10000*m1 / m2 - 5000;
	m1 = mag_z - mz_min;
	m2 = mz_max - mz_min;
	mag_z = 10000*m1 / m2 - 5000;
	twim_read_buf(qmc_addr, QMC7983_DATAX_A, 6);
//    north_heading = atan2(mag_y, mag_x);
//    Serial.printf("%d %d %d: %g\n", mag_x, mag_y, mag_z, north_heading);
}

void qmc_get_mag(int16_t *mx, int16_t *my, int16_t *mz)
{
	*mx = mag_x;
	*my = mag_y;
	*mz = mag_z;
}

int qmc_process_calibration()
{
	if(!in_mag_calibration)
	{
		mx_min = 32000;
		mx_max = -32000;
		my_min = 32000;
		my_max = -32000;
		mz_min = 32000;
		mz_max = -32000;
		
		in_mag_calibration = 1;
	}
}

void qmc_end_calibration()
{
	in_mag_calibration = 0;
}
void qmc_get_calibration_data(int16_t *min_x, int16_t *max_x, int16_t *min_y, int16_t *max_y, int16_t *min_z, int16_t *max_z)
{
	*min_x = mx_min;
	*max_x = mx_max;
	*min_y = my_min;
	*max_y = my_max;
	*min_z = mz_min;
	*max_z = mz_max;
}
void qmc_set_calibration_state(int16_t min_x, int16_t max_x, int16_t min_y, int16_t max_y, int16_t min_z, int16_t max_z)
{
	mx_min = min_x;
	mx_max = max_x;
	my_min = min_y;
	my_max = max_y;
	mz_min = min_z;
	mz_max = max_z;
}
