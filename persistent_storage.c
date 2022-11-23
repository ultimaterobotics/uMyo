#include "persistent_storage.h"
#include "nrf.h"
#include <stdint.h>

//around the flash end, storage size = page size
#define STORAGE_PAGE_ADDRESS 0x2B000

#define FLASH_READFLAG 0x00
#define FLASH_WRITEFLAG 0x01
#define FLASH_ERASEFLAG 0x02


void erase_page(uint32_t page)
{
    // Turn on flash write
    while (!NRF_NVMC->READY) ;
    NRF_NVMC->CONFIG = FLASH_ERASEFLAG;
    while (!NRF_NVMC->READY) ;

    // Erase page.
    NRF_NVMC->ERASEPAGE = page;
    while (!NRF_NVMC->READY) ;

    NRF_NVMC->CONFIG = FLASH_READFLAG;
    while (!NRF_NVMC->READY) ;
}

void write_start()
{
    // Enable write.
    while (!NRF_NVMC->READY) ;
	NRF_NVMC->CONFIG = FLASH_WRITEFLAG; 
    while (!NRF_NVMC->READY) ;	
}

void write_word(uint32_t address, uint32_t value)
{
    *(uint32_t*)address = value;
    while (!NRF_NVMC->READY) ;
}
void write_end()
{
	NRF_NVMC->CONFIG = FLASH_READFLAG; 
    while (!NRF_NVMC->READY) ;
}

sDevice_state read_current_state()
{
	uint32_t page_size = NRF_FICR->CODEPAGESIZE;
	uint32_t page = STORAGE_PAGE_ADDRESS / page_size;
	uint32_t start_addr = page_size * page;
	int values_cnt = page_size/4; //4 bytes per state
	int cur_pos = 0;
	uint32_t prev_val = 0xFFFFFFFF;
	while(cur_pos < values_cnt)
	{
		uint32_t addr = start_addr + cur_pos*4;
		uint32_t val = *(uint32_t*)addr;
		if(val == 0xFFFFFFFF) //empty value -> previous one is good
		{
			sDevice_state st;
			st.value = prev_val;
			return st;
		}
		prev_val = val;
		cur_pos++;
	}
	sDevice_state st;
	st.value = prev_val;
	return st;
}
void update_current_state(sDevice_state state)
{
	uint32_t page_size = NRF_FICR->CODEPAGESIZE;
	uint32_t page = STORAGE_PAGE_ADDRESS / page_size;
	uint32_t start_addr = page_size * page;
	int values_cnt = page_size/4; //4 bytes per state
	int cur_pos = 0;
	uint32_t prev_val = 0xFFFFFFFF;
	int pos_to_write = 0;
	while(cur_pos < values_cnt)
	{
		uint32_t addr = start_addr + cur_pos*4;
		uint32_t val = *(uint32_t*)addr;
		if(val == 0xFFFFFFFF) //empty value -> previous one is current state
		{
			if(val == state.value) return; //nothing to update
			if(cur_pos == 0)
			{
				pos_to_write = 0;
				break;
			}
			int can_overwrite = 1;
			uint32_t flash_zeroes_pos = ~prev_val; //ones where zeros
			uint32_t state_ones_pos = state.value; //ones where ones
			if(flash_zeroes_pos & state_ones_pos) can_overwrite = 0; //if none overlap, can reuse memory
			if(can_overwrite)
				pos_to_write = cur_pos-1;
			else
				pos_to_write = cur_pos;
			break;
		}
		prev_val = val;
		cur_pos++;
	}
	if(cur_pos >= values_cnt-1)
	{
		erase_page(page);
		pos_to_write = 0;
	}
	write_start();
	write_word(start_addr + pos_to_write*4, state.value);
	write_end();
}

