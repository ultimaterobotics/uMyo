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
	int record_size = sizeof(sDevice_state);
	int values_count = record_size/4; //number of 32-bit values
	int records_cnt = page_size/record_size; //16 bytes per state
	int cur_pos = 0;
	uint32_t prev_record[sizeof(sDevice_state)/4];
	for(int n = 0; n < values_count; n++)
		prev_record[n] = 0xFFFFFFFF;
	while(cur_pos < records_cnt)
	{
		uint32_t addr = start_addr + cur_pos*record_size;
		uint32_t val = *(uint32_t*)addr;
		//we check only the first value, can't be all FF for a valid record
		if(val == 0xFFFFFFFF) //empty value -> previous one is good
		{
			sDevice_state st;
			for(int n = 0; n < values_count; n++)
				st.values[n] = prev_record[n];
			return st;
		}
		for(int n = 0; n < values_count; n++)
		{
			uint32_t addr = start_addr + cur_pos*record_size + n*4;
			uint32_t val = *(uint32_t*)addr;
			prev_record[n] = val;
		}
		cur_pos++;
	}
	sDevice_state st;
	for(int n = 0; n < values_count; n++)
		st.values[n] = prev_record[n];
	return st;
}
void update_current_state(sDevice_state state)
{
	uint32_t page_size = NRF_FICR->CODEPAGESIZE;
	uint32_t page = STORAGE_PAGE_ADDRESS / page_size;
	uint32_t start_addr = page_size * page;
	int record_size = sizeof(sDevice_state);
	int values_count = record_size/4; //number of 32-bit values
	int records_cnt = page_size/record_size; //16 bytes per state
	int cur_pos = 0;
	uint32_t prev_record[sizeof(sDevice_state)/4];
	for(int n = 0; n < values_count; n++)
		prev_record[n] = 0xFFFFFFFF;
	int pos_to_write = 0;
	while(cur_pos < records_cnt)
	{
		uint32_t addr = start_addr + cur_pos*record_size;
		uint32_t val = *(uint32_t*)addr;
		if(val == 0xFFFFFFFF) //empty value -> previous one is current state
		{
			int need_update = 0;
			for(int n = 0; n < values_count; n++)
			{
				uint32_t addr = start_addr + cur_pos*record_size + n*4;
				uint32_t val = *(uint32_t*)addr;
				if(val != state.values[n]) need_update = 1;
			}
			if(!need_update) return; //nothing to update
			if(cur_pos == 0)
			{
				pos_to_write = 0;
				break;
			}
			int can_overwrite = 1;
			for(int n = 0; n < values_count; n++)
			{
				uint32_t flash_zeroes_pos = ~prev_record[n]; //ones where zeros
				uint32_t state_ones_pos = state.values[n]; //ones where ones
				if(flash_zeroes_pos & state_ones_pos) can_overwrite = 0; //if none overlap, can reuse memory
			}
			if(can_overwrite)
				pos_to_write = cur_pos-1;
			else
				pos_to_write = cur_pos;
			break;
		}
		for(int n = 0; n < values_count; n++)
		{
			uint32_t addr = start_addr + cur_pos*record_size + n*4;
			uint32_t val = *(uint32_t*)addr;
			prev_record[n] = val;
		}
		cur_pos++;
	}
	if(cur_pos >= records_cnt-1)
	{
		erase_page(page);
		pos_to_write = 0;
	}
	write_start();
	for(int n = 0; n < values_count; n++)
	{
		uint32_t addr = start_addr + pos_to_write*record_size + n*4;	
		write_word(addr, state.values[n]);
	}
	write_end();
}

