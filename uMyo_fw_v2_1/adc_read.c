#include "nrf.h"
#include "adc_read.h"
#include "fft_opt.h"
#include "fast_math.h"

uint16_t adc_buf[16];
uint16_t res_buf[16];
volatile uint8_t has_new_data = 0;
volatile uint8_t is_batt_measure = 0;
volatile uint16_t batt_measurement = 0;

ADC_PIN_CONFIG_REG conf_emg;

void adc_init()
{
	NRF_SAADC->ENABLE = 1;
	NRF_SAADC->CH[0].PSELP = adc_ain4;
	NRF_SAADC->CH[0].PSELN = adc_ain_nc;
	conf_emg.fields.RESP = 0;
	conf_emg.fields.RESN = 1; //pulldown
	conf_emg.fields.GAIN = adc_gain_1_6;
	conf_emg.fields.MODE_DIFF = 0;
	conf_emg.fields.REF_VDD4 = 0;
	conf_emg.fields.TACQ = adc_taq_5us;
	conf_emg.fields.BURST = 1;
	NRF_SAADC->CH[0].CONFIG = conf_emg.value;	
	NRF_SAADC->RESOLUTION = adc_res_14;
//	NRF_SAADC->OVERSAMPLE = adc_oversample_256x; //(5us + 2us) x 256 oversample -> ~558 Hz sample rate
	NRF_SAADC->OVERSAMPLE = adc_oversample_128x; //(5us + 2us) x 128 oversample -> ~1116 Hz sample rate
	NRF_SAADC->SAMPLERATE = 0; //control from task
	ADC_INT_REG intcfg;
	intcfg.value = 0;
	intcfg.fields.END = 1;
//	intcfg.fields.DONE = 1;
	intcfg.fields.RESULTDONE = 1;
	NRF_SAADC->INTEN = intcfg.value;
	NRF_SAADC->RESULT.PTR = adc_buf;
	NRF_SAADC->RESULT.MAXCNT = 8;
	NVIC_EnableIRQ(SAADC_IRQn);
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->TASKS_START = 1;
	while(!NRF_SAADC->EVENTS_STARTED) ;
	NRF_SAADC->TASKS_SAMPLE = 1;
}

volatile int batt_skip_cnt = 0;

void SAADC_IRQHandler()
{
	if(NRF_SAADC->EVENTS_RESULTDONE && !NRF_SAADC->EVENTS_END)
	{
		NRF_SAADC->EVENTS_RESULTDONE = 0;
		NRF_SAADC->TASKS_SAMPLE = 1;
	}
//	NRF_SAADC->EVENTS_END = 0;
//	NRF_SAADC->EVENTS_DONE = 0;
//	NRF_SAADC->EVENTS_RESULTDONE = 0;
	if(NRF_SAADC->EVENTS_END)
	{
		NRF_SAADC->EVENTS_END = 0;
		NRF_SAADC->EVENTS_RESULTDONE = 0;
		if(!is_batt_measure)
		{
			for(int x = 0; x < 8; x++)
				res_buf[x] = adc_buf[x];
			has_new_data = 1;
			batt_skip_cnt++;
			if(batt_skip_cnt > 1000)
			{
				batt_skip_cnt = 0;
				is_batt_measure = 1;
				NRF_SAADC->CH[0].PSELP = adc_ain5;
				conf_emg.fields.TACQ = adc_taq_20us;
				NRF_SAADC->CH[0].CONFIG = conf_emg.value;	
				NRF_SAADC->OVERSAMPLE = adc_oversample_off;
				NRF_SAADC->RESULT.PTR = adc_buf+8;
				NRF_SAADC->RESULT.MAXCNT = 1;
			}
			else
			{
				NRF_SAADC->RESULT.PTR = adc_buf;
				NRF_SAADC->RESULT.MAXCNT = 8;
			}
			NRF_SAADC->EVENTS_STARTED = 0;
			NRF_SAADC->TASKS_START = 1;
			while(!NRF_SAADC->EVENTS_STARTED) ;
			NRF_SAADC->TASKS_SAMPLE = 1;
			
	//		NRF_SAADC->TASKS_SAMPLE = 1;
	//		NRF_SAADC->TASKS_START = 1;
		}
		else
		{
			is_batt_measure = 0;
			batt_measurement = adc_buf[8];
			NRF_SAADC->CH[0].PSELP = adc_ain4;
			conf_emg.fields.TACQ = adc_taq_5us;
			NRF_SAADC->CH[0].CONFIG = conf_emg.value;	
			NRF_SAADC->OVERSAMPLE = adc_oversample_128x;
			NRF_SAADC->RESULT.PTR = adc_buf;
			NRF_SAADC->RESULT.MAXCNT = 8;
			NRF_SAADC->EVENTS_STARTED = 0;
			NRF_SAADC->TASKS_START = 1;
			while(!NRF_SAADC->EVENTS_STARTED) ;
			NRF_SAADC->TASKS_SAMPLE = 1;
			
	//		NRF_SAADC->TASKS_SAMPLE = 1;
	//		NRF_SAADC->TASKS_START = 1;
		}
	}
}

uint8_t adc_has_data()
{
	return has_new_data;	
}

int adc_get_data(int16_t *raw_data, float *sp_data)
{
	if(!has_new_data) return 0;
	for(int x = 0; x < 8; x++)
		raw_data[x] = res_buf[x];
	float bufX[16];
	float bufYr[16];
	float bufYi[16];
//	for(int x = 0; x < 8; x++)
//		bufX[x] = res_buf[x];
	
//	fft8_real(bufX, bufYr, bufYi);
//	for(int x = 0; x < 4; x++)
//		sp_data[x] = sqrt_f(bufYr[x]*bufYr[x] + bufYi[x]*bufYi[x]);
	
	has_new_data = 0;
	return 8;
}
uint32_t adc_read_battery()
{
	if(batt_measurement == 0) return 3600; //before we make the first reading, return something that won't trigger low power shutdown
	int bat_mv = batt_measurement * 0.65918f;// / 16.384 * 0.6 * 6 * 3
	return bat_mv;
}
