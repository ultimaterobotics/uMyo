/**
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf.h"
#include "leds.h"
#include "urf_radio.h"
#include "urf_star_protocol.h"
#include "urf_ble_peripheral.h"
#include "ble_const.h"
#include "urf_timer.h"
#include "adc_read.h"
#include "fft_opt.h"
#include "quat_math.h"
#include "lsm6ds3.h"
#include "persistent_storage.h"

extern uint32_t __StackLimit, __StackTop;      // symbols from nrf52_common.ld

static void paint_stack(void)
{
    for (uint32_t *p = &__StackLimit; p < &__StackTop; p++)
    {
        *p = 0xDEADBEEF;   // fill stack with a known pattern
    }
}

uint8_t pin_button = 19;
uint8_t pin_syson = 4; 

sDevice_state dev_state;
uint32_t dev_state_changed_time = 0;
uint8_t dev_state_changed = 0;

float battery_mv = 3500; //by default non zero to prevent power off before actual measurement
int battery_low_threshold = 3200;

uint8_t battery_level = 0;
uint8_t packet_id = 0;
uint8_t data_packet[258];

uint8_t unsent_cnt = 0;

uint8_t adc_data_id = 0;
int adc_buffer[130];
int emg_raw_buffer[130];
float cur_spectr[16] = {0};
uint8_t adc_buf_pos = 0;
uint8_t adc_buf_len = 128;
uint8_t adc_cnt = 0;

uint8_t param_send_id = 0;

uint8_t emg_raw_data_send = 1;

enum param_sends
{
	param_batt = 0,
	param_end
};

void prepare_data_packet()
{
	uint8_t idx = 0;
	packet_id++;
	if(packet_id > 128) packet_id = 0;
	data_packet[idx++] = packet_id;
	data_packet[idx++] = 0; //fill in the end
	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	data_packet[idx++] = (unit_id>>24)&0xFF;
	data_packet[idx++] = (unit_id>>16)&0xFF;
	data_packet[idx++] = (unit_id>>8)&0xFF;
	data_packet[idx++] = (unit_id)&0xFF;
	uint8_t send_cnt = 8;
	data_packet[idx++] = 80+send_cnt;

	param_send_id++;
	if(param_send_id == param_end) param_send_id = 0;

	param_send_id = 0; //force for now
	
	data_packet[idx++] = param_send_id;
	if(param_send_id == param_batt)
	{
		int bat_mv = adc_read_battery();
		if(bat_mv < 2000)
			battery_level = 0;
		else
			battery_level = (bat_mv - 2000)/10;
		if(bat_mv > 4500) battery_level = 255;
		data_packet[idx++] = battery_level;
		data_packet[idx++] = 100; //version_id
		data_packet[idx++] = 0;
	}
	
	data_packet[idx++] = adc_data_id;
	int start_cnt = adc_buf_pos - send_cnt;
	if(start_cnt < 0) start_cnt += adc_buf_len;
	for(int n = 0; n < send_cnt; n++)
	{
		data_packet[idx++] = adc_buffer[start_cnt]>>8;
		data_packet[idx++] = adc_buffer[start_cnt]&0xFF;
		start_cnt++;
		if(start_cnt >= adc_buf_len) start_cnt = 0;
	}
	for(int x = 0; x < 4; x++)
	{
		int16_t send_v = cur_spectr[x];
		data_packet[idx++] = send_v>>8;
		data_packet[idx++] = send_v&0xFF;
	}
	int16_t qww, qwx, qwy, qwz;
	int16_t raX, raY, raZ;
	int16_t rwX, rwY, rwZ;
	lsm_get_quat_packed(&qww, &qwx, &qwy, &qwz);
	lsm_get_acc(&raX, &raY, &raZ);
	lsm_get_W(&rwX, &rwY, &rwZ);
	data_packet[idx++] = qww>>8;
	data_packet[idx++] = qww&0xFF;
	data_packet[idx++] = qwx>>8;
	data_packet[idx++] = qwx&0xFF;
	data_packet[idx++] = qwy>>8;
	data_packet[idx++] = qwy&0xFF;
	data_packet[idx++] = qwz>>8;
	data_packet[idx++] = qwz&0xFF;

//	data_packet[idx++] = rwX>>8;
//	data_packet[idx++] = rwX&0xFF;
//	data_packet[idx++] = rwY>>8;
//	data_packet[idx++] = rwY&0xFF;
//	data_packet[idx++] = rwZ>>8;
//	data_packet[idx++] = rwZ&0xFF;

	data_packet[idx++] = raX>>8;
	data_packet[idx++] = raX&0xFF;
	data_packet[idx++] = raY>>8;
	data_packet[idx++] = raY&0xFF;
	data_packet[idx++] = raZ>>8;
	data_packet[idx++] = raZ&0xFF;
	
	int16_t roll = lsm_get_roll();
	int16_t yaw = lsm_get_yaw();
	int16_t pitch = lsm_get_pitch();

	data_packet[idx++] = yaw>>8;
	data_packet[idx++] = yaw&0xFF;
	data_packet[idx++] = pitch>>8;
	data_packet[idx++] = pitch&0xFF;
	data_packet[idx++] = roll>>8;
	data_packet[idx++] = roll&0xFF;
	
//	int16_t yaw, pitch, roll;
//	lsm_get_angles(&yaw, &pitch, &roll);
//	data_packet[idx++] = pitch>>8;
//	data_packet[idx++] = pitch&0xFF;
//	data_packet[idx++] = roll>>8;
//	data_packet[idx++] = roll&0xFF;

//	sLSM6DS3 *imu = lsm_get_object();
//	int sp0 = cur_spectr[0];
//	data_packet[idx++] = sp0>>8;
	data_packet[1] = idx;
}

void prepare_data_packet32()
{
	uint8_t idx = 0;
//	packet_id++;
//	if(packet_id > 128) packet_id = 0;
	data_packet[idx++] = adc_data_id;
	data_packet[idx++] = 0; //fill in the end
	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	data_packet[idx++] = (unit_id>>24)&0xFF;
	data_packet[idx++] = (unit_id>>16)&0xFF;
	data_packet[idx++] = (unit_id>>8)&0xFF;
	data_packet[idx++] = (unit_id)&0xFF;

	int bat_mv = adc_read_battery();
	if(bat_mv < 2000)
		battery_level = 0;
	else
		battery_level = (bat_mv - 2000)/10;
	if(bat_mv > 4500) battery_level = 255;
	data_packet[idx++] = battery_level;
	//7 bytes here, 2 checksum, 23 remains
	for(int x = 0; x < 4; x++)
	{
		int16_t send_v = cur_spectr[x];
		data_packet[idx++] = send_v>>8;
		data_packet[idx++] = send_v&0xFF;
	}
	//8 for spectrum, 15 remains
	int16_t qww, qwx, qwy, qwz;
	lsm_get_quat_packed(&qww, &qwx, &qwy, &qwz);
	data_packet[idx++] = qww>>8;
	data_packet[idx++] = qww&0xFF;
	data_packet[idx++] = qwx>>8;
	data_packet[idx++] = qwy>>8;
	data_packet[idx++] = qwz>>8;
	//5 for quaternion, 10 remains
	
	int start_cnt = adc_buf_pos - 8;
	if(start_cnt < 0) start_cnt += adc_buf_len;
	data_packet[idx++] = adc_buffer[start_cnt]>>8;
	data_packet[idx++] = adc_buffer[start_cnt]&0xFF;
	int start_v = adc_buffer[start_cnt];
	uint8_t sign_byte = 0;
	for(int n = 0; n < 7; n++)
	{
		start_cnt++;
		if(start_cnt >= adc_buf_len) start_cnt = 0;
		int v = adc_buffer[start_cnt];
		int dv = v - start_v;
		if(dv < -255) dv = -255;
		if(dv > 255) dv = 255;
		if(dv < 0)
		{
			dv = -dv;
			sign_byte |= 1<<(6-n);
		}
		data_packet[idx++] = dv;
	}
	data_packet[idx++] = sign_byte;
	data_packet[1] = idx + 2;
	
	uint8_t chk1 = 0;
	uint8_t chk2 = 0;
	for(int x = 0; x < idx; x++)
	{
		chk1 += data_packet[x];
		chk2 += chk1;
	}
	data_packet[idx++] = chk1;
	data_packet[idx++] = chk2;
}

uint8_t ble_mac[6];

int prepare_and_send_BLE()
{
	uint32_t ms = millis();
	static int cur_adv_delay = 10;
	static uint32_t radio_last_send_time = 0;
	if(ms - radio_last_send_time < cur_adv_delay) return 0;
	radio_last_send_time = ms;
	uint8_t rnd = adc_buffer[adc_buf_pos]&0xFF;
	cur_adv_delay = 2 + (rnd%8);

	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	uint8_t ble_mac[6];
	ble_mac[0] = 212;
	ble_mac[1] = (unit_id>>24)&0xFF;
	ble_mac[2] = (unit_id>>16)&0xFF;
	ble_mac[3] = (unit_id>>8)&0xFF;
	ble_mac[4] = (unit_id)&0xFF;
	ble_mac[5] = 215;

	int bat_mv = adc_read_battery();
	if(bat_mv < 2000)
		battery_level = 0;
	else
		battery_level = (bat_mv - 2000)/10;
	if(bat_mv > 4500) battery_level = 255;
	
	uint8_t pdu_data[40];
	uint8_t data[40];
	int pp = 0;
	for(int x = 0; x < 6; x++)
		pdu_data[pp++] = ble_mac[x];
	sBLE_flags flags;
	flags.value = 0;
	data[0] = flags.value;
	pp = ble_add_field_to_pdu(pdu_data, pp, data, 1, PDU_FLAGS);
	int nlen = sprintf((char*)data, "uMyo v2");
	pp = ble_add_field_to_pdu(pdu_data, pp, data, nlen, PDU_SHORT_NAME);
	
	//17 bytes remains for payload
	int dpos = 0;
	data[dpos++] = adc_data_id;
	data[dpos++] = battery_level;
	for(int x = 0; x < 4; x++)
	{
		int16_t send_v = cur_spectr[x];
		data[dpos++] = send_v>>8;
		data[dpos++] = send_v&0xFF;
	}
	int16_t qww, qwx, qwy, qwz;
	lsm_get_quat_packed(&qww, &qwx, &qwy, &qwz);
	data[dpos++] = qww>>8;
	data[dpos++] = qww&0xFF;
	data[dpos++] = qwx>>8;
	data[dpos++] = qwy>>8;
	data[dpos++] = qwz>>8;
	pp = ble_add_field_to_pdu(pdu_data, pp, data, dpos, PDU_MANUFACTURER_SPEC);
		
	static int adv_ch = 37; //cycle through advertising channels
	uint8_t pdu[40];

	ble_set_connection_mode(0);
	int len = ble_prepare_adv_pdu(pdu, pp, pdu_data, BLE_ADV_NONCONN_IND_TYPE, 0, 1);
	ble_send_advertisement_packet(len, pdu, adv_ch);
	adv_ch++;
	if(adv_ch > 39) adv_ch = 37;
	return 1;
}

int16_t emg_raw[16];
float emg_sp_data[16];
int emg_level_r = 0;
int emg_level_g = 0;
int emg_level_b = 0;

float filt_c1 = 0.5;
float filt_c2 = 0.5;
int filt_dp1 = 5;
int filt_dp2 = 6;
int d_val_out = 0;

float t50_coeff = 0.044802867 * 2.0 * M_PI; //t*2*PI makes 50 revolutions per second
float t60_coeff = 0.053763441 * 2.0 * M_PI; //t*2*PI makes 60 revolutions per second
float t50 = 0; //time for 50 hz calculations
float t60 = 0; //time for 60 hz calculations
float noise50_s = 0; //sin part of amplitude of 50 hz wave
float noise50_c = 0; //cos part of amplitude of 50 hz wave
float noise60_s = 0; //sin part of amplitude of 60 hz wave
float noise60_c = 0; //cos part of amplitude of 60 hz wave

int push_adc_data()
{
	if(adc_get_data(emg_raw, emg_sp_data))
	{
		static float avg_emg_long = 0;
		float bufX[16];
		float bufYr[16];
		float bufYi[16];
		
		for(int x = 0; x < 8; x++)
		{
//computationally expensive part
			t50 += t50_coeff;
			if(t50 > 2*M_PI) t50 -= 2*M_PI;
			t60 += t60_coeff;
			if(t60 > 2*M_PI) t60 -= 2*M_PI;
			float s50 = sin_f(t50);
			float c50 = cos_f(t50);
			float s60 = sin_f(t60);
			float c60 = cos_f(t60);
			noise50_s *= 0.99;
			noise50_c *= 0.99;
			noise50_s += 0.01*s50*emg_raw[x];
			noise50_c += 0.01*c50*emg_raw[x];
			noise60_s *= 0.99;
			noise60_c *= 0.99;
			noise60_s += 0.01*s60*emg_raw[x];
			noise60_c += 0.01*c60*emg_raw[x];
			int mains_is_50Hz = 0;
			if(noise50_s*noise50_s + noise50_c*noise50_c > noise60_s*noise60_s + noise60_c*noise60_c)
				mains_is_50Hz = 1;
//========================
			
			emg_raw_buffer[adc_buf_pos] = emg_raw[x];
			avg_emg_long *= 0.99f;
			avg_emg_long += 0.01*emg_raw[x];
			int fpos1 = adc_buf_pos - 11; //basic 50Hz filter
			if(!mains_is_50Hz) fpos1 = adc_buf_pos - 9; //basic 60Hz filter
			if(fpos1 < 0) fpos1 += adc_buf_len;
			adc_buffer[adc_buf_pos] = emg_raw_buffer[adc_buf_pos] + emg_raw_buffer[fpos1];// + filt_c2 * emg_raw_buffer[fpos2];
//			adc_buffer[adc_buf_pos] = emg_raw_buffer[adc_buf_pos];// + filt_c2 * emg_raw_buffer[fpos2];
			bufX[x] = adc_buffer[adc_buf_pos];
			adc_buf_pos++;
			if(adc_buf_pos >= adc_buf_len) adc_buf_pos = 0;
		}
	
		fft8_real(bufX, bufYr, bufYi);
		for(int x = 0; x < 4; x++)
		{
			float sv = sqrt_f(bufYr[x]*bufYr[x] + bufYi[x]*bufYi[x]);
			cur_spectr[x] = sv;
//			cur_spectr[x] *= 0.7;
//			cur_spectr[x] += 0.3*sv;
		}
						
		adc_data_id++;
		uint32_t ms = millis();
		static uint32_t prev_led_ms = 0;
		static int avg_corr = 700;
//		static float prev_avg_emg_long = 0;
//		float emg_speed = avg_emg_long - 8000 - prev_avg_emg_long;
//		prev_avg_emg_long = avg_emg_long - 8000;
//		static float avg_emg_speed = 0;
//		avg_emg_speed *= 0.9;
//		avg_emg_speed += 0.1*emg_speed;
		if(ms - prev_led_ms > 10)
		{
			prev_led_ms = ms;
			static float high_sp = 0;
			high_sp *= 0.9;
			high_sp += 0.1 * (cur_spectr[2] + cur_spectr[3]);
//			int vr, vg, vb;
//			float cscale = 1.0;
//			int h = 270 - high_sp * cscale;
//			int v = high_sp * cscale;
//			if(v > 100) v = 100;
//			if(h < 0) h = 0;
//			hsv2rgb(h, 100, v, &vr, &vg, &vb);
			static float i_corr = 0;
			float d_corr = avg_emg_long - 8000;// - avg_emg_speed*0.1 + i_corr * 0.01;
			i_corr += d_corr;
			i_corr *= 0.999;
			float c_coeff = 0.01;
			float dcabs = d_corr;
			if(dcabs < 0) dcabs = -dcabs;
			if(dcabs > 2000) c_coeff = 0.02;
			if(dcabs > 4000) c_coeff = 0.05;
			if(dcabs > 6000) c_coeff = 0.1;
			c_coeff = 0.000005;
			int d_val = avg_corr + c_coeff*d_corr*dcabs;
//			static int prev_d_val = 0;
			if(d_val > 1022) d_val = 1022;
			if(d_val < 1) d_val = 1;
//			if(d_val - prev_d_val < 2 && d_val - prev_d_val > -2) d_val = prev_d_val;
//			prev_d_val = d_val;
			int ledv = high_sp;
			ledv *= 2;
			int b_chan = ledv >> 1;
			int g_chan = 0;
			if(ledv > 200) g_chan = ledv-200;
			if(b_chan > 255) b_chan = 255;
			if(g_chan > 255) g_chan = 255;
			leds_pulse(0, g_chan, b_chan, d_val, 252);
			d_val_out = d_val;
//			if(avg_emg_long < 3000)
//				leds_pulse(0, 0, 180, 12);
//			else if(avg_emg_long > 12000)
//				leds_pulse(0, 0, 250, 12);
//			else
//				leds_pulse(0, 0, 210, 12);
//			leds_pulse(vr, vg, vb, 12);
		}
//		leds_set(255*((ms%2000)>1000), 0, 0, 0);
		return 1;
	}
	return 0;
}


void fast_clock_start()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}
void slow_clock_start()
{
	NRF_CLOCK->LFCLKSRC = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
}
void fast_clock_stop()
{
	slow_clock_start();
//	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
//	NRF_CLOCK->TASKS_LFCLKSTART = 1;
//	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
	NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}


void mode_lowbatt()
{
//	set_led(0);
//	nrf_delay_ms(1);
//	mcp3912_turnoff();
	time_stop();
	fast_clock_stop();
	NRF_SPI0->ENABLE = 0;
//	leds_pulse(0, 0, 0, 0, 1);
	delay_ms(2);
	NRF_GPIO->OUTCLR = 1<<pin_syson;
//	nrf_delay_ms(1);
//	NRF_POWER->TASKS_LOWPWR = 1;
//	NRF_POWER->DCDCEN = 1;
//	NRF_POWER->SYSTEMOFF = 1;
}
void stop_rtc()
{
	NRF_RTC1->TASKS_STOP = 1;
}
void start_rtc()
{
	stop_rtc();
	NRF_RTC1->TASKS_CLEAR = 1;
	NRF_RTC1->CC[0] = 220;
	NRF_RTC1->CC[1] = 0xFFFF;
	NRF_RTC1->CC[2] = 0xFFFF;
	NRF_RTC1->CC[3] = 0xFFFF;
	NRF_RTC1->PRESCALER = 0x3CFFFF;
	NRF_RTC1->INTENSET = (1<<16); //CC0 event
	NVIC_EnableIRQ(RTC1_IRQn);
	NRF_RTC1->TASKS_START = 1;
}
void RTC1_IRQHandler(void)
{
	/* Update compare counter */
	if (NRF_RTC1->EVENTS_COMPARE[0] != 0)
	{
		NRF_RTC1->EVENTS_COMPARE[0] = 0;
		NRF_RTC1->TASKS_CLEAR = 1;  // Clear Counter		    
//		if(U32_delay_ms) U32_delay_ms--; // used in V_hw_delay_ms()
		__SEV(); // to avoid race condition
		stop_rtc();
	}
}

void mode_idle()
{
//	leds_pause();
	time_pause();
	NRF_POWER->TASKS_LOWPWR = 1;
}
void mode_resume_idle()
{
	NRF_POWER->TASKS_CONSTLAT = 1;
	time_resume();
//	leds_resume();
}

void low_power_cycle()
{
	NRF_RADIO->POWER = 0;
//	mode_idle();
	__WFI();
//	mode_resume_idle();
	NRF_RADIO->POWER = 1;
}


enum
{
	radio_mode_fast32 = 0,
	radio_mode_ble,
	radio_mode_fast64
};

uint8_t radio_mode = radio_mode_fast32;//radio_mode_ble;

void switch_to_ble()
{
	rf_disable();
	rf_override_irq(NULL);
	schedule_event_stop();
	NRF_RADIO->POWER = 0;
	delay_ms(2);
	ble_init_radio();
	for(int x = 0; x < 3; x++)
	{
		leds_pulse(0, 0, 255, 0, 150); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}
}

void switch_to_fr32()
{
	rf_disable();
	rf_override_irq(NULL);
	schedule_event_stop();
	NRF_RADIO->POWER = 0;
	rf_dettach_rx_irq();
	rf_dettach_tx_irq();
//	rf_init(83, 250, 0);
	for(int x = 0; x < 3; x++)
	{
		leds_pulse(255, 0, 255, 0, 150); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}

	rf_init_ext(83, 1000, 0, 0, 0, 0, 32);
	
//	star_init(21, 1000, 2000, 0);
}
void switch_to_fr64()
{
	rf_disable();
	rf_override_irq(NULL);
	schedule_event_stop();
	NRF_RADIO->POWER = 0;
	rf_dettach_rx_irq();
	rf_dettach_tx_irq();
	for(int x = 0; x < 3; x++)
	{
		leds_pulse(0, 255, 0, 0, 150); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}
	
	star_init(21, 1000, 2000, 0);
}

uint32_t prev_short_time = 0;

void process_radio_switch()
{
	if(radio_mode == radio_mode_fast64)
	{
		radio_mode = radio_mode_fast32;
		switch_to_fr32();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}
	if(radio_mode == radio_mode_fast32)
	{
		radio_mode = radio_mode_ble;
		switch_to_ble();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}
	if(radio_mode == radio_mode_ble)
	{
		radio_mode = radio_mode_fast64;
		switch_to_fr64();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}
//	emg_raw_data_send = !emg_raw_data_send;
}

void process_btn_long()
{
	NRF_GPIO->OUTCLR = 1<<pin_syson;
}

void process_btn_read()
{
	static uint8_t btn_pressed = 0;
	static uint8_t short_press_pending = 0;
	static uint8_t conseq_short_presses = 0;
	static uint32_t short_press_issued_time = 0;
	static uint32_t btn_on = 0;
	static uint32_t btn_off = 0;
	uint32_t ms = millis();
	if(NRF_GPIO->IN & 1<<pin_button)
	{
		if(!btn_pressed)
		{
			btn_pressed = 1;
			btn_on = ms;
		}
		if(ms - btn_on > 25 && ms - btn_on < 1000)
			leds_pulse(0, 0, 255, -1, 30);
		if(ms - btn_on > 1000 && ms - btn_on < 5000)
			leds_pulse(255, 0, 0, -1, 30);
		if(ms - btn_on > 5000)
			leds_pulse(255, 0, 255, -1, 30);
	}
	else
	{
		if(btn_pressed)
		{
			btn_pressed = 0;
			btn_off = ms;
			uint32_t btn_time = btn_off - btn_on;
			if(btn_time > 25) //ignore too short presses - noise
			{
				if(btn_time < 1000)
				{
					if(ms - short_press_issued_time < 400) conseq_short_presses++;
					else conseq_short_presses = 0;
					short_press_pending = 1;
					short_press_issued_time = ms;
				}
				else if(btn_time < 5000)
					process_btn_long();
			}
		}
	}

	if(short_press_pending && ms - short_press_issued_time > 500)
	{
		if(conseq_short_presses >= 10)
		{
			uint8_t gx, gy, gz;
			lsm_get_zero_offsets_packed(&gx, &gy, &gz);
			dev_state.fields.zero_wx_packed = gx;
			dev_state.fields.zero_wy_packed = gy;
			dev_state.fields.zero_wz_packed = gz;
			dev_state_changed = 1;
			dev_state_changed_time = millis();
			for(int x = 0; x < 20; x++)
			{
				leds_pulse(255, 255, 0, 0, 150); 
				NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
				delay_ms(300);
			}
		}
		else process_radio_switch();
		short_press_pending = 0;
	}
}

int main(void)
{
	paint_stack(); 
	NRF_GPIO->DIRSET = 1<<pin_syson;
	NRF_GPIO->OUTSET = 1<<pin_syson;
	NRF_WDT->CRV = 1*32768; //1 second timeout
	NRF_WDT->TASKS_START = 1;
	
	NRF_UICR->NFCPINS = 0;

	NRF_GPIO->PIN_CNF[pin_button] = 0;

	fast_clock_start();
	time_start();
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
	leds_init(8, 7, 6, 26);
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
	lsm_init(14, 13, 15, 16, 17);

	NRF_POWER->DCDCEN = 1;

	for(int x = 0; x < 3; x++)
	{
		leds_pulse((x==0)*255, (x==1)*255, (x==2)*255, 0, 250); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}
	
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
//	star_init(21, 1000, 2000, 0);

	dev_state = read_current_state();
	if(dev_state.fields.radio_mode == radio_mode_ble)
		radio_mode = radio_mode_ble;
	if(dev_state.fields.radio_mode == radio_mode_fast32)
		radio_mode = radio_mode_fast32;
	if(dev_state.fields.radio_mode == radio_mode_fast64)
		radio_mode = radio_mode_fast64;
	if(dev_state.fields.zero_wx_packed != 0xFF)
	{
		lsm_set_zero_offsets_packed(dev_state.fields.zero_wx_packed, dev_state.fields.zero_wy_packed, dev_state.fields.zero_wz_packed);
	}
		
	if(radio_mode == radio_mode_fast64)
		switch_to_fr64();
	if(radio_mode == radio_mode_fast32)
		switch_to_fr32();
	if(radio_mode == radio_mode_ble)
		switch_to_ble();

	adc_init();

	uint32_t last_sent_ms = 0;

	NRF_RNG->TASKS_START = 1;
	NRF_RNG->SHORTS = 0;
	NRF_RNG->CONFIG = 1;
		
	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	
	int cur_rf32_dt = 10;
	
	star_set_id(unit_id);
	
	while(1)
	{
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		if(radio_mode == radio_mode_fast64) star_loop_step();
//		low_power_cycle();
		uint32_t ms = millis();
		
		if(dev_state_changed)
		{
			if(ms - dev_state_changed_time > 10000)
			{
				update_current_state(dev_state);
				dev_state_changed = 0;
			}
		}
		
		process_btn_read();
		
		if(adc_has_data())
			lsm_read();
		
		if(push_adc_data())
		{
			adc_cnt++;
			if(unsent_cnt < 30) unsent_cnt++;
			battery_mv = adc_read_battery();
			static int low_bat_cnt = 0;
			if(battery_mv < battery_low_threshold)
			{
				low_bat_cnt++;
				if(low_bat_cnt > 100)
				{
					mode_lowbatt();
				}
			}
			else
				low_bat_cnt = 0;
		}		
		if(radio_mode == radio_mode_fast64)
		{
			if(unsent_cnt > 0)
			{
				prepare_data_packet();
				star_queue_send(data_packet, data_packet[1]);
//				rf_send_and_listen(data_packet, data_packet[1]);
				last_sent_ms = ms;
				unsent_cnt = 0;
			}
		}
		if(radio_mode == radio_mode_fast32)
		{
			if(unsent_cnt > 0 && ms - last_sent_ms > cur_rf32_dt)
			{
				prepare_data_packet32();
				rf_send(data_packet, data_packet[1]);
//				rf_send_and_listen(data_packet, data_packet[1]);
				last_sent_ms = ms;
				unsent_cnt = 0;
				if(NRF_RNG->EVENTS_VALRDY)
				{
					cur_rf32_dt = 6 + ((NRF_RNG->VALUE)%8);
					NRF_RNG->EVENTS_VALRDY = 0;
					NRF_RNG->TASKS_START;
				}
				
			}
		}
		if(radio_mode == radio_mode_ble)
		{
			if(unsent_cnt > 0)
			{
				if(prepare_and_send_BLE()) unsent_cnt = 0;
			}
		}
//		if((ms%1000) == 499) leds_pulse(0, 255*(!had_adc), 255*had_adc, 10), had_adc = 0;
	}
}

