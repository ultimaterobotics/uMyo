#include "leds.h"
#include "nrf.h"
#include <stdint.h>

uint8_t led_pins[4];
uint32_t led_pin_mask[5];
uint32_t led_set_mask = 0;

volatile int led_pulse_length = 0;

int led_pwm_vals[4] = {0, 512, 1023, 0};
uint16_t pwm_seq[8];

void start_leds_pwm(int ms_length)
{
	if(NRF_PWM0->ENABLE)
	{
		NRF_PWM0->ENABLE = 0;
	}
//	NRF_PWM0->PSEL.OUT[3] = 0xFFFFFFFF;
	for(int x = 0; x < 4; x++)
	{
		NRF_PWM0->PSEL.OUT[x] = led_pins[x];
		pwm_seq[x] = led_pwm_vals[x];
		pwm_seq[4+x] = 1023;
	}
//	pwm_seq[3] = 0;
//	pwm_seq[7] = 0;
	NRF_PWM0->ENABLE = 1;
	NRF_PWM0->MODE = 0;
	NRF_PWM0->COUNTERTOP = 1024;
	NRF_PWM0->PRESCALER = 0;
	NRF_PWM0->DECODER = 2;
	NRF_PWM0->LOOP = 0;
	NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_seq;
	NRF_PWM0->SEQ[0].CNT = 8;
	NRF_PWM0->SEQ[0].REFRESH = 16*ms_length;
//	NRF_PWM0->INTEN = 1<<4; //SEQ0 END
	NRF_PWM0->INTEN = 1<<1; //STOPPED
//	if(ms_length == 0) NRF_PWM0->INTEN = 0;
	NRF_PWM0->TASKS_SEQSTART[0] = 1;
	NRF_PWM0->SHORTS = 1; //SEQ0END -> STOP
//	if(ms_length == 0) NRF_PWM0->SHORTS = 0;
}

void PWM0_IRQHandler()
{
	if(NRF_PWM0->EVENTS_STOPPED)
	{
		NRF_PWM0->ENABLE = 0;
		NRF_PWM0->EVENTS_STOPPED = 0;
//		NRF_GPIO->OUTSET = led_pin_mask[3];
	}
}


void leds_init(int pin_r, int pin_g, int pin_b, int pin_corr)
{
	led_pins[0] = pin_r;
	led_pins[1] = pin_g;
	led_pins[2] = pin_b;
	led_pins[3] = pin_corr;
	for(int x = 0; x < 4; x++)
	{
		led_pin_mask[x] = 1<<led_pins[x];
		NRF_GPIO->DIRSET = led_pin_mask[x];
	}
	led_pin_mask[4] = led_pin_mask[0] | led_pin_mask[1] | led_pin_mask[2];
	NRF_GPIO->OUTSET = led_pin_mask[4];
}

int val_to_cc(int val)
{
	int v2 = val*val;
//	v2 >>= 2;
	v2 >>= 6;
//	if(v2 == 0) v2 = 1;
	if(v2 > 1023) v2 = 1023; 
//	if(v2 > 16384) v2 = 16384; 
	return v2;
}

void leds_pulse(int r, int g, int b, int corr, int length)
{
	if(r >= 0) led_pwm_vals[0] = val_to_cc(r);
	if(g >= 0) led_pwm_vals[1] = val_to_cc(g);
	if(b >= 0) led_pwm_vals[2] = val_to_cc(b);
	if(corr >= 0) led_pwm_vals[3] = corr;
	start_leds_pwm(length);
}

void hsv2rgb(int h, float s, float v, int *r, int *g, int *b)
{
	v = v*0.01;
	s = s*0.01;
	if ( s == 0 ) //H 0...360, S, V from 0 to 1
	{
		*r = v * 255.0;
		*g = v * 255.0;
		*b = v * 255.0;
	}
	else
	{
//		float var_h = h * 6.0;
		float var_h = (float)h / 60.0;
		if ( var_h >= 6.0 ) var_h = 5.99;      //H must be < 1
		int var_i = var_h;             //Or ... var_i = floor( var_h )
		float var_1 = v * ( 1.0 - s );
		float var_2 = v * ( 1.0 - s * ( var_h - var_i ) );
		float var_3 = v * ( 1.0 - s * ( 1.0 - ( var_h - var_i ) ) );

		float var_r, var_g, var_b;
		
		if      ( var_i == 0 ) { var_r = v     ; var_g = var_3 ; var_b = var_1; }
		else if ( var_i == 1 ) { var_r = var_2 ; var_g = v     ; var_b = var_1; }
		else if ( var_i == 2 ) { var_r = var_1 ; var_g = v     ; var_b = var_3; }
		else if ( var_i == 3 ) { var_r = var_1 ; var_g = var_2 ; var_b = v;     }
		else if ( var_i == 4 ) { var_r = var_3 ; var_g = var_1 ; var_b = v;     }
		else                   { var_r = v     ; var_g = var_1 ; var_b = var_2; }

		*r = var_r * 255.0;                  //RGB results from 0 to 255
		*g = var_g * 255.0;
		*b = var_b * 255.0;
	}
}