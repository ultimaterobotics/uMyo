#include <stdint.h>

enum
{
	adc_gain_1_6 = 0,
	adc_gain_1_5,
	adc_gain_1_4,
	adc_gain_1_3,
	adc_gain_1_2,
	adc_gain_1,
	adc_gain_2,
	adc_gain_4
};

enum
{
	adc_taq_3us = 0,
	adc_taq_5us,
	adc_taq_10us,
	adc_taq_15us,
	adc_taq_20us,
	adc_taq_40us
};

enum
{
	adc_ain_nc = 0,
	adc_ain0,
	adc_ain1,
	adc_ain2,
	adc_ain3,
	adc_ain4,
	adc_ain5,
	adc_ain6,
	adc_ain7,
	adc_ain_vdd
};

enum 
{
	adc_res_8 = 0,
	adc_res_10,
	adc_res_12,
	adc_res_14
};

enum 
{
	adc_oversample_off = 0,
	adc_oversample_2x,
	adc_oversample_4x,
	adc_oversample_8x,
	adc_oversample_16x,
	adc_oversample_32x,
	adc_oversample_64x,
	adc_oversample_128x,
	adc_oversample_256x
};

typedef struct {
    union {
        struct { //LSB first
            unsigned RESP : 2;
            unsigned : 2;
            unsigned RESN : 2;
            unsigned : 2;
            unsigned GAIN : 3;
            unsigned : 1;
            unsigned REF_VDD4 : 1;
            unsigned : 3;
            unsigned TACQ : 3;
            unsigned : 1;
            unsigned MODE_DIFF : 1;
            unsigned : 3;
            unsigned BURST : 1;
            unsigned : 7;
        } fields;
        uint32_t value;
    };
} ADC_PIN_CONFIG_REG;

typedef struct {
    union {
        struct { //LSB first
            unsigned STARTED : 1;
            unsigned END : 1;
            unsigned DONE : 1;
            unsigned RESULTDONE : 1;
            unsigned CALIBRATEDONE : 1;
            unsigned STOPPED : 1;
            unsigned CH0LIMITH : 1;
            unsigned CH0LIMITL : 1;	
            unsigned CH1LIMITH : 1;
            unsigned CH1LIMITL : 1;	
            unsigned CH2LIMITH : 1;
            unsigned CH2LIMITL : 1;	
            unsigned CH3LIMITH : 1;
            unsigned CH3LIMITL : 1;	
            unsigned CH4LIMITH : 1;
            unsigned CH4LIMITL : 1;	
            unsigned CH5LIMITH : 1;
            unsigned CH5LIMITL : 1;	
            unsigned CH6LIMITH : 1;
            unsigned CH6LIMITL : 1;	
            unsigned CH7LIMITH : 1;
            unsigned CH7LIMITL : 1;	
            unsigned : 10;
        } fields;
        uint32_t value;
    };
} ADC_INT_REG;

void adc_init();
int adc_get_data(int16_t *raw_data, float *sp_data);
uint32_t adc_read_battery();
uint8_t adc_has_data();