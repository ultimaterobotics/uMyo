
void leds_init(int pin_r, int pin_g, int pin_b, int pin_corr);
void leds_pulse(int r, int g, int b, int corr, int length);
void hsv2rgb(int h, float s, float v, int *r, int *g, int *b);
