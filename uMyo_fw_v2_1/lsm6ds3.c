#include "lsm6ds3.h"
#include "spim_functions.h"
#include "quat_math.h"
#include "nrf.h"
#include "urf_timer.h"


sLSM6DS3 lsm;
volatile uint8_t lsm_in_reading = 0;
volatile uint8_t quat_update_pending = 0;

void lsm_write_reg8(uint8_t reg, uint8_t value)
{
	spi_write_reg8(reg, value, lsm.CS);
}

uint8_t lsm_read_reg8(uint8_t reg)
{
	return spi_read_reg8(reg | (1<<7), lsm.CS);
}

uint16_t lsm_read_reg16(uint8_t reg)
{
	return spi_read_reg16(reg | (1<<7), lsm.CS);
}



sQ Qsg; //from sensor to ground frame

uint32_t prev_lsm_mcs = 0;

float zwx = 0, zwy = 0, zwz = 0;
float zwx_calc = 0, zwy_calc = 0, zwz_calc = 0;

int encode_gyro_to8bit(float val)
{
	int v = val;
	v /= 4;
	if(v > 63) v = 64 + (v-64) / 4;
	if(v < -63) v = -64 + (v+64) / 4;
	if(v > 127) v = 127;
	if(v < -127) v = -127;
	return 127 + v;
}
int decode_gyro_from8bit(uint8_t encv)
{
	int v = encv - 127;
	if(v >= 0)
	{
		if(v > 63) v = 64*4 + (v-64)*16;
		else v = v*4;
	}
	else
	{
		if(v < -63) v = -64*4 + (v+64)*16;
		else v = v*4;
	}
	return v;
}

void lsm_get_zero_offsets_packed(uint8_t *ox, uint8_t *oy, uint8_t *oz)
{
	*ox = encode_gyro_to8bit(zwx_calc);
	*oy = encode_gyro_to8bit(zwy_calc);
	*oz = encode_gyro_to8bit(zwz_calc);
}
void lsm_set_zero_offsets_packed(uint8_t ox, uint8_t oy, uint8_t oz)
{
	zwx = decode_gyro_from8bit(ox);
	zwy = decode_gyro_from8bit(oy);
	zwz = decode_gyro_from8bit(oz);
	zwx *= lsm.rawG2SI;
	zwy *= lsm.rawG2SI;
	zwz *= lsm.rawG2SI;
}

uint8_t q_inited = 0;

void calculate_quat()
{
	uint32_t mcs = micros();
	uint32_t dt = mcs - prev_lsm_mcs;
	if(dt > 50000) dt = 50000;
	if(dt < 1000) return;
	if(!q_inited)
	{
		Qsg.x = 0;
		Qsg.y = 0;
		Qsg.z = 0;
		Qsg.w = 1;
		lsm.roll = 0;
		lsm.yaw = 0;
		lsm.pitch = 0;
		q_inited = 1;
	}
	prev_lsm_mcs = mcs;
	float dT = 0.000001 * (float)dt;
	if(millis() < 10000)
	{
		if(zwx_calc == 0) zwx_calc = lsm.rwX;
		if(zwy_calc == 0) zwy_calc = lsm.rwY;
		if(zwz_calc == 0) zwz_calc = lsm.rwZ;
		zwx_calc *= 0.99;
		zwy_calc *= 0.99;
		zwz_calc *= 0.99;
		zwx_calc += 0.01*lsm.rwX;
		zwy_calc += 0.01*lsm.rwY;
		zwz_calc += 0.01*lsm.rwZ;
	}
	sQ W;
	W.x = lsm.wX - zwx;
	W.y = lsm.wY - zwy;
	W.z = lsm.wZ - zwz;
	lsm.roll += dT * W.y;
	lsm.roll *= 0.9999;

	sV A;
	sV vW;
	A.x = lsm.aX;
	A.y = lsm.aY;
	A.z = lsm.aZ;
	v_renorm(&A);
	vW.x = W.x;
	vW.y = W.y;
	vW.z = W.z;
	
	float aproj = v_dot(&A, &vW);
	lsm.yaw += dT*aproj;
	
	float L2 = dT*sqrt_f(W.x*W.x + W.y*W.y + W.z*W.z)*0.5f;
	float sL = sin_f(L2);
	float cL = cos_f(L2);
	W.w = 0;
	q_renorm(&W);
	W.x *= sL;
	W.y *= sL;
	W.z *= sL;
	W.w = cL;

	sQ Qsg1;
	q_mult(&Qsg, &W, &Qsg1);
	q_renorm(&Qsg1);
	q_set(&Qsg, &Qsg1);

	sV tgt_A;
	tgt_A.x = 0;
	tgt_A.y = 0;
	tgt_A.z = -1;
	sQ Qgs;
	q_make_conj(&Qsg, &Qgs);
	rotate_v(&Qgs, &tgt_A);
	sQ corr_A;
	float corr_coeff_A = 0.0001;
	if(millis() < 5000)
	{
		corr_coeff_A = 0.01;
	}
    sV a_corr;
    v_mult(&tgt_A, &A, &a_corr); 
    corr_A.w = 1;
    corr_A.x = -a_corr.x*corr_coeff_A;
    corr_A.y = -a_corr.y*corr_coeff_A;
    corr_A.z = -a_corr.z*corr_coeff_A;
	q_renorm(&corr_A);
	sQ tmp;
	q_mult(&Qsg, &corr_A, &tmp);
	q_set(&Qsg, &tmp);
	q_renorm(&Qsg);	
}

uint8_t lsm_spi_buf[32];
void lsm_read_cplt()
{
	int16_t v;
	uint8_t *pbuf = lsm_spi_buf+1;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.T = v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.rwX = v;
	lsm.wX = lsm.rawG2SI * (float)v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.rwY = v;
	lsm.wY = lsm.rawG2SI * (float)v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.rwZ = v;
	lsm.wZ = lsm.rawG2SI * (float)v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.raX = v;
	lsm.aX = lsm.rawA2SI * (float)v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.raY = v;
	lsm.aY = lsm.rawA2SI * (float)v;
	v = *pbuf++; v += (*pbuf++) << 8;
	lsm.raZ = v;
	lsm.aZ = lsm.rawA2SI * (float)v;

	lsm.data_id++;
	
	lsm_in_reading = 0; 
	quat_update_pending = 1;
}

uint8_t lsm_read()
{
	if(quat_update_pending)
	{
		calculate_quat();
		quat_update_pending = 0;
	}
	if(lsm_in_reading) return 0;
	uint8_t status = lsm_read_reg8(LSM6_STATUS_REG);
	if((status & 0b11) != 0b11) return 0;
	spi_read_buf(LSM6_OUT_TEMP_L | (1<<7), 14, lsm_read_cplt, lsm_spi_buf, lsm.CS);
	lsm_in_reading = 1;
	return 1;
}

void lsm_init(uint8_t pin_COPI, uint8_t pin_CIPO, uint8_t pin_SCK, uint8_t pin_CS, uint8_t pin_INT)
{
	NRF_GPIO->DIRSET = 1<<pin_CS;
	lsm.CS = 1<<pin_CS;
	NRF_GPIO->OUTSET = lsm.CS;
	lsm.data_id = 0;
	
	spi_init(pin_SCK, pin_COPI, pin_CIPO);

	lsm.rawA2SI = 9.8 * 2.0 / 32768.0;// 2g mode
	lsm.rawG2SI = 500.0 / 180.0 * 3.1415926 / 28572.0;//32768.0;// 500 dps mode

	lsm_write_reg8(LSM6_FUNC_CFG_ACCESS, 1);
	LSM6DS3_ACCEL_CFG1_REG cfg1;
	cfg1.reg = 0;
	cfg1.f.odr = 0b0110; //416 Hz 
	cfg1.f.scale = 0; //2g
	lsm_write_reg8(LSM6_CTRL1_XL, cfg1.reg);
	LSM6DS3_GYRO_CFG2_REG cfg2;
	cfg2.reg = 0;
	cfg2.f.fs_125 = 0;
	cfg2.f.scale = 1; //500 dps
	cfg2.f.odr = 0b0110; //416 Hz
	lsm_write_reg8(LSM6_CTRL2_G, cfg2.reg);
//	LSM6DS3_INT_REG int_cfg;
//	int_cfg.reg = 0;
//	int_cfg.f.gyro_ready = 1;
//	lsm_write_reg8(LSM6_INT1_CTRL, int_cfg.reg);
}

sLSM6DS3 *lsm_get_object()
{
	return &lsm; 
}

void lsm_get_quat_packed(int16_t *qww, int16_t *qwx, int16_t *qwy, int16_t *qwz)
{
	*qww = Qsg.w * 32000.0;
	*qwx = Qsg.x * 32000.0;
	*qwy = Qsg.y * 32000.0;
	*qwz = Qsg.z * 32000.0;
}

void lsm_get_angles(int16_t *yaw, int16_t *pitch, int16_t *roll)
{
	float fyaw = atan2_f(2.0f*(Qsg.y*Qsg.z + Qsg.w*Qsg.x), Qsg.w*Qsg.w - Qsg.x*Qsg.x - Qsg.y*Qsg.y + Qsg.z*Qsg.z);
	float fpitch = asin_f(-2.0f*(Qsg.x*Qsg.z - Qsg.w*Qsg.y));
	float froll = atan2_f(2.0f*(Qsg.x*Qsg.y + Qsg.w*Qsg.z), Qsg.w*Qsg.w + Qsg.x*Qsg.x - Qsg.y*Qsg.y - Qsg.z*Qsg.z);
	*yaw = fyaw * 5000;
	*pitch = fpitch * 5000;
	*roll = froll * 5000;
}

void lsm_get_acc(int16_t *raX, int16_t *raY, int16_t *raZ)
{
	*raX = lsm.raX;
	*raY = lsm.raY;
	*raZ = lsm.raZ;
}

void lsm_get_W(int16_t *rwX, int16_t *rwY, int16_t *rwZ)
{
	*rwX = lsm.rwX;
	*rwY = lsm.rwY;
	*rwZ = lsm.rwZ;
}

int16_t lsm_get_roll()
{
	return lsm.roll * 1000;
}
int16_t lsm_get_yaw()
{
	return lsm.yaw * 1000;
}
int16_t lsm_get_pitch()
{
	return lsm.pitch * 1000;
}