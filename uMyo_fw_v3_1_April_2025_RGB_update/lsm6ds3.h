#include <stdint.h>

#define LSM6_FUNC_CFG_ACCESS 0x01
#define LSM6_SENSOR_SYNC_TIME_FRAME 0x04

#define LSM6_FIFO_CTRL1 0x06
#define LSM6_FIFO_CTRL2 0x07
#define LSM6_FIFO_CTRL3 0x08
#define LSM6_FIFO_CTRL4 0x09
#define LSM6_FIFO_CTRL5 0x0A

#define LSM6_ORIENT_CFG_G 0x0B
#define LSM6_INT1_CTRL 0x0D
#define LSM6_INT2_CTRL 0x0E
#define LSM6_WHO_AM_I 0x0F

#define LSM6_CTRL1_XL 0x10
#define LSM6_CTRL2_G 0x11
#define LSM6_CTRL3_C 0x12
#define LSM6_CTRL4_C 0x13
#define LSM6_CTRL5_C 0x14
#define LSM6_CTRL6_C 0x15
#define LSM6_CTRL7_G 0x16
#define LSM6_CTRL8_XL 0x17
#define LSM6_CTRL9_XL 0x18
#define LSM6_CTRL10_C 0x19

#define LSM6_MASTER_CONFIG 0x1A
#define LSM6_WAKE_UP_SRC 0x1B
#define LSM6_TAP_SRC 0x1C
#define LSM6_D6D_SRC 0x1D

#define LSM6_STATUS_REG 0x1E
#define LSM6_OUT_TEMP_L 0x20
#define LSM6_OUT_TEMP_H 0x21
#define LSM6_OUTX_L_G 0x22
#define LSM6_OUTX_H_G 0x23
#define LSM6_OUTY_L_G 0x24
#define LSM6_OUTY_H_G 0x25
#define LSM6_OUTZ_L_G 0x26
#define LSM6_OUTZ_H_G 0x27
#define LSM6_OUTX_L_XL 0x28
#define LSM6_OUTX_H_XL 0x29
#define LSM6_OUTY_L_XL 0x2A
#define LSM6_OUTY_H_XL 0x2B
#define LSM6_OUTZ_L_XL 0x2C
#define LSM6_OUTZ_H_XL 0x2D

#define LSM6_SENSORHUB1_REG 0x2E
#define LSM6_SENSORHUB2_REG 0x2F
#define LSM6_SENSORHUB3_REG 0x30
#define LSM6_SENSORHUB4_REG 0x31
#define LSM6_SENSORHUB5_REG 0x32
#define LSM6_SENSORHUB6_REG 0x33
#define LSM6_SENSORHUB7_REG 0x34
#define LSM6_SENSORHUB8_REG 0x35
#define LSM6_SENSORHUB9_REG 0x36
#define LSM6_SENSORHUB10_REG 0x37
#define LSM6_SENSORHUB11_REG 0x38
#define LSM6_SENSORHUB12_REG 0x39

#define LSM6_FIFO_STATUS1 0x3A
#define LSM6_FIFO_STATUS2 0x3B
#define LSM6_FIFO_STATUS3 0x3C
#define LSM6_FIFO_STATUS4 0x3D
#define LSM6_FIFO_DATA_OUT_L 0x3E
#define LSM6_FIFO_DATA_OUT_F 0x3F
#define LSM6_TIMESTAMP0_REG 0x40
#define LSM6_TIMESTAMP1_REG 0x41
#define LSM6_TIMESTAMP2_REG 0x42

#define LSM6_STEP_TIMESTAMP_L 0x49
#define LSM6_STEP_TIMESTAMP_H 0x4A
#define LSM6_STEP_COUNTER_L 0x4B
#define LSM6_STEP_COUNTER_H 0x4C

#define LSM6_SENSORHUB13_REG 0x4D
#define LSM6_SENSORHUB14_REG 0x4E
#define LSM6_SENSORHUB15_REG 0x4F
#define LSM6_SENSORHUB16_REG 0x50
#define LSM6_SENSORHUB17_REG 0x51
#define LSM6_SENSORHUB18_REG 0x52

#define LSM6_FUNC_SRC 0x53
#define LSM6_TAP_CFG 0x58



typedef struct sLSM6DS3
{
	uint32_t CS;
	float rawA2SI;
	float rawG2SI;
	
	float aX;
	float aY;
	float aZ;

	float wX;
	float wY;
	float wZ;
	
	float roll;
	float yaw;
	float pitch;
	
	int16_t raX;
	int16_t raY;
	int16_t raZ;
	
	int16_t rwX;
	int16_t rwY;
	int16_t rwZ;
	
	uint16_t step_cnt;
	
	uint32_t data_id;
	
	uint8_t raw_data[12];

	float G;
	
	float T;
	
	float qW;
	float qX;
	float qY;
	float qZ;
	
	uint32_t imu_read_time;
	uint32_t temp_read_time;
	uint32_t steps_read_time;
}sLSM6DS3;


typedef struct {
    union {
        uint8_t reg;
        struct {
            unsigned orient : 3;
            unsigned sign_z : 1;
            unsigned sign_y : 1;
            unsigned sign_x : 1;
			unsigned : 2;
        }f;
    };
} LSM6DS3_ORIENT_REG;


typedef struct {
    union {
        uint8_t reg;
        struct {
            unsigned acc_ready : 1;
            unsigned gyro_ready : 1;
            unsigned boot : 1;
            unsigned fifo_thr : 1;
            unsigned fifo_ovf : 1;
            unsigned fifo_full : 1;
            unsigned sign_motion : 1;
            unsigned step_det : 1;
        }f;
    };
} LSM6DS3_INT_REG;

typedef struct {
    union {
        uint8_t reg;
        struct {
            unsigned bw_filter : 2;
            unsigned scale : 2;
            unsigned odr : 4;
        }f;
    };
} LSM6DS3_ACCEL_CFG1_REG;

typedef struct {
    union {
        uint8_t reg;
        struct {
			unsigned : 1;
            unsigned fs_125 : 1;
            unsigned scale : 2;
            unsigned odr : 4;
        }f;
    };
} LSM6DS3_GYRO_CFG2_REG;

void lsm_init(uint8_t pin_COPI, uint8_t pin_CIPO, uint8_t pin_SCK, uint8_t pin_CS, uint8_t pin_INT);
uint8_t lsm_read();
sLSM6DS3 *lsm_get_object();
void lsm_get_angles(int16_t *yaw, int16_t *pitch, int16_t *roll);
void lsm_get_quat_packed(int16_t *qww, int16_t *qwx, int16_t *qwy, int16_t *qwz);
void lsm_get_acc(int16_t *raX, int16_t *raY, int16_t *raZ);
void lsm_get_W(int16_t *rwX, int16_t *rwY, int16_t *rwZ);
int16_t lsm_get_roll();
int16_t lsm_get_yaw();
int16_t lsm_get_pitch();
void lsm_get_zero_offsets_packed(uint8_t *ox, uint8_t *oy, uint8_t *oz);
void lsm_set_zero_offsets_packed(uint8_t ox, uint8_t oy, uint8_t oz);
uint8_t lsm_get_zero_test_result();