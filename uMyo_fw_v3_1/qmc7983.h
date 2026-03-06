#include <nrf.h>
#include <stdint.h>

// Legacy/old map (works for QMC5883L and the old "qmc7983" path)
#define QMC7983_DATAX_A 0x00
#define QMC7983_STATUS_A 0x06
#define QMC7983_TEMP_A 0x07
#define QMC7983_CFG_A 0x09
#define QMC7983_RESET_A 0x0A
#define QMC7983_SETRES_A 0x0B
#define QMC7983_OTP_RDY_A 0x0C
#define QMC7983_CHIPID_A 0x0D

// QMC5883P register map (different from L)
#define QMC5883P_CHIPID_A 0x00
#define QMC5883P_DATAX_A 0x01
#define QMC5883P_STATUS_A 0x09
#define QMC5883P_CTRL1_A 0x0A
#define QMC5883P_CTRL2_A 0x0B
#define QMC5883P_SIGN_A 0x29 // datasheet example writes 0x06 here (axis sign)

typedef struct
{
    union
    {
        uint8_t reg;
        struct
        {
            unsigned MODE : 2;
            unsigned ODR : 2;
            unsigned RNG : 2;
            unsigned OSR : 2;
        };
    };
} QMC7983_CFG;

typedef struct
{
    union
    {
        uint8_t reg;
        struct
        {
            unsigned DRDY : 1;
            unsigned OVL : 1;
            unsigned DOR : 1;
            unsigned : 5;
        };
    };
} QMC7983_STATUS;

void qmc_init();
void qmc_read();
void qmc_get_mag(int16_t *mx, int16_t *my, int16_t *mz);
int qmc_process_calibration();
void qmc_end_calibration();
void qmc_get_calibration_data(int16_t *min_x, int16_t *max_x,
                              int16_t *min_y, int16_t *max_y,
                              int16_t *min_z, int16_t *max_z);
void qmc_set_calibration_state(int16_t min_x, int16_t max_x,
                               int16_t min_y, int16_t max_y,
                               int16_t min_z, int16_t max_z);
