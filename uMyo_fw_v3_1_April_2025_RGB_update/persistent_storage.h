#include <stdint.h>

typedef struct
{
    union {
        struct { //LSB first
            unsigned radio_mode : 3;
            unsigned irrelevant1 : 5;
            unsigned zero_wx_packed : 8;
            unsigned zero_wy_packed : 8;
            unsigned zero_wz_packed : 8;
            unsigned magn_minx : 16;
            unsigned magn_maxx : 16;
            unsigned magn_miny : 16;
            unsigned magn_maxy : 16;
            unsigned magn_minz : 16;
            unsigned magn_maxz : 16;
        } fields;
        uint32_t values[4];
    };
}sDevice_state;

sDevice_state read_current_state();
void update_current_state(sDevice_state state);
