#ifndef RC_H
#define RC_H

#include "main.h"

extern uint8_t rxBuff[54];

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t l;
    uint8_t r;
} Rc_Mouse;

typedef struct
{
    uint8_t w, s, a, d;
    uint8_t shift, ctrl;
    uint8_t q, e, r, f, g;
    uint8_t z, x, c, v, b;
} Rc_Key;

typedef struct
{
    int16_t ch[4];
    uint8_t s[2];
    Rc_Mouse mouse;
    Rc_Key   key;
} Rc_Data;

extern Rc_Data rc;

void rc_init(void);
void rc_processdata(uint8_t* rxBuff);
int16_t offset(int16_t rc_val);

#endif // RC_H
