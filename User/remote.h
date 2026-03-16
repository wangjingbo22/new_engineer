#ifndef RC_H
#define RC_H

#include "main.h"

extern uint8_t rxBuff[54];

typedef struct
{
    int16_t ch[4]; 
    uint8_t s[2];
} Rc_Data;

extern Rc_Data rc;

void rc_init(void);
void rc_processdata(uint8_t* rxBuff);
int16_t offset(int16_t rc_val);

#endif // RC_H
