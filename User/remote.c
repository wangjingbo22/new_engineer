#include "remote.h"
#include "usart.h"
#include "string.h"

uint8_t rxBuff[54];
uint8_t* pData = rxBuff;

Rc_Data rc;

void rc_init(void)
{
    for(int i = 0;i<4;++i)
    {
        rc.ch[i] = 0;
    }
    rc.s[0] = 0;
    rc.s[1] = 0;
    rc.mouse.x = 0; rc.mouse.y = 0; rc.mouse.z = 0;
    rc.mouse.l = 0; rc.mouse.r = 0;
    rc.key.w = rc.key.s = rc.key.a = rc.key.d = 0;
    rc.key.shift = rc.key.ctrl = 0;
    rc.key.q = rc.key.e = rc.key.r = rc.key.f = rc.key.g = 0;
    rc.key.z = rc.key.x = rc.key.c = rc.key.v = rc.key.b = 0;

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    if(huart3.RxState == HAL_UART_STATE_READY)
    {
        huart3.pRxBuffPtr = pData;
        huart3.RxXferSize = 54;
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_DMA_Start(huart3.hdmarx, (uint32_t)&huart3.Instance->DR, (uint32_t)rxBuff, 54);
        SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    }
}

void rc_processdata(uint8_t* rxBuff)
{
    rc.ch[0] = offset(((int16_t)rxBuff[0] | ((int16_t)rxBuff[1] << 8)) & 0x07FF);
    rc.ch[1] = offset((((int16_t)rxBuff[1] >> 3) | ((int16_t)rxBuff[2] << 5)) & 0x07FF);
    rc.ch[2] = offset((((int16_t)rxBuff[2] >> 6) | ((int16_t)rxBuff[3] << 2) | ((int16_t)rxBuff[4] << 10)) & 0x07FF);
    rc.ch[3] = offset((((int16_t)rxBuff[4] >> 1) | ((int16_t)rxBuff[5]<<7)) & 0x07FF); 

    rc.s[0] = (rxBuff[5] >> 6) & 0x03;
    rc.s[1] =  (rxBuff[5] >> 4) & 0x03;

    rc.mouse.x = (int16_t)((uint16_t)rxBuff[6]  | ((uint16_t)rxBuff[7]  << 8));
    rc.mouse.y = (int16_t)((uint16_t)rxBuff[8]  | ((uint16_t)rxBuff[9]  << 8));
    rc.mouse.z = (int16_t)((uint16_t)rxBuff[10] | ((uint16_t)rxBuff[11] << 8));
    rc.mouse.l = rxBuff[12];
    rc.mouse.r = rxBuff[13];

    uint16_t key_val = (uint16_t)rxBuff[14] | ((uint16_t)rxBuff[15] << 8);
    rc.key.w     = (key_val >> 0)  & 1;
    rc.key.s     = (key_val >> 1)  & 1;
    rc.key.a     = (key_val >> 2)  & 1;
    rc.key.d     = (key_val >> 3)  & 1;
    rc.key.shift = (key_val >> 4)  & 1;
    rc.key.ctrl  = (key_val >> 5)  & 1;
    rc.key.q     = (key_val >> 6)  & 1;
    rc.key.e     = (key_val >> 7)  & 1;
    rc.key.r     = (key_val >> 8)  & 1;
    rc.key.f     = (key_val >> 9)  & 1;
    rc.key.g     = (key_val >> 10) & 1;
    rc.key.z     = (key_val >> 11) & 1;
    rc.key.x     = (key_val >> 12) & 1;
    rc.key.c     = (key_val >> 13) & 1;
    rc.key.v     = (key_val >> 14) & 1;
    rc.key.b     = (key_val >> 15) & 1;
}

int16_t offset(int16_t rc_val)
{
    int16_t temp;
    temp = rc_val - 1024;
    return temp;
}
