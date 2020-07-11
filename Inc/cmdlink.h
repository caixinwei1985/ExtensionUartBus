#ifndef _CMDLINK_H
#define _CMDLINK_H

#include "main.h"
//#include "stm32f0xx_hal.h"
#include "serialbufproc.h"
#define INBUFLENMAX (256l)
#define OUTBUFLENMAX (256l)
#define UART_CH_MAX     8

void hal_transfer_byte_IT(uint8_t ch);
void hal_receive_byte_IT(uint8_t ch);
uint8_t async_write(uint8_t ch,uint8_t* pbuf,uint8_t count);
uint8_t async_read(uint8_t ch,uint8_t* pbuf,uint8_t count);
uint8_t sync_read(uint8_t ch,uint8_t* pbuff,uint8_t count,uint16_t timeout);
uint8_t get_command(uint8_t ch,uint8_t* pbuf,e_framesrc);
void readtimer_update(void);
void init_cmdlink();
uint8_t get_XORchecksum(uint8_t* data, int16_t count);
uint32_t calc_msglength(uint32_t header);
#endif