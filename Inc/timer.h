#ifndef _TIMER_H
#define _TIMER_H
#include "stdint.h"
int8_t is_timeup(uint8_t id,int32_t interval);
void timer_counting();
void timer_init();
void timer_delay(int32_t ms);
#endif