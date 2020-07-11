
#include "stdint.h"
#define TIMER_NUM_MAX     5

static int32_t f_timers[TIMER_NUM_MAX] = {-1};
static volatile int32_t f_delay;
void timer_counting()
{
  int8_t i;
  for(i = 0;i < TIMER_NUM_MAX; i++)
  {
    if(f_timers[i] >0)
    {
      f_timers[i]--;
    }
  }
  if(f_delay > 0)
  {
      f_delay--;
  }
}

int8_t is_timeup(uint8_t id,int32_t interval)
{
  if(interval < 0)
    return -1;
  if(-1 == f_timers[id])
  {
    f_timers[id] = interval;
  }
  if(f_timers[id] > 0)
  {
    return 0;
  }
  if(0 == f_timers[id])
  {
    f_timers[id] = -1;
    return 1;
  }
  return 0;
}

void timer_delay(int32_t ms)
{
    f_delay = 1;
    while(f_delay);
    f_delay = ms;
    while(f_delay);
}
void timer_init()
{
    int i;
    for(i = 0; i < TIMER_NUM_MAX; i++)
    {
        f_timers[i] = -1;
    }
}