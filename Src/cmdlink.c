#include "cmdlink.h"
#include "timer.h"
#include "serialbufproc.h"
//#include "stm32f0xx_hal.h"


USART_TypeDef*  s_usarts[UART_CH_MAX] = {USART1,USART2,USART3,USART4, USART5, USART6,USART7,USART8};

uint8_t f_inbuf[UART_CH_MAX][INBUFLENMAX] = {0};
uint8_t f_outbuf[UART_CH_MAX][OUTBUFLENMAX] = {0};

uint8_t f_lastbyteflag[UART_CH_MAX] = {0};
// indicate the byte queue tranfer is going
uint8_t f_is_tranfering[UART_CH_MAX] = {0};
uint32_t recved_time;
//extern uint32_t systick;
//receive buffer writing index
volatile uint16_t f_inbuf_wr_idx[UART_CH_MAX] = {0}; 

//receive buffer reading index
volatile uint16_t f_inbuf_rd_idx[UART_CH_MAX] = {0};

//count of byte to read
volatile uint16_t f_inbuf_cnt[UART_CH_MAX] = {0};

volatile uint16_t f_outbuf_wr_idx[UART_CH_MAX] = {0};
volatile uint16_t f_outbuf_rd_idx[UART_CH_MAX] = {0};

//count of byte to transfer 
volatile uint16_t f_outbuf_cnt[UART_CH_MAX] = {0};

// read timeout timer counter
volatile int16_t f_inbuf_rd_to[UART_CH_MAX];

void readtimer_update()
{
    uint8_t ch = 0;
    for(ch = 0; ch < UART_CH_MAX; ch++)
    {
        if(f_inbuf_rd_to[ch] > 0)
            f_inbuf_rd_to[ch]--;
    }
}
void init_cmdlink()
{
    uint8_t ch = 0;
    for(ch = 0; ch < UART_CH_MAX; ch++)
    {
        f_inbuf_wr_idx[ch]  = 0;
        f_inbuf_rd_idx[ch]  = 0;
        f_outbuf_wr_idx[ch] = 0;
        f_outbuf_rd_idx[ch] = 0;
        f_inbuf_cnt[ch]     = 0;
        f_outbuf_cnt[ch]    = 0;
    }
}

void flush_cmdlink()
{
    uint8_t ch = 0;
    for(ch = 0; ch < UART_CH_MAX; ch++)
    {
        f_inbuf_wr_idx[ch]  = 0;
        f_inbuf_rd_idx[ch]  = 0;
        f_inbuf_cnt[ch]     = 0;
    }    
}

uint8_t sync_read(uint8_t ch,uint8_t* pbuff,uint8_t count,uint16_t timeout)
{
    uint8_t i;
    
    for( i = 0;i < count; i++)
    {
        f_inbuf_rd_to[ch] = timeout;
        while(f_inbuf_cnt[ch] == 0 && f_inbuf_rd_to[ch] > 0);
        
        if(f_inbuf_rd_to[ch] == 0)
        {
            return 0;
        }
        
        pbuff[i] = f_inbuf[ch][f_inbuf_rd_idx[ch]];
				f_inbuf[ch][f_inbuf_rd_idx[ch]] = 0xcc;
        f_inbuf_rd_idx[ch]++;
        if(INBUFLENMAX == f_inbuf_rd_idx[ch])
        {
          f_inbuf_rd_idx[ch] = 0;
        }
        s_usarts[ch]->CR1 &= ~USART_CR1_RXNEIE;
        f_inbuf_cnt[ch]--;
        s_usarts[ch]->CR1 |= USART_CR1_RXNEIE;
         
    }
    return i;
}
uint8_t async_read(uint8_t ch,uint8_t* pbuff,uint8_t count)
{
  uint8_t i;
  if(count > f_inbuf_cnt[ch] || count == 0)
    i = 0;
  else
  {
    for(i = 0;i < count;i++)
    {
      pbuff[i] = f_inbuf[ch][f_inbuf_rd_idx[ch]];
			f_inbuf[ch][f_inbuf_rd_idx[ch]] = 0XCC;
      f_inbuf_rd_idx[ch]++;
      if(INBUFLENMAX == f_inbuf_rd_idx[ch])
      {
        f_inbuf_rd_idx[ch] = 0;
      }
      s_usarts[ch]->CR1 &= ~USART_CR1_RXNEIE;
      f_inbuf_cnt[ch]--;
      s_usarts[ch]->CR1 |= USART_CR1_RXNEIE;
    }
  }
  return i;
}



/*********************************************************
*        name:async_write
* description:
*  parameters:
*return value:
**********************************************************/
uint8_t async_write(uint8_t ch,uint8_t* pbuff,uint8_t count)
{
  uint8_t i;
//  __disable_irq();
  
  if(count > OUTBUFLENMAX - f_outbuf_cnt[ch])
  {
    i = 0;
  }
  else
  {
    for(i = 0;i < count;i++)
    {
      f_outbuf[ch][f_outbuf_wr_idx[ch]] = pbuff[i];
        f_outbuf_wr_idx[ch]++;
 
      if(OUTBUFLENMAX == f_outbuf_wr_idx[ch])
      {
        f_outbuf_wr_idx[ch] = 0;
      }
      s_usarts[ch]->CR1 &= ~USART_CR1_TCIE;
      f_outbuf_cnt[ch]++;
      s_usarts[ch]->CR1 |= USART_CR1_TCIE;
    }
    
    // if the transfer is not going on, trigger the byte queue transfer
    s_usarts[ch]->CR1 &= ~USART_CR1_TCIE;
    if(f_is_tranfering[ch] == 0)
    {
      //只有当传输状态为结束时，才能在主线程中调用 字节发送 方法
        hal_transfer_byte_IT(ch);    
    }
    s_usarts[ch]->CR1 |= USART_CR1_TCIE;
  }
  
  return i;  
}

void hal_transfer_byte_IT(uint8_t ch)
{
  uint8_t dr;
  if(f_lastbyteflag[ch] == 1)
  {
    // 当缓冲中的最后一个字节发送完毕后，方能设置传输状态为结束
    f_is_tranfering[ch] = 0;
  }
  f_lastbyteflag[ch] = 0;
  if(f_outbuf_cnt[ch] > 0)
  {
    f_is_tranfering[ch] = 1;
    dr = f_outbuf[ch][f_outbuf_rd_idx[ch]++];
    if(OUTBUFLENMAX == f_outbuf_rd_idx[ch])
    {
      f_outbuf_rd_idx[ch] = 0;
    }
    s_usarts[ch]->TDR = dr;
    f_outbuf_cnt[ch]--;
    if(0 == f_outbuf_cnt[ch])
    {
      f_lastbyteflag[ch] = 1;
    }
  }
}

void hal_receive_byte_IT(uint8_t ch)
{
    
	f_inbuf[ch][f_inbuf_wr_idx[ch]] = s_usarts[ch]->RDR;
	f_inbuf_wr_idx[ch]++;
	if(INBUFLENMAX == f_inbuf_wr_idx[ch])
	{
			f_inbuf_wr_idx[ch] = 0;
	}
	f_inbuf_cnt[ch]++;     
}

uint32_t calc_msglength(uint32_t header)
{
  uint32_t len;
  if(header < 0x20)       
  {
    len = 1 ;
  }
  else if(header < 0x80)
  {
    len = 2 + ((header - 0x20) >> 4);
  }
  else if(header < 0xE0)
  {
    len = 8 + ((header - 0x80) >> 3);
  }
  else
  {
    len = 20 + ((header -0xE0) >> 2);
  } 
  return len;
}
uint8_t get_XORchecksum(uint8_t* data, int16_t count)
{
	int i;
	uint8_t checksum = 0;
	for(i = 0;i< count;i++)
	{
		checksum ^= data[i];
	}
	return checksum;
}
uint8_t get_command(uint8_t ch,uint8_t* pbuf,e_framesrc comefrom)
{
  uint8_t len,sumchk;
  uint8_t dataheader0,dataheader1;
  uint32_t i = 0;
  s_sbp_frame* pf = (s_sbp_frame*)pbuf;
  if(FROM_BUS == comefrom)
  {
    dataheader0 = FRAME_HEADER0;
    dataheader1 = FRAME_HEADER1;
  }
  else if(FROM_DEV == comefrom)
  {
    dataheader0 = 0x5a;
    dataheader1 = 0xa5;
  }
  while(1)
  {
    /*  B0  B1  B2  B3  B4  B5     
     *  00  00  00  00  00  00
     *  ^
    **/
    if(0==async_read(ch,pbuf,1))
    {
        break;
    }
    if(0xff == pf->header0)
    {
      pbuf++;
      /*  B0  B1  B2  B3  B4  B5     
       *  FF  00  00  00  00  00
       *      ^
      **/
      if(0 == sync_read(ch,pbuf,3,20))
      {
          break;
      }
      if(pf->header1 != 0xff)
      {
          break;
      }
      pbuf += 3;
      /*  B0  B1  B2  B3  B4  B5     
       *  FF  FF  00  LEN 00  00
       *                  ^
      **/
      len = pf->length+pf->length_h;
			if(sync_read(ch,pbuf,len-4,20))
					return len;
			else
			{
					break;
			}
    }
    else if(dataheader0 == pf->header0)
    {
      pbuf++;
      /*  B0  B1  B2  B3  B4  B5     
       *  AA  00  00  00  00  00
       *      ^
      **/
      if(0 == sync_read(ch,pbuf,3,5))
      {
        break;
      }
      if(pf->header1 != dataheader1)
      {
        break;
      }
      pbuf += 3;
      /*  B0  B1  B2  B3  B4  B5     
       *  AA  55  00  LEN 00  00
       *                  ^
      **/
      len = pf->length+pf->length_h;
      if(sync_read(ch,pbuf,len-4,5))
          return len;
      else
      {
          break;
      }
    }
    else
    {
        continue;
    }
  }
//  f_inbuf_cnt[ch] = 0;
//  f_inbuf_rd_idx[ch] = 0;
//  f_inbuf_wr_idx[ch] = 0;
  return 0;
}



