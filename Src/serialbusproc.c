#include "serialbufproc.h"
#include "main.h"
#include "isp.h"
#include "cmdlink.h"


uint32_t	moduleAddr;
uint8_t		is_TXBoard = 0;
uint8_t  	cmdfmt_map = 0;
uint8_t		termiantchar = 0x0d;
uint8_t		is_Last = 0;
uint8_t		is_First = 0;

volatile uint8_t TT_Mode = 0;  // transparent mode 0:Not,1:device0,2:device1,3:device2,4,device3,5:device4 
uint8_t   is_DLink = 0; // directly link
uint16_t  TT_timer = 0;
extern USART_TypeDef*  s_usarts[UART_CH_MAX];
e_channelTypedef device_chs[MAX_DEVICE_CHS] = {CH_DEVICE0,CH_DEVICE1,CH_DEVICE2,CH_DEVICE3,CH_DEVICE4};

void TT_timing_1ms()
{
	if(TT_timer > 0)
	{
		TT_timer--;
    if(TT_timer == 0)
    {
      TT_Mode = 0;
      LL_GPIO_SetOutputPin(SO_GPIO_Port,SO_Pin);
      LL_GPIO_SetOutputPin(LED1_GPIO_Port,LED1_Pin);
      LL_GPIO_ResetOutputPin(LED2_GPIO_Port,LED2_Pin);
    }
	}
}

__inline void Enable_DirectLink()
{
	// This code is relative to hardware designe.
	LL_GPIO_ResetOutputPin(DL_SEL_GPIO_Port,DL_SEL_Pin);
	LL_GPIO_ResetOutputPin(SO_GPIO_Port,SO_Pin);
	LL_GPIO_ResetOutputPin(LED1_GPIO_Port,LED1_Pin);
	LL_GPIO_SetOutputPin(LED2_GPIO_Port,LED2_Pin);
	is_DLink = 1;
}

__inline void Disable_DirectLink()
{
	LL_GPIO_SetOutputPin(DL_SEL_GPIO_Port,DL_SEL_Pin);
	LL_GPIO_SetOutputPin(SO_GPIO_Port,SO_Pin);
	LL_GPIO_SetOutputPin(LED1_GPIO_Port,LED1_Pin);
	LL_GPIO_ResetOutputPin(LED2_GPIO_Port,LED2_Pin);
	is_DLink = 0;
}
uint8_t Get_TTMode()
{
	return TT_Mode;
}

void Fresh_TT_timer()
{
	TT_timer = 5000;
}

e_channelTypedef* Get_DeviceCHs()
{
	return device_chs;
}

void Init_InterfacNode(void)
{
  uint8_t count =0,i;
  Disable_DirectLink();
  moduleAddr =  (LL_GPIO_ReadInputPort(A0_GPIO_Port) >> 4 & 0x0f)*5;

  // MO pin toggle 3 loops, if MI is identical signal(reversal to MO), the device is last one.
  for(i = 0;i < 3;i++)
  {
    LL_GPIO_SetOutputPin(MO_GPIO_Port,MO_Pin);
    HAL_Delay(1);
    if((LL_GPIO_ReadInputPort(MI_GPIO_Port) & MI_Pin) == RESET)
    {
      count++;
    }
    LL_GPIO_ResetOutputPin(MO_GPIO_Port,MO_Pin);
    HAL_Delay(1);
    if((LL_GPIO_ReadInputPort(MI_GPIO_Port) & MI_Pin) != RESET)
    {
      count++;
    }
  }
  if(count > 4)
  {
    is_Last = SET;
    LL_GPIO_SetOutputPin(MO_GPIO_Port,MO_Pin);
  }
  else
  {
    is_Last = RESET;
  }

}

/** @brife re-config uart
		@param device channel
		@param baudrate
		@param parity,0:No parity,1:Even,2:Odd
		@param stopbits,0:1stopbits,1:0.5stopbits,2:2stopbits,3:1.5stopbits
*/
static inline void config_Deviceport(uint8_t subaddr,uint32_t br,uint8_t parity,uint8_t stopbit)
{
	USART_TypeDef *usart;
	uint32_t div;
	
	usart = s_usarts[device_chs[subaddr]];
	div = 48000000/br;
  usart->CR1 &= ~(USART_CR1_TCIE | USART_CR1_RXNEIE);
	usart->CR1 &= ~USART_CR1_UE;			
	usart->BRR = div;
	if(0 == parity)
	{
		usart->CR1 &= ~USART_CR1_PCE;
	}
	else
	{
		usart->CR1 |= USART_CR1_PCE | USART_CR1_PS;
		if(1 == parity)
		{
			usart->CR1 &= ~USART_CR1_PS;
		}
	}
	
	usart->CR2 &= ~USART_CR2_STOP;
	usart->CR2 |= (stopbit << USART_CR2_STOP_Pos)&USART_CR2_STOP_Msk;	
	usart->CR1 |= USART_CR1_UE;	
  usart->CR1 |= (USART_CR1_TCIE | USART_CR1_RXNEIE);
}

void Proc_HostCommand(uint8_t* pbuf)
{
	uint8_t ack[10];
	s_sbp_frame* sbpf;
  int i;
  sbpf = (s_sbp_frame*)ack;
	sbpf->header0 = 0xff;
	sbpf->header1 = 0xff;
  sbpf->length_h = 0x00;
	sbpf->length = 2 + PACKET_HEADER_LENGTH + 1;
	sbpf->addr = 0xff;
	sbpf->packet[0] = 0x08;
	sbpf->packet[1] = moduleAddr;
	Send_UpperCommand(ack);
  
	sbpf = (s_sbp_frame*)pbuf;
	
  HAL_Delay(10);
	
	// Command 0xFF, start ISP
	if(sbpf->packet[0] == 0xff)
	{
		// to next level device
//    Send_LowerCommand(pbuf);
		HAL_Delay(10);
		// wait for IO signal indication
		BootConfig_System();
	}
  else if(sbpf->packet[0] == 0x00)
  {
    NVIC_SystemReset();
  }
  // Command 0x01, traversal all device nodes
  else if(sbpf->packet[0] == 0x01)
  {
    if(is_Last)
    {
      // Last node respone the command first
      // system channel
      sbpf->addr = 0xff;
      // command length
      sbpf->length = calc_msglength(0xC8)+6;
      // command header
      sbpf->packet[0] = 0xC8;
      // traversed devices count
      sbpf->packet[1] = 1;
      // 
      sbpf->packet[2] = moduleAddr;
      // calculate checksum
      pbuf[sbpf->length-1] = get_XORchecksum(pbuf,sbpf->length - 1);
      Send_UpperCommand(pbuf);
    }
    else
    {
      Send_LowerCommand(pbuf);
    }
  }
  // Command 0x03, Upper and Lower channel directly link
  else if(sbpf->packet[0] == 0x03)
  {
    if(is_Last)
      return ;
    HAL_Delay(10);
    // switch to directly link position
    Enable_DirectLink();	
    // to detect MI pin until it turns low
    while(1)
    {
      if((LL_GPIO_ReadInputPort(MI_GPIO_Port) & MI_Pin) == 0)
      {
        // check if the signal is steable 
        HAL_Delay(200);
        if((LL_GPIO_ReadInputPort(MI_GPIO_Port) & MI_Pin) == 0)
        {
          Disable_DirectLink();
          break;
        }
      }
    }
  }	
  else if(sbpf->packet[0] == 0x07)
  {
    // 
    TT_Mode = 0;// (sbpf->addr - moduleAddr) + 1;		
    Fresh_TT_timer();		
    // release the SO signal to indicate in special state
    LL_GPIO_ResetOutputPin(SO_GPIO_Port,SO_Pin);
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port,LED1_Pin);
    LL_GPIO_SetOutputPin(LED2_GPIO_Port,LED2_Pin);
    
  }
  else if(sbpf->packet[0] == 0x09)
  {
    termiantchar = sbpf->packet[1];
  }
  // Command 0x0A, set transit form, pbuf[5] bitmap: 0->Format stream, 1->String stream
  else if(sbpf->packet[0] == 0x0A)
  {
    cmdfmt_map = (sbpf->packet[1]&0x01) << (sbpf->addr - moduleAddr);
  }
  // Command 0x51 set device port config
  else if(sbpf->packet[0] == 0x51)
  {
    uint32_t br_set;
    uint32_t div;
    uint8_t parity,stopbit;
    uint8_t ch;
    USART_TypeDef* usart;
    ch = sbpf->addr - moduleAddr;
    br_set = (sbpf->packet[1] << 16) + (sbpf->packet[2] << 8) + sbpf->packet[3];
    parity = sbpf->packet[4];
    stopbit = sbpf->packet[5];
    config_Deviceport(ch,br_set,parity,stopbit);
  }
  else
  {
    if(!is_Last)
      Send_LowerCommand(pbuf);
  }
}

void Proc_LowerCommand(uint8_t *pbuf)
{
  s_sbp_frame* sbpf = (s_sbp_frame*)pbuf;
  // packet[0]:header	packet[1]:msg[0],number of modules,msg[1]...msg[16], module address, the last one in first byte
  if(sbpf->packet[0] == 0xC8)
  {
    sbpf->packet[2+ sbpf->packet[1]] = moduleAddr;
    sbpf->packet[1]++;
    pbuf[sbpf->length-1] = get_XORchecksum(pbuf,sbpf->length - 1);
  }
}

uint8_t Get_UpperCommand(uint8_t* pbuf)
{
  s_sbp_frame* pframe = (s_sbp_frame*)pbuf;
  if(get_command(CH_UPPER,pbuf,FROM_BUS))
  {
    int ch = 0;
    if(pframe->addr >= moduleAddr && pframe->addr < moduleAddr + MAX_DEVICE_CHS)
    {
      // get device channel
      if(FRAME_HEADER0 == pframe->header0)
      {
        s_sbp_frame* core = (s_sbp_frame*)pframe->packet;
        ch = pframe->addr%5;
        async_write(device_chs[ch],(uint8_t*)core,core->length);
      }
      else if(0xff == pframe->header0)
      {
        Proc_HostCommand(pbuf);
      }
    }
    else
    {
      // not corresponding address command, last node should 
      // process it,otherwise,pass it down
      if(RESET == is_Last)
      {
        Send_LowerCommand(pbuf);
      }
      else
      {
        Proc_HostCommand(pbuf);
      }
    }
  }
  return 0;
}	

uint8_t Send_UpperCommand(uint8_t* pbuf)
{
  s_sbp_frame* p = (s_sbp_frame*)pbuf;
  async_write(CH_UPPER,pbuf,p->length);
  return 0;
}

uint8_t Get_LowerCommand(uint8_t* pbuf)
{
  if(get_command(CH_LOWER,pbuf,FROM_BUS))
  {
    s_sbp_frame* sbpf = (s_sbp_frame*)pbuf;
    if(FRAME_HEADER0 == sbpf->header0)
    {
      Send_UpperCommand(pbuf);
    }
    else if(0xff == sbpf->header0)
    {
      Proc_LowerCommand(pbuf);
      Send_UpperCommand(pbuf);
    }
  }
  return 0;
}

uint8_t Send_LowerCommand(uint8_t* pbuf)
{
  s_sbp_frame* p = (s_sbp_frame*)pbuf;
  async_write(CH_LOWER,pbuf,p->length);
  return 0;
}


static uint8_t get_StringStream(uint8_t ch, uint8_t* pbuf, uint16_t* count)
{
  int i = 0;
  uint8_t c;
  if(async_read(ch,&c,1))
  {
    pbuf[i++] = c;
    while(sync_read(ch,&c,1,3))
    {
      pbuf[i++] = c;
      // reccive termination character
      if('\n' == c || 0x0d == c || termiantchar == c)
      {
        *count = i;
        return 1;
      }
    }
  }		
  return 0;
  }

void pack_DeviceCommand(uint8_t subAddr, uint8_t* srcBuf,uint8_t* destBuf)
{
  s_sbp_frame *pdest = (s_sbp_frame*)destBuf;
  s_sbp_frame *psrc = (s_sbp_frame*)srcBuf;
  uint8_t i, checksum = 0;

  pdest->header0 = FRAME_HEADER0;
  pdest->header1 = FRAME_HEADER1;
  pdest->length_h = 0x00;
  pdest->length = psrc->length + psrc->length_h + PACKET_HEADER_LENGTH + 1;
  pdest->addr = moduleAddr + subAddr;
  for(i = 0; i < psrc->length; i++)
  {
    pdest->packet[i] = srcBuf[i];
    checksum ^= srcBuf[i];
  }

  for( i = 0; i < PACKET_HEADER_LENGTH; i++)
  {
    checksum ^= destBuf[i];
  }
  destBuf[pdest->length-1] = checksum;

}
uint8_t Get_DeviceCommand(uint8_t subAddr, uint8_t* pbuf)
{
  // temp receive buffer to storing command data
  uint8_t tmp_buf[128];
  uint8_t rs = 0;
  if(!(cmdfmt_map & (1<<subAddr)))
  {
    if(get_command(device_chs[subAddr],tmp_buf,FROM_DEV))
    {
      rs = 1;
    }
  }
  else
  {
    uint16_t count;
    if(get_StringStream(device_chs[subAddr],tmp_buf,&count))
    {
      rs = 1;
    }
  }

  if(rs)
    pack_DeviceCommand(subAddr,tmp_buf,pbuf);
  return rs;
}

uint8_t Send_DeviceCommand(uint8_t ch, uint8_t* pbuf)
{
  return 0;
}