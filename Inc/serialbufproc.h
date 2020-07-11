#ifndef SERIAL_BUS_PROC_H
#define SERIAL_BUS_PROC_H
#include "stdint.h"


typedef enum	{
	CH_UPPER = 0,
	CH_LOWER = 4,
	CH_DEVICE0 = 3,
	CH_DEVICE1 = 1,
	CH_DEVICE2 = 2,
	CH_DEVICE3 = 6,
	CH_DEVICE4 = 7
}e_channelTypedef;

typedef enum{
  FROM_BUS,
  FROM_DEV
}e_framesrc;

#define FRAME_HEADER0 0xaa
#define FRAME_HEADER1 0x55


typedef struct{
	uint8_t header0;
	uint8_t header1;
  uint8_t length_h;
	uint8_t length;
	uint8_t addr;
	uint8_t packet[1];
}s_sbp_frame;

#define PACKET_HEADER_LENGTH  5
#define MAX_DEVICE_CHS	5


uint8_t Send_DeviceCommand(uint8_t ch, uint8_t* pbuf);
uint8_t Send_LowerCommand(uint8_t* pbuf);
uint8_t Send_UpperCommand(uint8_t* pbuf);
uint8_t Get_DeviceCommand(uint8_t ch, uint8_t* pbuf);
uint8_t Get_LowerCommand(uint8_t* pbuf);
uint8_t Get_UpperCommand(uint8_t* pbuf);
void Init_InterfacNode(void);
uint8_t Get_TTMode();
e_channelTypedef* Get_DeviceCHs();

void Disable_DirectLink();
void Enable_DirectLink();
void Fresh_TT_timer();
void TT_timing_1ms();
#endif