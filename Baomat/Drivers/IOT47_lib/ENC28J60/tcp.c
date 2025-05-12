#include "tcp.h"
extern const uint8_t macaddr[6];
extern uint8_t ip[4];
extern char debug_string[60];
extern uint8_t eth_buffer[BUFFER_LENGTH];
extern uint8_t ARP_table_index;
uint8_t ip_1[4];
uint8_t ip_2[4];
uint8_t mac_1[6];
uint8_t mac_2[6];
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table[5];
extern  struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table_PLC[5];

//unsigned char mac_defaul[6]={0x00,0x26,0x5e,0x3d,0x74,0x66};
uint16_t TCP_checksum(TCP_struct *TCP_Struct_Frame)
{
   uint32_t checksum;
   uint8_t *ptr;
   uint16_t length;
   length = swap16(TCP_Struct_Frame->TotoLength) - 20 + 8 ; //tinh length bat dau tu checksum
   ptr = (uint8_t *)&TCP_Struct_Frame->SourceIP;       //dia chi bat dau tinh checksum

   checksum=6 + length - 8;
   while(length>1) //cong het cac byte16 lai
    {
       checksum += (uint16_t) (((uint32_t)*ptr<<8)|*(ptr+1));
       ptr+=2;
       length-=2;
    };
    if(length) checksum+=((uint32_t)*ptr)<<8; //neu con le 1 byte
    while (checksum>>16) checksum=(uint16_t)checksum+(checksum>>16);
    //nghich dao bit
    checksum=~checksum;
    //hoan vi byte thap byte cao
    return swap16(checksum);
}   
int k;

	uint32_t dat_ack;
	uint16_t port;
	uint16_t i, datalength;
//void TCP_send(TCP_struct *TCP_Struct_Frame,uint16_t len,uint8_t *data,uint16_t data_length)
//{
//    for(i=0;i<data_length;i++)TCP_Struct_Frame->data[i] = data[i];
//    len+=data_length;
//    TCP_Struct_Frame->TotoLength = swap16(swap16(TCP_Struct_Frame->TotoLength) + data_length); //make totolength
//    TCP_Struct_Frame->CheckSum=0;
//    TCP_Struct_Frame->TCP_Checksums=0;
//    TCP_Struct_Frame->TCP_Flags = TCP_PSH|TCP_ACK;

//    TCP_Struct_Frame->CheckSum = NET_ipchecksum((uint8_t *)&TCP_Struct_Frame->Header_length);  //tinh checksum cho goi IO
//    TCP_Struct_Frame->TCP_Checksums = TCP_checksum(TCP_Struct_Frame);

//    NET_SendFrame((uint8_t *)TCP_Struct_Frame,len);
//}
extern uint8_t zero_ip;
extern int t;

void TCP_read(uint8_t *TCP_Frame,uint16_t len)
{
  TCP_struct *TCP_Struct_Frame = (TCP_struct *)TCP_Frame;
//		UART_putString("In ra cac goi tin nhan den\r\n");
//		sprintf(debug_string,"Source Ip %i.%i.%i.%i = Source Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->SourceIP[0],TCP_Struct_Frame->SourceIP[1],
//		TCP_Struct_Frame->SourceIP[2],	TCP_Struct_Frame->SourceIP[3],TCP_Struct_Frame->MAC_nguon[0],TCP_Struct_Frame->MAC_nguon[1],
//	TCP_Struct_Frame->MAC_nguon[2],TCP_Struct_Frame->MAC_nguon[3], TCP_Struct_Frame->MAC_nguon[4],TCP_Struct_Frame->MAC_nguon[5]);
//		UART_putString(debug_string);
//		sprintf(debug_string,"Dest Ip %i.%i.%i.%i = Dest Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->DestIP[0],TCP_Struct_Frame->DestIP[1],
//		TCP_Struct_Frame->DestIP[2],	TCP_Struct_Frame->DestIP[3],TCP_Struct_Frame->MAC_dich[0],TCP_Struct_Frame->MAC_dich[1],
//		TCP_Struct_Frame->MAC_dich[2],TCP_Struct_Frame->MAC_dich[3], TCP_Struct_Frame->MAC_dich[4],TCP_Struct_Frame->MAC_dich[5]);
//		UART_putString(debug_string);
memcpy(ip_1,TCP_Struct_Frame->DestIP,4);
memcpy(ip_2,TCP_Struct_Frame->SourceIP,4);
	for(int i =0; i<ARP_table_index; i++)
	{
		if(memcmp(ip_1,ARP_table[i].ip,4)==0)
		{
			memcpy(mac_1,ARP_table[i].mac,6);
		}
	}
memcpy(mac_2,TCP_Struct_Frame->MAC_nguon,4);

		if(TCP_Struct_Frame->TCP_Flags == (TCP_ACK|TCP_PSH)) //flag = 0x018
  {
		if(TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)] == 0x03 
			||TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)] == 0x10)
		{			
		 datalength= swap16(TCP_Struct_Frame->TotoLength) -20 - (TCP_Struct_Frame->data_offset >> 2);  // ( >> 4)*4 = >> 2

//     modbus_readpacket(TCP_Frame,len,TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)]);
		}}
		//Thay doi dia chi goi tin de chuyen tiep 			//1 la  PLC_dich, 2 la plc_nguon
    memcpy(TCP_Struct_Frame->SourceIP,ip_2,4); //hoan vi 1 la  PLC_vdk 1 la PLC_dich
    memcpy(TCP_Struct_Frame->DestIP,ip_1,4); //hoan vi source, 
		memcpy(TCP_Struct_Frame->MAC_nguon,macaddr,6);									
		memcpy(TCP_Struct_Frame->MAC_dich,mac_1,6);	


//		UART_putString("In ra cac goi tin chuyen di\r\n");
//		sprintf(debug_string,"Source Ip %i.%i.%i.%i = Source Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->SourceIP[0],TCP_Struct_Frame->SourceIP[1],
//		TCP_Struct_Frame->SourceIP[2],	TCP_Struct_Frame->SourceIP[3],TCP_Struct_Frame->MAC_nguon[0],TCP_Struct_Frame->MAC_nguon[1],
//	TCP_Struct_Frame->MAC_nguon[2],TCP_Struct_Frame->MAC_nguon[3], TCP_Struct_Frame->MAC_nguon[4],TCP_Struct_Frame->MAC_nguon[5]);
//		UART_putString(debug_string);
//		sprintf(debug_string,"Dest Ip %i.%i.%i.%i = Dest Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->DestIP[0],TCP_Struct_Frame->DestIP[1],
//		TCP_Struct_Frame->DestIP[2],	TCP_Struct_Frame->DestIP[3],TCP_Struct_Frame->MAC_dich[0],TCP_Struct_Frame->MAC_dich[1],
//		TCP_Struct_Frame->MAC_dich[2],TCP_Struct_Frame->MAC_dich[3], TCP_Struct_Frame->MAC_dich[4],TCP_Struct_Frame->MAC_dich[5]);
//		UART_putString(debug_string);
		NET_SendFrame((uint8_t *)TCP_Struct_Frame,len);
	
	}

