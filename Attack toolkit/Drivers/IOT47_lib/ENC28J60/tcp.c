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
extern int dao;
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table[5];
extern char send_status;
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}modbus_ARP_table[2];
uint8_t modbus_search = 1;
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
uint8_t zero_ip[4]={0,0,0,0};
void TCP_read(uint8_t *TCP_Frame,uint16_t len)
{
//  uint8_t *data_send=0;
//  uint16_t data_send_len=0;
  TCP_struct *TCP_Struct_Frame = (TCP_struct *)TCP_Frame;
				
					//	ARP_table[k].ip, ARP_table[i].ip, ARP_table[k].mac);
						
					//	ARP_send_request_fake(ARP_table[i].ip, ARP_table[k].ip, ARP_table[i].mac);
					
			
//kiem tra dia chi ip xem co phai no gui cho minh khong
	memcpy(ip_1,TCP_Struct_Frame->DestIP,4);
	memcpy(ip_2,TCP_Struct_Frame->SourceIP,4);
		for(int i =0; i<ARP_table_index; i++)
		{
			if(memcmp(ip_1,ARP_table[i].ip,4)==0)
			{
				memcpy(mac_1,ARP_table[i].mac,6);
			}
		}
	memcpy(mac_2,TCP_Struct_Frame->MAC_nguon,6);
	//UART_putString("nhan duoc goi tin\r\n");				
	uint8_t modbus_fcode = TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)];		
		if(TCP_Struct_Frame->TCP_Flags == (TCP_ACK|TCP_PSH)) //flag = 0x018
  {
		if(TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)] == 0x03 
			||TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)] == 0x10)
		{			
		 datalength= swap16(TCP_Struct_Frame->TotoLength) -20 - (TCP_Struct_Frame->data_offset >> 2);  // ( >> 4)*4 = >> 2
			
//		 sprintf(debug_string,"Do dai data goi tin la: %u\r\n",datalength); //in ra do dai goi tin
//     UART_putString(debug_string);
//		 UART_putString("Du lieu nhan duoc: ");
			
			if(send_status == 3)
				modbus_readpacket(TCP_Frame,len,TCP_Struct_Frame->data[7 + ((TCP_Struct_Frame->data_offset >> 2) - 20)]);
		}
			if(modbus_search == 1)
			{
				if((modbus_fcode == 0x10 && datalength >= 12)||(modbus_fcode == 0x03 && datalength == 12))
				{
					memcpy(modbus_ARP_table[0].ip,TCP_Struct_Frame->SourceIP,4);
					memcpy(modbus_ARP_table[1].ip,TCP_Struct_Frame->DestIP,4);
					memcpy(modbus_ARP_table[0].mac,TCP_Struct_Frame->MAC_nguon,6);
					memcpy(modbus_ARP_table[1].mac,TCP_Struct_Frame->MAC_dich,6);
				}
				modbus_search = 2;
				send_status = 2;
			}
	}
//		UART_putString("Source\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->SourceIP[0],TCP_Struct_Frame->SourceIP[1],TCP_Struct_Frame->SourceIP[2],
//		TCP_Struct_Frame->SourceIP[3],TCP_Struct_Frame->MAC_nguon[0],TCP_Struct_Frame->MAC_nguon[1],TCP_Struct_Frame->MAC_nguon[2],TCP_Struct_Frame->MAC_nguon[3]
//		,TCP_Struct_Frame->MAC_nguon[4],TCP_Struct_Frame->MAC_nguon[5]);
//		UART_putString(debug_string);
//		UART_putString("Dest\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->DestIP[0],TCP_Struct_Frame->DestIP[1],TCP_Struct_Frame->DestIP[2],
//		TCP_Struct_Frame->DestIP[3],TCP_Struct_Frame->MAC_dich[0],TCP_Struct_Frame->MAC_dich[1],TCP_Struct_Frame->MAC_dich[2],TCP_Struct_Frame->MAC_dich[3]
//		,TCP_Struct_Frame->MAC_dich[4],TCP_Struct_Frame->MAC_dich[5]);
//    UART_putString(debug_string);	
//	
//			UART_putString("- Source\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",ip_2[0],ip_2[1],ip_2[2],
//		ip_2[3],macaddr[0],macaddr[1],macaddr[2],macaddr[3]
//		,macaddr[4],macaddr[5]);
//		UART_putString(debug_string);
//		UART_putString("- Dest\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",ip_1[0],ip_1[1],ip_1[2],
//		ip_1[3],mac_1[0],mac_1[1],mac_1[2],mac_1[3]
//		,mac_1[4],mac_1[5]);
//    UART_putString(debug_string);	
		//Thay doi dia chi goi tin de chuyen tiep 			//1 la  PLC_dich, 2 la plc_nguon
    memcpy(TCP_Struct_Frame->SourceIP,ip_2,4); //hoan vi 1 la  PLC_vdk 1 la PLC_dich
    memcpy(TCP_Struct_Frame->DestIP,ip_1,4); //hoan vi source, 
		memcpy(TCP_Struct_Frame->MAC_nguon,macaddr,6);									
		memcpy(TCP_Struct_Frame->MAC_dich,mac_1,6);	
		NET_SendFrame((uint8_t *)TCP_Struct_Frame,len);
	
	}
			//sprintf(debug_string,"Nhan ban tin TCP co flag = %02X\r\n",TCP_Struct_Frame->TCP_Flags); //in ra co
			//UART_putString(debug_string);
     //tinh do dai cua goi data va in ra man hinh
//			UART_putString("Dest\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",ip_1[0],ip_1[1],ip_1[2],
//		ip_1[3],mac_1[0],mac_1[1],mac_1[2],mac_1[3]
//		,mac_1[4],mac_1[5]);
//		UART_putString(debug_string);
//			UART_putString("Sorce\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",ip_2[0],ip_2[1],ip_2[2],
//		ip_2[3],mac_2[0],mac_2[1],mac_2[2],mac_2[3]
//		,mac_2[4],mac_2[5]);
//		UART_putString(debug_string);
//		UART_putString("Source\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->SourceIP[0],TCP_Struct_Frame->SourceIP[1],TCP_Struct_Frame->SourceIP[2],
//		TCP_Struct_Frame->SourceIP[3],TCP_Struct_Frame->MAC_nguon[0],TCP_Struct_Frame->MAC_nguon[1],TCP_Struct_Frame->MAC_nguon[2],TCP_Struct_Frame->MAC_nguon[3]
//		,TCP_Struct_Frame->MAC_nguon[4],TCP_Struct_Frame->MAC_nguon[5]);
//		UART_putString(debug_string);
//		UART_putString("Dest\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->DestIP[0],TCP_Struct_Frame->DestIP[1],TCP_Struct_Frame->DestIP[2],
//		TCP_Struct_Frame->DestIP[3],TCP_Struct_Frame->MAC_dich[0],TCP_Struct_Frame->MAC_dich[1],TCP_Struct_Frame->MAC_dich[2],TCP_Struct_Frame->MAC_dich[3]
//		,TCP_Struct_Frame->MAC_dich[4],TCP_Struct_Frame->MAC_dich[5]);
//    UART_putString(debug_string);	
//		 datalength= swap16(TCP_Struct_Frame->TotoLength) -20 - (TCP_Struct_Frame->data_offset >> 2);  // ( >> 4)*4 = >> 2
//		 sprintf(debug_string,"Do dai data goi tin la: %u\r\n",datalength); //in ra do dai goi tin
//     UART_putString(debug_string);
		 //UART_putString("Du lieu nhan duoc: ");
//     for(i=0;i<datalength;i++)
//		{
//			sprintf(debug_string,"%02X:",TCP_Struct_Frame->data[i + ((TCP_Struct_Frame->data_offset >> 2) - 20) ]); //in ra ban tin
//			UART_putString(debug_string);
//		}
//     UART_putString("\r\n");
	
//		UART_putString("Source\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->SourceIP[0],TCP_Struct_Frame->SourceIP[1],TCP_Struct_Frame->SourceIP[2],
//		TCP_Struct_Frame->SourceIP[3],TCP_Struct_Frame->MAC_nguon[0],TCP_Struct_Frame->MAC_nguon[1],TCP_Struct_Frame->MAC_nguon[2],TCP_Struct_Frame->MAC_nguon[3]
//		,TCP_Struct_Frame->MAC_nguon[4],TCP_Struct_Frame->MAC_nguon[5]);
//		UART_putString(debug_string);
//		UART_putString("Dest\r\n");
//		sprintf(debug_string,"Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",TCP_Struct_Frame->DestIP[0],TCP_Struct_Frame->DestIP[1],TCP_Struct_Frame->DestIP[2],
//		TCP_Struct_Frame->DestIP[3],TCP_Struct_Frame->MAC_dich[0],TCP_Struct_Frame->MAC_dich[1],TCP_Struct_Frame->MAC_dich[2],TCP_Struct_Frame->MAC_dich[3]
//		,TCP_Struct_Frame->MAC_dich[4],TCP_Struct_Frame->MAC_dich[5]);
//    UART_putString(debug_string);
		
//    NET_SendFrame((uint8_t *)TCP_Struct_Frame,len);

