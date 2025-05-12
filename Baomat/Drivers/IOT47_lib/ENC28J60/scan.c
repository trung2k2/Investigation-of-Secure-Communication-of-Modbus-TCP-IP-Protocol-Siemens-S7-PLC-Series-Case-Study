#include "scan.h"
uint8_t eth_buffer1[BUFFER_LENGTH1];
extern char debug_string[60];
extern const uint8_t macaddr[6];
struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table_PLC[5];
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table[5];
uint8_t ARP_table_index_PLC=0;
extern uint8_t ARP_table_index;
extern uint8_t ip_1[4];
extern uint8_t ip_2[4];
extern uint8_t mac_1[6];
extern uint8_t mac_2[6];
extern uint16_t i, datalength;


int8_t ARP_table_checkIP_PLC(uint8_t *ip_check)  //kiem tra xem co ton tai ip trong bang chua
{
 int i;
 for(i=0;i<5;i++)
 {
   if(ip_check[0] == ARP_table_PLC[i].ip[0] && ip_check[1] == ARP_table_PLC[i].ip[1] && ip_check[2] == ARP_table_PLC[i].ip[2] 
		 && ip_check[3] == ARP_table_PLC[i].ip[3] )
      return i+1; //tra ve vi tri cua ip trong table
 }
 return -1;
}

void ARP_table_setIP_scan(uint8_t *ip_set, uint8_t *mac_set)
{
  if(ARP_table_checkIP_PLC(ip_set) == -1 ) //neu chua ton tai IP trong table
  {
    memcpy(ARP_table_PLC[ARP_table_index_PLC].ip,ip_set,4);
		for(i=0;i<5;i++)
		{
			if(memcmp(ip_set,ARP_table[i].ip,4)==0)
			{
				memcpy(mac_set,ARP_table[i].mac,6);
			}
		}
    memcpy(ARP_table_PLC[ARP_table_index_PLC].mac,mac_set,6);
    ARP_table_index_PLC++;if(ARP_table_index_PLC==7)ARP_table_index_PLC=0; //neu so luong ip vuot qua muc cho phep thi quay lai
  }
  //else
  //  UART_putString("IP da ton tai trong bang\r\n");
}
void TCP_read_scan(uint8_t *TCP_Frame,uint16_t len)
{
//  uint8_t *data_send=0;
//  uint16_t data_send_len=0;
  TCP_struct *TCP_Struct_Frame = (TCP_struct *)TCP_Frame;
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
	UART_putString("Day la goi TCP_read_scan\r\n");
//	sprintf(debug_string,"Source Port la: %i, Dest Port la: %i\r\n",TCP_Struct_Frame->Source_Port,TCP_Struct_Frame->Dest_Port); //in ra do dai goi tin
//				UART_putString(debug_string);
//	sprintf(debug_string,"Source Port la: %u, Dest Port la: %u\r\n",TCP_Struct_Frame->Source_Port,TCP_Struct_Frame->Dest_Port); //in ra do dai goi tin
//				UART_putString(debug_string);
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
		NET_SendFrame((uint8_t *)TCP_Struct_Frame,len);
	
	}
void NET_ipRead_scan(uint8_t *IP_Frame,uint16_t len)
{
	UART_putString("Day la goi NET_ipRead_scan\r\n");
  //chung ta se kiem tra xem goi tin do thuoc loai nao tcp hay udp hay icmp
  if(IP_Frame[23] == 0x06) //it is TCP packet
  {
		
     TCP_read_scan(IP_Frame,len);
  }
}
void NET_read_scan(uint8_t *net_buffer,uint16_t len)
{
	UART_putString("Da vao NET_read_scan \r\n");
  if(net_buffer[12] == 0x08 && net_buffer[13] == 0x06)
  {
    if(len>41)
    {
		UART_putString("Da vao ARP_read_packet_scan \r\n");
    ARP_read_packet(net_buffer,len);	
    }
  }
  else if(net_buffer[12] == 0x08 && net_buffer[13] == 0x00)
  {
		UART_putString("Da vao NET_ipRead_scan \r\n");
     NET_ipRead_scan(net_buffer,len);
  }
}

void NET_loop_scan(void)
{
 uint16_t len;
 while(1)
 {
    len=ENC28J60_read_packet(eth_buffer1,BUFFER_LENGTH1);
    if(len==0)return;
    else NET_read_scan(eth_buffer1,len);
 }
}
