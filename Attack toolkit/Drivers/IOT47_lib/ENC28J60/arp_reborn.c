#include "arp_reborn.h"
#include "arp.h"
#include "enc28j60.h"

extern const uint8_t macaddr[6];
extern const uint8_t ip[4];
extern char debug_string[60];
extern uint8_t ARP_table_index_reborn;
extern struct
  {
    uint8_t MAC_dich[6];                    // MAC dich
    uint8_t MAC_nguon[6];                   //MAC nguon
    uint16_t Ethernet_type;                 //ethernet type
    uint16_t Hardwave_type;                 //hardwave type
    uint16_t Protocol_type;                 //protocol type (ARP)
    uint16_t Size;                          //size
    uint16_t Opcode;                        //opcode
    uint8_t MAC_sender[6];                  //sender MAC
    uint8_t IP_sender[4];                   //sender IP
    uint8_t MAC_target[6];                  // Target MAC
    uint8_t IP_target[4];                   // Target IP
  }ARP_request;
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table[5];


extern uint32_t ARP_clear_time;

extern int8_t ARP_table_checkIP(uint8_t *ip_check);  //kiem tra xem co ton tai ip trong bang chua

extern void ARP_table_setIP(uint8_t *ip_set, uint8_t *mac_set);

	//
	
//

//
extern void ARP_clear_table(void);



void ARP_send_request_reborn(uint8_t *ip_dest, uint8_t *ip_source,uint8_t *MAC_dest, uint8_t * MAC_source)
{
  struct
		{
    uint8_t MAC_dich[6];                    // MAC dich
    uint8_t MAC_nguon[6];                   //MAC nguon
    uint16_t Ethernet_type;                 //ethernet type
    uint16_t Hardwave_type;                 //hardwave type
    uint16_t Protocol_type;                 //protocol type (ARP)
    uint16_t Size;                          //size
    uint16_t Opcode;                        //opcode
    uint8_t MAC_sender[6];                  //sender MAC
    uint8_t IP_sender[4];                   //sender IP
    uint8_t MAC_target[6];                  // Target MAC
    uint8_t IP_target[4];                   // Target IP
  }ARP_request;

	uint8_t MAC_dest1[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	//memcpy(ARP_request.MAC_nguon,MAC_source,6);
  memcpy(ARP_request.MAC_dich,MAC_dest1,6);
  memcpy(ARP_request.MAC_target,MAC_dest1,6);
  memcpy(ARP_request.IP_target,ip_dest,4);
  memcpy(ARP_request.MAC_nguon,MAC_source,6);
  memcpy(ARP_request.MAC_sender,MAC_source,6);
  memcpy(ARP_request.IP_sender,ip_source,4);
  ARP_request.Ethernet_type=0x0608;
  ARP_request.Hardwave_type=0x0100;
  ARP_request.Protocol_type=0x0008;
  ARP_request.Size=0x0406;
  ARP_request.Opcode=0x0100;     //request
  NET_SendFrame((uint8_t *)&ARP_request,42);   //gui goi ARP reponse di
}


//

void ARP_read_packet_reborn(uint8_t *ARP_Buff,uint16_t len)
{
  struct
  {
    uint8_t MAC_dich[6];                    // MAC dich
    uint8_t MAC_nguon[6];                   //MAC nguon
    uint16_t Ethernet_type;                 //ethernet type
    uint16_t Hardwave_type;                 //hardwave type
    uint16_t Protocol_type;                 //protocol type (ARP)
    uint16_t Size;                          //size
    uint16_t Opcode;                        //opcode
    uint8_t MAC_sender[6];                  //sender MAC
    uint8_t IP_sender[4];                   //sender IP
    uint8_t MAC_target[6];                  // Target MAC
    uint8_t IP_target[4];                   // Target IP
  }ARP_reponse;

  //tao 2 array de luu IP và MAC cua nguon
  uint8_t MAC_source[6];
  uint8_t IP_source[4];
  uint8_t IP_target[4];

  memcpy(MAC_source,(uint8_t *)&ARP_Buff[6],6);  //lay MAC cua nguon gui packet
  memcpy(IP_source ,(uint8_t *)&ARP_Buff[28],4); //lay IP cua nguon gui packet
  memcpy(IP_target ,(uint8_t *)&ARP_Buff[38],4); //lay IP cua nguon nhan packet

  //kiem tra xem co trung voi IP cua minh khong
  if(IP_target[0] != 192 ||  IP_target[1] != 168 || IP_target[2] != 0 || IP_target[3] != 102 ) return;

  if(ARP_Buff[21] == 0x00 && ARP_Buff[22] == 0x01) //neu Opcode = 0x0001 thi day la ban tin ARP request
  {	 uint8_t k[4]={192,168,0,102};
     //UART_putString("Da nhan 1 arp request\r\n");
     //tao goi tin ARP reponse
     memcpy(ARP_reponse.MAC_dich,MAC_source,6);   //dia chi MAC cua thang nhan goi tin reponse
     memcpy(ARP_reponse.MAC_target,MAC_source,6);
     memcpy(ARP_reponse.IP_target,IP_source,4);   //dia chi IP cua thang nhan goi tin reponse
     memcpy(ARP_reponse.MAC_nguon,macaddr,6);    //dia chi MAC cua thang gui ( chinh la MAC cua ENC28J60)
     memcpy(ARP_reponse.MAC_sender,macaddr,6);
     memcpy(ARP_reponse.IP_sender,k,4);   //dia chi IP cua thang gui goi tin reponse ( chinh la MAC cua ENC28J60)
     ARP_reponse.Ethernet_type=0x0608;
     ARP_reponse.Hardwave_type=0x0100;
     ARP_reponse.Protocol_type=0x0008;
     ARP_reponse.Size=0x0406;
     ARP_reponse.Opcode=0x0200;
     UART_putString("Gui gon tin reply_reborn\r\n");
     NET_SendFrame((uint8_t *)&ARP_reponse,42);   //gui goi ARP reponse di
  }
  else if(ARP_Buff[21] == 0x00 && ARP_Buff[22] == 0x02) //neu Opcode = 0x0001 thi day la ban tin ARP reponse
  {
    UART_putString("Da nhan 1 arp reponse_reborn\r\n");
    ARP_table_setIP(IP_source,MAC_source);
  }
}



