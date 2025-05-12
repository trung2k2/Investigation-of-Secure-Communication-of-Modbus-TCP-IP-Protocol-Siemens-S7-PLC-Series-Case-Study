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


