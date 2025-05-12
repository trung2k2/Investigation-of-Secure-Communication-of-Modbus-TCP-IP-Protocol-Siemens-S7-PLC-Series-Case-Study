#ifndef ARP_REPLY_H_
#define ARP_REPLY_H_
//--------------------------------------------------
//Include cac thu vien can thiet

#include "uart.h"
#include "net.h"
#include "arp.h"
//#include "arp_reborn.h"
//-------------------------------------------------
//dinh nghia thoi gian xoa bang arp
#define MAX_TIME_SAVE 10 //(phut)
//-------------------------------------------------
#define COUNT_TICK (MAX_TIME_SAVE*60000)
//-------------------------------------------------
void ARP_read_packet_reborn(uint8_t *ARP_Buff,uint16_t len);
void ARP_send_reply(uint8_t *ip_dest, uint8_t *ip_source, uint8_t * MAC_source, uint8_t * MAC_dest);
void help(int a,int b);
//void ARP_send_request_fake(uint8_t *ip_dest, uint8_t *sourceIP);
//--------------------------------------------------
#endif /* ARP__FAKE_H_ */

