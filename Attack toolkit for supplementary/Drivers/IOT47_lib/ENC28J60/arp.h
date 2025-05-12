#ifndef ARP_H_
#define ARP_H_
//--------------------------------------------------
//Include cac thu vien can thiet
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "net.h"
//-------------------------------------------------
//dinh nghia thoi gian xoa bang arp
#define MAX_TIME_SAVE 10 //(phut)
//-------------------------------------------------
#define COUNT_TICK (MAX_TIME_SAVE*60000)
//-------------------------------------------------
  typedef struct
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
  }__attribute__ ((packed)) ARP_struct;
//-------------------------------------------------
void ARP_read_packet(uint8_t *ARP_Buff,uint16_t len);
void ARP_send_request(uint8_t *ip_dest);
int8_t ARP_table_checkIP(uint8_t *ip_check);
void ARP_table_setIP(uint8_t *ip_set, uint8_t *mac_set);
void ARP_clear_table(void);
void ARP_table_get_MAC(uint8_t *ip_check,uint8_t * MAC_dest);
void ARP_report(void);
//--------------------------------------------------
#endif /* ARP_H_ */
