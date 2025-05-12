#ifndef SCAN_H_
#define SCAN_H_
//--------------------------------------------------
//Include cac thu vien can thiet
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
//#include "uart.h"
#include "enc28j60.h"
#include "arp.h"
#include "tcp.h"
#define BUFFER_LENGTH1 512
void NET_loop_scan(void);
void NET_read_scan(uint8_t *net_buffer,uint16_t len);
void NET_ipRead_scan(uint8_t *IP_Frame,uint16_t len);
void TCP_read_scan(uint8_t *TCP_Frame,uint16_t len);
int8_t ARP_table_checkIP_PLC(uint8_t *ip_check) ;
void ARP_table_setIP_scan(uint8_t *ip_set, uint8_t *mac_set);
void ARP_read_packet_scan(uint8_t *ARP_Buff,uint16_t len);





#endif /* NET_H_ */
