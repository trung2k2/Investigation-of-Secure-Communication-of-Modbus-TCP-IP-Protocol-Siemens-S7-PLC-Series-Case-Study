#ifndef TCP_H_
#define TCP_H_
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
typedef struct
  {
    uint8_t MAC_dich[6];     //--------------|
    uint8_t MAC_nguon[6];    //              |   => It is Ethernet Frame II
    uint16_t Ethernet_type;  //--------------|

    uint8_t  Header_length;  //--------------|   => IP
    uint8_t  Services;       //              |
    uint16_t TotoLength;     //              |
    uint16_t Identification; //              |
    uint16_t Flag;           //              |
    uint8_t  TimeToLive;     //              |
    uint8_t  Protocol;       //              |
    uint16_t CheckSum;       //              |
    uint8_t  SourceIP[4];    //              |
    uint8_t  DestIP[4];      //--------------|

    uint16_t Source_Port;//---------------|   //TCP
    uint16_t Dest_Port;  //               |
    uint32_t Sequence_Number;//           |
    uint32_t Acknowledgement;//           |
    uint8_t  data_offset;//               |
    uint8_t  TCP_Flags;//                 |
    uint16_t Window;//                    |
    uint16_t TCP_Checksums;//             |
    uint16_t Urgent_Pointer;//            |
    uint8_t  data[];//--------------------|
  }__attribute__ ((packed)) TCP_struct;
#define TCP_MIN_LENGTH 54

	
  //dinh nghia cac macro cho truong Flags
  #define TCP_CWR 0x80
  #define TCP_ECE 0x40
  #define TCP_URG 0x20
  #define TCP_ACK 0x10
  #define TCP_PSH 0x08
  #define TCP_RST 0x04
  #define TCP_SYN 0x02
  #define TCP_FIN 0x01

  #define FOR_SYN        0
  #define FOR_FIN        1
  #define FOR_PSH_ACK    2
  #define FOR_DISCONNECT 3
  #define FOR_CONNECT    4
	#define FOR_ACK        5
  #define FOR_SEND       6

  #define ENC28J60_TCP_PORT 20732
//--------------------------------------------------
//void TCP_make_herder(TCP_struct *TCP_Struct_Frame,uint16_t len,uint8_t type);
void TCP_read(uint8_t *TCP_Frame,uint16_t len);
uint16_t TCP_checksum(TCP_struct *TCP_Struct_Frame);
void TCP_send(TCP_struct *TCP_Struct_Frame,uint16_t len,uint8_t *data,uint16_t data_length);
uint8_t TCP_sendSYN(uint8_t *IP_server,uint16_t port,uint32_t timeout);
uint8_t TCP_Connect(uint8_t *IP_server,uint16_t port,uint32_t timeout,int8_t reconnect);
uint8_t TCP_Disconect(uint32_t timeout);
uint8_t TCP_sendData(uint8_t *data,uint16_t data_length,uint32_t timeout);
void TCP_readData(TCP_struct *TCP_Struct_Frame,uint16_t len);
//--------------------------------------------------
#endif /* TCP_H_ */
