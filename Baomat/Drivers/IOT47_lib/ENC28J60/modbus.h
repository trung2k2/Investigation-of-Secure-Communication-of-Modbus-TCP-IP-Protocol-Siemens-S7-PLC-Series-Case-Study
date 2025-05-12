#ifndef MODBUS_H_
#define MODBUS_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "net.h"
#include "tcp.h"
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
  }__attribute__ ((packed)) modbus_struct; //Read Query
#define read 0x03
	void modbus_read_packet(uint8_t* modbus_buff,uint8_t len);
	void modbus_read(uint8_t *TCP_Frame,uint16_t len);
	void modbus_write(uint8_t *TCP_Frame,uint16_t len);
	void modbus_readpacket(uint8_t* modbus_Frame,uint8_t len, uint8_t f_code);
	uint16_t modbus_checksum(modbus_struct *modbus_Struct_Frame);
#endif /* MODBUS_H_ */
