#include "modbus.h"
#include "net.h"
extern char debug_string[60];
extern uint16_t i;
uint16_t data_length;
uint8_t data_ref[50]={0}; //do dai toi da cua data
uint8_t data_save[50];
uint8_t q=0;
uint16_t modbus_checksum(modbus_struct *modbus_Struct_Frame)
{
   uint32_t checksum;
   uint8_t *ptr;
   uint16_t length;
   length = swap16(modbus_Struct_Frame->TotoLength) - 20 + 8 ; //tinh length bat dau tu checksum
   ptr = (uint8_t *)&modbus_Struct_Frame->SourceIP;       //dia chi bat dau tinh checksum

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
void modbus_readpacket(uint8_t* modbus_Frame,uint8_t len, uint8_t f_code)
{
		modbus_struct *modbus_Struct_Frame = (modbus_struct *)modbus_Frame;
		if(f_code == 0x03)
		{
			data_length= swap16(modbus_Struct_Frame->TotoLength) -20 - (modbus_Struct_Frame->data_offset >> 2);  // ( >> 4)*4 = >> 2
		 //UART_putString("Du lieu nhan duoc: ");
			if (data_length == 12)
			{
//				UART_putString("Day la goi QUERY READ\r\n");
			}
			else if (data_length > 12)
			{
//				UART_putString("Day la goi RESPONSE READ\r\n");
//				{
				for (i=0;i<data_length-9;i++)
				{
					modbus_Struct_Frame->data[i+9]=data_save[i];
				}
				modbus_Struct_Frame->CheckSum=0;
				modbus_Struct_Frame->TCP_Checksums=0;

				modbus_Struct_Frame->CheckSum = NET_ipchecksum((uint8_t *)&modbus_Struct_Frame->Header_length);  //tinh checksum cho goi IO
				modbus_Struct_Frame->TCP_Checksums = modbus_checksum(modbus_Struct_Frame);	
			}
		}
		if(f_code == 0x10)
		{
			data_length= swap16(modbus_Struct_Frame->TotoLength) -20 - (modbus_Struct_Frame->data_offset >> 2);  // ( >> 4)*4 = >> 2
		 //UART_putString("Du lieu nhan duoc: ");
			if (data_length == 12)
			{
//				UART_putString("Day la goi RESPONSE WRITE\r\n");
			}
			else if (data_length > 12)
			{
//				UART_putString("Day la goi QUERY WRITE\r\n");
				for (i=0;i<data_length-13;i++)
				{
					data_save[i]=modbus_Struct_Frame->data[i+13];
				}
				}
				if (q<=5){
				modbus_Struct_Frame->data[14] = 0x07;
				q++;}
				else if(q>=5 && q<=10){
				modbus_Struct_Frame->data[14] = 0x00;
				q=q+1;}
				else
					q=0;
				modbus_Struct_Frame->CheckSum=0;
				modbus_Struct_Frame->TCP_Checksums=0;
				modbus_Struct_Frame->CheckSum = NET_ipchecksum((uint8_t *)&modbus_Struct_Frame->Header_length);  //tinh checksum cho goi IO
				modbus_Struct_Frame->TCP_Checksums = modbus_checksum(modbus_Struct_Frame);

		}
}


