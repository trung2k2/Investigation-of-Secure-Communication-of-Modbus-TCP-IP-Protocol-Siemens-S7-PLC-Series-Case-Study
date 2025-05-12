#include "arp_reply.h"
#include "arp.h"
#include "enc28j60.h"

extern int a[2];
extern int k;
extern char debug_string[60];
extern struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table_static[5];
void help(int a,int b)
{
	UART_putString("He thong dang bi tan cong\r\nHai thiet bi bi tan cong la\r\n");
			sprintf(debug_string,"- Dia chi thiet bi %i Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",1,ARP_table_static[a].ip[0]
				,ARP_table_static[a].ip[1], ARP_table_static[a].ip[2],	ARP_table_static[a].ip[3],ARP_table_static[a].mac[0],
				ARP_table_static[a].mac[1],ARP_table_static[a].mac[2],ARP_table_static[a].mac[3],
				ARP_table_static[a].mac[4],ARP_table_static[a].mac[5]);
				UART_putString(debug_string);	
			sprintf(debug_string,"- Dia chi thiet bi %i Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",2,ARP_table_static[b].ip[0]
				,ARP_table_static[b].ip[1], ARP_table_static[b].ip[2],	ARP_table_static[b].ip[3],ARP_table_static[b].mac[0],
				ARP_table_static[b].mac[1],ARP_table_static[b].mac[2],ARP_table_static[b].mac[3],
				ARP_table_static[b].mac[4],ARP_table_static[b].mac[5]);
				UART_putString(debug_string);	
//			int m = a[0];
//			int n = a[1];

}
