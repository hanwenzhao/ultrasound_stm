// Reference: https://github.com/54zorb/stm32-lwip/tree/master/Src
#ifndef __udp_client_H
#define __udp_client_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip/udp.h"

 /* define port number */
#define UDP_REMOTE_PORT    8881 /* remote port */
#define UDP_LOCAL_PORT     8880 /* local port */

// define server IP
#define SERVER_IP1 169
#define SERVER_IP2 254
#define SERVER_IP3 212
#define SERVER_IP4 187

void udp_client_send(char *pData);
void udp_client_init(void);

#ifdef __cplusplus
}
#endif
#endif

