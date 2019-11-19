#include "udp_client.h"
#include "lwip.h"
#include "udp.h"
#include "string.h"

static void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
/* UDP control module */
static struct udp_pcb *upcb;

static void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    uint32_t i;

    /* data call back */
    udp_send(upcb, p);
    udp_sendto(upcb, p, addr, port);

    /* print received data */
    printf("get msg from %d:%d:%d:%d port:%d:\r\n",
        *((uint8_t *)&addr->addr), *((uint8_t *)&addr->addr + 1),
        *((uint8_t *)&addr->addr + 2), *((uint8_t *)&addr->addr + 3), port);

    if (p != NULL)
    {
        struct pbuf *ptmp = p;

        while(ptmp != NULL)
        {
            for (i = 0; i < p->len; i++)
            {
                printf("%c", *((char *)p->payload + i));
            }

            ptmp = p->next;
        }

        printf("\r\n");
    }

    /* release data buffer */
    pbuf_free(p);
}


void udp_client_send(char *pData)
{
    struct pbuf *p;

    /* free buffer */
    p = pbuf_alloc(PBUF_TRANSPORT, strlen(pData), PBUF_POOL);

    if (p != NULL)
    {
        /* fill buffer */
        pbuf_take(p, pData, strlen(pData));

        /* send upd pack */
        udp_send(upcb, p);

        /* free buffer */
        pbuf_free(p);
    }
}


void udp_client_init(void)
{
    ip_addr_t serverIP;
    err_t err;

    IP4_ADDR(&serverIP, SERVER_IP1, SERVER_IP2, SERVER_IP3, SERVER_IP4);

    /* create udp control module */
    upcb = udp_new();

    if (upcb!=NULL)
    {
        /* config local port */
        upcb->local_port = UDP_LOCAL_PORT;

        /* config server IP */
        err= udp_connect(upcb, &serverIP, UDP_REMOTE_PORT);

        if (err == ERR_OK)
        {
            /* callback func */
            udp_recv(upcb, udp_receive_callback, NULL);

            /* send udp pack */
            udp_client_send("udp client connected");

            printf("udp client connected\r\n");
        }
        else
        {
            udp_remove(upcb);

            printf("can not connect udp pcb\r\n");
        }
    }
}
