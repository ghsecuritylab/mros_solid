#include "solid_net_registerapi.h"
#include "solid_loader.h"

#include "solid_net_if.h"

#include "lwip/opt.h"
#include "lwip/raw.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

extern  void SOLID_NET_start_network(void);
extern  void lwip_thread_task(intptr_t exinf);

#define REG_SYM(name)	{ #name , (SOLID_ADDRESS)(name) }

static struct _NET_SYMBOL_ {
    const char* pName;
    SOLID_ADDRESS addr;
} NETSymbol[] = {

/*
 * start up
 */
REG_SYM(SOLID_NET_IF_Init),
REG_SYM(SOLID_NET_start_network),
REG_SYM(lwip_thread_task),

/*
 * Calback style APIs
 */
REG_SYM(tcp_close),
REG_SYM(tcp_shutdown),
REG_SYM(tcp_abort),
REG_SYM(tcp_bind),
REG_SYM(tcp_listen_with_backlog),
REG_SYM(tcp_recved),
REG_SYM(tcp_connect),
REG_SYM(tcp_new),
REG_SYM(tcp_new_ip_type),
REG_SYM(tcp_arg),
REG_SYM(tcp_recv),
REG_SYM(tcp_sent),
REG_SYM(tcp_err),
REG_SYM(tcp_accept),
REG_SYM(tcp_poll),
REG_SYM(udp_send),
REG_SYM(udp_sendto),
REG_SYM(udp_sendto_if),
REG_SYM(udp_sendto_if_src),
REG_SYM(udp_bind),
REG_SYM(udp_connect),
REG_SYM(udp_disconnect),
REG_SYM(udp_recv),
REG_SYM(udp_remove),
REG_SYM(udp_new),
REG_SYM(udp_new_ip_type),

#if LWIP_RAW
/*
 * RAW APIs
 */
REG_SYM(raw_connect),
REG_SYM(raw_recv),
REG_SYM(raw_sendto),
REG_SYM(raw_send),
REG_SYM(raw_remove),
REG_SYM(raw_new),
REG_SYM(raw_new_ip_type),

#endif  /* LWIP_RAW != 0 */

/*
 * NETCONN APIs
 */
REG_SYM(netconn_delete),
REG_SYM(netconn_bind),
REG_SYM(netconn_connect),
REG_SYM(netconn_recv),
#if LWIP_DNS
#if LWIP_IPV4 && LWIP_IPV6
REG_SYM(netconn_gethostbyname_addrtype),
#else /* !(LWIP_IPV4 && LWIP_IPV6) */
REG_SYM(netconn_gethostbyname),
#endif /* LWIP_IPV4 && LWIP_IPV6 */
#endif /* LWIP_DNS != 0 */
REG_SYM(netconn_listen_with_backlog),
REG_SYM(netconn_accept),
REG_SYM(netconn_recv_tcp_pbuf),
REG_SYM(netconn_write_partly),
REG_SYM(netconn_close),
REG_SYM(netconn_shutdown),
REG_SYM(netconn_disconnect),
REG_SYM(netconn_sendto),
REG_SYM(netconn_send),
#if LWIP_IGMP || (LWIP_IPV6 && LWIP_IPV6_MLD)
REG_SYM(netconn_join_leave_group),
#endif /* LWIP_IGMP || (LWIP_IPV6 && LWIP_IPV6_MLD) */
REG_SYM(netbuf_new),
REG_SYM(netbuf_delete),
REG_SYM(netbuf_alloc),
REG_SYM(netbuf_free),
REG_SYM(netbuf_ref),
REG_SYM(netbuf_chain),
REG_SYM(netbuf_data),
REG_SYM(netbuf_next),
REG_SYM(netbuf_first),

/*
 * socket/lwip APIs
 */
#if BYTE_ORDER != BIG_ENDIAN
REG_SYM(lwip_htonl),
REG_SYM(lwip_htons),
#endif /* BYTE_ORDER != BIG_ENDIAN */
#if 0
REG_SYM(lwip_ntohl),
REG_SYM(lwip_ntohs),
#endif
REG_SYM(lwip_accept),
REG_SYM(lwip_bind),
REG_SYM(lwip_shutdown),
REG_SYM(lwip_getpeername),
REG_SYM(lwip_getsockname),
REG_SYM(lwip_setsockopt),
REG_SYM(lwip_getsockopt),
REG_SYM(lwip_close),
REG_SYM(lwip_connect),
REG_SYM(lwip_listen),
REG_SYM(lwip_recv),
REG_SYM(lwip_recvfrom),
REG_SYM(lwip_send),
REG_SYM(lwip_sendmsg),
REG_SYM(lwip_sendto),
REG_SYM(lwip_socket),
REG_SYM(lwip_select),
REG_SYM(lwip_ioctl),
REG_SYM(lwip_read),
REG_SYM(lwip_write),
REG_SYM(lwip_writev),
REG_SYM(lwip_close),
REG_SYM(lwip_fcntl),
REG_SYM(lwip_ioctl),

#if LWIP_DNS && LWIP_SOCKET
REG_SYM(lwip_gethostbyname),
REG_SYM(lwip_gethostbyname_r),
REG_SYM(lwip_freeaddrinfo),
REG_SYM(lwip_getaddrinfo),
#endif /* LWIP_DNS && LWIP_SOCKET */

REG_SYM(ip4addr_ntoa_r),
REG_SYM(ip4addr_aton),

#if LWIP_IPV6
REG_SYM(ip6addr_ntoa_r),
REG_SYM(ip6addr_aton),
#endif /* LWIP_IPV6 */

};

void SOLID_NET_RegisterAPIs(void)
{
    unsigned int i;
    for (i = 0; i < sizeof(NETSymbol)/sizeof(struct _NET_SYMBOL_); i++) {
        SOLID_LDR_RegisterSymbol(NETSymbol[i].pName, NETSymbol[i].addr);
    }
}

