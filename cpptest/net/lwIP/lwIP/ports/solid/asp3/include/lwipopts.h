#ifndef LWIP_HDR_LWIPOPTS_H__
#define LWIP_HDR_LWIPOPTS_H__

/* Prevent having to link sys_arch.c (we don't test the API layers in unit tests) */
#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT            1
#define LWIP_NETCONN                    1
#define LWIP_SOCKET                     1
#define LWIP_NETIF_API                  1

/* Enable IPv4 */
#define LWIP_IPV4                       1

/* Enable IPv6 */
#define LWIP_IPV6                       0

/* Enable DHCP */
#define LWIP_DHCP                       1

/* Enable DNS module (resolver client) */
#define LWIP_DNS                        1

/* Minimal changes to opt.h required for tcp unit tests: */
#define MEM_ALIGNMENT                   4
#define ETH_PAD_SIZE                    2       /* to ensure alignment of header in pbuf */
#define MEM_SIZE                        16000
#define TCP_SND_QUEUELEN                40

#define	TCP_MSS							1460

#define MEMP_NUM_TCP_SEG                TCP_SND_QUEUELEN
#define TCP_SND_BUF                     (12 * TCP_MSS)
#define TCP_WND                         (10 * TCP_MSS)
#define LWIP_WND_SCALE                  1
#define TCP_RCV_SCALE                   0
#define PBUF_POOL_SIZE                  128 /* pbuf tests need ~200KByte */


#define	TCP_SNDLOWAT					(TCP_SND_BUF/2)

#define MEMP_NUM_TCPIP_MSG_API          16
#define	MEMP_NUM_TCPIP_MSG_INPKT		32

/* Minimal changes to opt.h required for etharp unit tests: */
#define ETHARP_SUPPORT_STATIC_ENTRIES   1

#define	LWIP_PROVIDE_ERRNO				1

#define	LWIP_TCPIP_CORE_LOCKING			1
#define	LWIP_TCPIP_CORE_LOCKING_INPUT	0

#define	LWIP_COMPAT_MUTEX				1
#define	LWIP_COMPAT_MUTEX_ALLOWED		1

#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_REMOVE_CALLBACK      0

#define	LWIP_DEBUG


#ifdef LWIP_DEBUG

#define LWIP_DBG_MIN_LEVEL				0
#define TIMERS_DEBUG					LWIP_DBG_OFF
#define SYS_DEBUG						LWIP_DBG_OFF
#define PPP_DEBUG						LWIP_DBG_OFF
#define MEM_DEBUG						LWIP_DBG_OFF
#define MEMP_DEBUG						LWIP_DBG_OFF
#define PBUF_DEBUG						LWIP_DBG_OFF
#define API_LIB_DEBUG					LWIP_DBG_OFF
#define API_MSG_DEBUG					LWIP_DBG_OFF
#define TCPIP_DEBUG						LWIP_DBG_OFF
#define NETIF_DEBUG						LWIP_DBG_OFF
#define SOCKETS_DEBUG					LWIP_DBG_OFF
#define DNS_DEBUG						LWIP_DBG_OFF
#define AUTOIP_DEBUG					LWIP_DBG_OFF
#define DHCP_DEBUG						LWIP_DBG_OFF
#define IP_DEBUG						LWIP_DBG_OFF
#define IP_REASS_DEBUG					LWIP_DBG_OFF
#define ICMP_DEBUG						LWIP_DBG_OFF
#define IGMP_DEBUG						LWIP_DBG_OFF
#define UDP_DEBUG						LWIP_DBG_OFF
#define TCP_DEBUG						LWIP_DBG_OFF
#define TCP_INPUT_DEBUG					LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG				LWIP_DBG_OFF
#define TCP_RTO_DEBUG					LWIP_DBG_OFF
#define TCP_CWND_DEBUG					LWIP_DBG_OFF
#define TCP_WND_DEBUG					LWIP_DBG_OFF
#define TCP_FR_DEBUG					LWIP_DBG_OFF
#define TCP_QLEN_DEBUG					LWIP_DBG_OFF
#define TCP_RST_DEBUG					LWIP_DBG_OFF

#endif	/* LWIP_DEBUG */


//#define TCP_TMR_INTERVAL       			500
//#define TCP_SLOW_INTERVAL				TCP_TMR_INTERVAL


#include	"arch/sys_arch.h"

#endif /* LWIP_HDR_LWIPOPTS_H__ */
