#ifndef LWIP_HDR_LWIPOPTS_H__
#define LWIP_HDR_LWIPOPTS_H__

#include "solid_mutex.h"

/* Prevent having to link sys_arch.c (we don't test the API layers in unit tests) */
#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

/* Enable DHCP to test it, disable UDP checksum to easier inject packets */
#define LWIP_DHCP                       1

/* Minimal changes to opt.h required for tcp unit tests: */
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        16000
#define TCP_SND_QUEUELEN                40
#define MEMP_NUM_TCP_SEG                TCP_SND_QUEUELEN
#define TCP_SND_BUF                     (12 * TCP_MSS)
#define TCP_WND                         (10 * TCP_MSS)
#define LWIP_WND_SCALE                  1
#define TCP_RCV_SCALE                   0
#define PBUF_POOL_SIZE                  400 /* pbuf tests need ~200KByte */

/* Minimal changes to opt.h required for etharp unit tests: */
#define ETHARP_SUPPORT_STATIC_ENTRIES   1

#include	"arch/sys_arch.h"

#endif /* LWIP_HDR_LWIPOPTS_H__ */
