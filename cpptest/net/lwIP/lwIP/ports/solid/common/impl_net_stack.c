#include	<stdint.h>
#include    <stdbool.h>
#include	"lwip/init.h"
#include	"lwip/pbuf.h"
#include	"lwip/netif.h"
#include	"lwip/tcpip.h"
#include	"lwip/timeouts.h"
#include	"lwip/dns.h"
#include	"lwip/dhcp.h"
#include	"lwip/autoip.h"
#include	"netif/ethernet.h"
#include	"lwip/netifapi.h"
#include	"solid_net_if.h"
#include	"impl_net_stack.h"

#include    "solid_log.h"
#include	"solid_cs_assert.h"

struct	netif	_netif_store_;
struct	netif	*_netif = NULL;

#if	LWIP_DHCP
struct	dhcp	_dhcp;
#endif	/* LWIP_DHCP */

#if	LWIP_AUTOIP
struct	autoip	_autoip;
#endif	/* LWIP_AUTOIP */

extern	err_t ethernetif_init(struct netif *netif);

#if LWIP_NETIF_STATUS_CALLBACK
static  void    __showIfStatus( struct netif *netif);
#define     HOOK_NETIF_STATUC_CALLBACK(netif)        netif_set_status_callback((netif), __showIfStatus)
#else
#define     HOOK_NETIF_STATUC_CALLBACK(netif)
#endif

void IMPL_NET_STACK_Init(void)
{
#if NO_SYS
	lwip_init();
#endif
}

void IMPL_NET_STACK_Start(void(*fn_done)(void*), void *arg)
{
#if NO_SYS
	fn_done(arg);
#else
	extern void sys_setup_OSresource(void);
	extern void sys_start_daemon(void (*)(void*), void*);
	sys_setup_OSresource();
	// tcpip_init(fn_done,arg);
	sys_start_daemon(fn_done,arg);
#endif
}

static	void _set_ip_addr_t(const uint8_t array[], size_t len, ip_addr_t	*stack_addr)
{
	if (array) {
		if ( len == 4 /* IPv4 */ ) {
			IP_ADDR4(stack_addr, array[0], array[1], array[2], array[3]);
#if LWIP_IPV6
		} else if (len == 16 /* IPv6 */ ) {
			IP_ADDR6(stack_addr,
				((array[0] << 24) | (array[1] << 16) | (array[2] << 8) |  array[3]),
				((array[4] << 24) | (array[5] << 16) | (array[6] << 8 )|  array[7]),
				((array[8] << 24) | (array[9] << 16) | (array[10] << 8 )| array[11]),
				((array[12] << 24) | (array[13] << 16) | (array[14] << 8 )| array[15]));
#endif	/* LWIP_IPV6 */

		} else {
			IP_ADDR4(stack_addr, 0, 0, 0, 0);
		}
	} else {
		IP_ADDR4(stack_addr, 0, 0, 0, 0);
	}
}

StackIfId IMPL_NET_STACK_AddInterface( const NetIfConfig_t *netifconfig)
{
	ip_addr_t		_ipaddr, _netmask, _gw;
    bool             isDynamicAddr = true;
    const DNSServerList_t *dnsconf;

	if (_netif == NULL ) {
		_netif = &_netif_store_;
	}

	_set_ip_addr_t(NULL, 0, &_ipaddr);
	_set_ip_addr_t(NULL, 0, &_netmask);
	_set_ip_addr_t(NULL, 0, &_gw);

    if (netifconfig) {
        if ( -1 != ipaddr_aton(netifconfig->ipaddr, &_ipaddr)) {
            isDynamicAddr = false;
			ipaddr_aton(netifconfig->netmask, &_netmask);
			ipaddr_aton(netifconfig->gw, &_gw);

#if 0
            /* for DEBUG */
            SOLID_LOG_printf("[Net] IP addr local:%s mask:%s gateway:%s\n",
                        netifconfig->ipaddr,
                            netifconfig->netmask,
                            netifconfig->gw);
#endif
        }

        dnsconf = &netifconfig->dns_server_list;

        if (dnsconf->num_of_dns > 0) {
#if LWIP_DNS
                const char*     nsaddr_str;
                ip_addr_t       ns_addr;
                int idx;

                if (dnsconf->num_of_dns > DNS_MAX_SERVERS) {
                        SOLID_LOG_printf("[Net] Warning: number of DNS servers exceeds limit.\n");
                        SOLID_LOG_printf("\tOnly first %d servers will be used.\n", DNS_MAX_SERVERS);
                }

                for (idx = 0; idx < dnsconf->num_of_dns; idx++ ) {
                        nsaddr_str = dnsconf->dns_servers[idx];
                        ipaddr_aton(nsaddr_str, &ns_addr);

                        if (idx < DNS_MAX_SERVERS) {
                                dns_setserver((u8_t)(0xff & idx), &ns_addr);
                                SOLID_LOG_printf("\t DNS%d:%s\n", (idx+1), nsaddr_str);
                        } else {
                                SOLID_LOG_printf("\t server %s was not registered.\n", nsaddr_str);

                        }
                }
#else
                SOLID_LOG_printf("[Net] this library was built with DNS option disabled.\n");
                SOLID_LOG_printf("\tAdd '#define LWIP_DNS   1(non-zero) to lwipopts.h to enable DNS features.\n");
#endif
        }

    }

    if (isDynamicAddr) {
            SOLID_LOG_printf("[Net] IP addr (auto)\n");
    }

#if NO_SYS
	netif_add(_netif, ip_2_ip4(&_ipaddr), ip_2_ip4(&_netmask), ip_2_ip4(&_gw), NULL, ethernetif_init, ethernet_input);
#else
	netif_add(_netif, ip_2_ip4(&_ipaddr), ip_2_ip4(&_netmask), ip_2_ip4(&_gw), NULL, ethernetif_init, tcpip_input);
#endif

    HOOK_NETIF_STATUC_CALLBACK(_netif);
	netif_set_default(_netif);


#if LWIP_IPV6
  	netif_create_ip6_linklocal_address(_netif, 1);
#endif	/* LWIP_IPV6 */

	if (isDynamicAddr) {
#if LWIP_DHCP
		dhcp_set_struct(_netif, &_dhcp );
#elif LWIP_AUTOIP
	autoip_set_struct(_netif, &_autoip );
#endif	/* LWIP_AUTOIP */
	}

	netif_set_up(_netif);

	if (isDynamicAddr) {
#if LWIP_DHCP
	dhcp_start(_netif);
#elif LWIP_AUTOIP	/* LWIP_DHCP */
	autoip_start(_netif);
#endif
	}

	return (StackIfId)_netif;
}

void IMPL_NET_STACK_InputPacket(pktbuf pb)
{
	solid_cs_assert(_netif);
#if ETH_PAD_SIZE
    if (pb) pbuf_header((struct pbuf*)pb, ETH_PAD_SIZE);
#endif
	if ( ERR_OK != _netif->input((struct pbuf*)pb, _netif)) {
    	pbuf_free((struct pbuf*)pb);
	}
}

void IMPL_NET_STACK_DriveProcess(void)
{
#if NO_SYS
	sys_check_timeouts();
#endif
}

void IMPL_NET_STACK_SetLinkUp(StackIfId ifid)
{
        struct  netif   *netif = (struct netif*)ifid;
#if NO_SYS
        netif_set_link_up(netif);
#else
        tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_up,netif, 0 /* non-blocking mode */);
#endif
}

void IMPL_NET_STACK_SetLinkDown(StackIfId ifid)
{
        struct  netif   *netif = (struct netif*)ifid;
#if NO_SYS
        netif_set_link_down(netif);
#else
        tcpip_callback_with_block( (tcpip_callback_fn)netif_set_link_down,netif, 0 /* non-blocking mode */);
#endif
}

#if LWIP_NETIF_STATUS_CALLBACK
static  void    __showIfStatus( struct netif *netif)
{
        SOLID_LOG_printf("[Net] IP interface status changed.\n");

#if LWIP_IPV4
        char    s4_local[IP4ADDR_STRLEN_MAX];
        char    s4_netmask[IP4ADDR_STRLEN_MAX];
        char    s4_gw[IP4ADDR_STRLEN_MAX];

        SOLID_LOG_printf("  IPv4 interface: localaddr:%s  netmask:%s gateway:%s\n"
                        , ip4addr_ntoa_r(netif_ip4_addr(netif), s4_local, sizeof(s4_local))
                        , ip4addr_ntoa_r(netif_ip4_netmask(netif), s4_netmask, sizeof(s4_netmask))
                        , ip4addr_ntoa_r(netif_ip4_gw(netif),s4_gw,sizeof(s4_gw)));

#endif /* LWIP_IPV4 */

#if LWIP_IPV6
#endif /* LWIP_IPV6 */

        int     i = 0, c = 0;
        const ip_addr_t   *ns;
        char    buf_ns[IPADDR_STRLEN_MAX];

        do {
                ns = dns_getserver(i);
                if (ns != IP_ADDR_ANY) {
                        if ( !ip_addr_isany(ns) )  {
                            if (c == 0)    SOLID_LOG_printf("   dns servers:");
                            SOLID_LOG_printf(" %s",ipaddr_ntoa_r(ns, buf_ns, sizeof(buf_ns)));
                            c++;
                        }
                        i++;
                } else {
                        SOLID_LOG_printf("\n");
                }

        } while (ns != IP_ADDR_ANY);

}

#endif /* LWIP_NETIF_STATUS_CALLBACK */

/*-----------------------------------------------------------------------------------------------------------*/

static  pktbuf      _solid_pb_alloc(size_t size);
static  void        _solid_pb_free(pktbuf pb);
static  void        _solid_pb_writeData(pktbuf pb, const void *data, size_t len);
static  const void *_solid_pb_readData(const pktbuf pb);
static  size_t      _solid_pb_getLength(const pktbuf pb);
static  int         _solid_pb_iterate(pktbuf pb, void (*fn)(void *data, size_t len, void *param), void *param);

const PacketBuffer_ops  impl_pktbuf_ops = {
    _solid_pb_alloc,
    _solid_pb_free,
    _solid_pb_writeData,
    _solid_pb_readData,
    _solid_pb_getLength,
    _solid_pb_iterate,
};

const PacketBuffer_ops*   IMPL_NET_STACK_GetPacketBufferOps(void)
{
    return &impl_pktbuf_ops;
}

static  pktbuf      _solid_pb_alloc(size_t size)
{
#if ETH_PAD_SIZE
        struct pbuf      *p;

        p = pbuf_alloc( PBUF_RAW, (u16_t)(0xffff & (size + ETH_PAD_SIZE)), PBUF_POOL );
        if (p) pbuf_header(p, -ETH_PAD_SIZE);
        return (pktbuf)p;
#else /* !ETH_PAD_SIZE */
	return	(pktbuf)pbuf_alloc( PBUF_RAW, (u16_t)(0xffff & size), PBUF_POOL );
#endif /* ETH_PAD_SIZE */
}

static  void        _solid_pb_free(pktbuf pb)
{
	pbuf_free((struct pbuf*)pb);
}

static  void        _solid_pb_writeData(pktbuf pb, const void *data, size_t len)
{
	pbuf_take((struct pbuf*)pb, data, (u16_t)(0xffff & len));
}

static  const void *_solid_pb_readData(const pktbuf pb) { return NULL;}

static  size_t      _solid_pb_getLength(const pktbuf pb)
{
	struct	pbuf*	pbuf = (struct pbuf*)pb;

	return 0xffff & (size_t)pbuf->tot_len;
}

static  int         _solid_pb_iterate(pktbuf pb, void (*fn)(void *data, size_t len, void *param), void *param)
{
	struct	pbuf	*pbuf = (struct pbuf*)pb;
	int				sum = 0;

	while (pbuf) {
		if ( pbuf->len > 0 ) {
			fn(pbuf->payload, (0xffff & (size_t)pbuf->len), param);
		}
		sum += pbuf->len;
		pbuf = pbuf->next;
	}

	return sum;
}

