#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "solid_intc.h"
#include "solid_timer.h"
#include "solid_mutex.h"
#include "solid_net_if.h"
#include "impl_net_dev.h"
#include "impl_net_stack.h"

#include "solid_net_registerapi.h"

#include "solid_log.h"
#include "solid_cs_assert.h"


const PacketBuffer_ops *pb_ops = NULL;

static SOLID_NET_DEV_INFO *dev_inf = NULL;

static void _callback_PacketReceived( pktbuf p);
static void _callback_FinishReceived(void);
static void _callback_LinkUp(void);
static void _callback_LinkDown(void);

static const SOLID_NET_DEV_CALLBACK_PKT callbackPacket = {
    _callback_PacketReceived,
    _callback_FinishReceived,
    _callback_LinkUp,
    _callback_LinkDown
};

static void _transmit_with_linkcheck(pktbuf pb);
static void _get_macaddr(uint8_t macaddr[], size_t len);

static SOLID_NET_IF_ADAPTER _adapterForStack = {
        _transmit_with_linkcheck,
        _get_macaddr
};


//#define		TRACE_QUEUE_IP_PACKET
//#define		TRACE_PRINT_PKTBUF
//#define		DEBUG_RX_FIFO
//
#define     ENABLE_POLL_LINK_TIMER

#define		INTNO_NET_SLOW_INTERRUPT	15			/* GIC Software Generated Interrupt */
#define		HEARTBEAT_INTERVAL			(500*1000)	/* 500msec interval				*/
#define		RX_PACKET_QUEUE_LENGTH		32

#if	SOLID_NET_IF_SOFT_RX_FIFO_LENGTH > 0
#define		RX_PACKET_QUEUE_LENGTH		SOLID_NET_IF_SOFT_RX_FIFO_LENGTH
#define		ENABLE_SOFT_RX_FIFO
#endif

#if defined(ENABLE_SOFT_RX_FIFO) && defined(TRACE_QUEUE_IP_PACKET)
static	void _traceIP(int __line__, int pos, const char* caption, pktbuf pb);
static	void _dumpIncomingQueue(void);
#define	TRACE_IP(pos, caption, pkt)		_traceIP(__LINE__, (pos), (caption), (pkt))
#define	DUMP_QUEUE()					_dumpIncomingQueue()
#else /* !(defined(RX_PACKET_QUEUE_LENGTH) && defined(TRACE_QUEUE_IP_PACKET)) */
#define	TRACE_IP(pos, caption, pkt)
#define	DUMP_QUEUE()
#endif /* defined(RX_PACKET_QUEUE_LENGTH) && defined(TRACE_QUEUE_IP_PACKET) */

#ifdef DEBUG_RX_FIFO
#define		STATIC
#define		PRINT_PATH()			SOLID_LOG_printf("___ %s(%d) ___\n", __func__, __LINE__ )
#else
#define		PRINT_PATH()
#endif

#define		RAISE_SLOW_INTERRUPT()		SOLID_INTC_CallSGI(INTNO_NET_SLOW_INTERRUPT,SOLID_INTC_CPUBITS_SELF)


#ifndef NET_REQUIRES_RTOS
/*
* タイマ通知時に呼び出される関数
*/
static void _heartbeat_timer_handler(void kj*param, SOLID_CPU_CONTEXT* context );

/*
*	タイマハンドラの設定情報
*/
static SOLID_TIMER_HANDLER _timer_inf = {
    NULL,						/* struct _SOLID_TIMER_HANDLER_ *pNext;     SOLID側で使用  */
    NULL,						/* struct _SOLID_TIMER_HANDLER_ *pCallQ;    SOLID側で使用  */
    0,							/* unsigned long nextticks;                 SOLID側で使用  */
    0,							/* unsigned long toticks;                   SOLID側で使用  */
    SOLID_TIMER_TYPE_INTERVAL,	/* int type;   		  タイマのタイプ(SOLID_TIMER_TYPE_XXX) */
    HEARTBEAT_INTERVAL,			/* unsigned long time;          タイマ通知をする時間(usec) */
    _heartbeat_timer_handler,	/* void (*func)(void*, SOLID_CPU_CONTEXT*);   タイマ通知時に呼び出される関数 */
    &_timer_inf					/* void* param;  タイマ通知時に呼び出される関数のパラメータ */
};

#endif	/* !NET_REQUIRES_RTOS */

#ifdef ENABLE_POLL_LINK_TIMER
/*
 *  リンクが無効な状態にあるときに１秒間隔でドライバにリンク状態の確認をさせるための
 *  タイマハンドラ
 *
 */

#define POLL_LINK_INTERVAL          (1000*1000)     // = 1sec

/*
* タイマ通知時に呼び出される関数
*/
static void _poll_link_timer_handler(void *param, SOLID_CPU_CONTEXT* context );

static void _start_poll_link(SOLID_NET_DEV_INFO *dev);
static void _stop_poll_link(SOLID_NET_DEV_INFO *dev);

#define START_POLL_LINK(dev)       _start_poll_link(dev)
#define STOP_POLL_LINK(dev)        _stop_poll_link(dev)


/*
*	タイマハンドラの設定情報
*/
static SOLID_TIMER_HANDLER _poll_link_timer_inf = {
    NULL,						/* struct _SOLID_TIMER_HANDLER_ *pNext;     SOLID側で使用  */
    NULL,						/* struct _SOLID_TIMER_HANDLER_ *pCallQ;    SOLID側で使用  */
    0,							/* unsigned long nextticks;                 SOLID側で使用  */
    0,							/* unsigned long toticks;                   SOLID側で使用  */
    SOLID_TIMER_TYPE_INTERVAL,	/* int type;   		  タイマのタイプ(SOLID_TIMER_TYPE_XXX) */
    POLL_LINK_INTERVAL,			/* unsigned long time;          タイマ通知をする時間(usec) */
    _poll_link_timer_handler,	/* void (*func)(void*, SOLID_CPU_CONTEXT*);   タイマ通知時に呼び出される関数 */
    NULL                        /* void* param;  タイマ通知時に呼び出される関数のパラメータ */
};

#else /* !ENABLE_POLL_LINK_TIMER */

#define START_POLL_LINK(dev)
#define STOP_POLL_LINK(dev)

#endif /* ENABLE_POLL_LINK_TIMER */


#ifdef ENABLE_SOFT_RX_FIFO
/*
* 割り込み発生時に呼び出される関数
*/
static int _slow_interrupt_handler(void *param, SOLID_CPU_CONTEXT* context);

/*
*	ソフトウェア割込みの設定情報
*/
static SOLID_INTC_HANDLER _sgi_inf = {
    INTNO_NET_SLOW_INTERRUPT,	/* int intno;       割り込み番号 0-15:SGI(CallSGIで生成) 16-31:PPI(各コア個別の割り込み)
													31-255:SPI(通常の割り込み) */
    1,							/* int priority;    処理優先度 0:高 - 255:(低) */
    0,							/* int config;      割り込み設定(ICFGR) 2bitのみ有効 SGI:無効 PPI:実装依存
													SPI:0X/レベルトリガ 1X:エッジトリガ */
    _slow_interrupt_handler,	/* int (*func)(void*, SOLID_CPU_CONTEXT*);  割り込み発生時に呼び出される関数 */
    &_sgi_inf					/* void* param;     割り込み発生時に関数に引き渡される第一引数 */
};
#endif	/* ENABLE_SOFT_RX_FIFO */

#ifdef ENABLE_SOFT_RX_FIFO
static	int	w_pos = 0;
static	int	r_pos = 0;

STATIC	pktbuf	_incomingQueue[ RX_PACKET_QUEUE_LENGTH ];

static	void	_enqueueIncomingPacket(pktbuf pb);
static	pktbuf	_dequeueIncomingPacket(void);
#endif	/* ENABLE_SOFT_RX_FIFO */

static	void	_onReadyToStart(void *arg);

void SOLID_NET_start_network(void)
{
	IMPL_NET_STACK_Start(_onReadyToStart, NULL /* not used so far. */);
}

/*
*	ネットワークの初期化
*/
void SOLID_NET_IF_Init(void)
{
	uint8_t	macaddr[6];

    SOLID_NET_RegisterAPIs();

	if (pb_ops == NULL) {
		pb_ops = IMPL_NET_STACK_GetPacketBufferOps();
		solid_cs_assert( pb_ops != NULL );
	}

	IMPL_NET_STACK_Init();

#ifdef ENABLE_SOFT_RX_FIFO
	int		i;
	for (i = 0; i < 0; i++ ) {
		_incomingQueue[i] = NULL;
	}
	w_pos = 0;
	r_pos = 0;
#endif	/* ENABLE_SOFT_RX_FIFO */

	/* MAC addressをボードから取得 */
	IMPL_NET_DEV_get_macaddr(macaddr, sizeof(macaddr));

    SOLID_LOG_printf("[Net] H/W MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                    macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4],
                    macaddr[5]);


	/* Ethernetドライバの初期化 . MACアドレスを指定 */
	dev_inf = IMPL_NET_DEV_init(macaddr, sizeof(macaddr), pb_ops);
	solid_cs_assert( dev_inf != NULL );

#ifndef NET_REQUIRES_RTOS
	SOLID_NET_start_network();
#endif
}

SOLID_NET_IF_ADAPTER  *SOLID_NET_IF_GetAdapter(const char *name)
{
        return  &_adapterForStack;
}

static	void	_onReadyToStart(void *arg)
{
#if !defined(NET_REQUIRES_RTOS) || defined(ENABLE_SOFT_RX_FIFO)
	int		err;
#endif /* !defined(NET_REQUIRES_RTOS) || defined(ENABLE_SOFT_RX_FIFO) */

	solid_cs_assert( dev_inf != NULL );

#ifndef NET_REQUIRES_RTOS
	/* タイマハンドラの設定 */
	err = SOLID_TIMER_RegisterTimer(&_timer_inf);
	solid_cs_assert(err == SOLID_ERR_OK );
#endif	/* !NET_REQUIRES_RTOS	*/

#ifdef ENABLE_SOFT_RX_FIFO
	/* ソフトウェア割込み(SGI)の設定 */
	err = SOLID_INTC_Register(&_sgi_inf);
	solid_cs_assert(err == SOLID_ERR_OK );
#endif	/* ENABLE_SOFT_RX_FIFO */

	dev_inf->stackIf = IMPL_NET_STACK_AddInterface( IMPL_NET_DEV_get_netconfig() );

	/* Ethernetのコールバック関数を設定*/
	dev_inf->setDeviceCallbacks( &callbackPacket );

	/* ソフトウェア割込みの許可 */
#ifdef NET_REQUIRES_RTOS
	extern int /*ER*/ ena_int(unsigned int /* INTNO */ intno); /* workaround, from kernel.h */
	ena_int( dev_inf->interruptNumber );
#else /* !NET_REQUIRES_RTOS	*/
	err = SOLID_INTC_Enable(INTNO_NET_SLOW_INTERRUPT);
	solid_cs_assert(err == SOLID_ERR_OK );
#endif	/* NET_REQUIRES_RTOS	*/

}

#ifdef ENABLE_SOFT_RX_FIFO
/*
* デバイスやタイマからの割り込みをトリガに
* タイマにより周期的にプロトコルスタックの処理を起動し、
* スタック内部のタイマ処理などをキックする
*
*/
static int _slow_interrupt_handler(void *param, SOLID_CPU_CONTEXT* context)
{
	pktbuf	pb;
	int		pkt_enq = 0;

	while (NULL != (pb = _dequeueIncomingPacket())) {
		/*
		* 	プロトコルスタックの受信処理を呼び出す
		*/
		pkt_enq++;
		IMPL_NET_STACK_InputPacket(pb);
	}

	DUMP_QUEUE();

	/*
	* スタック内部のタイマ処理などをキックする
	*/
	IMPL_NET_STACK_DriveProcess();

	return 0;
}
#endif	/* ENABLE_SOFT_RX_FIFO */


#ifndef NET_REQUIRES_RTOS
/*
* タイマにより周期的にプロトコルスタックの処理を起動し、
*/
static void _heartbeat_timer_handler(void *param, SOLID_CPU_CONTEXT* context )
{
	/* SGI発行 */
	RAISE_SLOW_INTERRUPT();
}
#endif	/* !NET_REQUIRES_RTOS */

#ifdef ENABLE_POLL_LINK_TIMER
/*
* タイマ通知時に呼び出される関数
*/
static void _poll_link_timer_handler(void *param, SOLID_CPU_CONTEXT* context )
{
        SOLID_NET_DEV_INFO  *dev_inf = (SOLID_NET_DEV_INFO*)param;

        if (dev_inf == NULL) {
                SOLID_LOG_printf("[%s] FATAL: NULL valued passed in 'param'", __func__);
                return;
        }

        if (dev_inf->pollLink()) {
            if (dev_inf->restartLink()) {
                _callback_LinkUp();
            }
        }

}

static void _start_poll_link(SOLID_NET_DEV_INFO *dev)
{
        unsigned long	status;
        status = SOLID_MUTEX_PushInt(); {
                if ( _poll_link_timer_inf.param == NULL )  {
                        _poll_link_timer_inf.param = dev;
                        SOLID_TIMER_RegisterTimer(&_poll_link_timer_inf);
                }
        } SOLID_MUTEX_PopInt(status);
}

static void _stop_poll_link(SOLID_NET_DEV_INFO *dev)
{
        unsigned long	status;
        status = SOLID_MUTEX_PushInt(); {
                _poll_link_timer_inf.param = NULL;
                SOLID_TIMER_UnRegisterTimer(&_poll_link_timer_inf);
        } SOLID_MUTEX_PopInt(status);
}

#endif  /* ENABLE_POLL_LINK_TIMER */

#if	defined(NET_INCLUDE_TEST_SERVER) && !defined(NET_REQUIRES_RTOS)
extern	void udpecho_raw_init(void);
extern	void tcpecho_raw_init(void);
#define	START_TEST_SERVERS()	do { udpecho_raw_init();tcpecho_raw_init();} while(0)
#else
#define	START_TEST_SERVERS()
#endif

/*
*	ネットワーク機能のテストコード
*/
void SOLID_NET_IF_Test(void)
{
	/* テストの実行(テストする項目はTBD) */
	START_TEST_SERVERS();
}

/*
*	ハードウェア受信割込みコールバック処理
*/

static int	_called_rx = 0;
static void _callback_PacketReceived( pktbuf p)
{
	PRINT_PATH();
	_called_rx++;
#ifdef ENABLE_SOFT_RX_FIFO
	// 一旦キューにつなぎ、ソフトウェア割込みで遅延処理をする
	solid_cs_assert(p);
	_enqueueIncomingPacket(p);
#else /* !ENABLE_SOFT_RX_FIFO */
	// プロトコルスタックのmboxに直接キューイングする
	IMPL_NET_STACK_InputPacket(p);
#endif /* ENABLE_SOFT_RX_FIFO */
}

static	int _called_rx_finished = 0;
static void _callback_FinishReceived(void)
{
	PRINT_PATH();
	_called_rx_finished++;

#ifdef ENABLE_SOFT_RX_FIFO
	/* SGI発行 */
	RAISE_SLOW_INTERRUPT();
#endif /* ENABLE_SOFT_RX_FIFO */
}

static void _callback_LinkUp(void)
{
        SOLID_LOG_printf("Link UP.\n");
        STOP_POLL_LINK(dev_inf);
        IMPL_NET_STACK_SetLinkUp(dev_inf->stackIf);
}

static void _callback_LinkDown(void)
{
        SOLID_LOG_printf("Link DOWN.\n");
        IMPL_NET_STACK_SetLinkDown(dev_inf->stackIf);
        START_POLL_LINK(dev_inf);
}

static void _transmit_with_linkcheck(pktbuf pb)
{
        if (!dev_inf->linkstat) {
                dev_inf->restartLink();
        }
        dev_inf->transmitPacket(pb);
}

static void _get_macaddr(uint8_t macaddr[], size_t len)
{
	IMPL_NET_DEV_get_macaddr(macaddr, len);
}

#ifdef ENABLE_SOFT_RX_FIFO

#ifdef TRACE_PRINT_PKTBUF
typedef struct {
	intptr_t	next;
	uint8_t		*payload;
	uint16_t	total_len;
	uint16_t	this_len;
} pbuf_snap_t;

static __inline__ void tracePBQ(char graph, int pos, pktbuf pb)
{
	pbuf_snap_t	*snap = (pbuf_snap_t*)pb;
	uint16_t	icmp_id, icmp_seq, ip_id;
	uint8_t		*dat;

	solid_cs_assert(snap);
	solid_cs_assert(snap->payload);

	dat = snap->payload;

	icmp_id = ((dat[0x26] << 8) & 0xff00) | dat[0x27];
	icmp_seq = ((dat[0x28] << 8) & 0xff00) | dat[0x29];
	ip_id = ((dat[0x12] << 8) & 0xff00) | dat[0x13];

	SOLID_LOG_printf("%c[%d](%4d) : %08x echo(%d/%d) ip(%d)\n",	graph, pos, snap->total_len, pb, icmp_id, icmp_seq, ip_id);
}

#define		TRACE_PB(mark, pos,pb)				tracePBQ((mark),(pos),(pb))
#else /* !TRACE_PRINT_PKTBUF */
#define		TRACE_PB(mark, pos,pb)
#endif /* TRACE_PRINT_PKTBUF */

/*
*	受信パケットのキューイング
*	(受信割込みハンドラから呼び出される)
*/
static	void	_enqueueIncomingPacket(pktbuf pb)
{
	unsigned long	status;

	status = SOLID_MUTEX_PushInt(); {

		solid_cs_assert(_incomingQueue[w_pos] == NULL);
		if (_incomingQueue[w_pos] == NULL) {
			_incomingQueue[w_pos] = pb;
			TRACE_PB('>', w_pos, pb);
			w_pos = (w_pos + 1) % RX_PACKET_QUEUE_LENGTH;
		}

	DUMP_QUEUE();

	} SOLID_MUTEX_PopInt(status);

	return;
}

/*
*	受信パケットをキューから取り出す
*	(ソフトウェア割込みハンドラから呼び出される)
*/
static	pktbuf	_dequeueIncomingPacket(void)
{
	unsigned long	status;
	pktbuf			pb = NULL;

	status = SOLID_MUTEX_PushInt(); {

		if (_incomingQueue[r_pos]) {
			pb = _incomingQueue[r_pos];
			_incomingQueue[r_pos] = NULL;
			TRACE_PB('<', r_pos, pb);
			r_pos = (r_pos + 1) % RX_PACKET_QUEUE_LENGTH;
		}

	} SOLID_MUTEX_PopInt(status);

	return pb;
}

#ifdef	TRACE_QUEUE_IP_PACKET

#define	ID_NULL				0xffff
#define	LEN_NULL			0xffff


typedef struct {
	int 	pos;
	const char	*caption;
	int		__line__;
	size_t	offset_len;
	size_t	offset_id;
	uint16_t	len;
	uint16_t	id;
} _traceIP_param;


static	void _treceIP_ope(void *data, size_t len, void *param)
{
	_traceIP_param *ppar = (_traceIP_param *)param;
	uint8_t		*bytes;

	if (ppar->offset_len < len ) {
		bytes = &((uint8_t*)data)[ppar->offset_len];
		ppar->len = ((uint16_t)(bytes[0] << 8) | (uint16_t)bytes[1]);	// host order = little endian
	} else {
		ppar->offset_len -= len;
	}

	if (ppar->offset_id < len ) {
		bytes = &((uint8_t*)data)[ppar->offset_id];
		ppar->id = (uint16_t)((bytes[0] << 8) | (uint16_t)bytes[1]);	// host order = little endian
	} else {
		ppar->offset_id -= len;
	}

	if (( ppar->len != LEN_NULL) && (ppar->id != ID_NULL)) {
		SOLID_LOG_printf("%s[%d] id:%d(0x%02x) len:%d(0x%02x) at line %d\n",
							ppar->caption, ppar->pos,
							ppar->id, ppar->id, ppar->len, ppar->len,
							ppar->__line__);
	}
}

static	void _traceIP(int __line__, int pos, const char* caption, pktbuf pb)
{
	_traceIP_param	par;

SOLID_LOG_printf("[%s] invoked\n",__func__);
	solid_cs_assert(pb_ops);

	par.pos = pos;
	par.caption = caption;
	par.__line__ = __line__;
	par.offset_len = 14 /*Ethrenet header*/ + 2 /* skip ver/hlen/tos/len */;
	par.offset_id = 14 /*Ethrenet header*/ + 4 /* skip ver/hlen/tos/len */;
	par.len = LEN_NULL;
	par.id = ID_NULL;

	pb_ops->iterate(pb, _treceIP_ope, &par);
}

static	void _dumpIncomingQueue(void)
{
	int		pos = 0;
	pktbuf	pb;


	for (pos = 0; pos < RX_PACKET_QUEUE_LENGTH; pos++ ) {
		pb = _incomingQueue[pos];
		if (pb) {
			_traceIP(__LINE__, pos, "-", pb);
		} else {
			SOLID_LOG_printf("-[%d] (NULL) at line %d\n", pos, __LINE__);
		}
	}

}

#endif	/* TRACE_QUEUE_IP_PACKET */
#endif	/* ENABLE_SOFT_RX_FIFO */

