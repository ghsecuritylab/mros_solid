#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "easy_arm.h"
#include "solid_intc.h"
#include "solid_net_if.h"
#include "solid_timer.h"
#include "solid_iores.h"
#include "solid_mutex.h"
#include "impl_net_dev.h"

#include "impl_rza1_eth.h"

#include "solid_cs_assert.h"

#define	_PRIVATE_DEBUG_

#define	DEFAULT_MTU		1518	/* EthernetヘッダとFCS分を含む */

#define	_roundup_(a,b)		(((a)+(b-1))/(b)*(b))
#define	_min_(a,b)			(((a)>(b))?(b):(a))
#define	_max_(a,b)			(((a)>(b))?(a):(b))

#define	MAX_TX_BUF		8
#define	MAX_RX_BUF		16

static	void _setDeviceCallbacks( const SOLID_NET_DEV_CALLBACK_PKT *cbpkt );
static	void _transmitPacket(pktbuf pb);

static	__inline__ bool	_isLinkUp(void);
static	bool	_restartLink(void);
static	bool	_pollLink(void);

static SOLID_NET_DEV_INFO	_this_dev_info = {
	NULL,	// プロトコルスタックで管理されるインタフェースID,上位側でセットされる
	_setDeviceCallbacks,
	_transmitPacket,
	0,	// interruptNumber, IORESの登録データを参照して動的に設定
	false,
	_restartLink,
	_pollLink
};

static	const PacketBuffer_ops	*_pbops = NULL;

SOLID_ADDRESS   _eth_io_base1;      /* 物理アドレス 0xE8203000 */
SOLID_ADDRESS   _eth_io_base2;      /* 物理アドレス 0xE8203400 */
SOLID_ADDRESS   _eth_io_base3;      /* 物理アドレス 0xE8204800 */

/*
*	受信/送信用ディスクリプタのサイズは16/32/64バイトが選択可能だが、
*   キャッシュのラインサイズ(32)未満のサイズの場合、キャッシュの操作中に
*   隣接するディスクリプタに意図しない書き戻しなどが発生してコントローラが
*	誤ったデータを読み込んでしまうことがあるため、副作用を避けるために
*	キャッシュのラインサイズに一致させている。
*/
typedef struct {
	uint32_t	RD[3];
	uint8_t		*va_buff;	/* pad領域にRD[2]の仮想アドレスを格納 */
	uint32_t	_pad[4];	/* 合計32byte */
} RX_DESC;

typedef struct {
	uint32_t	TD[3];
	uint8_t		*va_buff;	/* pad領域にTD[2]の仮想アドレスを格納 */
	SOLID_ADDRESS	pa_this;	/* このディスクリプタの物理アドレス */
	uint32_t	seqno;		/* 書込まれた順序を示す通し番号	*/
	uint32_t	_pad[2];	/* 合計32byte */
} TX_DESC;

#define		RZETH_BUF_ALIGN			32
#define		ARM_CACHE_LINE_SIZE		32

#define		BUFF_ALIGN_SIZE		_max_(RZETH_BUF_ALIGN, ARM_CACHE_LINE_SIZE)

#define		FRAME_BUF_SIZE		_roundup_(DEFAULT_MTU, BUFF_ALIGN_SIZE)

typedef struct {
	uint8_t	_firebreak1_[BUFF_ALIGN_SIZE];
	uint8_t	tx_buffers[MAX_TX_BUF][FRAME_BUF_SIZE];
	TX_DESC	tx_desc[MAX_TX_BUF];
	uint8_t	rx_buffers[MAX_RX_BUF][FRAME_BUF_SIZE];
	RX_DESC	rx_desc[MAX_RX_BUF];
	uint8_t	_firebreak2_[BUFF_ALIGN_SIZE];
} DMA_BUF_AREA;

static	DMA_BUF_AREA	_dma_buf	__attribute__((aligned(BUFF_ALIGN_SIZE)));

SOLID_ADDRESS		pa_tx_desc_top = 0;
SOLID_ADDRESS		pa_tx_desc_tail = 0;
SOLID_ADDRESS		pa_rx_desc_top = 0;
SOLID_ADDRESS		pa_rx_desc_tail = 0;

#define	tx_desc			_dma_buf.tx_desc
#define	rx_desc			_dma_buf.rx_desc
#define	tx_buffers		_dma_buf.tx_buffers
#define	rx_buffers		_dma_buf.rx_buffers

static	int		_rx_pos;
static	int		_tx_pos;

static void 	phy_reset(void);
static int		phy_is_full_duplex(void);
static void		phy_update_status(void);
static void		phy_kick_autonego(void);

static	void	init_tx_buffer(void);
static	void	init_rx_buffer(void);

#define	HOLD_USEC(usec)		SOLID_TIMER_WaitNsec((usec)*1000)

#ifdef	_PRIVATE_DEBUG_
#define	MAX_DEBUG_RECORDS		30

static	void	putRingLog( int __line__, uint32_t w1, uint32_t w2, uint32_t w3, uint32_t w4 );
#define	TRACE( w1, w2, w3, w4 )			putRingLog(__LINE__, (uint32_t)(w1),(uint32_t)(w2),(uint32_t)(w3),(uint32_t)(w4))

typedef	struct _RegTrace {
	int			pos;
	uint32_t	regVal[MAX_DEBUG_RECORDS];
} RegTrace;

static	RegTrace	_traceEESR = {0,};

#define	INT_NORMAL_MASK		(RZETH_INT_TWB_1 | RZETH_INT_TWB_0 | RZETH_INT_TC_1 | RZETH_INT_TC_0 | RZETH_INT_FR )

#define	INT_FILTER_RX	(RZETH_INT_FR | RZETH_INT_RDE | RZETH_INT_RMAF | RZETH_INT_ROC | RZETH_INT_RABT |\
						RZETH_INT_RRF | RZETH_INT_RTLF | RZETH_INT_RTSF | RZETH_INT_PRE | RZETH_INT_CERF | \
						RZETH_INT_RFCOF | RZETH_INT_RFOF )

#ifdef	RZETH_REDUCED_INT
#define	INT_MASK_SETTING		INT_FILTER_RX		/* 受信関連の割込みのみ設定 */
#else
#define	INT_MASK_SETTING		RZETH_INT_ALL 		/* 定義されている割込みをすべて有効にする(調査/デバッグ目的など) */
#endif

#define	LOG_EESR(val)		do {\
	if ((val)& ~INT_NORMAL_MASK) { \
		_traceEESR.regVal[_traceEESR.pos] = (val) & ~INT_NORMAL_MASK; \
		_traceEESR.pos = (_traceEESR.pos + 1) % MAX_DEBUG_RECORDS; \
	} \
} while(0)

#else	/* !_PRIVATE_DEBUG_	*/
#define	TRACE( w1, w2, w3, w4 )
#define	LOG_EESR(val)
#endif	/* !_PRIVATE_DEBUG_	*/

static	int	handleInterrupts(void *param, SOLID_CPU_CONTEXT *context);

static SOLID_INTC_HANDLER	_intr_handler_info = {
	0, // 割込み番号、_this_dev_info.interruptNumberに格納されている値を動的に設定する
	0,
	0,
	handleInterrupts,
	&_this_dev_info
};

static	void _stub_callback_received(pktbuf);
static	void _stub_callback_finished(void);
static	void _stub_callback_linkup(void);
static	void _stub_callback_linkdown(void);

static	fn_received _callback_received = _stub_callback_received;
static	fn_finished _callback_receiving_finished = _stub_callback_finished;
static	fn_linkup _callback_linkup = _stub_callback_linkup;
static	fn_linkdown _callback_linkdown = _stub_callback_linkdown;

SOLID_NET_DEV_INFO *IMPL_NET_DEV_init(const uint8_t macaddr[], size_t addrlen, const PacketBuffer_ops *pbops)
{
	int	err;
	SOLID_IORES_INFO	iores_eth_reg;

	_pbops = pbops;	/* パケット用バッファの操作関数 */

	/* コントローラのレジスタをマッピング */
	err = SOLID_IORES_Use("ETH1");
	solid_cs_assert( err == SOLID_ERR_OK );

	err = SOLID_IORES_Use("ETH2");
	solid_cs_assert( err == SOLID_ERR_OK );

	err = SOLID_IORES_Use("ETH3");
	solid_cs_assert( err == SOLID_ERR_OK );

	SOLID_IORES_GetInfo("ETH1", &iores_eth_reg);
	solid_cs_assert( iores_eth_reg.addr );
	_eth_io_base1 = iores_eth_reg.addr;

	_this_dev_info.interruptNumber = iores_eth_reg.extra;

	SOLID_IORES_GetInfo("ETH2", &iores_eth_reg);
	solid_cs_assert( iores_eth_reg.addr );
	_eth_io_base2 = iores_eth_reg.addr;

	SOLID_IORES_GetInfo("ETH3", &iores_eth_reg);
	solid_cs_assert( iores_eth_reg.addr );
	_eth_io_base3 = iores_eth_reg.addr;


	/* コントローラリセット*/
		/* クロック設定 ET_TXCLK,ET_RXCLK (impl_boot) */

		/* E-DMAC起動 */
	*RZETH_EDSR0 = 0x3;		/* EDSR.ENT=1, ENR=1 */
	__DMB();

	/* ソフトウェアリセット */
	*RZETH_EDMR0 = 0x3; 	/* EDMR.SWRR<-1, .SWRT<-1 */
	__DMB();

	/* ソフトウェアリセット後, 内部バスクロック Bck 64cyclesの間レジスタにアクセス禁止 */
	HOLD_USEC(1);

	/* ディスクリプタとバッファの初期化 */
	init_tx_buffer();
	init_rx_buffer();

	while( *RZETH_EDMR0 & 0x03 );	/* EDMR.SWRR,SWRTが0になるまで待ち合わせ */

	/* PHY 初期化（要タイミング確認）*/

	phy_reset();
	phy_update_status();

	/* ディスクリプタリング登録 */
	*RZETH_TDLAR0 = pa_tx_desc_top; 	/* TDLAR <- TX_DESC 先頭 */
	*RZETH_TDFAR0 = pa_tx_desc_top; 	/* TDFAR <- TX_DESC 先頭 */
	*RZETH_TDFXR0 = pa_tx_desc_tail; 	/* TDFXR <- TX_DESC 最終 */
	*RZETH_TDFFR0 = 1;				/* TDFFR.TDLF <- '1'(=最終)  */

	*RZETH_RDLAR0 = pa_rx_desc_top; 	/* RDLAR <- RX_DESC 先頭 */
	*RZETH_RDFAR0 = pa_rx_desc_top; 	/* RDFAR <- RX_DESC 先頭 */
	*RZETH_RDFXR0 = pa_rx_desc_tail; 	/* RDFXR <- RX_DESC 最終 */
	*RZETH_RDFFR0 = 1;				/* RDFFR.RDLF <- '1'(=最終)  */

	__DMB();

	/* 各種レジスタ設定 */



	{
		/* CAMエントリテーブルの初期化 */
		int		n;
		volatile	uint32_t	*adr_hi = RZETH_TSU_ADRH0;
		volatile	uint32_t	*adr_lo = RZETH_TSU_ADRL0;

		*RZETH_TSU_TEN = 0;	/* CAMエントリをすべて無効にする */
		__DMB();

		for (n = 0; n < RZETH_MAX_CAM_ENTRY; n++) {

			while(*RZETH_TSU_ADSBSY ){
				HOLD_USEC(1);
			}

			*adr_hi = 0;
			*adr_lo = 0;
			__DMB();

			adr_hi += 2;
			adr_lo += 2;
		}

	}

	solid_cs_assert( sizeof(TX_DESC) == sizeof(RX_DESC ));
	solid_cs_assert(( sizeof(TX_DESC) == 32 ) || (sizeof(TX_DESC) == 64 ));
	*RZETH_EDMR0 = ( 0x40 | ((0x3 & (sizeof(TX_DESC)/32)) << 4));/* EDMR.DE<-1 (little endian), .DL[1:0]<- ディスクリプタ長 */
	*RZETH_EESIPR0 = INT_MASK_SETTING;	/* EESIPR: 割込みマスクの設定 */

		/* TRSCER:*/

	*RZETH_TFTR0 = 0;	/* TFTR: 送信FIFO thresh = store&forward */
	*RZETH_RMCR0 = 1;	/* RMCR: 受信方式(連続) -> ソフトと同期せずに連続で受信する方式 RRはクリアされない */
	*RZETH_RPADIR0 = 0; /* RPADIR: 受信時のパディング設定 => パディングなし */

	*RZETH_FDR0 = /* TX FIFO */(((2048/*bytes*//256) - 1) << 8 ) | /* RX FIFO */ (((4096/*bytes*//256) - 1));
	*RZETH_FCFTR0 = ((24/*frames*/ - 1) << 16 ) | ((4096/*bytes*//256)-1); /* FCFTR: 受信FIFO thresh */

		/* ECMR: E-MACモード*/
	*RZETH_ECMR0 = 	0x2000	/* MCT: CAMエントリテーブル使用 */ |
					(phy_is_full_duplex()?0x2:0) /* DM: 全二重設定 */
					;

	*RZETH_ECSIPR0 = 0x11;	/* ECSIPR: E-MAC割込み許可設定 PFROIP&ICDIP <- 1 */

	/*
	*	MACアドレスをコントローラに設定
	*/
	solid_cs_assert( addrlen == 6 );
	*RZETH_MAHR0 = ((macaddr[0] << 24) | (macaddr[1] << 16) | (macaddr[2] << 8) | macaddr[3] );	/* MAHR: MACアドレス設定 */
	*RZETH_MALR0 = ((macaddr[4] << 8) | macaddr[5] );	/* MALR: MACアドレス設定 */

	*RZETH_RFLR0 = FRAME_BUF_SIZE; /* RFLR: 受信フレーム長上限 */

	*RZETH_APR0 = 1;		/* APR: 自動PAUSE TIME設定 => 512bit x 1 bit-time (where 1bit-time= 10ns@100Mbps, 100ns@10Mbps */
	*RZETH_MPR0 = 0;		/* MPR: 手動PAUSE TIME設定 => フロー制御なし*/
	*RZETH_TPAUSER0 = 1; 	/* TPAUSER: PAUSEフレーム再送回数上限 => 1回*/

	__DMB();

	/*
	*	ホスト側の割込みを設定
	*/

	_intr_handler_info.intno = _this_dev_info.interruptNumber;
	SOLID_INTC_Register( &_intr_handler_info );

	SOLID_INTC_Enable(_this_dev_info.interruptNumber);

	/* 起動 */
		/* EDTRR.TR = 3;(ここではしなくてよい) */
	*RZETH_EDRRR0 = 1; /* EDRRR.RR <- 1, 受信要求 */
	*RZETH_ECMR0 |= 0x60; /* ECMR.TE=1, .RE=1, 送受信許可 */

	__DMB();

	return &_this_dev_info;
}

static	bool _restartLink(void)
{
	/* MACの動作を一時停止 */
	*RZETH_ECMR0 &= ~0x60; /* ECMR.TE=0, .RE=0, 送受信許可 */
	__DMB();
	*RZETH_EDTRR0 &= ~3;	/* EDTRR.TR <- [00] 送信停止 */
	*RZETH_EDRRR0 &= ~1; 	/* EDRRR.RR <- 0, 受信動作停止 */
	__DMB();

	SOLID_INTC_Disable(_this_dev_info.interruptNumber);

	/* PHY 初期化 */	// autonegoだけすればよい？
	phy_reset();

	if (_isLinkUp()) {
		/* ECMR.DM: 全二重設定 */
		*RZETH_ECMR0 = 	(*RZETH_ECMR0 & ~2) | (phy_is_full_duplex()?0x2:0);
		__DMB();
	}

	SOLID_INTC_Enable(_this_dev_info.interruptNumber);

	/* 起動 */
	*RZETH_EDRRR0 = 1; /* EDRRR.RR <- 1, 受信要求 */
	*RZETH_ECMR0 |= 0x60; /* ECMR.TE=1, .RE=1, 送受信許可 */

	__DMB();

	return _isLinkUp();
}

static	bool	_pollLink(void)
{
					phy_update_status();
					return _isLinkUp();
}


static	int	handleInterrupts(void *param, SOLID_CPU_CONTEXT *context)
{
	uint32_t	eesr = *RZETH_EESR0;

	solid_cs_assert(_pbops);

	LOG_EESR(eesr);
	TRACE(eesr,0,0,0);

	if (!_isLinkUp()) {
		phy_update_status();
		if (_isLinkUp())	_callback_linkup();	// リンクが確立した事を通知
	}

	do {
		if (eesr & INT_FILTER_RX) {
			/*
			*	受信ディスクリプタからデータを読み出し
			*/
			RX_DESC	*rd = &rx_desc[_rx_pos];
			size_t		l;
			pktbuf		pb;
			int			num = 0;

			SOLID_MEM_CACHE_Invalidate((SOLID_ADDRESS)rd, sizeof(RX_DESC));

			while (0 == (rd->RD[0] & 0x80000000 /* RACT */)) {

				TRACE(rd,rd->RD[0],rd->RD[1],rd->RD[2]);

				/* エラーが発生していなければコールバック関数を呼び出す */
				if ( 0 == (rd->RD[0] & 0x08000000 /* RFE */ ) ){
					l = rd->RD[1] & 0xffff;	/* RDL */

					SOLID_MEM_CACHE_Invalidate((SOLID_ADDRESS)rd->va_buff, l);

					pb = _pbops->alloc(l);
					if (pb) {
						_pbops->writeData(pb, rd->va_buff, l);
						_callback_received(pb);
						num++;
					}

				}
				rd->RD[1] &= 0xffff0000;	/* RDLクリア */
				rd->RD[0] = ((rd->RD[0] & 0x40000000) | 0x80000000); /* RACT = '1',RDLEは落とさないこと */

				SOLID_MEM_CACHE_Clean((SOLID_ADDRESS)rd,sizeof(RX_DESC));

				TRACE(rd,rd->RD[0],rd->RD[1],rd->RD[2]);

				_rx_pos = (_rx_pos + 1) % MAX_RX_BUF;
				rd = &rx_desc[_rx_pos];
				SOLID_MEM_CACHE_Invalidate((SOLID_ADDRESS)rd, sizeof(RX_DESC));
			}

			/*
			*	すべて読み出したことをスタック側に通知する
			*/
			if (num)	_callback_receiving_finished();
		}

		if (eesr & RZETH_INT_RDE) {
			/*
			*	受信ディスクリプタが枯渇している場合、リカバリが必要
			*/
			TRACE(*RZETH_TDLAR0, *RZETH_TDFAR0, *RZETH_TDFXR0, *RZETH_TDFFR0 );

			if (*RZETH_EDRRR0) {
				*RZETH_EDRRR0 = 0; /* EDRRR.RR <- 0, 受信要求 */
				__DMB();
			}

			init_rx_buffer();	/* 受信ディスクリプタ全体を再構築 */

			/*
			* 	レジスタに初期値を再設定
			*/
			*RZETH_RDLAR0 = pa_rx_desc_top; 	/* RDLAR <- RX_DESC 先頭 */
			*RZETH_RDFAR0 = pa_rx_desc_top; 	/* RDFAR <- RX_DESC 先頭 */
			*RZETH_RDFXR0 = pa_rx_desc_tail; 	/* RDFXR <- RX_DESC 最終 */
			*RZETH_RDFFR0 = 1;				/* RDFFR.RDLF <- '1'(=最終)  */

			__DMB();
			TRACE(*RZETH_TDLAR0, *RZETH_TDFAR0, *RZETH_TDFXR0, *RZETH_TDFFR0 );
		}

		if (eesr & RZETH_INT_ECI ) {
			// EESR.ECI , ECSR.ICDが立っている場合にはケーブルが抜かれたなどの理由で
			// リンクが切れている事を示している
			uint32_t	ecsr;

			ecsr = *RZETH_ECSR0;
			if ( ecsr & RZETH_ECSR_ICD ) {
					phy_update_status();
					if (!_isLinkUp())	_callback_linkdown();	// リンクが切れていることを通知
			}
			*RZETH_ECSR0 = ecsr; // E-MACステータス割込み要因のクリア
			__DMB();
		}

		if (eesr & (SHETH_INT_CND | SHETH_INT_DLC)) {
			phy_update_status();
			if (_isLinkUp()) {
				phy_kick_autonego();
			}
		}

		/*
		*	割り込み要因のクリア
		*/
		*RZETH_EESR0 |= (RZETH_INT_ALL | SHETH_INT_CND | SHETH_INT_CND);
		__DMB();
		eesr = *RZETH_EESR0;
	} while (eesr & (RZETH_INT_ALL | SHETH_INT_CND | SHETH_INT_CND));


	/*
	*	受信動作が停止している場合には再開する
	*/
	if (*RZETH_EDRRR0 == 0 ) {
		*RZETH_EDRRR0 = 1; /* EDRRR.RR <- 1, 受信要求 */
		__DMB();
		TRACE(eesr,*RZETH_EDRRR0,0,0);
	}

	return 0;
}

static	void	init_tx_buffer(void)
{
	TX_DESC				*va_txd;
	uint8_t				*va_buff;
	SOLID_ADDRESS		pa_buffer;
	int					i;

	/*	送信ディスクリプタの初期化 */
	for ( i = 0; i < MAX_TX_BUF; i++ ) {
		va_txd = &tx_desc[i];
		va_buff = tx_buffers[i];

		SOLID_MEM_VA2PA((SOLID_ADDRESS)va_buff, &pa_buffer);	/* to refactor: 毎回必要か？ */

		va_txd->TD[0] = 0x30000000; /* TFP[1:0] = '11' */
		va_txd->TD[1] = 0; 			/* データ長　*/
		va_txd->TD[2] = pa_buffer; 	/* TBA <- バッファの物理アドレス */

		va_txd->va_buff = va_buff;	/* ホスト側からのアクセス用バッファの論理アドレス */

		SOLID_MEM_VA2PA((SOLID_ADDRESS)va_txd, &va_txd->pa_this);
		va_txd->seqno = 0;
	}

	tx_desc[MAX_TX_BUF-1].TD[0] |= 0x40000000; /* 最後だけ最終フラグを設定 */

	/* コントローラが参照できる様にキャッシュを書き戻して反映 */
	SOLID_MEM_CACHE_Clean((SOLID_ADDRESS)tx_desc, sizeof(tx_desc));

	/* 先頭と末尾の物理アドレスを取得 */
	if (( pa_tx_desc_top == 0 ) || ( pa_tx_desc_tail == 0)) {
		SOLID_MEM_VA2PA((SOLID_ADDRESS)&tx_desc[0], &pa_tx_desc_top);
		SOLID_MEM_VA2PA((SOLID_ADDRESS)&tx_desc[MAX_TX_BUF-1], &pa_tx_desc_tail);
	}

	_tx_pos = 0;
}

static	void	init_rx_buffer(void)
{
	RX_DESC				*va_rxd;
	uint8_t				*va_buff;
	SOLID_ADDRESS		pa_buffer;
	int					i;

	/*	受信ディスクリプタの初期化 */
	for ( i = 0; i < MAX_RX_BUF; i++ ) {
		va_rxd = &rx_desc[i];
		va_buff = rx_buffers[i];

		SOLID_MEM_VA2PA((SOLID_ADDRESS)va_buff, &pa_buffer);	/* to refactor: 毎回必要か？ */

		va_rxd->RD[0] = 0x80000000; /* RACT = '1' */
		va_rxd->RD[1] = ((FRAME_BUF_SIZE) << 16); /* バッファ長　*/
		va_rxd->RD[2] = pa_buffer; 	/* TBA <- バッファの物理アドレス */

		va_rxd->va_buff = va_buff;	/* ホスト側からのアクセス用バッファの論理アドレス */
	}

	rx_desc[MAX_RX_BUF-1].RD[0] |= 0x40000000; /* 最後だけ最終フラグを設定 */

	/* コントローラが参照できる様にキャッシュを書き戻して反映 */
	SOLID_MEM_CACHE_Clean((SOLID_ADDRESS)rx_desc, sizeof(rx_desc));

	/* 先頭と末尾の物理アドレスを取得 */
	if (( pa_rx_desc_top == 0 ) || ( pa_rx_desc_tail == 0)) {
		SOLID_MEM_VA2PA((SOLID_ADDRESS)&rx_desc[0], &pa_rx_desc_top);
		SOLID_MEM_VA2PA((SOLID_ADDRESS)&rx_desc[MAX_RX_BUF-1], &pa_rx_desc_tail);
	}

	_rx_pos = 0;
}

static	void _setDeviceCallbacks( const SOLID_NET_DEV_CALLBACK_PKT *cbpkt )
{
	unsigned long stat = SOLID_MUTEX_PushInt();
	{
		_callback_received = cbpkt->fn_r;
		_callback_receiving_finished = cbpkt->fn_f;
		_callback_linkup = cbpkt->fn_lup;
		_callback_linkdown = cbpkt->fn_ldown;
	}
	SOLID_MUTEX_PopInt(stat);

	phy_update_status();
	if (_isLinkUp()) {
			_callback_linkup();
	} else {
			_callback_linkdown();
	}
}

typedef	struct {
	size_t		pos;
	TX_DESC		*desc;
} TX_BUFF_LOC;

static	void _setTXData(void *data, size_t len, void *param)
{
	TX_BUFF_LOC	*loc = (TX_BUFF_LOC*)param;

	solid_cs_assert( loc );
	TX_DESC		*desc = loc->desc;

	memcpy(&desc->va_buff[loc->pos], data, len);
	loc->pos += len;
}


static	void _transmitPacket(pktbuf pb)
{
	TX_DESC			*desc;
	TX_BUFF_LOC		loc;
	unsigned long	stat;
	static uint32_t		seq = 1;


	solid_cs_assert(_pbops);

	stat = SOLID_MUTEX_PushInt();
	{

		desc = &tx_desc[_tx_pos];
		SOLID_MEM_CACHE_Invalidate((SOLID_ADDRESS)desc, sizeof(TX_DESC));
		TRACE(_tx_pos,desc,desc->TD[0],desc->TD[1]);
		__DMB();
		TRACE(*RZETH_TDLAR0, *RZETH_TDFAR0, *RZETH_TDFXR0, *RZETH_TDFFR0 );

		if (0 == (desc->TD[0] & 0x80000000 )) {

			/*
			*	スタックのバッファからコントローラが参照するエリアに
			*   データをコピーする
			*/

			loc.pos = 0;
			loc.desc = desc;

			_pbops->iterate(pb, _setTXData, &loc);

			if ( loc.pos > 0 ) {

				/*
				*	送信ディスクリプタ設定
				*/
				desc->TD[0] &= 0x40000000; /* TDLE以外をクリア */
				desc->TD[1] = 0xffff0000 & (loc.pos << 16 );

				desc->TD[0] |= 0xb0000000;	/* TACT=1, TFP[] = '11' */

				desc->seqno = seq++;

				SOLID_MEM_CACHE_Clean((SOLID_ADDRESS)desc,sizeof(TX_DESC));
				SOLID_MEM_CACHE_Clean((SOLID_ADDRESS)desc->va_buff,loc.pos);

				TRACE(_tx_pos,desc,desc->TD[0],desc->TD[1]);

				/*
				*	コントローラに送信要求
				*/

				*RZETH_EDTRR0 = 3;	/* TR[1:0] <- 0b11 */
				__DMB();

				_tx_pos = (_tx_pos + 1) % MAX_TX_BUF;
			}
		} else {
			TRACE(desc,desc->TD[0],desc->TD[1],desc->TD[2]);
		}
	}
	SOLID_MUTEX_PopInt(stat);

	return;
}

static	void _stub_callback_received(pktbuf pb) {
	solid_cs_assert(_pbops);
	_pbops->free(pb);
}

static	void _stub_callback_finished(void) { }
static	void _stub_callback_linkup(void) { }
static	void _stub_callback_linkdown(void) { }

/*************************************************************************/

#define	MII_REG_BASIC_CONTROL			0
	#define	MII_CONTROL_SOFT_RESET			0x8000
	#define	MII_CONTROL_AUTONEGO_ENABLE		(1<<12)
	#define	MII_CONTROL_RESTART_AUTONEGO	(1<<9)

#define	MII_REG_BASIC_STATUS			1
	#define	MII_STATUS_100BASE_TX_FD		0x4000
	#define	MII_STATUS_100BASE_TX_HD		0x2000
	#define	MII_STATUS_10BASE_TX_FD			0x1000
	#define	MII_STATUS_10BASE_TX_HD			0x0100
	#define	MII_STATUS_AUTONEGO_COMPLETE	0x0020
	#define	MII_STATUS_AUTONEGO_ABILITY		(1<<3)
	#define	MII_STATUS_LINK_STATUS			(1<<2)

#define	MII_REG_LINK_PARTNER_ABLITY		5
	#define	MII_LPA_100BASE_TX_FD		0x4000
	#define	MII_LPA_100BASE_TX_HD		0x2000
	#define	MII_LPA_10BASE_TX_FD			0x1000
	#define	MII_LPA_10BASE_TX_HD			0x0100
	#define	MII_LPA_SELECTOR_MASK			0x1f
	#define	MII_LPA_SELECTOR_IEEE802_3		0b00001
	#define	MII_LPA_ISVALID(reg)			(((reg) & MII_LPA_SELECTOR_MASK) == MII_LPA_SELECTOR_IEEE802_3)

static	void		_mii_write_reg(int regno, uint16_t val);
static	uint16_t	_mii_read_reg(int regno);

static	uint16_t	_last_phy_status = 0;

static	bool	_isLinkUp(void)
{
		return ((_last_phy_status & MII_STATUS_LINK_STATUS) != 0);
}

static void 	phy_reset(void)
{
	//int			retries = 10;
	uint16_t	status = 0;
	int			count;

	_mii_write_reg(MII_REG_BASIC_CONTROL, MII_CONTROL_SOFT_RESET);
	count = 20; /* = 2000msec  total */
	while ( count > 0 ) {
		HOLD_USEC( 100 * 1000);
		status = _mii_read_reg(MII_REG_BASIC_STATUS);
		if ((status != 0xffff) && ( status & MII_STATUS_AUTONEGO_COMPLETE)) {
			break;
		}
		count--;
	}

	if (status != 0xffff) {
		_last_phy_status = status;
	}
}

static int	phy_is_full_duplex(void)
{
	uint16_t	link_ability = 0;

	link_ability = _mii_read_reg(MII_REG_LINK_PARTNER_ABLITY);

	if (MII_LPA_ISVALID(link_ability)) {
		return (link_ability & (MII_LPA_100BASE_TX_FD | MII_LPA_10BASE_TX_FD));
	} else {
		return (0!=0);	/* false */
	}
}

static void		phy_update_status(void)
{
	uint16_t	bsr = _mii_read_reg(MII_REG_BASIC_STATUS);

	if (bsr != 0xffff) {
		_last_phy_status = bsr;
		_this_dev_info.linkstat = ((_last_phy_status & MII_STATUS_LINK_STATUS) != 0);
	}
}

static void		phy_kick_autonego(void)
{
	uint16_t	bcr = _mii_read_reg(MII_REG_BASIC_CONTROL);
	bcr |= (MII_CONTROL_AUTONEGO_ENABLE | MII_CONTROL_RESTART_AUTONEGO );
	_mii_write_reg(MII_REG_BASIC_CONTROL, bcr);
}

/*-----------------------------------------------------------------------*/

#define	PHYAD		0		/* configuration (=ボードの回路設計)によって異なる */

static void	_mii_preamble(void);
static void	_mii_ST(void);
static void _mii_OP_READ(void);
static void	_mii_OP_WRITE(void);
static void	_mii_phy_address(uint8_t phyadr);
static void	_mii_reg_address(uint8_t regadr);
static void	_mii_TA(void);
static void	_mii_put_data16(uint16_t val);
static uint16_t	_mii_get_data16(void);
static void	_mii_X(void);
static void	_mii_Z0(void);

static void	_mii_write_reg(int regno, uint16_t val)
{
	_mii_preamble();
	_mii_ST();
	_mii_OP_WRITE();
	_mii_phy_address(PHYAD);
	_mii_reg_address(regno & 0x1ff);
	_mii_TA();
	_mii_put_data16(val);
	_mii_X();
}

static	uint16_t _mii_read_reg(int regno)
{
	uint16_t	val = 0;

	_mii_preamble();
	_mii_ST();
	_mii_OP_READ();
	_mii_phy_address(PHYAD);
	_mii_reg_address(regno & 0x1ff);
	_mii_Z0();
	val = _mii_get_data16();
	_mii_Z0();

	return val;
}

static void	_mii_write_bit(uint8_t bit);
static void	_mii_write_bits(uint16_t dat, int bits);
static uint8_t _mii_read_bit(void);

static void	_mii_preamble(void)
{
	int		i;
	for (i = 0; i < 32; i++ ) {
		_mii_write_bit(1);
	}
}

static void	_mii_ST(void)
{
	_mii_write_bit(0);
	_mii_write_bit(1);
}

static void _mii_OP_READ(void)
{
	_mii_write_bit(1);
	_mii_write_bit(0);
}

static void	_mii_OP_WRITE(void)
{
	_mii_write_bit(0);
	_mii_write_bit(1);
}

static void	_mii_phy_address(uint8_t phyadr)
{
	_mii_write_bits(phyadr,5);
}

static void	_mii_reg_address(uint8_t regadr)
{
	_mii_write_bits(regadr,5);
}

static void	_mii_TA(void)
{
	_mii_write_bit(1);
	_mii_write_bit(0);
}

static void	_mii_put_data16(uint16_t val)
{
	_mii_write_bits(val,16);
}

static uint16_t	_mii_get_data16(void)
{
	uint16_t	val = 0;
	int			b;

	for (b = 0; b < 16; b++ ) {
		val <<= 1;
		val |= (_mii_read_bit()?1:0);
	}

	return val;
}

#define	RZETH_PIR_MDI			0x8
#define	RZETH_PIR_MDO			0x4
#define	RZETH_PIR_MMD			0x2
#define	RZETH_PIR_MDC			0x1

#define	RZETH_MII_SMI_MASK		(RZETH_PIR_MDO|RZETH_PIR_MMD|RZETH_PIR_MDC)

#define	PHY_8710_TCLKL			(200)	// nsec
#define	PHY_8710_TCLKH			(200)	// nsec
#define	PHY_8710_TSETUP			(10)	// nsec
#define	PHY_8710_THOLD			(10)	// nsec

#define	HOLD_CLK_LO()			SOLID_TIMER_WaitNsec(PHY_8710_TCLKL)
#define	HOLD_CLK_HI()			SOLID_TIMER_WaitNsec(PHY_8710_TCLKH)
#define	HOLD_DAT_SETUP()		SOLID_TIMER_WaitNsec(PHY_8710_TSETUP)
#define	HOLD_DATA()				SOLID_TIMER_WaitNsec(PHY_8710_THOLD)

static void	_mii_X(void)
{
	*RZETH_PIR0 &= ~(RZETH_PIR_MMD | RZETH_PIR_MDC);
	__DMB();
	HOLD_CLK_LO();
}

static void	_mii_Z0(void)
{
	/*
	*	MMD=0,MDC=0
	*/
	*RZETH_PIR0	= (*RZETH_PIR0 & ~(RZETH_PIR_MMD | RZETH_PIR_MDC));
	__DMB();
	HOLD_DAT_SETUP();

	/*
	*	MMD=0,MDC=1
	*/
	*RZETH_PIR0	= ((*RZETH_PIR0 & ~(RZETH_PIR_MMD)) | RZETH_PIR_MDC);
	__DMB();
	HOLD_CLK_HI();

	/*
	*	MMD=0,MDC=0
	*/
	*RZETH_PIR0	= (*RZETH_PIR0 & ~(RZETH_PIR_MMD | RZETH_PIR_MDC));
	__DMB();
	HOLD_CLK_LO();
}

static void	_mii_write_bit(uint8_t bit)
{
	/*
	*	MMD=1,MDO=data,MDC=0
	*/
	*RZETH_PIR0	= ((*RZETH_PIR0 & ~RZETH_MII_SMI_MASK) | RZETH_PIR_MMD | ((bit)?RZETH_PIR_MDO:0));
	__DMB();
	HOLD_DAT_SETUP();

	/*
	*	MMD=1,MDO=data,MDC=1
	*
	*/
	*RZETH_PIR0	= ((*RZETH_PIR0 & ~RZETH_MII_SMI_MASK) | RZETH_PIR_MMD | ((bit)?RZETH_PIR_MDO:0) | RZETH_PIR_MDC);
	__DMB();
	HOLD_CLK_HI();

	/*
	*	MMD=1,MDO=data,MDC=0
	*/
	*RZETH_PIR0	= ((*RZETH_PIR0 & ~RZETH_MII_SMI_MASK) | RZETH_PIR_MMD | ((bit)?RZETH_PIR_MDO:0));
	__DMB();
	HOLD_CLK_LO();
}

static void	_mii_write_bits(uint16_t dat, int bits)
{
	uint16_t	mask = 1 << (bits - 1);

	do {
		_mii_write_bit( (dat & mask)?1:0 );
		mask >>= 1;
	} while (mask >= 1);
}

static uint8_t _mii_read_bit(void)
{
	uint8_t	ret = 0;

	/*
	*	MMD=0,MDC=1
	*/
	*RZETH_PIR0	= ((*RZETH_PIR0 & ~(RZETH_PIR_MMD)) | RZETH_PIR_MDC);
	__DMB();
	HOLD_CLK_HI();

	ret = ((*RZETH_PIR0 & RZETH_PIR_MDI)?1:0);

	/*
	*	MMD=0,MDC=0
	*/
	*RZETH_PIR0	= (*RZETH_PIR0 & ~(RZETH_PIR_MMD | RZETH_PIR_MDC));
	__DMB();
	HOLD_CLK_LO();

	return ret;
}

#ifdef	_PRIVATE_DEBUG_

typedef struct {
	int __line__;
	uint32_t w1 ;
	uint32_t w2 ;
	uint32_t w3 ;
	uint32_t w4 ;
} dbgRecord;

typedef struct {
	int			pos;
	dbgRecord	rec[MAX_DEBUG_RECORDS];
} RingDbgLog;

static RingDbgLog _ringLog= {
	0,
};


static	void	putRingLog( int __line__, uint32_t w1, uint32_t w2, uint32_t w3, uint32_t w4 )
{
	dbgRecord	*rec;

	rec = &_ringLog.rec[_ringLog.pos];

	rec->__line__ = __line__;
	rec->w1=w1;
	rec->w2=w2;
	rec->w3=w3;
	rec->w4=w4;

	_ringLog.pos = (_ringLog.pos + 1) % MAX_DEBUG_RECORDS;
}
#endif	/* _PRIVATE_DEBUG_ */
