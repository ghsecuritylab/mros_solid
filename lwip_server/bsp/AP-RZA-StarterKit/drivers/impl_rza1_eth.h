#ifndef	_IMPL_RZA1_ETH_H_
#define	_IMPL_RZA1_ETH_H_

#include	<stdint.h>
#include	"solid_mem.h"

#define	RZETH_REG_BASE1_PA				0xE8203000
#define	RZETH_REG_BASE2_PA				0xE8203400
#define	RZETH_REG_BASE3_PA				0xE8204800

#define	RZETH_REG_AREA1_SIZE			0x40
#define	RZETH_REG_AREA2_SIZE			0x37C
#define	RZETH_REG_AREA3_SIZE			0x200

extern	SOLID_ADDRESS	_eth_io_base1;		/* 物理アドレス 0xE8203000 */
extern	SOLID_ADDRESS	_eth_io_base2;		/* 物理アドレス 0xE8203400 */
extern	SOLID_ADDRESS	_eth_io_base3;		/* 物理アドレス 0xE8204800 */

/*
*	レジスタアクセス用define定義
*/

#define	RZETH_EDSR0		((volatile uint32_t*)(_eth_io_base1+0x0))	/* E-DMAC起動レジスタ */
#define	RZETH_TDLAR0	((volatile uint32_t*)(_eth_io_base1+0x10))	/* 送信ディスクリプタリスト先頭アドレスレジスタ */
#define	RZETH_TDFAR0	((volatile uint32_t*)(_eth_io_base1+0x14))	/* 送信ディスクリプタフェッチアドレスレジスタ */
#define	RZETH_TDFXR0	((volatile uint32_t*)(_eth_io_base1+0x18))	/* 送信ディスクリプタ処理済アドレスレジスタ */
#define	RZETH_TDFFR0		((volatile uint32_t*)(_eth_io_base1+0x1C))	/* 送信ディスクリプタ最終フラグレジスタ */
#define	RZETH_RDLAR0		((volatile uint32_t*)(_eth_io_base1+0x30))	/* 受信ディスクリプタリスト先頭アドレスレジスタ */
#define	RZETH_RDFAR0		((volatile uint32_t*)(_eth_io_base1+0x34))	/* 受信ディスクリプタフェッチアドレスレジスタ */
#define	RZETH_RDFXR0		((volatile uint32_t*)(_eth_io_base1+0x38))	/* 受信ディスクリプタ処理済アドレスレジスタ */
#define	RZETH_RDFFR0		((volatile uint32_t*)(_eth_io_base1+0x3C))	/* 受信ディスクリプタ最終フラグレジスタ */
#define	RZETH_EDMR0		((volatile uint32_t*)(_eth_io_base2+0x0))	/* E-DMACモードレジスタ */
#define	RZETH_EDTRR0		((volatile uint32_t*)(_eth_io_base2+0x8))	/* E-DMAC送信要求レジスタ */
#define	RZETH_EDRRR0		((volatile uint32_t*)(_eth_io_base2+0x10))	/* E-DMAC受信要求レジスタ */
#define	RZETH_EESR0		((volatile uint32_t*)(_eth_io_base2+0x28))	/* E-MAC/E-DMACステータスレジスタ */
#define	RZETH_EESIPR0		((volatile uint32_t*)(_eth_io_base2+0x30))	/* E-MAC/E-DMACステータス割り込み許可レジスタ */
#define	RZETH_TRSCER0		((volatile uint32_t*)(_eth_io_base2+0x38))	/* 送受信ステータスコピー指示レジスタ */
#define	RZETH_RMFCR0		((volatile uint32_t*)(_eth_io_base2+0x40))	/* ミスドフレームカウンタレジスタ */
#define	RZETH_TFTR0		((volatile uint32_t*)(_eth_io_base2+0x48))	/* 送信FIFOしきい値指定レジスタ */
#define	RZETH_FDR0		((volatile uint32_t*)(_eth_io_base2+0x50))	/* FIFO容量指定レジスタ */
#define	RZETH_RMCR0		((volatile uint32_t*)(_eth_io_base2+0x58))	/* 受信方式制御レジスタ */
#define	RZETH_RPADIR0		((volatile uint32_t*)(_eth_io_base2+0x60))	/* 受信データパディング挿入設定レジスタ */
#define	RZETH_FCFTR0		((volatile uint32_t*)(_eth_io_base2+0x68))	/* オーバフロー予告FIFOしきい値設定レジスタ */
#define	RZETH_CSMR		((volatile uint32_t*)(_eth_io_base2+0xE4))	/* インテリジェントチェックサムモードレジスタ */
#define	RZETH_CSSBM		((volatile uint32_t*)(_eth_io_base2+0xE8))	/* インテリジェントチェックサムスキップ済みバイト数モニタレジスタ */
#define	RZETH_CSSMR		((volatile uint32_t*)(_eth_io_base2+0xEC))	/* インテリジェントチェックサム機能モニタレジスタ */
#define	RZETH_ECMR0		((volatile uint32_t*)(_eth_io_base2+0x100))	/* E-MACモードレジスタ */
#define	RZETH_RFLR0		((volatile uint32_t*)(_eth_io_base2+0x108))	/* 受信フレーム長上限レジスタ */
#define	RZETH_ECSR0		((volatile uint32_t*)(_eth_io_base2+0x110))	/* E-MACステータスレジスタ */
#define	RZETH_ECSIPR0		((volatile uint32_t*)(_eth_io_base2+0x118))	/* E-MAC割り込み許可レジスタ */
#define	RZETH_PIR0		((volatile uint32_t*)(_eth_io_base2+0x120))	/* PHY部インタフェースレジスタ */
#define	RZETH_APR0		((volatile uint32_t*)(_eth_io_base2+0x154))	/* 自動PAUSEフレーム設定レジスタ */
#define	RZETH_MPR0		((volatile uint32_t*)(_eth_io_base2+0x158))	/* 手動PAUSEフレーム設定レジスタ */
#define	RZETH_PFTCR0		((volatile uint32_t*)(_eth_io_base2+0x15C))	/* PAUSEフレーム送信カウンタ */
#define	RZETH_PFRCR0		((volatile uint32_t*)(_eth_io_base2+0x160))	/* PAUSEフレーム受信カウンタ */
#define	RZETH_TPAUSER0		((volatile uint32_t*)(_eth_io_base2+0x164))	/* 自動PAUSEフレーム再送回数設定レジスタ */
#define	RZETH_MAHR0		((volatile uint32_t*)(_eth_io_base2+0x1C0))	/* MACアドレス上位設定レジスタ */
#define	RZETH_MALR0		((volatile uint32_t*)(_eth_io_base2+0x1C8))	/* MACアドレス下位設定レジスタ */
#define	RZETH_CEFCR0		((volatile uint32_t*)(_eth_io_base2+0x340))	/* CRCエラーフレーム受信カウンタレジスタ */
#define	RZETH_FRECR0		((volatile uint32_t*)(_eth_io_base2+0x348))	/* フレーム受信エラーカウンタレジスタ */
#define	RZETH_TSFRCR0		((volatile uint32_t*)(_eth_io_base2+0x350))	/* 64バイト未満フレーム受信カウンタレジスタ */
#define	RZETH_TLFRCR0		((volatile uint32_t*)(_eth_io_base2+0x358))	/* 指定バイト超フレーム受信カウンタレジスタ */
#define	RZETH_RFCR0		((volatile uint32_t*)(_eth_io_base2+0x360))	/* 端数ビットフレーム受信カウンタレジスタ */
#define	RZETH_MAFCR0		((volatile uint32_t*)(_eth_io_base2+0x378))	/* マルチキャストアドレスフレーム受信カウンタレジスタ */
#define	RZETH_ARSTR		((volatile uint32_t*)(_eth_io_base3+0x0))	/* ソフトウェアリセットレジスタ */
#define	RZETH_TSU_CTRST		((volatile uint32_t*)(_eth_io_base3+0x4))	/* TSUカウンタリセットレジスタ */
#define	RZETH_TSU_VTAG0		((volatile uint32_t*)(_eth_io_base3+0x58))	/* VLANtag設定レジスタ */
#define	RZETH_TSU_ADSBSY		((volatile uint32_t*)(_eth_io_base3+0x60))	/* CAMエントリテーブル設定ビジーレジスタ */
#define	RZETH_TSU_TEN		((volatile uint32_t*)(_eth_io_base3+0x64))	/* CAMエントリテーブルイネーブル設定レジスタ */
#define	RZETH_TXNLCR0		((volatile uint32_t*)(_eth_io_base3+0x80))	/* 送信フレーム数カウンタレジスタ */
#define	RZETH_TXALCR0		((volatile uint32_t*)(_eth_io_base3+0x84))	/* 送信フレーム数カウンタレジスタ */
#define	RZETH_RXNLCR0		((volatile uint32_t*)(_eth_io_base3+0x88))	/* 受信フレーム数カウンタレジスタ */
#define	RZETH_RXALCR0		((volatile uint32_t*)(_eth_io_base3+0x8C))	/* 受信フレーム数カウンタレジスタ */
#define	RZETH_TSU_ADRH0		((volatile uint32_t*)(_eth_io_base3+0x100))	/* CAMエントリテーブル0Hレジスタ */
#define	RZETH_TSU_ADRL0		((volatile uint32_t*)(_eth_io_base3+0x104))	/* CAMエントリテーブル0Lレジスタ */
#define	RZETH_TSU_ADRH1		((volatile uint32_t*)(_eth_io_base3+0x108))	/* CAMエントリテーブル1Hレジスタ */
#define	RZETH_TSU_ADRL1		((volatile uint32_t*)(_eth_io_base3+0x10C))	/* CAMエントリテーブル1Lレジスタ */
#define	RZETH_TSU_ADRH2		((volatile uint32_t*)(_eth_io_base3+0x110))	/* CAMエントリテーブル2Hレジスタ */
#define	RZETH_TSU_ADRL2		((volatile uint32_t*)(_eth_io_base3+0x114))	/* CAMエントリテーブル2Lレジスタ */
#define	RZETH_TSU_ADRH3		((volatile uint32_t*)(_eth_io_base3+0x118))	/* CAMエントリテーブル3Hレジスタ */
#define	RZETH_TSU_ADRL3		((volatile uint32_t*)(_eth_io_base3+0x11C))	/* CAMエントリテーブル3Lレジスタ */
#define	RZETH_TSU_ADRH4		((volatile uint32_t*)(_eth_io_base3+0x120))	/* CAMエントリテーブル4Hレジスタ */
#define	RZETH_TSU_ADRL4		((volatile uint32_t*)(_eth_io_base3+0x124))	/* CAMエントリテーブル4Lレジスタ */
#define	RZETH_TSU_ADRH5		((volatile uint32_t*)(_eth_io_base3+0x128))	/* CAMエントリテーブル5Hレジスタ */
#define	RZETH_TSU_ADRL5		((volatile uint32_t*)(_eth_io_base3+0x12C))	/* CAMエントリテーブル5Lレジスタ */
#define	RZETH_TSU_ADRH6		((volatile uint32_t*)(_eth_io_base3+0x130))	/* CAMエントリテーブル6Hレジスタ */
#define	RZETH_TSU_ADRL6		((volatile uint32_t*)(_eth_io_base3+0x134))	/* CAMエントリテーブル6Lレジスタ */
#define	RZETH_TSU_ADRH7		((volatile uint32_t*)(_eth_io_base3+0x138))	/* CAMエントリテーブル7Hレジスタ */
#define	RZETH_TSU_ADRL7		((volatile uint32_t*)(_eth_io_base3+0x13C))	/* CAMエントリテーブル7Lレジスタ */
#define	RZETH_TSU_ADRH8		((volatile uint32_t*)(_eth_io_base3+0x140))	/* CAMエントリテーブル8Hレジスタ */
#define	RZETH_TSU_ADRL8		((volatile uint32_t*)(_eth_io_base3+0x144))	/* CAMエントリテーブル8Lレジスタ */
#define	RZETH_TSU_ADRH9		((volatile uint32_t*)(_eth_io_base3+0x148))	/* CAMエントリテーブル9Hレジスタ */
#define	RZETH_TSU_ADRL9		((volatile uint32_t*)(_eth_io_base3+0x14C))	/* CAMエントリテーブル9Lレジスタ */
#define	RZETH_TSU_ADRH10		((volatile uint32_t*)(_eth_io_base3+0x150))	/* CAMエントリテーブル10Hレジスタ */
#define	RZETH_TSU_ADRL10		((volatile uint32_t*)(_eth_io_base3+0x154))	/* CAMエントリテーブル10Lレジスタ */
#define	RZETH_TSU_ADRH11		((volatile uint32_t*)(_eth_io_base3+0x158))	/* CAMエントリテーブル11Hレジスタ */
#define	RZETH_TSU_ADRL11		((volatile uint32_t*)(_eth_io_base3+0x15C))	/* CAMエントリテーブル11Lレジスタ */
#define	RZETH_TSU_ADRH12		((volatile uint32_t*)(_eth_io_base3+0x160))	/* CAMエントリテーブル12Hレジスタ */
#define	RZETH_TSU_ADRL12		((volatile uint32_t*)(_eth_io_base3+0x164))	/* CAMエントリテーブル12Lレジスタ */
#define	RZETH_TSU_ADRH13		((volatile uint32_t*)(_eth_io_base3+0x168))	/* CAMエントリテーブル13Hレジスタ */
#define	RZETH_TSU_ADRL13		((volatile uint32_t*)(_eth_io_base3+0x16C))	/* CAMエントリテーブル13Lレジスタ */
#define	RZETH_TSU_ADRH14		((volatile uint32_t*)(_eth_io_base3+0x170))	/* CAMエントリテーブル14Hレジスタ */
#define	RZETH_TSU_ADRL14		((volatile uint32_t*)(_eth_io_base3+0x174))	/* CAMエントリテーブル14Lレジスタ */
#define	RZETH_TSU_ADRH15		((volatile uint32_t*)(_eth_io_base3+0x178))	/* CAMエントリテーブル15Hレジスタ */
#define	RZETH_TSU_ADRL15		((volatile uint32_t*)(_eth_io_base3+0x17C))	/* CAMエントリテーブル15Lレジスタ */
#define	RZETH_TSU_ADRH16		((volatile uint32_t*)(_eth_io_base3+0x180))	/* CAMエントリテーブル16Hレジスタ */
#define	RZETH_TSU_ADRL16		((volatile uint32_t*)(_eth_io_base3+0x184))	/* CAMエントリテーブル16Lレジスタ */
#define	RZETH_TSU_ADRH17		((volatile uint32_t*)(_eth_io_base3+0x188))	/* CAMエントリテーブル17Hレジスタ */
#define	RZETH_TSU_ADRL17		((volatile uint32_t*)(_eth_io_base3+0x18C))	/* CAMエントリテーブル17Lレジスタ */
#define	RZETH_TSU_ADRH18		((volatile uint32_t*)(_eth_io_base3+0x190))	/* CAMエントリテーブル18Hレジスタ */
#define	RZETH_TSU_ADRL18		((volatile uint32_t*)(_eth_io_base3+0x194))	/* CAMエントリテーブル18Lレジスタ */
#define	RZETH_TSU_ADRH19		((volatile uint32_t*)(_eth_io_base3+0x198))	/* CAMエントリテーブル19Hレジスタ */
#define	RZETH_TSU_ADRL19		((volatile uint32_t*)(_eth_io_base3+0x19C))	/* CAMエントリテーブル19Lレジスタ */
#define	RZETH_TSU_ADRH20		((volatile uint32_t*)(_eth_io_base3+0x1A0))	/* CAMエントリテーブル20Hレジスタ */
#define	RZETH_TSU_ADRL20		((volatile uint32_t*)(_eth_io_base3+0x1A4))	/* CAMエントリテーブル20Lレジスタ */
#define	RZETH_TSU_ADRH21		((volatile uint32_t*)(_eth_io_base3+0x1A8))	/* CAMエントリテーブル21Hレジスタ */
#define	RZETH_TSU_ADRL21		((volatile uint32_t*)(_eth_io_base3+0x1AC))	/* CAMエントリテーブル21Lレジスタ */
#define	RZETH_TSU_ADRH22		((volatile uint32_t*)(_eth_io_base3+0x1B0))	/* CAMエントリテーブル22Hレジスタ */
#define	RZETH_TSU_ADRL22		((volatile uint32_t*)(_eth_io_base3+0x1B4))	/* CAMエントリテーブル22Lレジスタ */
#define	RZETH_TSU_ADRH23		((volatile uint32_t*)(_eth_io_base3+0x1B8))	/* CAMエントリテーブル23Hレジスタ */
#define	RZETH_TSU_ADRL23		((volatile uint32_t*)(_eth_io_base3+0x1BC))	/* CAMエントリテーブル23Lレジスタ */
#define	RZETH_TSU_ADRH24		((volatile uint32_t*)(_eth_io_base3+0x1C0))	/* CAMエントリテーブル24Hレジスタ */
#define	RZETH_TSU_ADRL24		((volatile uint32_t*)(_eth_io_base3+0x1C4))	/* CAMエントリテーブル24Lレジスタ */
#define	RZETH_TSU_ADRH25		((volatile uint32_t*)(_eth_io_base3+0x1C8))	/* CAMエントリテーブル25Hレジスタ */
#define	RZETH_TSU_ADRL25		((volatile uint32_t*)(_eth_io_base3+0x1CC))	/* CAMエントリテーブル25Lレジスタ */
#define	RZETH_TSU_ADRH26		((volatile uint32_t*)(_eth_io_base3+0x1D0))	/* CAMエントリテーブル26Hレジスタ */
#define	RZETH_TSU_ADRL26		((volatile uint32_t*)(_eth_io_base3+0x1D4))	/* CAMエントリテーブル26Lレジスタ */
#define	RZETH_TSU_ADRH27		((volatile uint32_t*)(_eth_io_base3+0x1D8))	/* CAMエントリテーブル27Hレジスタ */
#define	RZETH_TSU_ADRL27		((volatile uint32_t*)(_eth_io_base3+0x1DC))	/* CAMエントリテーブル27Lレジスタ */
#define	RZETH_TSU_ADRH28		((volatile uint32_t*)(_eth_io_base3+0x1E0))	/* CAMエントリテーブル28Hレジスタ */
#define	RZETH_TSU_ADRL28		((volatile uint32_t*)(_eth_io_base3+0x1E4))	/* CAMエントリテーブル28Lレジスタ */
#define	RZETH_TSU_ADRH29		((volatile uint32_t*)(_eth_io_base3+0x1E8))	/* CAMエントリテーブル29Hレジスタ */
#define	RZETH_TSU_ADRL29		((volatile uint32_t*)(_eth_io_base3+0x1EC))	/* CAMエントリテーブル29Lレジスタ */
#define	RZETH_TSU_ADRH30		((volatile uint32_t*)(_eth_io_base3+0x1F0))	/* CAMエントリテーブル30Hレジスタ */
#define	RZETH_TSU_ADRL30		((volatile uint32_t*)(_eth_io_base3+0x1F4))	/* CAMエントリテーブル30Lレジスタ */
#define	RZETH_TSU_ADRH31		((volatile uint32_t*)(_eth_io_base3+0x1F8))	/* CAMエントリテーブル31Hレジスタ */
#define	RZETH_TSU_ADRL31		((volatile uint32_t*)(_eth_io_base3+0x1FC))	/* CAMエントリテーブル31Lレジスタ */

/*
*	割込みビットマスク定義 (EESR,ECSIPR)
*/

#define	RZETH_INT_TWB_1	0x80000000	/*  ライトバック完了[1] */
#define	RZETH_INT_TWB_0	0x40000000	/*  ライトバック完了[0] */
#define	RZETH_INT_TC_1	0x20000000	/*  フレーム送信完了 */
#define	RZETH_INT_TUC	0x10000000	/*  送信アンダフローフレームライトバック完了 */
#define	RZETH_INT_ROC	0x08000000	/*  受信オーバフローフレームライトバック完了 */
#define	RZETH_INT_TABT	0x04000000	/*  送信中断検出 */
#define	RZETH_INT_RABT	0x02000000	/*  受信中断検出 */
#define	RZETH_INT_RFCOF	0x01000000	/*  受信フレームカウンタオーバフロー */
#define	RZETH_INT_ECI	0x00400000	/*  E-MACステータスレジスタ要因 */
#define	RZETH_INT_TC_0	0x00200000	/*  フレーム送信完了 */
#define	RZETH_INT_TDE	0x00100000	/*  送信ディスクリプタ枯渇 */
#define	RZETH_INT_TFUF	0x00080000	/*  送信FIFOアンダフロー */
#define	RZETH_INT_FR	0x00040000	/*  フレーム受信 */
#define	RZETH_INT_RDE	0x00020000	/*  受信ディスクリプタ枯渇 */
#define	RZETH_INT_RFOF	0x00010000	/*  受信FIFOオーバフロー */
#define	RZETH_INT_RMAF	0x00000080	/*  マルチキャストアドレスフレーム受信 */
#define	RZETH_INT_RRF	0x00000010	/*  端数ビットフレーム受信 */
#define	RZETH_INT_RTLF	0x00000008	/*  ロングフレーム受信エラー */
#define	RZETH_INT_RTSF	0x00000004	/*  ショートフレーム受信エラー */
#define	RZETH_INT_PRE	0x00000002	/*  PHY-LSI受信エラー */
#define	RZETH_INT_CERF	0x00000001	/*  受信フレームCRCエラー */

/*
 * RZ/A1の仕様書には記載がない割込みビット
 */
#define SHETH_INT_CND       (1<<11)     // キャリア未検出
#define SHETH_INT_DLC       (1<<10)     // キャリア消失検出

/*
 *	E-MACステータスレジスタ(ECSR)
 */
#define	RZETH_ECSR_PFROI	(1<<4)	/* PAUSEフレーム再送リトライオーバ */
#define	RZETH_ECSR_ICD		(1<<0)	/* 不正キャリア検出 */

#define	RZETH_INT_ALL 	(\
	RZETH_INT_TWB_1	 | RZETH_INT_TWB_0	 | RZETH_INT_TC_1	 | RZETH_INT_TUC	 | \
	RZETH_INT_ROC	 | RZETH_INT_TABT	 | RZETH_INT_RABT	 | RZETH_INT_RFCOF	 | \
	RZETH_INT_ECI	 | RZETH_INT_TC_0	 | RZETH_INT_TDE	 | RZETH_INT_TFUF	 | \
	RZETH_INT_FR	 | RZETH_INT_RDE	 | RZETH_INT_RFOF	 | RZETH_INT_RMAF	 | \
	RZETH_INT_RRF	 | RZETH_INT_RTLF	 | RZETH_INT_RTSF	 | RZETH_INT_PRE	 | \
	RZETH_INT_CERF	 | SHETH_INT_CND	 | SHETH_INT_DLC)

#define	RZETH_MAX_CAM_ENTRY				32

#define	INTNO_ETHERI					359

#ifdef	NET_REQUIRES_RTOS
void IMPL_NET_DEV_ASP_InterruptHandler(void);
#endif

#endif	/* _IMPL_RZA1_ETH_H_ */
