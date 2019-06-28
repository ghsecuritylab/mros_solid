#include "impl_boot.h"
#include "impl_gpio.h"
#include "easy_arm.h"
#include "easy_arm_l2c310.h"

#define PL310_PHYADDRESS (0x3FFFF000)
#define	L2C_REG15_PWR_CTRL	((volatile unsigned long*)(0x3fffff80))

#define FRQCR	((volatile unsigned short*)(0xFCFE0010))
#define FRQCR2	((volatile unsigned short*)(0xFCFE0014))

#define STBCR4	((volatile unsigned char*)(0xfcfe0424))
#define STBCR5	((volatile unsigned char*)(0xfcfe0428))
#define STBCR7	((volatile unsigned char*)(0xfcfe0430))
#define STBCR9	((volatile unsigned char*)(0xfcfe0438))
#define STBCR10 ((volatile unsigned char*)(0xfcfe043C))

#define PORTN_BASE   (0xFCFE3000)
#define JPORTN_BASE  (0xFCFE7B00)
#define PM1  ((volatile unsigned short *)(PORTN_BASE + 0x0300 + 1 * 4))
#define PMC1 ((volatile unsigned short *)(PORTN_BASE + 0x0400 + 1 * 4))
#define PIBC1 ((volatile unsigned short *)(PORTN_BASE + 0x4000 + 1 * 4))

#define PMC2 ((volatile unsigned short *)(PORTN_BASE + 0x0400 + 2 * 4))
#define PFC2 ((volatile unsigned short *)(PORTN_BASE + 0x0500 + 2 * 4))
#define PFCE2 ((volatile unsigned short *)(PORTN_BASE + 0x0600 + 2 * 4))
#define PFCAE2 ((volatile unsigned short *)(PORTN_BASE + 0x0A00 + 2 * 4))
#define PIPC2 ((volatile unsigned short *)(PORTN_BASE + 0x4200 + 2 * 4))

#define PM8  ((volatile unsigned short*)(PORTN_BASE+0x0300+8*4))
#define PMC8 ((volatile unsigned short*)(PORTN_BASE+0x0400+8*4))
#define PFC8 ((volatile unsigned short*)(PORTN_BASE+0x0500+8*4))
#define PFCE8 ((volatile unsigned short*)(PORTN_BASE+0x0600+8*4))
#define PFCAE8 ((volatile unsigned short*)(PORTN_BASE+0x0A00+8*4))
#define PIBC8 ((volatile unsigned short*)(PORTN_BASE+0x4000+8*4))
#define PIPC8 ((volatile unsigned short*)(PORTN_BASE+0x4200+8*4))

#define SYSCR1 ((volatile unsigned char *)0xFCFE0400)
#define SYSCR2 ((volatile unsigned char *)0xFCFE0404)
#define SYSCR3 ((volatile unsigned char *)0xFCFE0408)

#ifdef BOARD_AP_RZA_0A
#define CS3BCR ((volatile unsigned long *)0x3FFFC010)
#define CS3WCR ((volatile unsigned long *)0x3FFFC034)
#define SDCR   ((volatile unsigned long *)0x3FFFC04C)
#define RTCOR  ((volatile unsigned long *)0x3FFFC058)
#define RTCSR  ((volatile unsigned long *)0x3FFFC050)
/* Area3 SDRAM モードレジスタ */
#define AREA3_SDRAM_MODE_REG (*(volatile unsigned short *)0x3FFFE440)
#define __DMB() asm volatile("dmb":::"memory")

static void init_gpio_for_ddr(void)
{
	GPIO_PIBC(6)	 = 0x0000;
	GPIO_PBDC(6)	 = 0x0000;
	GPIO_PM(6)		 = 0xFFFF;
	GPIO_PMC(6)		 = 0x0000;
	GPIO_PBDC(6)	 = 0x0000;
	GPIO_PFC(6)		 = 0x0000;
	GPIO_PFCE(6)	 = 0x0000;
	GPIO_PFCAE(6)	 = 0x0000;
	GPIO_PIPC(6)	 = 0xFFFF;
	GPIO_P(6)		 = 0x0000;
	GPIO_PMC(6)		 = 0xFFFF;
	GPIO_PM(6)		 = 0x0000;
	GPIO_PIBC(6)	 = 0x0000;

	GPIO_PIBC(7)	 = 0x0000;
	GPIO_PBDC(7)	 = 0x0000;
	GPIO_PM(7)		 = 0xFFFF;
	GPIO_PMC(7)		 = 0x0000;
	GPIO_PBDC(7)	 = 0x0000;
	GPIO_PFC(7)		 = 0x0000;
	GPIO_PFCE(7)	 = 0x0000;
	GPIO_PFCAE(7)	 = 0x0000;
	GPIO_PIPC(7)	 = 0xFFFF;
	GPIO_P(7)		 = 0x0000;
	GPIO_PMC(7)		 = 0xFFFF;
	GPIO_PM(7)		 = 0x0000;
	GPIO_PIBC(7)	 = 0x0000;

	GPIO_PIBC(8)	 = 0x0000;
	GPIO_PBDC(8)	 = 0x0000;
	GPIO_PM(8)		 = 0xFFFF;
	GPIO_PMC(8)		 = 0x0000;
	GPIO_PBDC(8)	 = 0x0000;
	GPIO_PFC(8)		 = 0x0000;
	GPIO_PFCE(8)	 = 0x0000;
	GPIO_PFCAE(8)	 = 0x0000;
	GPIO_PIPC(8)	 = 0xFFFF;
	GPIO_P(8)		 = 0x0000;
	GPIO_PMC(8)		 = 0x00FF;
	GPIO_PM(8)		 = 0x0000;
	GPIO_PIBC(8)	 = 0x0000;

	return;
}

static void init_ddr_ap_rza_0a(void)
{
	volatile int cnt = 300;

	init_gpio_for_ddr();

	/* パワーオン待機 */
	while (cnt)
	{
		cnt--;
	}

	/* BSC設定 */
	/* Area3: SDRAM 32MB */
	*CS3BCR = (0ul << 28) | /* WR, WWサイクル間アイドル */
			  (0ul << 25) | /* 別空間RWサイクル間アイドル */
			  (0ul << 22) | /* 同一空間RWサイクル間アイドル */
			  (0ul << 19) | /* 別空間RRサイクル間アイドル */
			  (0ul << 16) | /* 同一空間RRサイクル間アイドル */
			  (4ul << 12) | /* SDRAM */
			  (1ul << 11) | /* 16Bit */
			  (2ul << 9) |  /* Reserved */
			  (0ul << 0);   /* Reserved */

	*CS3WCR = (2ul << 13) | /* プリチャージ完了待ちサイクル：２ */
			  (2ul << 10) |
			  (1ul << 7) | /* CAS:2 */
			  (1ul << 3) |
			  (2ul << 0);

	*SDCR = (1ul << 11) | /* リフレッシュを行う */
		    (0ul << 10) | /* オートリフレッシュ */
		    (0ul << 9) |  /* パワーダウンしない */
		    (0ul << 8) |  /* オートプリチャージモード */
		    (2ul << 3) |  /* RA:13Bit */
		    (1ul << 0);   /* CA:9Bit */

	*RTCOR = (0xA55Aul << 16) | /* ライトプロテクト解除 */
			 (15ul << 0);

	*RTCSR = (0xA55Aul << 16) | /* ライトプロテクト解除 */
			 (3ul << 3) |	   /* CKIO / 64 */
	 		 (0ul << 0);		   /* リフレッシュ回数１ */


	__DMB();
	/* SDRAMモードレジスタ */
	AREA3_SDRAM_MODE_REG = 0;

	return;
}
#endif

static void InitSPI(int ch)
{
    // SPIのクロック供給を許可
    *STBCR10 &= ~(0x80 >> ch);

    // I/Oの設定
    switch (ch) {
    case 0: // P2_12,13,14,15を使用: 兼用モードは2
	GPIO_PMC(2)   |=  0xF000;
	GPIO_PFCAE(2) &= ~0xF000;
	GPIO_PFCE(2)  &= ~0xF000;
	GPIO_PFC(2)   |=  0xF000;
	GPIO_PIPC(2)  |=  0xF000;
	// P1_6 をGPIO(入力)として使用
	GPIO_PMC(1)   &= ~0x0040;
	GPIO_PM(1)    |=  0x0040;
	GPIO_PIBC(1)  |=  0x0040;
	break;
    case 2: // P8_3,4,5,6を使用: 兼用モードは3
	GPIO_PMC(8)   |=  0x0078;
	GPIO_PFCAE(8) &= ~0x0078;
	GPIO_PFCE(8)  |=  0x0078;
	GPIO_PFC(8)   &=  0x0078;
	GPIO_PIPC(8)  |=  0x0078;
	// P8_2 をGPIO(入力)として使用
	GPIO_PMC(8)   &= ~0x0004; // PORT MODE
	GPIO_PM(8)    |=  0x0004; // IN
	GPIO_PIBC(8)  |=  0x0004; // buffer enable
	break;
    }
}

static void InitSerial(int ch)
{
    switch (ch) {
    case 4:
	// Port:5, 1:RxD4,0:TxD4

	GPIO_PIBC(5) &= ~0x0003; /* ポート入力バッファ制御レジスタ */ // 入力バッファ禁止
	GPIO_PBDC(5) &= ~0x0003; /* ポート双方向制御レジスタ */       // 双方向モード禁止
	GPIO_PM(5) |= 0x0003; /* ポートモードレジスタ */	      // 入力モード(出力禁止)
	GPIO_PMC(5) &= ~0x0003; /* ポートモード制御レジスタ */	// ポートモード
	GPIO_PFC(5) &= ~0x0003; /* ポート機能制御レジスタ*/	   // 兼用モード=第5兼用機能
	GPIO_PFCE(5) &= ~0x0003; /* ポート機能制御拡張レジスタ */     // 兼用モード=第5兼用機能
	GPIO_PFCAE(5) |= 0x0003; /* ポート機能制御追加拡張レジスタ */ // 兼用モード=第5兼用機能
	GPIO_PIPC(5) |= 0x0003; /* ポートIP制御レジスタ */	    // 兼用機能(=UART)から入出力を制御
	GPIO_P(5) &= ~0x0003; /* ポートレジスタ */		      // 入出力レジスタをクリア

	GPIO_PMC(5) |= 0x0003; /* ポートモード制御レジスタ */	 // 兼用モードに変更
	GPIO_PM(5) &= ~0x0003; /* ポートモードレジスタ */	     // 出力許可
	GPIO_PIBC(5) &= ~0x0003; /* ポート入力バッファ制御レジスタ */ // 入力バッファ禁止

	GPIO_PIBC(5) |= 0x0020; /* ポート入力バッファ制御レジスタ */ // 入力バッファ許可
	break;
    case 5:
	// Port:6, 7:RxD5,6:TxD5
	GPIO_PIBC(6) &= ~0x00c0; /* ポート入力バッファ制御レジスタ */ // 入力バッファ禁止
	GPIO_PBDC(6) &= ~0x00c0; /* ポート双方向制御レジスタ */       // 双方向モード禁止
	GPIO_PM(6) |= 0x00c0; /* ポートモードレジスタ */	      // 入力モード(出力禁止)
	GPIO_PMC(6) &= ~0x00c0; /* ポートモード制御レジスタ */	// ポートモード
	GPIO_PFC(6) &= ~0x00c0; /* ポート機能制御レジスタ*/	   // 兼用モード=第5兼用機能
	GPIO_PFCE(6) &= ~0x00c0; /* ポート機能制御拡張レジスタ */     // 兼用モード=第5兼用機能
	GPIO_PFCAE(6) |= 0x00c0; /* ポート機能制御追加拡張レジスタ */ // 兼用モード=第5兼用機能
	GPIO_PIPC(6) |= 0x00c0; /* ポートIP制御レジスタ */	    // 兼用機能(=UART)から入出力を制御
	GPIO_P(6) &= ~0x00c0; /* ポートレジスタ */		      // 入出力レジスタをクリア

	GPIO_PMC(6) |= 0x00c0; /* ポートモード制御レジスタ */	 // 兼用モードに変更
	GPIO_PM(6) &= ~0x00c0; /* ポートモードレジスタ */	     // 出力許可
	GPIO_PIBC(6) &= ~0x00c0; /* ポート入力バッファ制御レジスタ */ // 入力バッファ禁止

	GPIO_PIBC(6) |= 0x0080; /* ポート入力バッファ制御レジスタ */ // 入力バッファ許可
	break;
    }
}

void initEthernet_AP_RZA_1A(void)
{
	uint16_t	mask;

	/* Ethernetコントローラで使用する端子の設定 (ボード依存: AP-RZA-1A) */

	/* P5 9 ALT2 */
	mask = 0x200;

	GPIO_PMC(5) |= mask;	/* 兼用モード */
	GPIO_PFCAE(5) &= ~mask;	/* 第2兼用モード */
	GPIO_PFCE(5) &= ~mask;	/* 第2兼用モード */
	GPIO_PFC(5) |= mask;	/* 第2兼用モード */
	GPIO_PIPC(5) |= mask;	/* 兼用機能からの制御 */

	/* P7 1-7,9-15 ALT3 */
	mask = 0xfefe;

	GPIO_PMC(7) |= mask;	/* 兼用モード */
	GPIO_PFCAE(7) &= ~mask;	/* 第3兼用モード */
	GPIO_PFCE(7) |= mask;	/* 第3兼用モード */
	GPIO_PFC(7) &= ~mask;	/* 第3兼用モード */
	GPIO_PIPC(7) |= mask;	/* 兼用機能からの制御 */

	/* P8 0,1 ALT3 */
	mask = 0x3;

	GPIO_PMC(8) |= mask;	/* 兼用モード */
	GPIO_PFCAE(8) &= ~mask;	/* 第3兼用モード */
	GPIO_PFCE(8) |= mask;	/* 第3兼用モード */
	GPIO_PFC(8) &= ~mask;	/* 第3兼用モード */
	GPIO_PIPC(8) |= mask;	/* 兼用機能からの制御 */

	/* P8 7 ALT5 */
	mask = 0x80;

	GPIO_PMC(8) |= mask;	/* 兼用モード */
	GPIO_PFCAE(8) |= mask;	/* 第5兼用モード */
	GPIO_PFCE(8) &= ~mask;	/* 第5兼用モード */
	GPIO_PFC(8) &= ~mask;	/* 第5兼用モード */
	GPIO_PIPC(8) |= mask;	/* 兼用機能からの制御 */
}

void initEEPROM_AP_RZA_1A(void)
{
	uint16_t	mask;

	/* MACアドレスが格納されているEEPROMにアクセスするためのI2C端子の設定 (ボード依存: AP-RZA-1A) */

	/* RIIC ch1, P1 2,3  ALT1 */
	mask = 0xc;
	GPIO_PIBC(1) &= ~mask;	/* ポート入力バッファ制御レジスタ */ 	// 入力バッファ禁止
	GPIO_PBDC(1) &= ~mask;	/* ポート双方向制御レジスタ */       	// 双方向モード禁止
	GPIO_PM(1) |= mask;		/* ポートモードレジスタ */	      		// 入力モード(出力禁止)
	GPIO_PMC(1) &= ~mask;	/* ポートモード制御レジスタ */			// ポートモード
	GPIO_PBDC(1) |= mask;	/* ポート双方向制御レジスタ */ 			// 双方向モード許可
	GPIO_PFC(1) &= ~mask;	/* ポート機能制御レジスタ*/	   			// 兼用モード=第1兼用機能
	GPIO_PFCE(1) &= ~mask;	/* ポート機能制御拡張レジスタ */     	// 兼用モード=第1兼用機能
	GPIO_PFCAE(1) &= ~mask;	/* ポート機能制御追加拡張レジスタ */	// 兼用モード=第1兼用機能
	GPIO_PIPC(1) |= mask;	/* ポートIP制御レジスタ */				// 兼用機能からの制御
	GPIO_P(1) &= ~mask; 	/* ポートレジスタ */		      		// 入出力レジスタをクリア
	GPIO_PMC(1) |= mask;	/* ポートモード制御レジスタ */	 		// 兼用モードに変更
	GPIO_PM(1) &= ~mask;		/* ポートモードレジスタ */	      		// 出力許可
	GPIO_PIBC(1) |= mask;	/* ポート入力バッファ制御レジスタ */ 	// 入力バッファ許可
}

static void L2C_init(unsigned long l2c_base)
{
	if (L2C_Is_Enabled(l2c_base))
		return;

	L2C_Inv_Way(l2c_base);
	L2C_Enable(l2c_base, 1);
	L2C_Sync(l2c_base);
	L2C_Reg_Wait_Mask(l2c_base, REG7_CACHE_SYNC, 1);
}


/* デバイスの初期化関数 : ここで、SOLIDが使用するメモリ(DRAM等)やCPUのIOの初期化を行ってください。*/
/* I/Oのアドレス等は物理アドレスを使用してください。                                              */
void IMPL_BOOT_Init()
{
	// RAM ライトイネーブル
	*SYSCR1 = 0xFF;
	*SYSCR2 = 0xFF;
	*SYSCR3 = 0x0F;
	
	
    // L2Cacheのスタンバイモードを許可
    *L2C_REG15_PWR_CTRL = 0x00000001;

	L2C_init(PL310_PHYADDRESS);

    // クロックパルス発振器の初期化
    *FRQCR  = 0x1035;
    *FRQCR2 = 0x0001;

    // 使用するペリフェラルへのクロック供給
    *STBCR5 &= ~0x02;	/* MSTP51(OSTM0へのクロック供給)をon */
    *STBCR5 &= ~0x01;	/* MSTP51(OSTM1へのクロック供給)をon */
#ifdef BOARD_AP_RZA_0A
	*STBCR4 &= ~0x08;	/* MSTP43(SCIF4へのクロック供給)をon */
#else
    *STBCR4 &= ~0x04;	/* MSTP42(SCIF5へのクロック供給)をon */
#endif
    *STBCR7 &= ~0x10;	/* MSTP74(イーサネットコントローラへのクロック供給)をon */
    *STBCR9 &= ~0x40;	/* MSTP96(I2Cバスインタフェースチャネル1へのクロック供給)をon */

    // Pin Function の初期化
#ifdef BOARD_AP_RZA_0A
    InitSPI(0);
#else
    InitSPI(2);
#endif

#ifdef BOARD_AP_RZA_0A
    InitSerial(4);
#else
    InitSerial(5);
#endif

#ifndef BOARD_AP_RZA_0A
	initEthernet_AP_RZA_1A();
	initEEPROM_AP_RZA_1A();
#endif

#ifdef BOARD_AP_RZA_0A
	init_ddr_ap_rza_0a();
#endif
}

/* SOLIDへの引数の指定 */
void IMPL_BOOT_GetArgument(unsigned char* args, unsigned long size)
{
    args[0] = 0;
}

/* コアサービスのバイナリ情報を取得: solid_cs.binの先頭のデータを転送してください */
void IMPL_BOOT_GetCsInfo(void *pBuf, unsigned long bufSize)
{
    extern void *__start_solid_cs;
    int i;
    unsigned long *pDst = (unsigned long *)pBuf;
    unsigned long *pSrc = (unsigned long *)(&__start_solid_cs);
    for (i = 0; i < (bufSize + 3) / 4; i++) {
	*pDst = *pSrc;
	pDst++;
	pSrc++;
    }
}

/* コアサービスのバイナリを転送: soli_cs.binを指定位置に転送してください */
void IMPL_BOOT_LoadCs(void *pAddr)
{
    // 既にロード状態のため、何もしない
}

