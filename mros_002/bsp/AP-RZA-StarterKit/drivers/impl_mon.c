#include <stdint.h>
#include "easy_arm.h"
#include "impl_mon.h"
#include "solid_iores.h"

/* 各レジスタのオフセット */
#define SCIF_OFF_SCSMR				0
#define	SCIF_OFF_SCBRR				0x4
#define	SCIF_OFF_SCSCR				0x8
#define	SCIF_OFF_SCFTDR				0xc
#define	SCIF_OFF_SCFSR				0x10
#define	SCIF_OFF_SCFRDR				0x14
#define	SCIF_OFF_SCFCR				0x18
#define	SCIF_OFF_SCFDR				0x1c
#define	SCIF_OFF_SCSPTR				0x20
#define	SCIF_OFF_SCLSR				0x24
#define	SCIF_OFF_SCEMR				0x28

static SOLID_IORES_INFO info;

#define	g_UARTAddress (info.addr)

#define SCIF_SCSMR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCSMR)
#define	SCIF_SCBRR  (uint8_t*)(g_UARTAddress+SCIF_OFF_SCBRR)
#define	SCIF_SCSCR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCSCR)
#define	SCIF_SCFTDR (uint8_t*)(g_UARTAddress+SCIF_OFF_SCFTDR)
#define	SCIF_SCFSR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCFSR)
#define	SCIF_SCFRDR (uint8_t*)(g_UARTAddress+SCIF_OFF_SCFRDR)
#define	SCIF_SCFCR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCFCR)
#define	SCIF_SCFDR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCFDR)
#define	SCIF_SCSPTR (uint16_t*)(g_UARTAddress+SCIF_OFF_SCSPTR)
#define	SCIF_SCLSR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCLSR)
#define	SCIF_SCEMR  (uint16_t*)(g_UARTAddress+SCIF_OFF_SCEMR)

#define		read_mem8(b)		(*((volatile uint8_t*)(b)))
#define		read_mem16(h)		(*((volatile uint16_t*)(h)))
#define		write_mem8(b,v) 	((*((volatile uint8_t*)(b))) = ((uint8_t)(v)))
#define		write_mem16(h,v) 	((*((volatile uint16_t*)(h))) = ((uint16_t)(v)))

int IMPL_MON_Init(SOLID_MON_INFO *pInfo)
{
    if (SOLID_IORES_GetInfo("MON_UART", &info) != SOLID_ERR_OK) {
	return 0;
    }

    /* GIC関連の通知 */
    pInfo->intno    = info.extra;  /* ch 5, RXI5 */
    pInfo->bEdgeTrigger = 1;	/* edge trigger */
    pInfo->bUseFiq  = 0;
    pInfo->logMode = 2;

    /* シリアルの初期化 */
    __DMB();

    write_mem16(SCIF_SCSCR,0);	    /* disable SCIF	*/
    write_mem16(SCIF_SCFCR,0x6);    /* clear TX and RX FIFO. */

    __DMB();

    {
	uint16_t	scfsr;
	scfsr = read_mem16(SCIF_SCFSR);
	/*
	*	以下のビットが立っていた場合には0でクリアする
	*	ER,TEND,TDFE,BRK,RDF,DR
	*/
	if (scfsr &= 0xf3) write_mem16(SCIF_SCFSR,0); 
    }	

    write_mem16(SCIF_SCSMR,0); /* 調歩同期式, 8bit/non-parity/stop-bit1, CLK=P1phi */
    write_mem16(SCIF_SCEMR,0); /* ~BGCM,~ABCS */

    #define CLK_P1_MHz 	64 * 1024 * 1024
    #define	BAUD_RATE 	115200
    write_mem8(SCIF_SCBRR,(CLK_P1_MHz/(32 * BAUD_RATE) - 1)); /* <= baud=115200, P1-phi=64MHz, CLK n=0(SCSMR)	*/

    __DMB();

    {
	uint16_t	scfsr;
    	scfsr = read_mem16(SCIF_SCLSR);
	/*
	*	以下のビットが立っていた場合には0でクリアする
	*	ORER
	*/
	if (scfsr &= 1) write_mem16(SCIF_SCLSR,0);
    }

    write_mem16(SCIF_SCFCR,(read_mem16(SCIF_SCFCR)&~0x6)); 	/* enable TX and RX FIFO. */
    
    __DMB();


    /*
    *	送受信の開始	
    *	TE,REを有効
    *	CKE{1:0}は内部クロック、SCK端子はクロックとしては無視
    *	RIEをデバッグ開始割り込みとして使用
    */
    write_mem16(SCIF_SCSCR,0x70);

    return 2;
}

void IMPL_MON_Suspend(void)
{
    write_mem16(SCIF_SCSCR,0x30);
}
 
void IMPL_MON_Resume(void)
{
    // メモリに変化があるので、再度取得
    SOLID_IORES_GetInfo("MON_UART", &info);

    write_mem16(SCIF_SCSCR,0x70);
}

void IMPL_MON_DoReset(void)
{
}

void IMPL_MON_Putc(unsigned char c)
{
    /* FIFO空きチェック */
    __DMB();
    while (((read_mem16(SCIF_SCFDR) >> 8) & 0x1F) == 0x10) ;

    /* 1byte送信 */
    write_mem8(SCIF_SCFTDR, (uint8_t)c);		/* write data -> SCFTDR */
}

int IMPL_MON_Poll()
{
    return ((read_mem16(SCIF_SCFDR) & 0x1F) > 0);
}

unsigned char IMPL_MON_Getc()
{
    while (IMPL_MON_Poll() == 0);

    return read_mem8(SCIF_SCFRDR);
}

void IMPL_MON_ClearInterrupt(void)
{
    read_mem16(SCIF_SCFSR);
    write_mem16(SCIF_SCFSR,0);
    __DMB();
}

void IMPL_MON_EnterBreakMode(void)
{
}

void IMPL_MON_LeaveBreakMode(void)
{
}
