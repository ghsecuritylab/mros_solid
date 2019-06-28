/*
 *		RZ/1A 内蔵 FIFO UART（SCIF5）用 簡易SIOドライバ
 */

#include <stdint.h>
#include "impl_uart.h"
#include "solid_mem.h"
#include "solid_iores.h"
#include "solid_cs_assert.h"
#include "easy_arm.h"

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

/* ここからは論理アドレスで扱う */
static SOLID_ADDRESS	g_UARTAddress;
#define	SCIF_LOGICAL_BASE(n)	g_UARTAddress

#define SCIF_SCSMR(n)	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCSMR)
#define	SCIF_SCBRR(n) 	(uint8_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCBRR)
#define	SCIF_SCSCR(n)	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCSCR)
#define	SCIF_SCFTDR(n) 	(uint8_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCFTDR)
#define	SCIF_SCFSR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCFSR)
#define	SCIF_SCFRDR(n) 	(uint8_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCFRDR)
#define	SCIF_SCFCR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCFCR)
#define	SCIF_SCFDR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCFDR)
#define	SCIF_SCSPTR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCSPTR)
#define	SCIF_SCLSR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCLSR)
#define	SCIF_SCEMR(n) 	(uint16_t*)(SCIF_LOGICAL_BASE((n))+SCIF_OFF_SCEMR)

#define		read_mem8(b)		(*((volatile uint8_t*)(b)))
#define		read_mem16(h)		(*((volatile uint16_t*)(h)))
#define		write_mem8(b,v) 	((*((volatile uint8_t*)(b))) = ((uint8_t)(v)))
#define		write_mem16(h,v) 	((*((volatile uint16_t*)(h))) = ((uint16_t)(v)))

#define SCIF_USE_CH (0)

#ifdef IMPL_UART_DEBUG
static	char output_mirror[512] = "empty";
static	int		mir_pos =0;
static __inline__	_mirror_out(char c) {
	output_mirror[mir_pos] = c;
	mir_pos = (mir_pos + 1) % sizeof(output_mirror);	
}
#define	MIRROR(c)			_mirror_out(c)
#else
#define	MIRROR(c)
#endif

/*
*	制約事項:
*	-コンソールログを想定した送信機能のみの実装
*	-受信機能は未実装
*	--> 割込みは受信・送信とも使用していない
*	-現状は引数に関係なく、SCIFのチャネルは5で固定。
*	 (AP-RZA-1AではGPIP6経由でUART端子に結線されている)
*
*/

/*
 *  シリアルI/Oポートのオープン
 */
void
IMPL_UART_open(int portno)
{
	SOLID_IORES_INFO info;
	SOLID_IORES_GetInfo("LOG_UART", &info);
	g_UARTAddress = info.addr;

	__DMB();

	write_mem16(SCIF_SCSCR(SCIF_USE_CH),0); /* disable SCIF	*/
	write_mem16(SCIF_SCFCR(SCIF_USE_CH),0x6); 	/* clear TX and RX FIFO. */

	__DMB();

	{
		uint16_t	scfsr;
		scfsr = read_mem16(SCIF_SCFSR(SCIF_USE_CH));
		/*
		*	以下のビットが立っていた場合には0でクリアする
		*	ER,TEND,TDFE,BRK,RDF,DR
		*/
		if (scfsr &= 0xf3) write_mem16(SCIF_SCFSR(SCIF_USE_CH),0); 
	}	

	write_mem16(SCIF_SCSMR(SCIF_USE_CH),0); /* 調歩同期式, 8bit/non-parity/stop-bit1, CLK=P1phi */
	write_mem16(SCIF_SCEMR(SCIF_USE_CH),0); /* ~BGCM,~ABCS */

	#define CLK_P1_MHz 	64 * 1024 * 1024
	#define	BAUD_RATE 	115200
	write_mem8(SCIF_SCBRR(SCIF_USE_CH),(CLK_P1_MHz/(32 * BAUD_RATE) - 1)); /* <= baud=115200, P1-phi=64MHz, CLK n=0(SCSMR)	*/

	__DMB();

	{
		uint16_t	scfsr;
		scfsr = read_mem16(SCIF_SCLSR(SCIF_USE_CH));
		/*
		*	以下のビットが立っていた場合には0でクリアする
		*	ORER
		*/
		if (scfsr &= 1) write_mem16(SCIF_SCLSR(SCIF_USE_CH),0);
	}


	write_mem16(SCIF_SCFCR(SCIF_USE_CH),(read_mem16(SCIF_SCFCR(SCIF_USE_CH))&~0x6)); 	/* enable TX and RX FIFO. */

	__DMB();


	/*
	*	送受信の開始	
	*	TE,REを有効
	*	CKE{1:0}は内部クロック、SCK端子はクロックとしては無視
	*	TIE,RIE,REIEは割込みデバッグ中に障害になるので初期状態では無効
	*/
	write_mem16(SCIF_SCSCR(SCIF_USE_CH),0x30);
	
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
IMPL_UART_close(int portno)
{
	write_mem16(SCIF_SCSCR(SCIF_USE_CH),0); /* disable SCIF	*/
	write_mem16(SCIF_SCFCR(SCIF_USE_CH),0x6); 	/* clear TX and RX FIFO. */
	__DMB();
}

static __inline__	void _write1Byte(int portno, char c)
{
	uint16_t	scfsr;
	uint16_t	scfsr_mask = (1<<5)|(1<<6);

	while (((read_mem16(SCIF_SCFSR(portno))&(1<<5))==0)); 	/* check TDFE. */ 
	__DMB();

	write_mem8(SCIF_SCFTDR(portno), (uint8_t)c);		/* write data -> SCFTDR */
	__DMB();
	MIRROR(c);

	do {
		__DMB();
		scfsr = read_mem16(SCIF_SCFSR(portno));
	} while ((scfsr & scfsr_mask) != scfsr_mask); 		/* test TDFE=1 and TEND=1	*/

	write_mem16(SCIF_SCFSR(portno), (scfsr & ~scfsr_mask));	/* reset TDFE & TEND */
	__DMB();
}

/*
 *  シリアルI/Oポートへの文字送信
 */
int
IMPL_UART_putChar(int portno, char c)
{
	/* TODO portno */

	solid_cs_assert(*SCIF_SCBRR(SCIF_USE_CH) != (uint8_t)0);

	_write1Byte(SCIF_USE_CH, c);

	while( (read_mem16(SCIF_SCFSR(SCIF_USE_CH))&(1<<6)) == 0);	/* wait for TEND to deassert. */

	return((0==0));
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int
IMPL_UART_getChar(int portno)
{
	uint16_t	scfsr;
	uint16_t	sclsr;
	uint16_t	scfdr;
	uint8_t		scfrdr;

	scfsr = read_mem16(SCIF_SCFSR(SCIF_USE_CH)); /* check SCFSR.ER .DR .BRK*/
	sclsr = read_mem16(SCIF_SCLSR(SCIF_USE_CH)); /* check SCLSR.ORER */

	__DMB();

	while (( scfsr & 0x91 )|| ( sclsr & 0x1)) {

		if (scfsr & 0x1) {
			scfrdr = read_mem8(SCIF_SCFRDR(SCIF_USE_CH)); /* discard data from SCFRDR (FIFO) */
			__DMB();
		}

		write_mem16(SCIF_SCFSR(SCIF_USE_CH), (scfsr & ~(0x91)));	/* clear SCFSR.ER .DR .BRK */
		__DMB();
		write_mem16(SCIF_SCLSR(SCIF_USE_CH), 0 );					/* clear SCLSR.ORER */
		__DMB();
		scfsr = read_mem16(SCIF_SCFSR(SCIF_USE_CH)); /* check SCFSR.ER .DR .BRK*/
		sclsr = read_mem16(SCIF_SCLSR(SCIF_USE_CH)); /* check SCLSR.ORER */
	}

	/* check SCFSR.RDF */
	if (0 == (scfsr & 0x02 )) {
		return -1; /* data empty */
	} 

	scfrdr = read_mem8(SCIF_SCFRDR(SCIF_USE_CH)); /* read data from SCFRDR (FIFO) */
	__DMB();

	/* if EMPTY? */
	scfdr = read_mem16(SCIF_SCFDR(SCIF_USE_CH));
	if (0 == (scfdr & 0x1f)) {
		scfsr = read_mem16(SCIF_SCFSR(SCIF_USE_CH));
		write_mem16(SCIF_SCFSR(SCIF_USE_CH), (scfsr & ~(0x2))); /* clear SCFSR.RDF */
		__DMB();
	}

	return (0xff & (int)scfrdr);
}

/*
 *  シリアルI/Oポートへの文字列送信
 */
int IMPL_UART_puts(int portno, const char *s)
{
	int		cnt = 0;
	/* TODO portno */

	solid_cs_assert(*SCIF_SCBRR(SCIF_USE_CH) != (uint8_t)0);

	while (*s) {
		_write1Byte(SCIF_USE_CH, *s);	
		s++;	
		cnt++;
	}

	__DMB();
	while( (read_mem16(SCIF_SCFSR(SCIF_USE_CH))&(1<<6)) == 0);	/* wait for TEND to deassert. */

	return cnt;
}

/*
 *  シリアルI/Oポートへのデータ書き込み
 */
int IMPL_UART_write(int portno, const char *data, size_t length)
{
	int		cnt = 0;
	/* TODO portno */

	solid_cs_assert(*SCIF_SCBRR(SCIF_USE_CH) != (uint8_t)0);

	while (cnt < length) {
		_write1Byte(SCIF_USE_CH, *data);	
		data++;
		cnt++;
	}

	__DMB();
	while( (read_mem16(SCIF_SCFSR(SCIF_USE_CH))&(1<<6)) == 0);	/* wait for TEND to deassert. */

	return cnt;
}

/*
*　割込みの許可
*/
void IMPL_UART_enable_int(int portno, unsigned long mask)
{
	uint16_t	scscr;
	scscr = read_mem16(SCIF_SCSCR(portno));

	if (mask & SOLID_UART_INT_RX)	scscr |= 0x40;
	if (mask & SOLID_UART_INT_TXEMPTY)	scscr |= 0x80;
	if (mask & SOLID_UART_INT_ERROR)	scscr |= 0x8;

	write_mem16(SCIF_SCSCR(portno),scscr);
	__DMB();
}

/*
*  割込みの禁止
*/
void IMPL_UART_disable_int(int portno, unsigned long mask)
{
	uint16_t	scscr;
	scscr = read_mem16(SCIF_SCSCR(portno));

	if (mask & SOLID_UART_INT_RX)	scscr &= ~0x40;
	if (mask & SOLID_UART_INT_TXEMPTY)	scscr &= ~0x80;
	if (mask & SOLID_UART_INT_ERROR)	scscr &= ~0x8;

	write_mem16(SCIF_SCSCR(portno),scscr);
	__DMB();
}
