#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "easy_arm.h"
#include "solid_iores.h"
#include "solid_log.h"
#include "impl_net_dev.h"
#include "solid_cs_assert.h"

//#define	TEST_E2PROM_READ

static	uint8_t	_ethernetMACEmpty[] = {0,0,0,0,0,0};
static	uint8_t	_ethernetMACAddress[] = {0,0,0,0,0,0};

#define	I2C_ADDR_IC2_AP_RZA_1A		0b1010000
#define	ROM_WORDADDR_MAC					0

static	int readEEPROM(uint8_t	slave_addr, uint8_t word_addr, uint8_t *buf, size_t len);
#ifdef	TEST_E2PROM_READ
static	void runReadEEPROMTest(void);
#define	RUN_ROM_READ_TEST() 		runReadEEPROMTest()
#else	/* !TEST_E2PROM_READ */
#define	RUN_ROM_READ_TEST()
#endif	/* TEST_E2PROM_READ */

void IMPL_NET_DEV_get_macaddr(uint8_t macaddr[], size_t len)
{
		RUN_ROM_READ_TEST();
		if (0 == memcmp(_ethernetMACEmpty, _ethernetMACAddress, sizeof(_ethernetMACAddress))) {
				readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, ROM_WORDADDR_MAC, _ethernetMACAddress, len);
		}
		memcpy( macaddr, _ethernetMACAddress, len);
}

#ifdef	TEST_E2PROM_READ
const static uint8_t	_expected_data[] = {0x00,0x0c,0x7b,0x3b,0x01,0x0c,0xff,0xff,0xff,0xff,0xff,0xff};
void runReadEEPROMTest(void)
{
		int		i;
		uint8_t	work_buf[sizeof(_expected_data)];
		uint8_t	*p_dst;
		uint8_t	off;

		#define	ERASEBUF()			memset(work_buf, 0xee, sizeof(work_buf))
		#define	CHECK(roff,len)		solid_cs_assert( 0 == memcmp(&(_expected_data[roff]), work_buf, (len)));

		/* single byte read * 6 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		for ( i = 0; i < 6; i++, p_dst++) {
				readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, (off + i), p_dst, 1);
		}
		CHECK(off,6);

		/* 2 bytes read * 3 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		for ( i = 0; i < 3; i++, p_dst += 2) {
				readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, (off + i*2), p_dst, 2);
		}
		CHECK(off,6);

		/* 3 bytes read * 2 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		for ( i = 0; i < 2; i++, p_dst += 3) {
				readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, (off + i*3), p_dst, 3);
		}
		CHECK(off,6);

		/* 6 bytes read * 1 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, off, p_dst, 6);
		CHECK(off,6);

		/* 4bytes read * 2 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		for ( i = 0; i < 2; i++, p_dst += 4) {
				readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, (off + i*4), p_dst, 4);
		}
		CHECK(off,8);

		/* 12 bytes read * 1 */
		ERASEBUF();
		p_dst = work_buf;
		off = 0;
		readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, off, p_dst, 12);
		CHECK(off,12);

		/* 6bytes read offset(+3) * 1 */
		ERASEBUF();
		p_dst = work_buf;
		off = 3;
		readEEPROM(I2C_ADDR_IC2_AP_RZA_1A, off, p_dst, 6);
		CHECK(off,6);

}

#endif	/* TEST_E2PROM_READ */

#define	I2C_CH			1

#define		IGNORE_DUMMY_DATA(val)		(void)(val)

#define		CR1_ICE		(1<<7)
#define		CR1_IICRST	(1<<6)
#define		CR1_SCLO	(1<<3)
#define		CR1_SDAO	(1<<2)
#define		CR1_SCLI	(1<<1)
#define		CR1_SDAI	(1<<0)

#define		CR2_BBSY	(1<<7)
#define		CR2_MST		(1<<6)
#define		CR2_SP		(1<<3)
#define		CR2_RS		(1<<2)
#define		CR2_ST		(1<<1)

#define		MR1_BCWP	(1<<3)

#define		MR2_DLCS	(1<<7)
#define		MR2_SDDL(c)	((0x7&(c))<<4)

#define		MR3_WAIT	(1<<6)
#define		MR3_RDRFS	(1<<5)
#define		MR3_ACKWP	(1<<4)
#define		MR3_ACKBT	(1<<3)

#define		SER_SAR0E	(1<<0)

#define		FER_SCLE	(1<<6)
#define		FER_NFE		(1<<5)
#define		FER_NACKE	(1<<4)
#define		FER_SALE	(1<<3)
#define		FER_NALE	(1<<2)
#define		FER_MALE	(1<<1)

#define		SR2_TDRE	(1<<7)
#define		SR2_TEND	(1<<6)
#define		SR2_RDRF	(1<<5)
#define		SR2_NACKF	(1<<4)
#define		SR2_STOP	(1<<3)

typedef struct {
		uint32_t    cr1;
		uint32_t    cr2;
		uint32_t    mr1;
		uint32_t    mr2;
		uint32_t    mr3;
		uint32_t    fer;
		uint32_t    ser;
		uint32_t    ier;
		uint32_t    sr1;
		uint32_t    sr2;
		uint32_t    sar0;
		uint32_t    sar1;
		uint32_t    sar2;
		uint32_t    brl;
		uint32_t    brh;
		uint32_t    drt;
		uint32_t    drr;
} _riic_reg_t;

//#define TRACLE_SIGNALS
#ifdef TRACLE_SIGNALS

#include	<stdbool.h>

#define		MAX_I2C_SIGNAL_LOG		100

typedef struct {
		int		__line__;
		bool	sclo;
		bool	sdao;
		bool	scli;
		bool	sdai;
} _rec_signals_t;

static	_rec_signals_t		_i2c_sig_log[MAX_I2C_SIGNAL_LOG];
static	int					_i2c_sig_log_pos = 0;

static __inline__ void _log_i2c_signals(int line, volatile _riic_reg_t *reg)
{
	_rec_signals_t		*rec = &_i2c_sig_log[_i2c_sig_log_pos++];
	uint32_t			cr1 = reg->cr1;

	rec->__line__ = line;
	rec->sclo = (0 != (cr1 & CR1_SCLO));
	rec->sdao = (0 != (cr1 & CR1_SDAO));
	rec->scli = (0 != (cr1 & CR1_SCLI));
	rec->sdai = (0 != (cr1 & CR1_SDAI));

	if (_i2c_sig_log_pos >= MAX_I2C_SIGNAL_LOG) _i2c_sig_log_pos = 0;
}

#define		monitor_pins(reg) 	_log_i2c_signals(__LINE__, (reg))

#else /* !TRACLE_SIGNALS */
#define		monitor_pins(reg)
#endif /* TRACLE_SIGNALS */


static void waitRDRF( volatile _riic_reg_t *reg )
{
		while (!(reg->sr2 & SR2_RDRF));
		__DMB();
		monitor_pins(reg);
		reg->sr2 &= ~SR2_RDRF;
		__DMB();
		monitor_pins(reg);
}

typedef enum {
		RIIC_ACK = true,
		RIIC_NACK = false
} riic_ack_t;

static void setACKBit( volatile _riic_reg_t *reg, riic_ack_t ack )
{
		uint32_t	dummy;
		reg->mr3 |= MR3_ACKWP;
		__DMB();
		monitor_pins(reg);

		dummy = reg->mr3;
		__DMB();
		monitor_pins(reg);

		IGNORE_DUMMY_DATA(dummy);

		if (ack) {
			reg->mr3 &= ~MR3_ACKBT;	// ACK
		} else {
			reg->mr3 |= MR3_ACKBT;	// NACK^
		}
		__DMB();
		monitor_pins(reg);
}

static int	writeWord(volatile _riic_reg_t *reg, uint8_t data)
{
		while (!(reg->sr2 & SR2_TDRE))
				;

		monitor_pins(reg);

		reg->sr2 &= ~SR2_TDRE;
		__DMB();
		monitor_pins(reg);

		reg->drt = data;
		__DMB();
		monitor_pins(reg);

		if (reg->sr2 & SR2_NACKF ) {
				return -1;
		}

		return 0;
}

typedef enum {
		RWACT_NONE,
		RWACT_SET_NACK,
		RWACT_SET_LOW_HOLD,
		RWACT_RESET_LOW_HOLD,
		RWACT_STOP_COND,
} _rw_act_t;

static int	readWord(volatile _riic_reg_t *reg, _rw_act_t act_before_read)
{
		uint8_t			val;

		waitRDRF(reg);
		__DMB();
		monitor_pins(reg);

		switch (act_before_read) {
				case RWACT_NONE:
						/* nothing to do here */
						break;
				case RWACT_SET_NACK:
						setACKBit(reg, RIIC_NACK);
						break;
				case RWACT_SET_LOW_HOLD:
						reg->mr3 |= MR3_WAIT;
						__DMB();
						monitor_pins(reg);
						break;
				case RWACT_RESET_LOW_HOLD:
						reg->mr3 &= ~MR3_WAIT;
						__DMB();
						monitor_pins(reg);
						break;
				case RWACT_STOP_COND:
						reg->sr2 &= ~SR2_STOP;
						__DMB();
						monitor_pins(reg);

						reg->cr2 |= CR2_SP;
						__DMB();
						monitor_pins(reg);
						break;
				default:
						return -1;
						break;
		}

		val = (uint8_t)(0xff & reg->drr);
		__DMB();
		monitor_pins(reg);

		return (0xff & (int)val);
}

static	int readEEPROM(uint8_t	slave_addr, uint8_t word_addr, uint8_t *buf, size_t len)
{
		int	err;
		SOLID_IORES_INFO		iores_riic1;
		volatile _riic_reg_t	*reg;
		uint8_t					dummy;
		uint8_t					*p = buf;
		int						ret = 0;
		size_t					cnt = 0;
		int						rdat;

		if (len < 1 ) {
				return -1;
		}

		/*
		 * I2Cコントローラ(RIIC)レジスタのマッピング
		 */

		err = SOLID_IORES_Use("RIIC1");
		solid_cs_assert( err == SOLID_ERR_OK );

		SOLID_IORES_GetInfo("RIIC1",&iores_riic1);
		solid_cs_assert( iores_riic1.addr );

		reg = (volatile _riic_reg_t*)iores_riic1.addr;

		/*
		 * I2Cコントローラの初期化
		 */

		monitor_pins(reg);

		reg->cr1 &= ~CR1_ICE;	// SCL,SDA端子駆動停止
		__DMB();
		monitor_pins(reg);
		reg->cr1 |= CR1_IICRST;	// RIICリセット
		__DMB();
		monitor_pins(reg);
		reg->cr1 |= CR1_ICE;	// 内部リセット
		__DMB();
		monitor_pins(reg);

		//	通信クロック設定
		reg->mr1 = (((0b011/* CKS=P0/8 */) << 4) | MR1_BCWP);	// where P0:33,30MHz
		__DMB();
		monitor_pins(reg);
		// バスビットレート(low)設定
		reg->brl = (0xe0 | 25);			// where P0:33,30MHz
		__DMB();
		monitor_pins(reg);
		// バスビットレート(high)設定
		reg->brh = (0xe0 | 25); 		//  where P0:33,30MHz
		__DMB();
		monitor_pins(reg);

		monitor_pins(reg);
		reg->cr1 &= ~CR1_IICRST;	// RIICリセット解除
		__DMB();
		monitor_pins(reg);

		/* setup 終了 */

		/*
		 * 読み出すアドレスをwriteコマンドで指定
		 */
		while (reg->cr2 & CR2_BBSY);
		__DMB();
		monitor_pins(reg);

		reg->cr2 |= CR2_ST;		// start condition
		__DMB();
		monitor_pins(reg);

		while ((reg->cr2 & CR2_ST))
				;

		monitor_pins(reg);

		/* start condition 完了 */

		/* アドレスを指定するためにdummy writeコマンドを発行 */

		while (!(reg->cr2 & CR2_BBSY))
				;

		monitor_pins(reg);
		solid_cs_assert( (reg->cr2 & CR2_MST) );

		// write slave addres + 'W'
		if (writeWord(reg, (((0x7f & slave_addr) << 1 ) | 0))) {
				SOLID_LOG_printf("got NACK from device\n");
				goto error_cleanup;
		}

		// write word address
		if (writeWord(reg, word_addr )) {
				SOLID_LOG_printf("got NACK from device\n");
				goto error_cleanup;
		}

		while (!(reg->sr2 & SR2_TEND))	// wait for transfer complete
				;

		/*
		 * データをEEPROMから受信
		 */
		// restart condition (start -> start)
		while (!(reg->cr2 & CR2_BBSY));
		__DMB();
		monitor_pins(reg);

		reg->cr2 |= CR2_RS;		// restart condition
		__DMB();
		monitor_pins(reg);

		while ((reg->cr2 & CR2_RS))
				;

		monitor_pins(reg);

		while (!(reg->cr2 & CR2_BBSY))
				;

		monitor_pins(reg);
		solid_cs_assert( (reg->cr2 & CR2_MST) );

		// write slave addres + 'R'
		if (writeWord(reg, (((0x7f & slave_addr) << 1 ) | 1))) {
				SOLID_LOG_printf("got NACK from device\n");
				goto error_cleanup;
		}

		{ // dummy read

			dummy = readWord(reg, RWACT_NONE);
			if ( dummy < 0 ) {
				SOLID_LOG_printf("failed at 'dummy-read' \n");
				goto error_cleanup;
			}
			IGNORE_DUMMY_DATA(dummy);

		} // dummy read

		if (len == 1) {
				setACKBit(reg, RIIC_NACK);
		} else if ( len == 2) {

				rdat = readWord(reg, RWACT_SET_NACK);
				if ( rdat < 0 ) {
						SOLID_LOG_printf("failed at 'read 1/2' \n");
						goto error_cleanup;
				}
				*p++ = (uint8_t)(0xff & rdat);

		} else {
				cnt = len - 3;

				while (cnt > 0 ) {

						rdat = readWord(reg, RWACT_NONE);
						if ( rdat < 0 ) {
								SOLID_LOG_printf("failed at 'read %d/%d' \n", ((len-2) - cnt), len);
								goto error_cleanup;
						}
						*p++ = (uint8_t)(0xff & rdat);

						cnt--;
				}

				// last byte -2
				rdat = readWord(reg, RWACT_SET_LOW_HOLD);
				if ( rdat < 0 ) {
						SOLID_LOG_printf("failed at 'read %d/%d' \n", (len -2), len);
						goto error_cleanup;
				}
				*p++ = (uint8_t)(0xff & rdat);

				// last byte -1
				rdat = readWord(reg, RWACT_SET_NACK);
				if ( rdat < 0 ) {
						SOLID_LOG_printf("failed at 'read %d/%d' \n", (len -2), len);
						goto error_cleanup;
				}
				*p++ = (uint8_t)(0xff & rdat);
		}

		{ // last 1byte read

				rdat = readWord(reg, RWACT_STOP_COND);
				if ( rdat < 0 ) {
						SOLID_LOG_printf("failed at 'read %d/%d' \n", (len -2), len);
						goto error_cleanup;
				}
				*p++ = (uint8_t)(0xff & rdat);


				reg->mr3 |= MR3_WAIT;
				__DMB();
				monitor_pins(reg);
		}

		ret = (int)(p - buf);

cleanup_and_exit:
		while (!(reg->sr2 & SR2_STOP));

		reg->mr3 &= ~MR3_WAIT;
		__DMB();
		monitor_pins(reg);

		reg->sr2 &= ~SR2_NACKF;
		__DMB();
		monitor_pins(reg);

		reg->sr2 &= ~SR2_STOP;
		__DMB();
		monitor_pins(reg);

		// I2Cコントローラレジスタのマッピングを解放する
		err = SOLID_IORES_Unuse("RIIC1");
		solid_cs_assert( err == SOLID_ERR_OK );

		return ret;

error_cleanup:

		reg->sr2 &= ~SR2_STOP;
		reg->cr2 |= CR2_SP;
		__DMB();
		monitor_pins(reg);

		dummy = (uint8_t)(0xff & reg->drr);
		__DMB();
		monitor_pins(reg);
		IGNORE_DUMMY_DATA(dummy);

		ret = -1;
		goto cleanup_and_exit;
}
