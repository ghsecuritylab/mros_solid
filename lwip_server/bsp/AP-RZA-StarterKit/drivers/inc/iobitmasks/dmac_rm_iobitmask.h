/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2013 - 2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* $FileName: dmac_rm_iobitmask.h $
* $Module: PFV $ $PublicVersion: 1.00 $ (=PFV_VERSION)
* $Rev: 37 $
* $Date:: 2013-12-12 13:59:31 +0900#$
* Description : PFV register bit mask define header
*******************************************************************************/
#ifndef DMAC_RM_IOBITMASK_H
#define DMAC_RM_IOBITMASK_H

/*[R_DMAC_RM_SET_REGISTER_BIT_FIELD]*/
/*[R_DMAC_RM_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_DMAC_RM_SET_REGISTER_BIT_FIELD( \
		in_out_Register, RegisterName, BitName, Value ) \
	R_DMAC_RM_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		in_out_Register, RegisterName, BitName, Value, DMAC_T__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "DMAC__BIT_WIDTH__##RegisterName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		in_out_Register, RegisterName, BitName, Value, BitWidth ) \
	R_DMAC_RM_SET_REGISTER_BIT_FIELD_SUB0( \
		in_out_Register, RegisterName##__##BitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_SET_REGISTER_BIT_FIELD_SUB0( in_out_Register, RegisterBitName, Value, BitWidth ) \
	R_DMAC_RM_SET_REGISTER_BIT_FIELD_SUB( in_out_Register, RegisterBitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_DMAC_RM_SET_REGISTER_BIT_FIELD_SUB( in_out_Register, RegisterBitName, Value, BitWidth ) \
	R_OSPL_SET_TO_##BitWidth##_BIT_REGISTER( (volatile uint##BitWidth##_t*)(in_out_Register), \
		DMAC_T__MASK##BitWidth##__##RegisterBitName, \
		DMAC_T__SHIFT__##RegisterBitName, \
		(uint##BitWidth##_t)(Value) )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_DMAC_RM_GET_REGISTER_BIT_FIELD]*/
/*[R_DMAC_RM_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_DMAC_RM_GET_REGISTER_BIT_FIELD( \
		RegisterValue, RegisterName, BitName ) \
	R_DMAC_RM_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		RegisterValue, RegisterName, BitName, DMAC_T__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "DMAC__BIT_WIDTH__##RegisterName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( RegisterValue, RegisterName, BitName, BitWidth ) \
	R_DMAC_RM_GET_REGISTER_BIT_FIELD_SUB0( RegisterValue, RegisterName##__##BitName, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_GET_REGISTER_BIT_FIELD_SUB0( RegisterValue, RegisterBitName, BitWidth ) \
	R_DMAC_RM_GET_REGISTER_BIT_FIELD_SUB( RegisterValue, RegisterBitName, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_DMAC_RM_GET_REGISTER_BIT_FIELD_SUB( RegisterValue, RegisterBitName, BitWidth ) \
	R_OSPL_GET_FROM_##BitWidth##_BIT_REGISTER( (volatile const uint##BitWidth##_t*) &(RegisterValue), \
		DMAC_T__MASK##BitWidth##__##RegisterBitName, \
		DMAC_T__SHIFT__##RegisterBitName )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_DMAC_RM_IS_OVERFLOW_BIT_FIELD]*/
/*[R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_DMAC_RM_IS_OVERFLOW_BIT_FIELD( RegisterName, BitName, Value ) \
	R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH( RegisterName, BitName, Value, \
		DMAC_T__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "DMAC__BIT_WIDTH__##RegisterName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH( RegisterName, BitName, Value, BitWidth ) \
	R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_SUB0( RegisterName##__##BitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */ /* ->SEC M5.1.3 */
#define  R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_SUB0( RegisterBitName, Value, BitWidth ) \
	R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_SUB( RegisterBitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_DMAC_RM_IS_OVERFLOW_BIT_FIELD_SUB( RegisterBitName, Value, BitWidth ) \
	R_DMAC_T_IsOverflowBitField##BitWidth##_Sub( \
		DMAC_T__MASK##BitWidth##__##RegisterBitName, \
		DMAC_T__SHIFT__##RegisterBitName, \
		Value )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_DMAC_T_IsOverflowBitField32_Sub]*/
/*[R_DMAC_T_IsOverflowBitField16_Sub]*/
/*[R_DMAC_T_IsOverflowBitField8_Sub]*/
INLINE bool_t  R_DMAC_T_IsOverflowBitField32_Sub( uint32_t const  Mask,
        int_fast32_t const  Shift,  uint32_t const  Value )
{
    return  ( ( (uint32_t)(Value) & ~( (uint32_t)(Mask) >> (Shift) ) ) != 0u );
}

INLINE bool_t  R_DMAC_T_IsOverflowBitField16_Sub( uint16_t const  Mask,
        int_fast32_t const  Shift,  uint16_t const  Value )
{
    return  ( ( (uint_fast16_t)(Value) & ~( (uint_fast16_t)(Mask) >> (Shift) ) ) != 0u );
}

INLINE bool_t  R_DMAC_T_IsOverflowBitField8_Sub( uint8_t const  Mask,
        int_fast32_t const  Shift, uint8_t const  Value )
{
    return  ( ( (uint_fast8_t)(Value) & ~( (uint_fast8_t)(Mask) >> (Shift) ) ) != 0u );
}



/* [DMAC_RM.CHCTRL_n] */
/*  DMAC_T = DMAC_RM (for C90 31characters) */
/*  DMAC_T__BIT_WIDTH__(RegisterName) */
/*  DMAC_T__MASK32__(RegisterName)__(BitFieldName) */
/*  DMAC_T__SHIFT__(RegisterName)__(BitFieldName) */

#define  DMAC_T__BIT_WIDTH__CHCTRL_n  32

#define  CHCTRL_n__CLRINTMSK  B_CLRINTMSK
#define  CHCTRL_n__SETINTMSK  B_SETINTMSK
#define  CHCTRL_n__CLRSUS     B_CLRSUS
#define  CHCTRL_n__SETSUS     B_SETSUS
#define  CHCTRL_n__CLRTC      B_CLRTC
#define  CHCTRL_n__CLREND     B_CLREND
#define  CHCTRL_n__CLRRQ      B_CLRRQ
#define  CHCTRL_n__SWRST      B_SWRST
#define  CHCTRL_n__STG        B_STG
#define  CHCTRL_n__CLREN      B_CLREN
#define  CHCTRL_n__SETEN      B_SETEN

/* DMAC_T = DMAC_RM (C90) */
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLRINTMSK = 0x00020000 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_SETINTMSK = 0x00010000 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLRSUS    = 0x00000200 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_SETSUS    = 0x00000100 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLRTC     = 0x00000040 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLREND    = 0x00000020 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLRRQ     = 0x00000010 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_SWRST     = 0x00000008 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_STG       = 0x00000004 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_CLREN     = 0x00000002 };
enum { /*uint32_t*/      DMAC_T__MASK32__B_SETEN     = 0x00000001 };

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLRINTMSK = 17 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_SETINTMSK = 16 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLRSUS    =  9 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_SETSUS    =  8 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLRTC     =  6 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLREND    =  5 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLRRQ     =  4 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_SWRST     =  3 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_STG       =  2 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_CLREN     =  1 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_SETEN     =  0 };


/* [DMAC_RM.CHCFG_n] */

#define  DMAC_T__BIT_WIDTH__CHCFG_n  32

/* DMAC_T = DMAC_RM (C90) */
#define                  DMAC_T__MASK32__CHCFG_n__DMS    0x80000000u
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__REN  = 0x40000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__RSW  = 0x20000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__RSEL = 0x10000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__SBE  = 0x08000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__DEM  = 0x01000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__TM   = 0x00400000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__DAD  = 0x00200000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__SAD  = 0x00100000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__DDS  = 0x000F0000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__SDS  = 0x0000F000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__AM   = 0x00000700 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__LVL  = 0x00000040 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__HIEN = 0x00000020 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__LOEN = 0x00000010 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__REQD = 0x00000008 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHCFG_n__SEL  = 0x00000007 };

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__DMS  = 31 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__REN  = 30 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__RSW  = 29 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__RSEL = 28 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__SBE  = 27 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__DEM  = 24 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__TM   = 22 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__DAD  = 21 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__SAD  = 20 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__DDS  = 16 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__SDS  = 12 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__AM   =  8 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__LVL  =  6 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__HIEN =  5 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__LOEN =  4 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__REQD =  3 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHCFG_n__SEL  =  0 };


/*[DMAC_RM.CHCFG_n.DMS]*/
/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__DMS_REGISTER_MODE = 0 };
enum { /*int_fast32_t*/  DMAC_T__DMS_LINK_MODE     = 1 };


/*[DMAC_RM.CHCFG_n.RSEL]*/
/* DMAC_T = DMAC_RM (C90) */
/*int_fast32_t*/
#define  DMAC_T__NEXT_REGISTER_SET_MIN_NUM  0
/*int_fast32_t*/  #define  DMAC_T__NEXT_REGISTER_SET_MAX_NUM  1


/* [DMAC_RM.DCTRL_nn] DCTRL_0_7(DCTRL_nn) */

#define  DMAC_T__BIT_WIDTH__DCTRL_nn  32

/* DMAC_T = DMAC_RM (C90) */
#define                  DMAC_T__MASK32__DCTRL_nn__LWCA    0xF0000000u
enum { /*uint32_t*/      DMAC_T__MASK32__DCTRL_nn__LWPR  = 0x07000000 };
enum { /*uint32_t*/      DMAC_T__MASK32__DCTRL_nn__LDCA  = 0x00F00000 };
enum { /*uint32_t*/      DMAC_T__MASK32__DCTRL_nn__LDPR  = 0x00070000 };
enum { /*uint32_t*/      DMAC_T__MASK32__DCTRL_nn__LVINT = 0x00000002 };
enum { /*uint32_t*/      DMAC_T__MASK32__DCTRL_nn__PR    = 0x00000001 };

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__LWCA  = 28 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__LWPR  = 24 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__LDCA  = 20 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__LDPR  = 16 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__LVINT =  1 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__DCTRL_nn__PR    =  0 };


/* [DMAC_RM.CHSTAT_n] */

#define  DMAC_T__BIT_WIDTH__CHSTAT_n  32

#define  CHSTAT_n__INTMSK  B_INTMSK

/* DMAC_T = DMAC_RM (C90) */
enum { /*uint32_t*/      DMAC_T__MASK32__B_INTMSK       = 0x00010000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__MODE = 0x00000800 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__DER  = 0x00000400 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__DW   = 0x00000200 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__DL   = 0x00000100 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__SR   = 0x00000080 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__TC   = 0x00000040 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__END  = 0x00000020 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__ER   = 0x00000010 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__SUS  = 0x00000008 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__TACT = 0x00000004 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__RQST = 0x00000002 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHSTAT_n__EN   = 0x00000001 };

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_INTMSK         = 16 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__MODE   = 11 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__DER    = 10 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__DW     =  9 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__DL     =  8 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__SR     =  7 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__TC     =  6 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__END    =  5 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__ER     =  4 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__SUS    =  3 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__TACT   =  2 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__RQST   =  1 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHSTAT_n__EN     =  0 };


/* [DMAC_RM.CHITVL_n] */

/*  DMAC_T = DMAC_RM (for C90 31characters) */
#define  DMAC_T__BIT_WIDTH__CHITVL_n  32

/*  DMAC_T = DMAC_RM (for C90 31characters) */
enum { /*uint32_t*/      DMAC_T__MASK32__CHITVL_n__ITVL = 0x0000FFFF };

/*  DMAC_T = DMAC_RM (for C90 31characters) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHITVL_n__ITVL = 0 };


/* [DMAC_RM.CHEXT_n] */

/*  DMAC_T = DMAC_RM (for C90 31characters) */
#define  DMAC_T__BIT_WIDTH__CHEXT_n  32

/*  DMAC_T = DMAC_RM (for C90 31characters) */
enum { /*uint32_t*/      DMAC_T__MASK32__CHEXT_n__DCA = 0x0000F000 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHEXT_n__DPR = 0x00000700 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHEXT_n__SCA = 0x000000F0 };
enum { /*uint32_t*/      DMAC_T__MASK32__CHEXT_n__SPR = 0x00000007 };

/*  DMAC_T = DMAC_RM (for C90 31characters) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHEXT_n__DCA = 12 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHEXT_n__DPR =  8 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHEXT_n__SCA =  4 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__CHEXT_n__SPR =  0 };


/* [DMAC_RM.DMARSn] */

#define  DMAC_T__BIT_WIDTH__DMARSn  32

#define  DMARSn__CH1__MID  B_MID1
#define  DMARSn__CH1__RID  B_RID1
#define  DMARSn__CH0__MID  B_MID0
#define  DMARSn__CH0__RID  B_RID0

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__MASK32__B_MID1 = 0x01FC0000 };
enum { /*int_fast32_t*/  DMAC_T__MASK32__B_RID1 = 0x00030000 };
enum { /*int_fast32_t*/  DMAC_T__MASK32__B_MID0 = 0x000001FC };
enum { /*int_fast32_t*/  DMAC_T__MASK32__B_RID0 = 0x00000003 };

/* DMAC_T = DMAC_RM (C90) */
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_MID1 = 18 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_RID1 = 16 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_MID0 =  2 };
enum { /*int_fast32_t*/  DMAC_T__SHIFT__B_RID0 =  0 };


#endif /* DMAC_RM_IOBITMASK_H */
