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
* $FileName: pfv_iobitmask.h $
* $Module: PFV $ $PublicVersion: 1.00 $ (=PFV_VERSION)
* $Rev: 51 $
* $Date:: 2014-03-14 18:42:33 +0900#$
* Description : PFV register bit mask define header
*******************************************************************************/
#ifndef PFV_IOBITMASK_H
#define PFV_IOBITMASK_H


/*[R_PFV_SET_REGISTER_BIT_FIELD]*/
/*[R_PFV_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_PFV_SET_REGISTER_BIT_FIELD( \
		in_out_Register, RegisterName, BitName, Value ) \
	R_PFV_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		in_out_Register, RegisterName, BitName, Value, \
		PFV__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "PFV__BIT_WIDTH__##RegisterName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_SET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		in_out_Register, RegisterName, BitName, Value, BitWidth ) \
	R_PFV_SET_REGISTER_BIT_FIELD_SUB0( \
		in_out_Register, RegisterName##__##BitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_SET_REGISTER_BIT_FIELD_SUB0( \
		in_out_Register, RegisterBitName, Value, BitWidth ) \
	R_PFV_SET_REGISTER_BIT_FIELD_SUB( \
		in_out_Register, RegisterBitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_PFV_SET_REGISTER_BIT_FIELD_SUB( \
		in_out_Register, RegisterBitName, Value, BitWidth ) \
	R_OSPL_SET_TO_##BitWidth##_BIT_REGISTER( \
		(volatile uint##BitWidth##_t*)(in_out_Register), \
		PFV__MASK##BitWidth##__##RegisterBitName, \
		PFV__SHIFT__##RegisterBitName, \
		(uint##BitWidth##_t)(Value) )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_PFV_GET_REGISTER_BIT_FIELD]*/
/*[R_PFV_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_PFV_GET_REGISTER_BIT_FIELD( \
		RegisterValue, RegisterName, BitName ) \
	R_PFV_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		RegisterValue, RegisterName, BitName, \
		PFV__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "PFV__BIT_WIDTH__##RegisterName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_GET_REGISTER_BIT_FIELD_WITH_REG_WIDTH( \
		RegisterValue, RegisterName, BitName, BitWidth ) \
	R_PFV_GET_REGISTER_BIT_FIELD_SUB0( \
		RegisterValue, RegisterName##__##BitName, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_GET_REGISTER_BIT_FIELD_SUB0( \
		RegisterValue, RegisterBitName, BitWidth ) \
	R_PFV_GET_REGISTER_BIT_FIELD_SUB( \
		RegisterValue, RegisterBitName, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_PFV_GET_REGISTER_BIT_FIELD_SUB( \
		RegisterValue, RegisterBitName, BitWidth ) \
	R_OSPL_GET_FROM_##BitWidth##_BIT_REGISTER( \
		(volatile const uint##BitWidth##_t*) &(RegisterValue), \
		PFV__MASK##BitWidth##__##RegisterBitName, \
		PFV__SHIFT__##RegisterBitName )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_PFV_IS_OVERFLOW_BIT_FIELD]*/
/*[R_PFV_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH]*/
/* ->MISRA 19.12 */ /* ->MISRA 19.13 */ /* ->SEC M5.1.2 (1) */

#define  R_PFV_IS_OVERFLOW_BIT_FIELD( \
		RegisterName, BitName, Value ) \
	R_PFV_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH( \
		RegisterName, BitName, Value, PFV__BIT_WIDTH__##RegisterName )

/* ->MISRA 19.7 : Expand "PFV__BIT_WIDTH__##RegisterName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_IS_OVERFLOW_BIT_FIELD_WITH_REG_WIDTH( \
		RegisterName, BitName, Value, BitWidth ) \
	R_PFV_IS_OVERFLOW_BIT_FIELD_SUB0( \
		RegisterName##__##BitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

/* ->MISRA 19.7 : Expand "RegisterName##__##BitName" macro */
/* ->SEC M5.1.3 */
#define  R_PFV_IS_OVERFLOW_BIT_FIELD_SUB0( \
		RegisterBitName, Value, BitWidth ) \
	R_PFV_IS_OVERFLOW_BIT_FIELD_SUB( \
		RegisterBitName, Value, BitWidth )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */

#define  R_PFV_IS_OVERFLOW_BIT_FIELD_SUB( \
		RegisterBitName, Value, BitWidth ) \
	R_PFV_IsOverflowBitField##BitWidth##_Sub( \
		PFV__MASK##BitWidth##__##RegisterBitName, \
		PFV__SHIFT__##RegisterBitName, \
		Value )
/* <-MISRA 19.12 */ /* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/*[R_PFV_IsOverflowBitField32_Sub]*/
/*[R_PFV_IsOverflowBitField16_Sub]*/
/*[R_PFV_IsOverflowBitField8_Sub]*/
INLINE bool_t  R_PFV_IsOverflowBitField32_Sub( uint32_t const  Mask,
        int_fast32_t const  Shift,  uint32_t const  Value )
{
    return  ( ( (uint32_t)(Value) &
                ~( (uint32_t)(Mask) >> (Shift) ) )
              != 0u );
}

INLINE bool_t  R_PFV_IsOverflowBitField16_Sub( uint16_t const  Mask,
        int_fast32_t const  Shift,  uint16_t const  Value )
{
    return  ( ( (uint_fast16_t)(Value) &
                ~( (uint_fast16_t)(Mask) >> (Shift) ) )
              != 0u );
}

INLINE bool_t  R_PFV_IsOverflowBitField8_Sub( uint8_t const  Mask,
        int_fast32_t const  Shift, uint8_t const  Value )
{
    return  ( ( (uint_fast8_t)(Value) &
                ~( (uint_fast8_t)(Mask) >> (Shift) ) )
              != 0u );
}



/* [PFV.PFVCR] : 0xE8205000 */
/*  PFV__BIT_WIDTH__(RegisterName) */
/*  PFV__MASK32__(RegisterName)__(BitFieldName) */
/*  PFV__SHIFT__(RegisterName)__(BitFieldName) */

#define  PFV__BIT_WIDTH__PFVCR  32

enum   /*uint32_t */
{
    PFV__MASK32__PFVCR__PFVE       = 0x00010000,  /* Enable */
    PFV__MASK32__PFVCR__DTH_ON     = 0x00001000,  /* Dither. (47.3.4)  */
    PFV__MASK32__PFVCR__IFMT       = 0x00000C00,
    PFV__MASK32__PFVCR__OFMT       = 0x00000300,
    PFV__MASK32__PFVCR__DINSWAP32  = 0x0000000C,  /* PFV_ARGB8888, ... */
    PFV__MASK32__PFVCR__DOUTSWAP32 = 0x00000003   /* PFV_ARGB8888, ... */
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__PFVE       = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__DTH_ON     = 12 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__IFMT       = 10 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__OFMT       =  8 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__DINSWAP32  =  2 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVCR__DOUTSWAP32 =  0 };


/* [PFV.PFVICR] : 0xE8205004 */

#define  PFV__BIT_WIDTH__PFVICR  32

enum  /*uint32_t */
{
    PFV__MASK32__PFVICR__PFVEEN    = 0x00000040,
    PFV__MASK32__PFVICR__IFEN      = 0x00000020,
    PFV__MASK32__PFVICR__OFEN      = 0x00000010,
    PFV__MASK32__PFVICR__IDTRG     = 0x0000000C,
    PFV__MASK32__PFVICR__ODTRG     = 0x00000003
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFVICR__PFVEEN    =  6 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVICR__IFEN      =  5 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVICR__OFEN      =  4 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVICR__IDTRG     =  2 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVICR__ODTRG     =  0 };


/* [PFV.PFVISR] : 0xE8205008 */

#define  PFV__BIT_WIDTH__PFVISR  32

enum   /*uint32_t */
{
    PFV__MASK32__PFVISR__IFOVF  = 0x00000008,
    PFV__MASK32__PFVISR__OFUDF  = 0x00000004,
    PFV__MASK32__PFVISR__IFEMP  = 0x00000002,
    PFV__MASK32__PFVISR__OFFUL  = 0x00000001
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFVISR__IFOVF =  3 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVISR__OFUDF =  2 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVISR__IFEMP =  1 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVISR__OFFUL =  0 };


/* [PFV.PFVID] : 0xE820500C */

#define  PFV__BIT_WIDTH__PFVID  32

#define/*uint32_t */     PFV__MASK32__PFVID__ID    0xFFFFFFFFu   /* Input data */
enum { /*uint16_t */     PFV__MASK16__PFVID__ID  =     0xFFFF };

enum { /*int_fast32_t*/  PFV__SHIFT__PFVID__ID = 0 };


/* [PFV.PFVOD] : 0xE8205010 */

#define  PFV__BIT_WIDTH__PFVOD  32

#define/*uint32_t */     PFV__MASK32__PFVOD__OD    0xFFFFFFFFu   /* Output data */
enum { /*uint16_t */     PFV__MASK16__PFVOD__OD  =     0xFFFF };

enum { /*int_fast32_t*/  PFV__SHIFT__PFVOD__OD = 0 };


/* [PFV.PFVIFSR] : 0xE8205014 */

#define  PFV__BIT_WIDTH__PFVIFSR  32

enum { /*uint32_t */     PFV__MASK32__PFVIFSR__IFVD  = 0x0000003F };

enum { /*int_fast32_t*/  PFV__SHIFT__PFVIFSR__IFVD = 0 };


/* [PFV.PFVOFSR] : 0xE8205018 */

#define  PFV__BIT_WIDTH__PFVOFSR  32

enum { /*uint32_t */     PFV__MASK32__PFVOFSR__OFVD  = 0x0000003F };

enum { /*int_fast32_t*/  PFV__SHIFT__PFVOFSR__OFVD = 0 };


/* [PFV.PFVACR] : 0xE8205020 */

#define  PFV__BIT_WIDTH__PFVACR  32

enum { /*uint32_t */     PFV__MASK32__PFVACR__ALPHA  = 0x000000FF };

enum { /*int_fast32_t*/  PFV__SHIFT__PFVACR__ALPHA = 0 };


/* [PFV.PFV_MTX_MODE] : 0xE8205024 */

#define  PFV__BIT_WIDTH__PFV_MTX_MODE  32

#define  PFV_MTX_MODE__PFV_MTX_MD   PFV_MODE_MD

enum { /*uint32_t */     PFV__MASK32__PFV_MODE_MD  = 0x00000003 };

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_MODE_MD = 0 };


/* [PFV.PFV_MTX_YG_ADJ0] : 0xE8205028 */

#define  PFV__BIT_WIDTH__PFV_MTX_YG_ADJ0  32

#define  PFV_MTX_YG_ADJ0__PFV_MTX_YG   PFV_YG
#define  PFV_MTX_YG_ADJ0__PFV_MTX_GG   PFV_GG

enum   /*uint32_t */
{
    PFV__MASK32__PFV_YG  = 0x00FF0000,
    PFV__MASK32__PFV_GG  = 0x000007FF
};

enum { /*int_fast32_t */   PFV__SHIFT__PFV_YG  = 16 };
enum { /*int_fast32_t */   PFV__SHIFT__PFV_GG  =  0 };


/* [PFV.PFV_MTX_YG_ADJ1] : 0xE820502C */

#define  PFV__BIT_WIDTH__PFV_MTX_YG_ADJ1  32

#define  PFV_MTX_YG_ADJ1__PFV_MTX_GB   PFV_GB
#define  PFV_MTX_YG_ADJ1__PFV_MTX_GR   PFV_GR


enum   /*uint32_t */
{
    PFV__MASK32__PFV_GB  = 0x07FF0000,
    PFV__MASK32__PFV_GR  = 0x000007FF
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_GB = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFV_GR =  0 };


/* [PFV.PFV_MTX_CBB_ADJ0] : 0xE8205030 */

#define  PFV__BIT_WIDTH__PFV_MTX_CBB_ADJ0  32

#define  PFV_MTX_CBB_ADJ0__PFV_MTX_B    PFV_B
#define  PFV_MTX_CBB_ADJ0__PFV_MTX_BG   PFV_BG

enum   /*uint32_t */
{
    PFV__MASK32__PFV_B  = 0x00FF0000,
    PFV__MASK32__PFV_BG = 0x000007FF
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_B  = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFV_BG =  0 };


/* [PFV.PFV_MTX_CBB_ADJ1] : 0xE8205034 */

#define  PFV__BIT_WIDTH__PFV_MTX_CBB_ADJ1  32

#define  PFV_MTX_CBB_ADJ1__PFV_MTX_BB   PFV_BB
#define  PFV_MTX_CBB_ADJ1__PFV_MTX_BR   PFV_BR

enum   /*uint32_t */
{
    PFV__MASK32__PFV_BB = 0x07FF0000,
    PFV__MASK32__PFV_BR = 0x000007FF
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_BB = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFV_BR =  0 };


/* [PFV.PFV_MTX_CRR_ADJ0] : 0xE8205038 */

#define  PFV__BIT_WIDTH__PFV_MTX_CRR_ADJ0  32

#define  PFV_MTX_CRR_ADJ0__PFV_MTX_R    PFV_R
#define  PFV_MTX_CRR_ADJ0__PFV_MTX_RG   PFV_RG

enum   /*uint32_t */
{
    PFV__MASK32__PFV_R = 0x00FF0000,
    PFV__MASK32__PFV_RG = 0x000007FF
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_R = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFV_RG =  0 };


/* [PFV.PFV_MTX_CRR_ADJ1] : 0xE820503C */

#define  PFV__BIT_WIDTH__PFV_MTX_CRR_ADJ1  32

#define  PFV_MTX_CRR_ADJ1__PFV_MTX_RB   PFV_RB
#define  PFV_MTX_CRR_ADJ1__PFV_MTX_RR   PFV_RR

enum   /*uint32_t */
{
    PFV__MASK32__PFV_RB = 0x07FF0000,
    PFV__MASK32__PFV_RR = 0x000007FF
};

enum { /*int_fast32_t*/  PFV__SHIFT__PFV_RB = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFV_RR =  0 };


/* [PFV.PFVSZR] : 0xE8205040 */

#define  PFV__BIT_WIDTH__PFVSZR  32

#define/*uint32_t */     PFV__MASK32__PFVSZR__PFVSZX   0xFFFF0000u
#define/*uint32_t */     PFV__MASK32__PFVSZR__PFVSZY   0x0000FFFF

enum { /*int_fast32_t*/  PFV__SHIFT__PFVSZR__PFVSZX = 16 };
enum { /*int_fast32_t*/  PFV__SHIFT__PFVSZR__PFVSZY =  0 };


/* [CPG.STBCR13] : 0xFCFE0470 */

#define  PFV__BIT_WIDTH__STBCR13  8

enum   /*uint8_t */
{
    PFV__MASK8__STBCR13__MSTP131 = 0x02,  /* PFV0 */
    PFV__MASK8__STBCR13__MSTP132 = 0x04   /* PFV1 */
};

enum { /*int_fast32_t*/  PFV__SHIFT__STBCR13__MSTP131 =  1 };
enum { /*int_fast32_t*/  PFV__SHIFT__STBCR13__MSTP132 =  2 };


#endif /* PFV_IOBITMASK_H */
