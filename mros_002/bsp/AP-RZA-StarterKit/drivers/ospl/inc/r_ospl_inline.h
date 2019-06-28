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
* Copyright (C) 2012 - 2016 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/***********************************************************************
* File: r_ospl_inline.h
*    OS Porting Layer API
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef R_OSPL_INLINE_H
#define R_OSPL_INLINE_H
#ifndef  NOT_DEFINE_INLINE_FUNCTION


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/
/* In "r_ospl_typedef.h" */

/******************************************************************************
Macro definitions
******************************************************************************/
/* In "r_ospl_typedef.h" */

/******************************************************************************
Variable Externs
******************************************************************************/
/* In "r_ospl_typedef.h" */

/******************************************************************************
Functions Prototypes
******************************************************************************/


/***********************************************************************
* Implement: R_OSPL_CALLER_GetRootChannelNum
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_CALLER_GetRootChannelNum
#endif
INLINE int_fast32_t  R_OSPL_CALLER_GetRootChannelNum( const r_ospl_caller_t* const  self )
{
    int_fast32_t  root_channel_num;

    IF_DQ( self == NULL )
    {
        root_channel_num = 0;
    }
    else
    {
        root_channel_num = self->I_LockVTable->GetRootChannelNum( self->I_Lock );
    }

    return  root_channel_num;
}


/***********************************************************************
* Implement: R_OSPL_FTIMER_TimeToCount
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_FTIMER_TimeToCount
#endif
INLINE uint32_t  R_OSPL_FTIMER_TimeToCount( const r_ospl_ftimer_spec_t* const  ts,
        uint32_t const  msec )
{
    uint32_t  count;

    IF_DQ( ts == NULL )
    {
        count = 0;
    }
    else
    {
        count = ( ((msec * ts->msec_Denominator) + ts->msec_Numerator) - 1u ) / ts->msec_Numerator;
    }
    return  count;
}


/***********************************************************************
* Implement: R_OSPL_FTIMER_CountToTime
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_FTIMER_CountToTime
#endif
INLINE uint32_t  R_OSPL_FTIMER_CountToTime( const r_ospl_ftimer_spec_t* const  ts,
        uint32_t const  Count )
{
    uint32_t  time;

    IF_DQ( ts == NULL )
    {
        time = 0;
    }
    else
    {
        time = ( Count * ts->msec_Numerator ) / ts->msec_Denominator;
    }
    return  time;
}


/***********************************************************************
* Implement: BIT_And_Sub
************************************************************************/
#ifdef _SH
#pragma inline BIT_And_Sub
#endif
INLINE uint_fast32_t  BIT_And_Sub( bit_flags_fast32_t const  Variable,
                                   bit_flags_fast32_t const  ConstValue )
{
    return  ((Variable) & (ConstValue));
}


/***********************************************************************
* Implement: R_OSPL_EVENT_OBJECT_Cast
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_EVENT_OBJECT_Cast
#endif
INLINE r_ospl_event_flags_t  R_OSPL_EVENT_OBJECT_Cast( r_ospl_event_object_id_t const  in_EventObjectId )
{
    return  (r_ospl_event_flags_t) in_EventObjectId;
}


/***********************************************************************
* Implement: R_OSPL_IsSetBitsCount1
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_IsSetBitsCount1
#endif
INLINE bool_t  R_OSPL_IsSetBitsCount1( uint32_t  in_Value )
{
    return  ( in_Value << R_OSPL_CountLeadingZeros( in_Value ) ) == 0x80000000u;
}


/***********************************************************************
* Implement: R_OSPL_MergeErrNum
************************************************************************/
#ifdef _SH
#pragma inline R_OSPL_MergeErrNum
#endif
INLINE errnum_t  R_OSPL_MergeErrNum( errnum_t const  CurrentError,  errnum_t const  AppendError )
{
    errnum_t  e;

    if ( CurrentError != 0 )
    {
        e = CurrentError;
    }
    else
    {
        e = AppendError;
    }
    return  e;
}


/***********************************************************************
* Implement: R_OSPL_SET_TO_32_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else

#ifdef _SH
#pragma inline R_OSPL_SET_TO_32_BIT_REGISTER
#endif
INLINE void  R_OSPL_SET_TO_32_BIT_REGISTER( volatile uint32_t* const  Register,
        uint32_t const  Mask,  int_fast32_t const  Shift,  uint32_t const  Value )
{
    uint32_t  reg_value;

    IF_DQ ( Register == NULL ) {}
    else
    {
        reg_value = *Register;
        reg_value = ( reg_value & ~Mask ) | ( ( Value << Shift ) & Mask );
        *Register = reg_value;
    }
}

#endif


/***********************************************************************
* Implement: R_OSPL_SET_TO_16_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else

#ifdef __CC_ARM
/* This inline functions is not expanded on __CC_ARM 5.15 */
#endif
#ifdef _SH
#pragma inline R_OSPL_SET_TO_16_BIT_REGISTER
#endif
INLINE void  R_OSPL_SET_TO_16_BIT_REGISTER( volatile uint16_t* const  Register,
        uint16_t const  Mask,  int_fast32_t const  Shift,  uint16_t const  Value )
{
    uint16_t  reg_value;

    IF_DQ ( Register == NULL ) {}
    else
    {
        reg_value = *Register;
        reg_value = (uint16_t)( ( (uint_fast32_t) reg_value & ~(uint_fast32_t) Mask ) |
                                ( ( (uint_fast32_t) Value << Shift ) & (uint_fast32_t) Mask ) );
        /* Cast is for SEC R2.4.2 */
        *Register = reg_value;
    }
}

#endif


/***********************************************************************
* Implement: R_OSPL_SET_TO_8_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else

#ifdef __CC_ARM
/* This inline functions is not expanded on __CC_ARM 5.15 */
#endif
#ifdef _SH
#pragma inline R_OSPL_SET_TO_8_BIT_REGISTER
#endif
INLINE void  R_OSPL_SET_TO_8_BIT_REGISTER( volatile uint8_t* const  Register,
        uint8_t const  Mask,  int_fast32_t const  Shift,  uint8_t const  Value )
{
    uint8_t  reg_value;

    IF_DQ ( Register == NULL ) {}
    else
    {
        reg_value = *Register;
        reg_value = (uint8_t)( ( (uint_fast32_t) reg_value & ~(uint_fast32_t) Mask ) |
                               ( ( (uint_fast32_t) Value << Shift ) & (uint_fast32_t) Mask ) );
        /* Cast is for SEC R2.4.2 */
        *Register = reg_value;
    }
}

#endif


/***********************************************************************
* Implement: R_OSPL_GET_FROM_32_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

#ifdef __CC_ARM
/* This inline functions is not expanded on __CC_ARM 5.15 */
#endif
#ifdef _SH
#pragma inline R_OSPL_GET_FROM_32_BIT_REGISTER
#endif
INLINE uint32_t  R_OSPL_GET_FROM_32_BIT_REGISTER( volatile const uint32_t* const  RegisterAddress,
        uint32_t const  Mask,  int_fast32_t const  Shift )
{
    uint32_t  reg_value;

    IF_DQ ( RegisterAddress == NULL )
    {
        enum { num = 0x0EDEDEDE };  /* SEC M1.10.1 */
        reg_value = num;
    }
    else
    {
        reg_value = *RegisterAddress;
        reg_value = ( reg_value & Mask ) >> Shift;
    }
    return  reg_value;
}

#endif


/***********************************************************************
* Implement: R_OSPL_GET_FROM_16_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

#ifdef __CC_ARM
/* This inline functions is not expanded on __CC_ARM 5.15 */
#endif
#ifdef _SH
#pragma inline R_OSPL_GET_FROM_16_BIT_REGISTER
#endif
INLINE uint16_t  R_OSPL_GET_FROM_16_BIT_REGISTER( volatile const uint16_t* const  RegisterAddress,
        uint16_t const  Mask,  int_fast32_t const  Shift )
{
    uint16_t  reg_value;

    IF_DQ ( RegisterAddress == NULL )
    {
        enum { num = 0xDEDE };  /* SEC M1.10.1 */
        reg_value = num;
    }
    else
    {
        reg_value = *RegisterAddress;
        reg_value = (uint16_t)( ( (uint_fast32_t) reg_value & (uint_fast32_t) Mask ) >> Shift );
        /* Cast is for SEC R2.4.2 */
    }
    return  reg_value;
}

#endif


/***********************************************************************
* Implement: R_OSPL_GET_FROM_8_BIT_REGISTER
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO
#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

#ifdef __CC_ARM
/* This inline functions is not expanded on __CC_ARM 5.15 */
#endif
#ifdef _SH
#pragma inline R_OSPL_GET_FROM_8_BIT_REGISTER
#endif
INLINE uint8_t  R_OSPL_GET_FROM_8_BIT_REGISTER( volatile const uint8_t* const  RegisterAddress,
        uint8_t const  Mask,  int_fast32_t const  Shift )
{
    uint8_t  reg_value;

    IF_DQ ( RegisterAddress == NULL )
    {
        enum { num = 0xDE };  /* SEC M1.10.1 */
        reg_value = num;
    }
    else
    {
        reg_value = *RegisterAddress;
        reg_value = (uint8_t)( ( (uint_fast32_t) reg_value & (uint_fast32_t) Mask ) >> Shift );
        /* Cast is for SEC R2.4.2 */
    }
    return  reg_value;
}

#endif


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif  /* NOT_DEFINE_INLINE_FUNCTION */
#endif /* R_OSPL_H */

