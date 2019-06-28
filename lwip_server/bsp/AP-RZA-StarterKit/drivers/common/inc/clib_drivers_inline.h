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
* Copyright (C) 2013 - 2015 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/***********************************************************************
* File: clib_drivers_inline.h
*    Common code for drivers and more.
*
* - $Module: CLibCommon $ $PublicVersion: 0.90 $ (=CLIB_VERSION)
* - $Rev: 30 $
* - $Date:: 2014-02-13 21:21:47 +0900#$
************************************************************************/

#ifndef  CLIB_DRIVERS_INLINE_H
#define  CLIB_DRIVERS_INLINE_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/
/* in clib_drivers_typedef.h */

/******************************************************************************
Macro definitions
******************************************************************************/
/* in clib_drivers_typedef.h */

/******************************************************************************
Variable Externs
******************************************************************************/
/* in clib_drivers_typedef.h */

/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* Implement: R_int_t_to_int8_t
*    Cast with range check
*
* Arguments:
*    input  - Input value
*    output - Output value
*
* Return Value:
*    Error Code. 0=No Error.
************************************************************************/
INLINE errnum_t  R_int_t_to_int8_t( int_t input, int8_t* output )
{
    return  R_int32_t_to_int8_t( input, output );
}


/***********************************************************************
* Implement: R_Mod_N
*    Fast mod operation
*
* Arguments:
*    value - Left operand
*
* Return Value:
*    value % N
*
* Description:
* - This is shared description of
*   R_Mod_2s, R_Mod_4s, R_Mod_8s, R_Mod_16s, R_Mod_32s, R_Mod_64s,
*   R_Mod_2u, R_Mod_4u, R_Mod_8u, R_Mod_16u, R_Mod_32u, R_Mod_64u.
* - Porting layer of faster than % operator.
* - MISRA & IPA SEC confirmed version.
* - Minus value is expected 2's complement.
* - Not same C99 % operator.
*   Sample: R_Mod_4s( -7 ) == +1.  Sample: -7 % 4 == -3
************************************************************************/


/***********************************************************************
* Implement: R_Mod_2s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_2s
#endif
INLINE int_fast32_t  R_Mod_2s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 1;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_4s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_4s
#endif
INLINE int_fast32_t  R_Mod_4s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 3;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_8s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_8s
#endif
INLINE int_fast32_t  R_Mod_8s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 7;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_16s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_16s
#endif
INLINE int_fast32_t  R_Mod_16s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 15;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_32s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_32s
#endif
INLINE int_fast32_t  R_Mod_32s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 31;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_64s
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_64s
#endif
INLINE int_fast32_t  R_Mod_64s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 63;  /* SEC M1.10.1 */
    return  ((int_fast32_t)((uint_fast32_t)(value) & mask));
}


/***********************************************************************
* Implement: R_Mod_2u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_2u
#endif
INLINE uint_fast32_t  R_Mod_2u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 1u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Mod_4u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_4u
#endif
INLINE uint_fast32_t  R_Mod_4u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 3u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Mod_8u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_8u
#endif
INLINE uint_fast32_t  R_Mod_8u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 7u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Mod_16u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_16u
#endif
INLINE uint_fast32_t  R_Mod_16u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 15u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Mod_32u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_32u
#endif
INLINE uint_fast32_t  R_Mod_32u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 31u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Mod_64u
*    Fast mod operation. See <R_Mod_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Mod_64u
#endif
INLINE uint_fast32_t  R_Mod_64u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 63u;  /* SEC M1.10.1 */
    return  ((value) & mask);
}


/***********************************************************************
* Implement: R_Ceil_N
*    Fast ceil operation
*
* Arguments:
*    value - Left operand
*
* Return Value:
*    Ceil( value / N )
*
* Description:
* - This is shared description of
*   R_Ceil_2s, R_Ceil_4s, R_Ceil_8s, R_Ceil_16s, R_Ceil_32s, R_Ceil_64s,
*   R_Ceil_2u, R_Ceil_4u, R_Ceil_8u, R_Ceil_16u, R_Ceil_32u, R_Ceil_64u.
* - Porting layer of fast ceil operation.
* - Function version is confirmed with MISRA & IPA SEC.
************************************************************************/


/***********************************************************************
* Implement: R_Ceil_2s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_2s
#endif
INLINE int_fast32_t  R_Ceil_2s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 1;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_4s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_4s
#endif
INLINE int_fast32_t  R_Ceil_4s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 3;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_8s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_8s
#endif
INLINE int_fast32_t  R_Ceil_8s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 7;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_16s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_16s
#endif
INLINE int_fast32_t  R_Ceil_16s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 15;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_32s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_32s
#endif
INLINE int_fast32_t  R_Ceil_32s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 31;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_64s
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_64s
#endif
INLINE int_fast32_t  R_Ceil_64s( int_fast32_t const  value )
{
    const uint_fast32_t  mask = 63;
    return  ((int_fast32_t)((uint_fast32_t)((value)+mask)&~mask));
}


/***********************************************************************
* Implement: R_Ceil_2u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_2u
#endif
INLINE uint_fast32_t  R_Ceil_2u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 1;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Ceil_4u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_4u
#endif
INLINE uint_fast32_t  R_Ceil_4u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 3;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Ceil_8u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_8u
#endif
INLINE uint_fast32_t  R_Ceil_8u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 7;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Ceil_16u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_16u
#endif
INLINE uint_fast32_t  R_Ceil_16u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 15;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Ceil_32u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_32u
#endif
INLINE uint_fast32_t  R_Ceil_32u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 31;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Ceil_64u
*    Fast ceil operation. See <R_Ceil_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Ceil_64u
#endif
INLINE uint_fast32_t  R_Ceil_64u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 63;
    return  (((value)+mask)&~mask);
}


/***********************************************************************
* Implement: R_Floor_N
*    Fast floor operation
*
* Arguments:
*    value - Left operand
*
* Return Value:
*    Floor( value / N )
*
* Description:
* - This is shared description of
*   R_Floor_2s, R_Floor_4s, R_Floor_8s, R_Floor_16s, R_Floor_32s, R_Floor_64s,
*   R_Floor_2u, R_Floor_4u, R_Floor_8u, R_Floor_16u, R_Floor_32u, R_Floor_64u.
* - Porting layer of fast floor operation.
* - Function version is confirmed with MISRA & IPA SEC.
************************************************************************/


/***********************************************************************
* Implement: R_Floor_2s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_2s
#endif
INLINE int_fast32_t  R_Floor_2s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 1;
    return  ((value)&~mask);  /* [R_Floor_2s] faster than (x)-(x%2) */
}


/***********************************************************************
* Implement: R_Floor_4s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_4s
#endif
INLINE int_fast32_t  R_Floor_4s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 3;
    return  ((value)&~mask);  /* [R_Floor_4s] faster than (x)-(x%4) */
}


/***********************************************************************
* Implement: R_Floor_8s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_8s
#endif
INLINE int_fast32_t  R_Floor_8s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 7;
    return  ((value)&~mask);  /* [R_Floor_8s] faster than (x)-(x%8) */
}


/***********************************************************************
* Implement: R_Floor_16s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_16s
#endif
INLINE int_fast32_t  R_Floor_16s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 15;
    return  ((value)&~mask);  /* [R_Floor_16s] faster than (x)-(x%16) */
}


/***********************************************************************
* Implement: R_Floor_32s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_32s
#endif
INLINE int_fast32_t  R_Floor_32s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 31;
    return  ((value)&~mask);  /* [R_Floor_32s] faster than (x)-(x%32) */
}


/***********************************************************************
* Implement: R_Floor_64s
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_64s
#endif
INLINE int_fast32_t  R_Floor_64s( int_fast32_t const  value )
{
    const int_fast32_t  mask = 63;
    return  ((value)&~mask);  /* [R_Floor_64s] faster than (x)-(x%64) */
}


/***********************************************************************
* Implement: R_Floor_2u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_2u
#endif
INLINE uint_fast32_t  R_Floor_2u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 1;
    return  ((value)&~mask);  /* [R_Floor_2u] faster than (x)-(x%2) */
}


/***********************************************************************
* Implement: R_Floor_4u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_4u
#endif
INLINE uint_fast32_t  R_Floor_4u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 3;
    return  ((value)&~mask);  /* [R_Floor_4u] faster than (x)-(x%4) */
}


/***********************************************************************
* Implement: R_Floor_8u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_8u
#endif
INLINE uint_fast32_t  R_Floor_8u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 7;
    return  ((value)&~mask);  /* [R_Floor_8u] faster than (x)-(x%8) */
}


/***********************************************************************
* Implement: R_Floor_16u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_16u
#endif
INLINE uint_fast32_t  R_Floor_16u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 15;
    return  ((value)&~mask);  /* [R_Floor_16u] faster than (x)-(x%16) */
}


/***********************************************************************
* Implement: R_Floor_32u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_32u
#endif
INLINE uint_fast32_t  R_Floor_32u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 31;
    return  ((value)&~mask);  /* [R_Floor_32u] faster than (x)-(x%32) */
}


/***********************************************************************
* Implement: R_Floor_64u
*    Fast floor operation. See <R_Floor_N>.
************************************************************************/
#ifdef _SH
#pragma inline  R_Floor_64u
#endif
INLINE uint_fast32_t  R_Floor_64u( uint_fast32_t const  value )
{
    const uint_fast32_t  mask = 63;
    return  ((value)&~mask);  /* [R_Floor_64u] faster than (x)-(x%64) */
}


/* Section: Global */
#ifdef __cplusplus
}  /* extern "C" */
#endif


/***********************************************************************
* Implement: is_float_equal
*    Judgement whether float value is equal with some error (epsilon).
************************************************************************/
INLINE bool_t  is_float_equal( float a, float b, float epsilon )
{
    float  sub = a - b;

    return  sub < epsilon  &&  sub > -epsilon;
}


/***********************************************************************
* Implement: is_double_equal
*    Judgement whether double float value is equal with some error (epsilon).
************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

INLINE bool_t  is_double_equal( double a, double b, double epsilon )
{
    double  sub = a - b;

    return  sub < epsilon  &&  sub > -epsilon;
}


/* Section: Global */
#ifdef __cplusplus
}  /* extern "C" */
#endif


/***********************************************************************
* Implement: is_float_equal (double)
*    Judgement whether double float value is equal with some error (epsilon).
************************************************************************/
#ifdef __cplusplus
inline bool_t  is_float_equal( double a, double b, double epsilon )
{
    double  sub = a - b;

    return  sub < epsilon  &&  sub > -epsilon;
}
#endif


/* Section: Global */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif  /* CLIB_DRIVERS_INLINE_H */
