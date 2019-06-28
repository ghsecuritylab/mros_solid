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
* File: r_ospl_typedef_1.h
*    Part of "r_ospl_typedef.h". For mutual reference.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef R_OSPL_TYPEDEF_1_H
#define R_OSPL_TYPEDEF_1_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "Project_Config.h"
#include  <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Type: errnum_t
*    Error number
************************************************************************/
typedef int_fast32_t   errnum_t;


/***********************************************************************
* Type: bool8_t
*    Boolean type in 8 bit variable. This is not C99.
************************************************************************/
typedef uint8_t  bool8_t;


/***********************************************************************
* Type: bool16_t
*    Boolean type in 16 bit variable. This is not C99.
************************************************************************/
typedef uint16_t  bool16_t;


/***********************************************************************
* Type: bool32_t
*    Boolean type in 32 bit variable. This is not C99.
************************************************************************/
typedef uint32_t  bool32_t;


/***********************************************************************
* Type: bit_flags_fast32_t
*    Bit flags as "uint_fast32_t"
************************************************************************/
typedef uint_fast32_t  bit_flags_fast32_t;


/***********************************************************************
* Type: bit_flags32_t
*    Bit flags as "uint32_t"
************************************************************************/
typedef uint32_t       bit_flags32_t;


/***********************************************************************
* Type: bit_flags16_t
*    Bit flags as "uint16_t"
************************************************************************/
typedef uint16_t       bit_flags16_t;


/***********************************************************************
* Type: bit_flags8_t
*    Bit flags as "uint8_t"
************************************************************************/
typedef uint8_t       bit_flags8_t;


/***********************************************************************
* Type: byte_t
*    Byte type
************************************************************************/
typedef uint8_t        byte_t;


/***********************************************************************
* Type: ssize_t
*    Signed size type. This is a POSIX specification.
************************************************************************/
#if  ! ( defined( IS_MBED_USED )  &&  defined( __GNUC__ )  &&  ! defined( __CC_ARM ) )
typedef int  ssize_t;
#endif


/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
/* In "r_ospl.h" */


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* R_OSPL_TYPEDEF_1_H */

