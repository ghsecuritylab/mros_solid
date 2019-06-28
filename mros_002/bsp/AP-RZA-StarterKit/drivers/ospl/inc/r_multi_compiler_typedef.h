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
* File: r_multi_compiler_typedef.h
*    Compiler Porting Layer. Data types.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 42 $
* - $Date:: 2014-06-03 16:54:02 +0900#$
************************************************************************/

#ifndef R_MULTI_COMPILER_TYPEDEF_H
#define R_MULTI_COMPILER_TYPEDEF_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Macro: R_OSPL_SECTION_INLINE_VERSION
************************************************************************/
#define R_OSPL_SECTION_INLINE_VERSION  7


/***********************************************************************
* Macro: R_OSPL_LIST_UP_INLINE_BODY
*    Define or not define
************************************************************************/
/* #define  R_OSPL_LIST_UP_INLINE_BODY */


/***********************************************************************
* Macro: R_OSPL_MAKE_INLINE_BODY
*    Define or not define
************************************************************************/
/* #define  R_OSPL_MAKE_INLINE_BODY */


/***********************************************************************
* Macro: INLINE
*    Inline function
*
* Description:
*    "__INLINE" is reserved by compiler.
************************************************************************/
/* ->QAC 1252 : QAC considers that && is not short-circuit evaluation */
#if defined( __STDC_VERSION__ ) && __STDC_VERSION__ >= 199901L  &&  ! defined( __ICCARM__ ) /* For C99 */ \
&&  ! IS_MBED_USED  ||  defined( __cplusplus )
/* <-QAC 1252 */
#if defined( R_OSPL_MAKE_INLINE_BODY )  &&  ! defined( R_OSPL_LIST_UP_INLINE_BODY )
#define  INLINE  extern inline
#else
#define  INLINE  inline
#endif

/* ->QAC 1252 : QAC considers that && is not short-circuit evaluation */
#elif defined( __STDC_VERSION__ ) && __STDC_VERSION__ >= 199901L  &&  defined( __ICCARM__ ) /* For IAR C99 */
/* <-QAC 1252 */
#if defined( R_OSPL_MAKE_INLINE_BODY )
#define  INLINE
/* "extern inline" raises a error, if with "@" operator */
#else
#define  INLINE  inline  /* Special inline */
#endif

#elif defined( __CC_ARM )  &&  ! defined( __GNUC__ )  /* For ARMCC not C99, not -gnu */
#if defined( R_OSPL_MAKE_INLINE_BODY )  &&  ! defined( R_OSPL_LIST_UP_INLINE_BODY )
#if R_OSPL_LIBRARY_MAKING
#define  INLINE  static __inline
#else
#define  INLINE  extern __inline  /* Special inline */
#endif
#else
#if R_OSPL_LIBRARY_MAKING
#define  INLINE  static __inline
#else
#define  INLINE  extern __inline  /* Special inline */
#endif
#endif
/* Function bodys are in shared "i.<FunctionName>" section */

#elif defined( __GNUC__ )  /* For gcc */
#if  IS_MBED_USED
#define  INLINE  static __inline
#else
#if defined( R_OSPL_MAKE_INLINE_BODY )  &&  ! defined( R_OSPL_LIST_UP_INLINE_BODY )
#define  INLINE  __inline__  /* extern inline of C99 */
#else
#define  INLINE  extern __inline__  /* inline of C99 */
#endif
#endif

#elif defined( _SH )  /* For SH compiler */
#if defined( R_OSPL_MAKE_INLINE_BODY )  &&  ! defined( R_OSPL_LIST_UP_INLINE_BODY )
#define  INLINE
#else
#ifdef  __cplusplus
#define  INLINE  inline
#else
#define  INLINE  static
/* "extern" is ignored, if 'C1400 (W) Function "..." in #pragma inline is not expanded' */
#endif
#endif
/* No inline qualifier */
/* #ifdef _SH */
/* #pragma inline <function_name> */
/* #endif */

#else
#error
#endif


/***********************************************************************
* Macro: STATIC_INLINE
*    Static inline in C source file
*
* Description:
*    "__STATIC_INLINE" is reserved by compiler.
************************************************************************/

#ifndef  __cplusplus

#ifdef __CC_ARM
#define  STATIC_INLINE  static __inline
#endif

#ifdef __ICCARM__
#define  STATIC_INLINE  static inline
#endif

#if  defined( __GNUC__ )  &&  ! defined( __CC_ARM )
#define  STATIC_INLINE  static inline
#endif

#ifdef _SH
#define  STATIC_INLINE  static
#endif

#else

#define  STATIC_INLINE  static inline

#endif  /* __cplusplus */


/***********************************************************************
* Macro: R_OSPL_SECTION
*    Names section name to function or varaible
************************************************************************/
/***********************************************************************
* Macro: R_OSPL_SECTION_FOR_ZERO_INIT
*    Names section name to zero initialized varaible
************************************************************************/
/* ->MISRA 19.10 : Cannot ( ) */ /* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#if defined( __CC_ARM )
#define  R_OSPL_SECTION( SectionName, Declaration ) \
		__attribute__ ((section (SectionName)))  Declaration

#define  R_OSPL_SECTION_FOR_ZERO_INIT( SectionName, Declaration ) \
		__attribute__ ((section (SectionName), zero_init))  Declaration

#elif defined( __GNUC__ )  &&  ! defined( __CC_ARM )
#define  R_OSPL_SECTION( SectionName, Declaration ) \
		__attribute__ ((section (SectionName)))  Declaration

#define  R_OSPL_SECTION_FOR_ZERO_INIT( SectionName, Declaration ) \
		__attribute__ ((section (SectionName)))  Declaration

#elif defined( __ICCARM__ )
#define  R_OSPL_SECTION( SectionName, Declaration ) \
		Declaration @ SectionName

#define  R_OSPL_SECTION_FOR_ZERO_INIT( SectionName, Declaration ) \
		Declaration @ SectionName

#elif defined( _SH )  /* For SH compiler */
#define  R_OSPL_SECTION( SectionName, Declaration ) \
		Declaration
/* No section qualifier */
/* #ifdef _SH */
/* #pragma section <section_name> */
/* #endif */
/*   :   */
/* <Not extern code> */
/*   :   */
/* #ifdef _SH */
/* #pragma section */
/* #endif */

#define  R_OSPL_SECTION_FOR_ZERO_INIT( SectionName, Declaration ) \
		Declaration

#else
#error
#endif
/* <-MISRA 19.10 */ /* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Macro: R_OSPL_SECTION_FOR_INLINE
*    Names section name to inline function
************************************************************************/
/* ->MISRA 19.10 : Cannot ( ) */ /* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#if defined( __CC_ARM )  ||  defined( __GNUC__ )
#if defined( R_OSPL_LIST_UP_INLINE_BODY )
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
			__attribute__ ((section ("INLINE_BODY")))  Declaration
#else
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
			__attribute__ ((section (SectionName)))  Declaration
#endif

#elif defined( __ICCARM__ )
#if defined( R_OSPL_MAKE_INLINE_BODY )
#if defined( R_OSPL_LIST_UP_INLINE_BODY )
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
				Declaration @ "INLINE_BODY"
#else
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
				Declaration @ SectionName
#endif
#else
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
			Declaration
#endif

#elif defined( _SH )  /* For SH compiler */
#define  R_OSPL_SECTION_FOR_INLINE( SectionName, Declaration ) \
		Declaration
/* No section qualifier */
/* #ifdef _SH */
/* #pragma section <section_name> */
/* #endif */
/*   :   */
/* <Not extern code> */
/*   :   */
/* #ifdef _SH */
/* #pragma section */
/* #endif */

#else
#error
#endif
/* <-MISRA 19.10 */ /* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Macro: R_OSPL_ALIGNMENT
*    Alignments first addres of global variable
*
* Arguments:
*    ByteCount                     - Value of alignment
*    Declaration_without_Semicolon - Declaration of the variable
*
* Example:
*    > R_OSPL_ALIGNMENT( 0x100,
*    > int  g_Buffer[0x1000] );
************************************************************************/
/* ->MISRA 19.10 : Cannot ( ) */ /* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#if defined( __CC_ARM )
#define  R_OSPL_ALIGNMENT( ByteCount, Declaration_without_Semicolon ) \
		__align( ByteCount )  Declaration_without_Semicolon

#elif defined( __GNUC__ )  &&  ! defined( __CC_ARM )
#define  R_OSPL_ALIGNMENT( ByteCount, Declaration_without_Semicolon ) \
		__attribute__((aligned( ByteCount )))  Declaration_without_Semicolon

#elif defined( __ICCARM__ )
#define  R_OSPL_ALIGNMENT( ByteCount, Declaration_without_Semicolon ) \
		R_OSPL_ALIGNMENT_SUB( ByteCount, Declaration_without_Semicolon )
#define  R_OSPL_ALIGNMENT_SUB( ByteCount, Declaration_without_Semicolon ) \
		_Pragma( "diag_suppress=Pe606" ) \
		_Pragma( "diag_suppress=Pa060" ) \
		_Pragma( "diag_suppress=Pe609" ) \
		R_OSPL_ALIGNMENT_##ByteCount() \
		Declaration_without_Semicolon \
		R_OSPL_ALIGNMENT_##0x4()

#define  R_OSPL_ALIGNMENT_0x4()      _Pragma( "data_alignment=0x4" )
#define  R_OSPL_ALIGNMENT_0x8()      _Pragma( "data_alignment=0x8" )
#define  R_OSPL_ALIGNMENT_0x10()     _Pragma( "data_alignment=0x10" )
#define  R_OSPL_ALIGNMENT_0x20()     _Pragma( "data_alignment=0x20" )
#define  R_OSPL_ALIGNMENT_0x40()     _Pragma( "data_alignment=0x40" )
#define  R_OSPL_ALIGNMENT_0x80()     _Pragma( "data_alignment=0x80" )
#define  R_OSPL_ALIGNMENT_0x100()    _Pragma( "data_alignment=0x100" )
#define  R_OSPL_ALIGNMENT_0x200()    _Pragma( "data_alignment=0x200" )
#define  R_OSPL_ALIGNMENT_0x400()    _Pragma( "data_alignment=0x400" )
#define  R_OSPL_ALIGNMENT_0x800()    _Pragma( "data_alignment=0x800" )
#define  R_OSPL_ALIGNMENT_0x1000()   _Pragma( "data_alignment=0x1000" )
#define  R_OSPL_ALIGNMENT_0x2000()   _Pragma( "data_alignment=0x2000" )
#define  R_OSPL_ALIGNMENT_0x4000()   _Pragma( "data_alignment=0x4000" )
#define  R_OSPL_ALIGNMENT_0x8000()   _Pragma( "data_alignment=0x8000" )
#define  R_OSPL_ALIGNMENT_0x10000()  _Pragma( "data_alignment=0x10000" )
#define  R_OSPL_ALIGNMENT_0x20000()  _Pragma( "data_alignment=0x20000" )
#define  R_OSPL_ALIGNMENT_0x40000()  _Pragma( "data_alignment=0x40000" )
#define  R_OSPL_ALIGNMENT_0x80000()  _Pragma( "data_alignment=0x80000" )
#define  R_OSPL_ALIGNMENT_0x100000() _Pragma( "data_alignment=0x100000" )

#define  R_OSPL_ALIGNMENT_4()        _Pragma( "data_alignment=4" )
#define  R_OSPL_ALIGNMENT_8()        _Pragma( "data_alignment=8" )
#define  R_OSPL_ALIGNMENT_16()       _Pragma( "data_alignment=16" )
#define  R_OSPL_ALIGNMENT_32()       _Pragma( "data_alignment=32" )
#define  R_OSPL_ALIGNMENT_64()       _Pragma( "data_alignment=64" )
#define  R_OSPL_ALIGNMENT_128()      _Pragma( "data_alignment=128" )
#define  R_OSPL_ALIGNMENT_256()      _Pragma( "data_alignment=256" )
#define  R_OSPL_ALIGNMENT_512()      _Pragma( "data_alignment=512" )
#define  R_OSPL_ALIGNMENT_1024()     _Pragma( "data_alignment=1024" )
#define  R_OSPL_ALIGNMENT_2048()     _Pragma( "data_alignment=2048" )
#define  R_OSPL_ALIGNMENT_4096()     _Pragma( "data_alignment=4096" )
#define  R_OSPL_ALIGNMENT_8192()     _Pragma( "data_alignment=8192" )
#define  R_OSPL_ALIGNMENT_16384()    _Pragma( "data_alignment=16384" )
#define  R_OSPL_ALIGNMENT_32768()    _Pragma( "data_alignment=32768" )
#define  R_OSPL_ALIGNMENT_65536()    _Pragma( "data_alignment=65536" )

#elif defined( _SH )
#define  R_OSPL_ALIGNMENT( ByteCount, Declaration_without_Semicolon ) \
		Declaration_without_Semicolon
/* No alignment qualifier */
/* Set aligned address by "Map section information" */

#else
#error
#endif
/* <-MISRA 19.10 */ /* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/* Form: C Language Header */
/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
/* In "r_multi_compiler.h" */

/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* R_MULTI_COMPILER_TYPEDEF_H */
