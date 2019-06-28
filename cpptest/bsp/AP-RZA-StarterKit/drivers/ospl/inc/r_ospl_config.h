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
* File: ospl_config.h
*    Configuration of OSPL. For RZ/A1 RTX BSP.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/



/* This file is included from "Project_Config.h" */


#ifndef R_OSPL_CONFIG_H
#define R_OSPL_CONFIG_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "platform_config.h"

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
* Constant: R_OSPL_VERSION
*    Version number of OSPL
*
* Description:
*    The value can not be changed.
*    101 = version 1.01.
*    Hundreds place is version number of OSPL specification.
*    Tens place and one's place are minor version number in specified OS and board.
************************************************************************/
#define  R_OSPL_VERSION  96


/***********************************************************************
* Constant: R_OSPL_VERSION_STRING
*    String of version number of OSPL.
*
* Description:
*    The value can not be changed.
************************************************************************/
#define  R_OSPL_VERSION_STRING  "0.96"


/***********************************************************************
* Constant: R_OSPL_IS_PREEMPTION
*    Whether preemptive RTOS or not.
*
* Description:
*    The value can not be changed.
*    The value is 1 or 0.
*    This value is 0, if the environment was OS less.
*    It is necessary to configure to pseudo multithreading, if this value was 0.
************************************************************************/
#define  R_OSPL_IS_PREEMPTION   BSP_CFG_RTOS_USED


/***********************************************************************
* Define: R_OSPL_NDEBUG
*    Debug configuration or Release configuration.
*
* Description:
*    The value can be changed.
*    Define (=Release) or not define (=Debug).
*    This is same as "NDEBUG" of standard library.
*    The system can run with the debug configuration OSPL and the release
*    configuration application.
*    If the library (compiled binary) called the debug configuration OSPL,
*    compile the OSPL source with debug configuration.
************************************************************************/
#ifndef  R_OSPL_NDEBUG_DEFINED

#define  R_OSPL_NDEBUG

#define  R_OSPL_NDEBUG_DEFINED
#endif


/***********************************************************************
* Define: R_OSPL_MINIMUM
*    Minimum OS porting layer.
*
* Description:
*    This is under development. Set 0.
*
*    The value can be changed.
*    The value is 0 or 1.
************************************************************************/
#ifndef  R_OSPL_MINIMUM
#define  R_OSPL_MINIMUM  0
#endif


/***********************************************************************
* Define: R_OSPL_ERROR_BREAK
*    Whether it is supported to break, when error was raised.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
************************************************************************/
#ifndef  R_OSPL_ERROR_BREAK
#if  defined( R_OSPL_NDEBUG )  ||  R_OSPL_MINIMUM
#define  R_OSPL_ERROR_BREAK  0
#else
#define  R_OSPL_ERROR_BREAK  1
#endif
#endif


/***********************************************************************
* Define: R_OSPL_PRINTF
*    Printf output device from OSPL.
*
* Description:
*    The value of "R_OSPL_PRINTF" can be changed to the following values.
*    : R_OSPL_PRINTF_ENABLED    - 1
*    : R_OSPL_PRINTF_DISABLED   - 0
*    : R_OSPL_PRINTF_TO_INT_LOG - 2
************************************************************************/
#ifndef  R_OSPL_PRINTF
#ifdef  R_OSPL_NDEBUG
#define  R_OSPL_PRINTF  R_OSPL_PRINTF_DISABLED
#else
#define  R_OSPL_PRINTF  R_OSPL_PRINTF_ENABLED
#endif
#endif
#define  R_OSPL_PRINTF_ENABLED     1
#define  R_OSPL_PRINTF_DISABLED    0
#define  R_OSPL_PRINTF_TO_INT_LOG  2


/***********************************************************************
* Define: R_OSPL_TLS_ERROR_CODE
*    Whether it is supported that error code is stored in thread local storage
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
*    This value must be 1 by some application or some library with OSPL.
************************************************************************/
#ifndef  R_OSPL_TLS_ERROR_CODE
#define  R_OSPL_TLS_ERROR_CODE  0
#endif


/***********************************************************************
* Define: R_OSPL_STACK_CHECK_CODE
*    Whether it is supported stack check
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
*    This value must be 1 by some application or some library with OSPL.
************************************************************************/
#ifndef  R_OSPL_STACK_CHECK_CODE
#ifndef R_OSPL_NDEBUG
#define  R_OSPL_STACK_CHECK_CODE  1
#else
#define  R_OSPL_STACK_CHECK_CODE  0
#endif
#endif


/***********************************************************************
* Define: R_OSPL_TLS_EVENT_CODE
*    Whether it is enabled to manage event using.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
*    This value must be 1 by some application or some library with OSPL.
************************************************************************/
#ifndef  R_OSPL_TLS_EVENT_CODE
#if  R_OSPL_TLS_ERROR_CODE
#define  R_OSPL_TLS_EVENT_CODE  1
#else
#define  R_OSPL_TLS_EVENT_CODE  0
#endif
#endif


/***********************************************************************
* Define: R_OSPL_DETECT_BAD_EVENT
*    Whether it is enabled detecting bad event.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
*    This value must be 1 by some application or some library with OSPL.
************************************************************************/
#ifndef  R_OSPL_DETECT_BAD_EVENT
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_TLS_EVENT_CODE
#define  R_OSPL_DETECT_BAD_EVENT  1
#else
#define  R_OSPL_DETECT_BAD_EVENT  0
#endif
#endif


/***********************************************************************
* Define: R_OSPL_EVENT_MUST_ALLOCATE
*    Whether event must be allocated.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
#ifndef  R_OSPL_EVENT_MUST_ALLOCATE
#define  R_OSPL_EVENT_MUST_ALLOCATE  1
#endif
#endif


/***********************************************************************
* Define: R_OSPL_EVENT_OBJECT_CODE
*    Whether it is enabled event object.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
*    This value must be 1 by some application or some library with OSPL.
************************************************************************/
#define  R_OSPL_EVENT_OBJECT_CODE  0


#if  R_OSPL_EVENT_OBJECT_CODE
#if  ! R_OSPL_TLS_ERROR_CODE
#error
#endif
#endif


/***********************************************************************
* Define: R_OSPL_CPU_LOAD
*    Whether it is supported to measure CPU load (for OS less)
*
* Description:
*    The value can be changed, if <R_OSPL_IS_PREEMPTION> was 0.
*    The value is 1 or 0.
************************************************************************/
#ifndef  R_OSPL_CPU_LOAD
#if  ! R_OSPL_IS_PREEMPTION
#if  ! R_OSPL_MINIMUM
#define  R_OSPL_CPU_LOAD  1
#else
#define  R_OSPL_CPU_LOAD  0
#endif
#else
#define  R_OSPL_CPU_LOAD  0
#endif
#endif


/***********************************************************************
* Define: R_OSPL_LIBRARY_MAKING
*    Whether current project makes library.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
************************************************************************/
#ifndef  R_OSPL_LIBRARY_MAKING
#define  R_OSPL_LIBRARY_MAKING  0
#endif


/***********************************************************************
* Constant: R_BOOL_IS_SIGNED
*    Whether compiler defines that bool_t type is signed.
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
************************************************************************/
#define  R_BOOL_IS_SIGNED  1


/***********************************************************************
* Define: R_OSPL_SUPPORT_EVENT_GET
*    Whether event flag function supports getting event.
*
* Description:
*    It is recommended to not support this for portability.
*    The value is 1 or 0.
************************************************************************/
#define  R_OSPL_SUPPORT_EVENT_GET  0


/***********************************************************************
* Define: R_OSPL_BIT_FIELD_ACCESS_MACRO
*    Whether bit field access function is provided as macro or function.
*
* Description:
*    The value can be changed.
*    The value is 1 (=macro) or 0 (=function).
************************************************************************/
#define  R_OSPL_BIT_FIELD_ACCESS_MACRO  1


/***********************************************************************
* Define: R_OSPL_FTIMER_IS
*    Which channel of timer to use.
*
*    : R_OSPL_FTIMER_IS_OSTM0    - OSTM0
*    : R_OSPL_FTIMER_IS_OSTM1    - OSTM1
*    : R_OSPL_FTIMER_IS_MTU2_1_2 - MTU2 ch1 and ch2
*
* Description:
*    The value can be changed.
*    The symbol of value is depend on target timer.
************************************************************************/
#define  R_OSPL_FTIMER_IS        R_OSPL_FTIMER_IS_MTU2_1_2
#define  R_OSPL_FTIMER_IS_OSTM0     0
#define  R_OSPL_FTIMER_IS_OSTM1     1
#define  R_OSPL_FTIMER_IS_MTU2_1_2  2

#if R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM0
#error  OSTM0 is used by RTX
#endif


/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* R_OSPL_H */

