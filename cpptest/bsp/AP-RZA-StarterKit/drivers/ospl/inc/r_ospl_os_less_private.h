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
* File: ospl_os_less_private.h
*    OS Porting Layer private functions for OS less
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef OSPL_OS_LESS_PRIVATE_H
#define OSPL_OS_LESS_PRIVATE_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_ospl_os_less_typedef.h"
#include "./r_ospl.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Structure: r_ospl_master_t
*    Variables for OSPL
************************************************************************/
typedef struct st_r_ospl_master_t  r_ospl_master_t;
struct st_r_ospl_master_t
{
#if ! R_OSPL_IS_PREEMPTION
    /* Variable: CurrentThread */
    r_ospl_thread_def_t*  CurrentThread;

#if ! R_OSPL_MINIMUM
    /* Variable: IdleCallback */
    r_ospl_idle_callback_t  IdleCallback;
#endif
#endif
#if ! R_OSPL_MINIMUM
    /* Variable: IsFreeRunTimerSpec */
    bool_t  IsFreeRunTimerSpec;

    /* Variable: FreeRunTimerSpec */
    r_ospl_ftimer_spec_t  FreeRunTimerSpec;

    /* Variable: MaxOneTimeoutTime */
    uint32_t  MaxOneTimeoutTime;
#endif
};


/* Section: Global */
/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* Function: R_OSPL_GetPrivateContext
*    Returns <r_ospl_master_t> type variable.
*
* Arguments:
*    None
*
* Return Value:
*    <r_ospl_master_t> type variable.
************************************************************************/
r_ospl_master_t*  R_OSPL_GetPrivateContext(void);


/***********************************************************************
* Function: R_OSPL_MEMORY_Flush_Sub
*    Sub routine of <R_OSPL_MEMORY_Flush>
*
* Arguments:
*    None
*
* Return Value:
*    None
************************************************************************/
void    R_OSPL_MEMORY_Flush_Sub(void);


/***********************************************************************
* Function: R_OSPL_MEMORY_GetCacheLineSize
*    GetCacheLineSize
*
* Arguments:
*    None
*
* Return Value:
*    CacheLineSize
************************************************************************/
size_t  R_OSPL_MEMORY_GetCacheLineSize(void);


/***********************************************************************
* Function: R_OSPL_MEMORY_RangeFlush_Sub
*    Sub routine of <R_OSPL_MEMORY_RangeFlush>
*
* Arguments:
*    Start           Start
*    Over          - Over
*    CacheLineSize - CacheLineSize
*
* Return Value:
*    None
************************************************************************/
void    R_OSPL_MEMORY_RangeFlush_Sub( uintptr_t Start,  uintptr_t Over,  size_t CacheLineSize );


#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* OSPL_OS_LESS_PRIVATE_H */

