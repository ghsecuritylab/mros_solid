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
* File: ospl_os_less.h
*    OS Porting Layer API for OS less Compatibility
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef OSPL_OS_LESS_H
#define OSPL_OS_LESS_H

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
/* In "ospl_os_less_typedef.h" */

/******************************************************************************
Macro definitions
******************************************************************************/
/* In "ospl_os_less_typedef.h" */

/******************************************************************************
Variable Externs
******************************************************************************/
/* In "ospl_os_less_typedef.h" */

/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* Function: R_OSPL_THREAD_SetOnWait
*    Set waiting behavior of current thread.
*
* Arguments:
*    OnWait - Behavior on waiting
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    - Case of <R_OSPL_IS_PREEMPTION> = 0:
*
*    Initial value is <R_OSPL_WAIT_POLLING>.
*    If this function was called from the interrupt context, E_STATE error is raised.
*
*    - Case of <R_OSPL_IS_PREEMPTION> = 1:
*
*    This function is for compatibility only.
*    Arguments are ignored.
*    This function returns 0.
*
*    Refer to: <R_OSPL_THREAD_GetIsWaiting>
************************************************************************/
#ifdef _SH
#pragma inline  R_OSPL_THREAD_SetOnWait
#endif
INLINE errnum_t  R_OSPL_THREAD_SetOnWait( r_ospl_wait_t const  OnWait )
{
    return  0;
}


/***********************************************************************
* Function: R_OSPL_THREAD_GetOnWait
*    Get waiting behavior of current thread.
*
* Arguments:
*    None
*
* Return Value:
*    Behavior on waiting
*
* Description:
*    - Case of <R_OSPL_IS_PREEMPTION> = 0:
*
*    Initial value is <R_OSPL_WAIT_POLLING>.
*    If this function was called from the interrupt context,
*    this function returns <R_OSPL_WAIT_POLLING>.
*
*    - Case of <R_OSPL_IS_PREEMPTION> = 1:
*
*    This function is for compatibility only.
*    This function returns R_OSPL_WAIT_POLLING.
*    But it does not polling on waiting
*
*    Refer to: <R_OSPL_THREAD_SetOnWait>
************************************************************************/
#ifdef _SH
#pragma inline  R_OSPL_THREAD_GetOnWait
#endif
INLINE r_ospl_wait_t  R_OSPL_THREAD_GetOnWait(void)
{
    return  R_OSPL_WAIT_POLLING;
}


/***********************************************************************
* Function: R_OSPL_THREAD_GetIsWaiting
*    Get whether the current thread is waiting or not.
*
* Arguments:
*    None
*
* Return Value:
*    Whether the current thread is waiting or not
*
* Description:
*    - Case of <R_OSPL_IS_PREEMPTION> = 0:
*
*    If <R_OSPL_WAIT_PM_THREAD> was set by <R_OSPL_THREAD_SetOnWait> function,
*    some waiting functions return soon even if the state is waiting and
*    <R_OSPL_THREAD_GetIsWaiting> function returns true.
*
*    If time out was set to 0, <R_OSPL_THREAD_GetIsWaiting> function returns
*    true at time out, even if any value was passed to <R_OSPL_THREAD_SetOnWait>
*    function,
*
*    If this function was called from the interrupt context, this function
*    returns false and <ASSERT_D> in this function notifies in debug configuration.
*
*    - Case of <R_OSPL_IS_PREEMPTION> = 1:
*
*    This function is for compatibility only.
*    This function returns false.
*
* Example:
*    > e= R_OSPL_Delay( 100 ); IF(e){goto fin;}
*    > if ( R_OSPL_THREAD_GetIsWaiting() ) { e=0; goto fin; }
************************************************************************/
#ifdef _SH
#pragma inline  R_OSPL_THREAD_GetIsWaiting
#endif
INLINE bool_t  R_OSPL_THREAD_GetIsWaiting(void)
{
    return  false;
}


/***********************************************************************
* Function: R_OSPL_THREAD_ExitWaiting
*    Exit waiting state, if current thread was waiting state.
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    - Case of <R_OSPL_IS_PREEMPTION> = 0:
*
*    The thread returned true from <R_OSPL_THREAD_GetIsWaiting> function
*    after a waiting function must call the waiting function again.
*    After exiting waiting state, other operation and other waiting can
*    be done. If time out was set to 0, exiting does not have to do.
*
*    If it was detected in OSPL API that necessary exiting was not done,
*    E_STATE error is raised. However sometimes the state can not be detected.
*    In this case, time out will be not correct.
*
*    This function does not do anything called from the interrupt context.
*    "ASSERT_D" in this function notifies in debug configuration.
*
*    - Case of <R_OSPL_IS_PREEMPTION> = 1:
*
*    This function is for compatibility only.
*    This function does not do anything.
************************************************************************/
#ifdef _SH
#pragma inline  R_OSPL_THREAD_ExitWaiting
#endif
INLINE void  R_OSPL_THREAD_ExitWaiting(void)
{
}


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* OSPL_OS_LESS_H */

