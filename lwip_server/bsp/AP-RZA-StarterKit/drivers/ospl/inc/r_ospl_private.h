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
* File: r_ospl_private.h
*    OS Porting Layer private API for OS less
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef R_OSPL_PRIVATE_H
#define R_OSPL_PRIVATE_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "Project_Config.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Structure: r_ospl_table_searched_t
*    Result of searching
************************************************************************/
typedef struct st_r_ospl_table_searched_t  r_ospl_table_searched_t;
struct st_r_ospl_table_searched_t
{

    /* Variable: SortedKeyIndex */
    int_fast32_t  SortedKeyIndex;

    /* Variable: IsFound */
    bool_t  IsFound;
};


/* Section: Global */
/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: R_OSPL_EVENT_WATCH
*    Debug tool. 0 or 1
************************************************************************/
#define  R_OSPL_EVENT_WATCH  0


/***********************************************************************
* Constant: R_OSPL_EVENT_CHECK_BIT_MASK
*    Mask of bit number of event for <R_OSPL_DETECT_BAD_EVENT>.
************************************************************************/
#define  R_OSPL_EVENT_CHECK_BIT_MASK  0x000F0000


/***********************************************************************
* Constant: R_OSPL_EVENT_CHECK_BIT_SHIFT
*    Shift count of bit number of event for <R_OSPL_DETECT_BAD_EVENT>.
************************************************************************/
#define  R_OSPL_EVENT_CHECK_BIT_SHIFT  16


/***********************************************************************
* Constant: R_OSPL_EVENT_CHECK_ORDER_MASK
*    Mask of order number of event for <R_OSPL_DETECT_BAD_EVENT>.
************************************************************************/
#define  R_OSPL_EVENT_CHECK_ORDER_MASK  0x0FF00000


/***********************************************************************
* Constant: R_OSPL_EVENT_CHECK_ORDER_SHIFT
*    Shift count of order number of event for <R_OSPL_DETECT_BAD_EVENT>.
************************************************************************/
#define  R_OSPL_EVENT_CHECK_ORDER_SHIFT  20


/***********************************************************************
* Constant: R_OSPL_BY_ELEMENT_OBJECT
*    See <R_OSPL_ByEventObject>.
************************************************************************/
#define  R_OSPL_BY_ELEMENT_OBJECT  ( (r_ospl_thread_id_t) 0x00000001 )


/***********************************************************************
* Macro: IF_C
*     <IF> with context.
************************************************************************/
#if R_OSPL_ERROR_BREAK

/* ->MISRA 19.4 : Abnormal termination. Compliant with C language syntax. */ /* ->SEC M1.8.2 */
#define  IF_C( Condition, Context ) \
		if ( IS( R_OSPL_IF_C_Macro_Sub( \
			IS( (int_fast32_t)( Condition ) ), Context, __FILE__, __LINE__ ) ) )
/* (int_fast32_t) cast is for QAC warning of implicit cast unsigned to signed */
/* != 0 is for QAC warning of MISRA 13.2 Advice */
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */

#else  /* ! R_OSPL_ERROR_BREAK */

/* ->MISRA 19.4 : Abnormal termination. Compliant with C language syntax. */ /* ->SEC M1.8.2 */
#define  IF_C( Condition, Context )  if ( Condition )
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */
#endif


/***********************************************************************
* Macro: IF_DC
*     <IF_D> with context.
************************************************************************/
/* ->MISRA 19.4 : Compliant with C language syntax. */ /* ->SEC M1.8.2 */
/* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#ifndef R_OSPL_NDEBUG
#define  IF_DC( Condition, Context )  IF_C ( Condition, Context )
#else
#define  IF_DC( Condition, Context )  if ( false )
#endif
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */


/***********************************************************************
* Macro: IF_DQC
*     <IF_DQ> with context.
************************************************************************/
/* ->MISRA 19.4 : Compliant with C language syntax. */ /* ->SEC M1.8.2 */
#if defined(__QAC_SH_H__)
#define  IF_DQC  IF_C
#else
#define  IF_DQC  IF_DC
#endif
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */


/***********************************************************************
* Macro: ASSERT_RC
*     <ASSERT_R> with context.
************************************************************************/
#ifndef  __cplusplus
#define  ASSERT_RC( Condition, Context, goto_fin_Statement ) \
		do{ IF_C(!(Condition), Context) { goto_fin_Statement; } } while(0)
/* do-while is CERT standard PRE10-C */
#else
#define  ASSERT_RC( Condition, Context, goto_fin_Statement ) \
		{ IF_C(!(Condition), Context) { goto_fin_Statement; } }  /* no C5236(I) */
#endif


/***********************************************************************
* Macro: ASSERT_DC
*     <ASSERT_D> with context.
************************************************************************/
#ifndef R_OSPL_NDEBUG
#define  ASSERT_DC  ASSERT_RC
#else
/* ->MISRA 19.7 : Function's argument can not get "goto_fin_Statement" */ /* ->SEC M5.1.3 */
#define  ASSERT_DC( Condition, Context, goto_fin_Statement )  R_NOOP()
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */
#endif


/***********************************************************************
* Macro: R_OSPL_BAD_EVENT_ERROR
*    Whether <R_OSPL_RaiseUnrecoverable> is called, when not allocated event was detected.
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
#if  R_OSPL_EVENT_MUST_ALLOCATE
#define  R_OSPL_BAD_EVENT_ERROR()  R_OSPL_RaiseUnrecoverable( E_OTHERS )
#else
#define  R_OSPL_BAD_EVENT_ERROR()  R_NOOP()
#endif
#endif


/******************************************************************************
Variable Externs
******************************************************************************/

#if ! IS_MBED_USED
#if  defined( __CC_ARM )

/* ->QAC 0289 */ /* ->QAC 1002 */ /* ->MISRA 5.1 */
extern uint32_t  Image$$BEGIN_OF_CACHED_RAM_BARRIER$$Base;
extern uint32_t  Image$$BEGIN_OF_NOCACHE_RAM_BARRIER$$Base;
extern uint32_t  Image$$BEGIN_OF_NOCACHE_RAM_BARRIER$$ZI$$Limit;
extern uint32_t  Image$$END_OF_INTERNAL_RAM_BARRIER$$ZI$$Limit;
/* <-QAC 0289 */ /* <-QAC 1002 */ /* <-MISRA 5.1 */

#endif
#else  /* IS_MBED_USED */
extern uint32_t  Image$$RW_DATA_NC$$Base;
#endif


/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* Function: R_OSPL_InitializeIfNot
*    Initializes the internal of OSPL, if OSPL was not initialized.
*
* Arguments:
*    None
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_InitializeIfNot(void);


/***********************************************************************
* Function: R_OSPL_Initialize
*    Initializes the internal of OSPL
*
* Arguments:
*    NullConfig - Specify NULL
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    Initializes internal mutual exclusion objects.
*    However, "R_OSPL_Initialize" function does not have to be called for
*    OSPL of "R_OSPL_IS_PREEMPTION = 0".
*    "E_ACCESS_DENIED" error is raised, when the OSPL API that it is
*    necessary to call "R_OSPL_Initialize" before calling the API was called.
************************************************************************/
errnum_t  R_OSPL_Initialize( const void* const  NullConfig );


/***********************************************************************
* Function: R_OSPL_LockUnlockedChannel
*    LockUnlockedChannel
*
* Arguments:
*    out_ChannelNum   - ChannelNum
*    HardwareIndexMin - HardwareIndexMin
*    HardwareIndexMax - HardwareIndexMax
*
* Return Value:
*    Error Code. 0=No Error.
************************************************************************/
#if ! BSP_CFG_USER_LOCKING_ENABLED
errnum_t  R_OSPL_LockUnlockedChannel( int_fast32_t* out_ChannelNum,
                                      mcu_lock_t  HardwareIndexMin,  mcu_lock_t  HardwareIndexMax );
#endif


/***********************************************************************
* Function: R_OSPL_LockCurrentThreadError_Sub
************************************************************************/
r_ospl_error_t*  R_OSPL_LockCurrentThreadError_Sub( r_ospl_thread_id_t  in_Thread,
        r_ospl_if_not_t const  in_TypeOfIfNot );


/***********************************************************************
* Function: R_OSPL_GetCurrentThreadError_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
r_ospl_error_t*  R_OSPL_GetCurrentThreadError_Sub( r_ospl_thread_id_t  in_Thread,
        r_ospl_if_not_t const  in_TypeOfIfNot );
#endif


/***********************************************************************
* Function: R_OSPL_GetCurrentThreadError_Sub
************************************************************************/
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
r_ospl_event_object_id_t  R_OSPL_EVENT_GetEventObject_Sub( r_ospl_thread_id_t const  in_ThreadId,
        r_ospl_event_flags_t const  in_SetFlags );
#endif


/***********************************************************************
* Function: R_OSPL_C_LOCK_Lock_Sub
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Lock_Sub( r_ospl_c_lock_t* const  self,  r_ospl_error_t* const  err );


/***********************************************************************
* Function: R_OSPL_C_LOCK_Unlock_Sub
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Unlock_Sub( r_ospl_c_lock_t* const  self,  r_ospl_error_t* const  err );


/***********************************************************************
* Function: R_OSPL_CLEAR_ERROR_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
void  R_OSPL_CLEAR_ERROR_Sub( r_ospl_error_t*  err );
#else
#ifdef _SH
#pragma inline R_OSPL_CLEAR_ERROR_Sub
#endif
INLINE void  R_OSPL_CLEAR_ERROR_Sub( r_ospl_error_t*  err )
{
    R_UNREFERENCED_VARIABLE( err );    /* QAC 3138 */
}
#endif


/***********************************************************************
* Function: R_OSPL_EVENT_SetForDebug
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
void  R_OSPL_EVENT_SetForDebug( r_ospl_thread_id_t const  in_ThreadId,  bit_flags32_t const  in_SetFlags );
#endif


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* R_OSPL_PRIVATE_H */
