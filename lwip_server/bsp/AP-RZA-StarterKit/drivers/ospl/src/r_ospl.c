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
* File: ospl.c
*    OS Porting Layer. Functions not depended on any peripherals and RTOS for RZ/A1.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "iodefine.h"
#include  "iobitmasks/cpg_iobitmask.h"
#include  "r_ospl_private.h"
#include  "r_ospl_os_less_private.h"
#if R_OSPL_IS_PREEMPTION
#include  "r_ospl_RTOS_private.h"
#endif
#include  "locking.h"
#ifdef __ICCARM__
#include <intrinsics.h>
#endif
#ifdef  __GNUC__
#if ! IS_MBED_USED
//#include  "irq.h" //tokuyama@kmg 2016.09.13
#endif
#endif
#if R_OSPL_DEBUG_TOOL
#include  <stdio.h>
#endif
#ifndef R_OSPL_NDEBUG
#include  <string.h>
#endif

#include "kernel.h" //tokuyama@kmg 2016.09.14
//#include "itron.h" //tokuyama@kmg 2016.09.14
#include <syslog.h>

#ifdef DEBUG_OSPL
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

#define	UNUSED_PARAM(param)             (void)(param)


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: R_OSPL_IS_ASYNC_LIST
*    Whether <r_ospl_async_t> type variables is listed up for debug.
*
* Description:
*    0 or 1. The value can be changed.
************************************************************************/
#define  R_OSPL_IS_ASYNC_LIST  0


/***********************************************************************
* Define: R_OSPL_DEBUG_THREAD_COUNT
*    Maximum thread count for thread local storage of OSPL.
*
* Description:
*    The value can be changed.
*    It is not only debuging but also TLS (thread local storage).
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
#ifndef  R_OSPL_DEBUG_THREAD_COUNT
#define  R_OSPL_DEBUG_THREAD_COUNT  8
#endif
#endif


/***********************************************************************
* Define: R_OSPL_DEBUG_EVENT_OBJECT_COUNT
*    Maximum event object count for detecting not allocated event.
*
* Description:
*    The value can be changed.
************************************************************************/
#if  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_DETECT_BAD_EVENT  ||  R_OSPL_EVENT_OBJECT_CODE
#ifndef  R_OSPL_DEBUG_EVENT_OBJECT_COUNT
#define  R_OSPL_DEBUG_EVENT_OBJECT_COUNT  8
#endif
#endif


/***********************************************************************
* Define: GS_INTERRUPT_STACK_SIZE
*    For stack check only. It is not real size.
*
* Description:
*    Case of OS less, interrupt context shares main stack.
*
*    Case of RTX, this should set less than "SVC_Stack_Size"
*    (for "IRQ_Handler") in "(project)\ARM\startup_(chip_name).s file.
************************************************************************/
#if  R_OSPL_IS_PREEMPTION
#define  GS_INTERRUPT_STACK_SIZE  0x800
#endif


/***********************************************************************
* Define: GS_INTERRUPT_STACK_BUFFER_SIZE
*
* Description:
*    Size is determined by argument of "R_OSPL_SET_END_OF_STACK()" in
*    interrupt context.
************************************************************************/
#if  R_OSPL_IS_PREEMPTION
#define  GS_INTERRUPT_STACK_BUFFER_SIZE  0x100

R_STATIC_ASSERT_GLOBAL( GS_INTERRUPT_STACK_SIZE > GS_INTERRUPT_STACK_BUFFER_SIZE, "" );
#endif


/***********************************************************************
* Define: GS_LIST_UP_TLS_ATTACHED_THREAD
*     0 or 1. For Debug.
************************************************************************/
#define  GS_LIST_UP_TLS_ATTACHED_THREAD  0


/***********************************************************************
* Define: GS_DEBUG_T_LOCK
*     0 or 1. For Debug. T-Lock is used in RTOS environment only.
************************************************************************/
#define  GS_DEBUG_T_LOCK  0


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/

static void  R_OSPL_TABLE_Search_Sub( const r_ospl_table_t* const  self,
                                      const void* const  Key,  r_ospl_table_searched_t* const  out_Searched );
static void  R_OSPL_TABLE_Free_Sub( r_ospl_table_t* const  self,  const void* const  Key,  int_fast32_t  Index );

#if R_OSPL_ERROR_BREAK
#if  ! R_OSPL_IS_PREEMPTION
void  R_OSPL_CHECK_STACK_OVERFLOW_SUB(void);
#else
void  R_OSPL_CHECK_STACK_OVERFLOW_SUB( r_ospl_error_t*  Context );
#endif
#endif


/***********************************************************************
* Variable: gs_allocated_event_objects
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
static R_OSPL_TABLE_DEF( gs_allocated_event_objects,  R_OSPL_DEBUG_EVENT_OBJECT_COUNT,  R_OSPL_TABLE_T_LOCK );
#endif


/***********************************************************************
* Variable: gs_error
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
static r_ospl_error_t  gs_error[ R_OSPL_DEBUG_THREAD_COUNT ];
static r_ospl_error_t  gs_error_for_interrupt;
#else
static r_ospl_error_t  gs_dummy_error;
#endif


/***********************************************************************
* Variable: gs_global_error_table
*    as <r_ospl_table_t>
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
static R_OSPL_TABLE_DEF( gs_global_error_table,  R_OSPL_DEBUG_THREAD_COUNT,  R_OSPL_TABLE_I_LOCK );
#endif


/***********************************************************************
* Variable: gs_global_error
*    as <r_ospl_global_error_t>
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE

/* ->MISRA 8.7 : If not defined R_OSPL_ERROR_BREAK, this is referenced by only 1 function. */ /* ->SEC M2.2.1 */
static r_ospl_global_error_t  gs_global_error =
    /* <-MISRA 8.7 */ /* <-SEC M2.2.1 */
{
    R_OSPL_TABLE( gs_global_error_table ),
    gs_error,  /* .ErrorArray */

#if R_OSPL_ERROR_BREAK
    0,          /* .RaisedGlobalErrorID */  /* Necessary T-Lock */
    0,          /* .BreakGlobalErrorID */
#endif
#if R_OSPL_DETECT_BAD_EVENT
    0           /* /PreviousEventNum */
#endif
};
#endif


/***********************************************************************
* Implement: R_OSPL_GetVersion
************************************************************************/
int32_t  R_OSPL_GetVersion(void)
{
    return  R_OSPL_VERSION;
}


/***********************************************************************
* Implement: R_OSPL_IsPreemption
************************************************************************/
bool_t  R_OSPL_IsPreemption(void)
{
    return  R_OSPL_IS_PREEMPTION;
}


/***********************************************************************
* Implement: R_OSPL_FLAG32_InitConst
************************************************************************/
void  R_OSPL_FLAG32_InitConst( volatile r_ospl_flag32_t* const  self )
{
    IF_DQ( self == NULL )
    {
        goto fin;
    }

    self->Flags = 0;

fin:
    return;
}


/***********************************************************************
* Implement: R_OSPL_FLAG32_Set
************************************************************************/
void  R_OSPL_FLAG32_Set( volatile r_ospl_flag32_t* const  self, bit_flags32_t const  SetFlags )
{
    IF_DQ( self == NULL )
    {
        goto fin;
    }

    self->Flags |= SetFlags;

fin:
    return;
}


/***********************************************************************
* Implement: R_OSPL_FLAG32_Clear
************************************************************************/
void  R_OSPL_FLAG32_Clear( volatile r_ospl_flag32_t* const  self, bit_flags32_t const  ClearFlags1 )
{
    IF_DQ( self == NULL )
    {
        goto fin;
    }

    self->Flags &= ~ClearFlags1;

fin:
    return;
}


/***********************************************************************
* Implement: R_OSPL_FLAG32_Get
************************************************************************/
bit_flags32_t  R_OSPL_FLAG32_Get( volatile const r_ospl_flag32_t* const  self )
{
    bit_flags32_t  return_value;

    IF_DQ( self == NULL )
    {
        return_value = 0;
        goto fin;
    }

    return_value = self->Flags;

fin:
    return  return_value;
}


/***********************************************************************
* Implement: R_OSPL_FLAG32_GetAndClear
************************************************************************/
bit_flags32_t  R_OSPL_FLAG32_GetAndClear( volatile r_ospl_flag32_t* const  self )
{
    bit_flags32_t  return_value;

    IF_DQ( self == NULL )
    {
        return_value = 0;
        goto fin;
    }

    return_value = self->Flags;
    self->Flags = 0;

fin:
    return  return_value;
}


/***********************************************************************
* Implement: R_OSPL_ASYNC_SetDefaultPreset
************************************************************************/
void  R_OSPL_ASYNC_SetDefaultPreset( r_ospl_async_t* const  in_out_Async )
{
    errnum_t  e;

    bit_flags_fast32_t  flags = in_out_Async->Flags;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_ASYNC_SetDefaultPreset()_E");

    if ( (flags & R_F_OSPL_MaskOfPreset) == R_F_OSPL_AsynchronousPreset )
    {
        if ( IS_BIT_NOT_SET( flags, R_F_OSPL_A_Thread ) )
        {
            in_out_Async->A_Thread = R_OSPL_THREAD_GetCurrentId();
            flags |= R_F_OSPL_A_Thread;
        }
        if ( IS_BIT_NOT_SET( flags, R_F_OSPL_A_EventValue ) )
        {
#if  R_OSPL_TLS_ERROR_CODE
            in_out_Async->A_EventValue = R_OSPL_UNUSED_FLAG;
#else
            in_out_Async->A_EventValue = R_OSPL_A_FLAG;
#endif
            flags |= R_F_OSPL_A_EventValue;
        }

        e= R_OSPL_EVENT_Allocate(
               &in_out_Async->A_Thread,  in_out_Async->A_Thread,
               &in_out_Async->A_EventValue,  in_out_Async->A_EventValue );
        IF(e!=0)
        {
            goto fin;
        }
    }
    flags &= ~R_F_OSPL_MaskOfPreset;

    in_out_Async->Flags = flags;


    e=0;
fin:
#if  R_OSPL_IS_ASYNC_LIST
    {
        static uint_fast32_t  count;
        bool_t               was_all_enabled = R_OSPL_DisableAllInterrupt();

        count += 1;

        if ( was_all_enabled )
        {
            R_OSPL_EnableAllInterrupt();
        }

        R_D_AddToIntLog( (int_fast32_t)( 0xAC000000 + count ) );
        R_D_AddToIntLog( (int_fast32_t) in_out_Async );  /* ... Check it. */

        if ( count == 0 ) /* ... It is able to be changed. */
        {
            R_DEBUG_BREAK();
        }
    }
#endif


    if ( e != 0 )
    {
        R_OSPL_RaiseUnrecoverable( e );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_ASYNC_SetDefaultPreset()_X");
}


/***********************************************************************
* Implement: R_OSPL_ASYNC_CopyExceptAThread
************************************************************************/
void  R_OSPL_ASYNC_CopyExceptAThread( const r_ospl_async_t* const  Source,
                                      r_ospl_async_t* const  Destination )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_ASYNC_CopyExceptAThread()_E");
	IF_DQ(Destination == NULL)
    {
        goto fin;
    }
    IF_DQ( Source == NULL )
    {
        goto fin;
    }

    *Destination = *Source;
    Destination->A_Thread = R_OSPL_THREAD_NULL;

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_ASYNC_CopyExceptAThread()_X");
	return;
}


/***********************************************************************
* Implement: R_OSPL_EnableAllInterrupt
************************************************************************/
void  R_OSPL_EnableAllInterrupt(void)
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EnableAllInterrupt()_E");
	//    __enable_irq(); // tokuyama@kmg 2016.09.14 Toppers に任せる
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EnableAllInterrupt()_X");
}


/***********************************************************************
* Implement: R_OSPL_DisableAllInterrupt
************************************************************************/
bool_t  R_OSPL_DisableAllInterrupt(void)
{

#if 0 //tokuyama@kmg 2016.09.14 Toppersに任せる
#ifdef __ICCARM__
    bool_t  was_enabled = ( ( __get_interrupt_state() & 0x80 ) == 0 );
    __disable_irq();
    return  was_enabled;
#else
    return  (bool_t)( __disable_irq() == 0 );
#endif
#else
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_DisableAllInterrupt()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_DisableAllInterrupt()_X");
	return (1 == 1);
#endif /* 0 */
}


/***********************************************************************
* Implement: R_BSP_InterruptsEnable
************************************************************************/
void  R_BSP_InterruptsEnable(void)
{
#if 0 //tokuyama@kmg 2016.09.14 Toppersに任せる
    __enable_irq();
#endif /* 0 */
	TOPPERS_SYSLOG(LOG_INFO, "R_BSP_InterruptEnable()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_BSP_InterruptEnable()_X");
}


/***********************************************************************
* Implement: R_BSP_InterruptsDisable
************************************************************************/
void  R_BSP_InterruptsDisable(void)
{
#if 0 //tokuyama@kmg 2016.09.14 Toppersに任せる
#ifdef __ICCARM__
    __disable_irq();
#else
    int_fast32_t  ret;

    ret = __disable_irq();
    R_UNREFERENCED_VARIABLE( ret );  /* QAC 3200 : This is not error information */
#endif
#endif /* 0 */
	TOPPERS_SYSLOG(LOG_INFO, "R_BSP_InterruptDisable()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_BSP_InterruptDisable()_X");
}


/***********************************************************************
* Implement: R_OSPL_LockChannel
************************************************************************/
errnum_t  R_OSPL_LockChannel( int_fast32_t in_ChannelNum, int_fast32_t* out_ChannelNum,
                              mcu_lock_t  in_HardwareIndexMin,  mcu_lock_t  in_HardwareIndexMax )
{
    errnum_t    e;
    bool_t      is_success;
    mcu_lock_t  hardware_index = BSP_NUM_LOCKS;  /* BSP_NUM_LOCKS is avoid for warning of "Used before set symbol" */
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_LockChannel()_E");
    ASSERT_D( in_HardwareIndexMin <= in_HardwareIndexMax,  e=E_OTHERS; goto fin );

    if ( in_ChannelNum == R_OSPL_UNLOCKED_CHANNEL )
    {
        ASSERT_D( out_ChannelNum != NULL,  e=E_OTHERS; goto fin );

#if BSP_CFG_USER_LOCKING_ENABLED
        for ( hardware_index = in_HardwareIndexMin;  hardware_index <= in_HardwareIndexMax;
                hardware_index += 1 )
        {
            is_success = R_BSP_HardwareLock( hardware_index );
            if ( is_success )
            {
                break;
            }
        }
        IF ( hardware_index > in_HardwareIndexMax )
        {
            e=E_FEW_ARRAY;
            goto fin;
        }
#else
        e= R_OSPL_LockUnlockedChannel( out_ChannelNum,
                                       in_HardwareIndexMin, in_HardwareIndexMax );
        IF(e)
        {
            goto fin;
        }
        out_ChannelNum = NULL;
#endif
    }
    else
    {
        hardware_index = (mcu_lock_t)( in_HardwareIndexMin + in_ChannelNum );

        IF_D ( hardware_index < in_HardwareIndexMin  ||  hardware_index > in_HardwareIndexMax )
        {
            e=E_FEW_ARRAY;
            goto fin;
        }

        is_success = R_BSP_HardwareLock( hardware_index );
        IF ( ! is_success )
        {
            e=E_ACCESS_DENIED;
            goto fin;
        }
    }

    if ( out_ChannelNum != NULL )
    {
        *out_ChannelNum = hardware_index - in_HardwareIndexMin;
        /* Warning of "hardware_index" can be ignored. */
        /* Because this code "hardware_index" is not executed by code of "out_ChannelNum = NULL" */
    }

    e=0;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_LockChannel()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_UnlockChannel
************************************************************************/
errnum_t  R_OSPL_UnlockChannel( int_fast32_t in_ChannelNum,  errnum_t  e,
                                mcu_lock_t  in_HardwareIndexMin,  mcu_lock_t  in_HardwareIndexMax )
{
    bool_t      is_success;
    mcu_lock_t  hardware_index;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_UnlockChannel()_E");
    ASSERT_D( in_HardwareIndexMin <= in_HardwareIndexMax,  e= R_OSPL_MergeErrNum( e, E_OTHERS ) );

    hardware_index = (mcu_lock_t)( in_HardwareIndexMin + in_ChannelNum );

    if ( hardware_index >= in_HardwareIndexMin  &&  hardware_index <= in_HardwareIndexMax )
    {
        is_success = R_BSP_HardwareUnlock( hardware_index );
        IF ( ! is_success )
        {
            e= R_OSPL_MergeErrNum( e, E_ACCESS_DENIED );
        }
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_UnlockChannel()_X");
    return  e;
}


/***********************************************************************
* Implement: R_OSPL_C_LOCK_InitConst
************************************************************************/
void  R_OSPL_C_LOCK_InitConst( r_ospl_c_lock_t* const  self )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_InitConst()_E");
	IF_DQ(self == NULL)
    {
        goto fin;
    }

    self->IsLocked = false;

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_InitConst()_X");
	return;
}


/***********************************************************************
* Implement: R_OSPL_C_LOCK_Lock
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Lock( r_ospl_c_lock_t* const  self )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Lock()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Lock()_X");
	return R_OSPL_C_LOCK_Lock_Sub(self, NULL);
}


/***********************************************************************
* Implement: R_OSPL_C_LOCK_Lock_Sub
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Lock_Sub( r_ospl_c_lock_t* const  self,  r_ospl_error_t* const  err )
{
    errnum_t  e;

    R_UNREFERENCED_VARIABLE( err );  /* When "IF_QDC" was disabled */
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Lock_Sub()_E");

    /* IF_DQC( self == NULL,  err ) { e=E_OTHERS; goto fin; } */
    /* SH Compiler warns "C0004 (I) Constant as condition", if not comment out. */

    IF_C ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL,  err )    /* Interrupt */
    {
        e = E_NOT_THREAD;
        R_OSPL_RaiseUnrecoverable( e );
        goto fin;
    }

    IF_C ( IS( self->IsLocked ),  err )
    {
        e=E_ACCESS_DENIED;
        goto fin;
    }

    self->IsLocked = true;

    e=0;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Lock_Sub()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_C_LOCK_Unlock
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Unlock( r_ospl_c_lock_t* const  self )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Unlock()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Unlock()_X");
	return R_OSPL_C_LOCK_Unlock_Sub(self, NULL);
}


/***********************************************************************
* Implement: R_OSPL_C_LOCK_Unlock_Sub
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Unlock_Sub( r_ospl_c_lock_t* const  self,  r_ospl_error_t* const  err )
{
    errnum_t  e;

    R_UNREFERENCED_VARIABLE( err );  /* When "IF_C" was disabled */
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Unlock_Sub()_E");

    if ( self != NULL )
    {

        IF_C ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL,  err )    /* Interrupt */
        {
            e = E_NOT_THREAD;
            R_OSPL_RaiseUnrecoverable( e );
            goto fin;
        }

        IF_C ( ! self->IsLocked,  err )
        {
            /* Check not unlock the object that was initialized by other thread */
            e = E_ACCESS_DENIED;
            goto fin;
        }

        self->IsLocked = false;
    }

    e=0;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_Unlock_Sub()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_I_LOCK_LockStub
************************************************************************/
bool_t  R_OSPL_I_LOCK_LockStub( void* const  self_ )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_LockStub()_E");
	R_IT_WILL_BE_NOT_CONST(self_);
    R_UNREFERENCED_VARIABLE( self_ );
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_LockStub()_X");
	return false;
}


/***********************************************************************
* Implement: R_OSPL_I_LOCK_UnlockStub
************************************************************************/
void  R_OSPL_I_LOCK_UnlockStub( void* const  self_ )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_UnlockStub()_E");
	R_IT_WILL_BE_NOT_CONST(self_);
    R_UNREFERENCED_VARIABLE( self_ );
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_C_LOCK_UnlockStub()_X");
}


/***********************************************************************
* Implement: R_OSPL_I_LOCK_RequestFinalizeStub
************************************************************************/
void  R_OSPL_I_LOCK_RequestFinalizeStub( void* const  self_ )
{
    R_IT_WILL_BE_NOT_CONST( self_ );
    R_UNREFERENCED_VARIABLE( self_ );
}


/***********************************************************************
* Implement: R_OSPL_MEMORY_Barrier
************************************************************************/
void  R_OSPL_MEMORY_Barrier(void)
{
    /* ->QAC 1006 : asm */
    __asm("DSB");
    /* <-QAC 1006 */
}


/***********************************************************************
* Implement: R_OSPL_InstructionSyncBarrier
************************************************************************/
void  R_OSPL_InstructionSyncBarrier(void)
{
    /* ->QAC 1006 : asm */
    __asm("ISB");
    /* <-QAC 1006 */
}


/***********************************************************************
* Implement: R_OSPL_CALLER_Initialize
************************************************************************/
void  R_OSPL_CALLER_Initialize( r_ospl_caller_t* const  self,  r_ospl_async_t* const  Async,
                                volatile void* const  PointerToState,  int_t const  StateValueOfOnInterrupting,
                                void* const  I_Lock, const r_ospl_i_lock_vtable_t* const  I_LockVTable )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CALLER_Initialize()_E");

	ASSERT_D( Async != NULL,  R_NOOP() );
    ASSERT_D( PointerToState != NULL,  R_NOOP() );
    IF_DQ( self == NULL ) {}
    else
    {
        self->Async = Async;
        self->PointerToState = (volatile int_fast32_t*) PointerToState;
        self->StateValueOfOnInterrupting = StateValueOfOnInterrupting;
        self->I_Lock = I_Lock;
        self->I_LockVTable = I_LockVTable;
    }

#ifndef R_OSPL_NDEBUG
    /* Set sentinel */
    ASSERT_D( IS_ALL_BITS_NOT_SET( Async->Flags, R_F_OSPL_ASYNC_FLAGS_SENTINEL_MASK ),
              R_NOOP() );
    ASSERT_D( IS_ALL_BITS_NOT_SET( R_F_OSPL_ASYNC_FLAGS_SENTINEL_VALUE,
                                   (bit_flags_fast32_t)(~R_F_OSPL_ASYNC_FLAGS_SENTINEL_MASK) ),  R_NOOP() );

    Async->Flags |= R_F_OSPL_ASYNC_FLAGS_SENTINEL_VALUE;
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CALLER_Initialize()_X");
}


/***********************************************************************
* Implement: R_OSPL_FTIMER_IsPast
************************************************************************/
errnum_t  R_OSPL_FTIMER_IsPast( const r_ospl_ftimer_spec_t* const  ts,
                                uint32_t const  Now,  uint32_t const  TargetTime,  bool_t* const  out_IsPast )
{
    uint32_t const         target_minus_now = TargetTime - Now;
    static const uint32_t  minus_flag = 0x80000000u;
    errnum_t       e;
    bool_t const   is_past = IS_BIT_SET( (uint_fast32_t) target_minus_now, (uint_fast32_t) minus_flag );

    IF_DQ( ts == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    IF_DQ( out_IsPast == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    if ( IS( is_past ) )
    {
        uint32_t const  now_minus_target = Now - TargetTime;

        ASSERT_R( now_minus_target <= ts->ExtensionOfCount,  e=E_TIME_OUT; goto fin );
        /* Error: Over "r_ospl_ftimer_spec_t::ExtensionOfCount" */
    }

    *out_IsPast = is_past;

    e=0;
fin:
    return  e;
}


/***********************************************************************
* Implement: R_OSPL_TABLE_InitConst
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
*    This function has T-Lock area.
************************************************************************/
void  R_OSPL_TABLE_InitConst( r_ospl_table_t* const  self,
                              void* const  in_Area,  size_t const  in_AreaByteSize,  r_ospl_table_flags_t const  in_Flags )
{
#if  ! defined( R_OSPL_NDEBUG ) ||  R_OSPL_IS_PREEMPTION
    errnum_t  e;
#endif
    bool_t    was_all_enabled = false;
#if R_OSPL_IS_PREEMPTION
    bool_t    is_lock = false;
#endif

#if defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)  /* IF_DQ */
    if ( self == NULL )
    {
#ifndef R_OSPL_NDEBUG
        e=E_OTHERS;
#endif
        goto fin;
    }
#endif

#if R_OSPL_IS_PREEMPTION
    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_T_LOCK ) )
    {
        if ( R_OSPL_THREAD_GetCurrentId() != R_OSPL_THREAD_NULL )    /* If not interrrupt context */
        {
            e= R_OSPL_Start_T_Lock();
            if(e!=0)
            {
                goto fin;
            }
            is_lock = true;  /* T-Lock to "self" */
        }
    }
#endif

    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_I_LOCK ) )
    {
        was_all_enabled = R_OSPL_DisableAllInterrupt();
    }

    self->Area       = in_Area;
    self->Count      = 0;
    self->MaxCount   = (int_fast32_t)( in_AreaByteSize / sizeof( r_ospl_table_block_t ) );
    self->KeyCache   = NULL;
    self->FirstFreeIndex = 0;
    self->TableFlags = in_Flags | R_OSPL_TABLE_IS_KEY_SORTED | R_OSPL_TABLE_IS_IN_CRITICAL;


    /* Set "self->Area[].NextFreeIndex" */
    {
        r_ospl_table_block_t* const  block_array = (r_ospl_table_block_t*) in_Area;
        int_fast32_t                 index;
        int_fast32_t const           max_index = self->MaxCount - 1;

#if defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)  /* IF_DQ */
        if ( block_array == NULL )
        {
#ifndef R_OSPL_NDEBUG
            e=E_OTHERS;
#endif
            goto fin;
        }
#endif

        for ( index = 0;  index < max_index;  index += 1 )
        {
            block_array[ index ].NextFreeIndex = (int16_t)( index + 1 );
        }
        block_array[ max_index ].NextFreeIndex = R_OSPL_TABLE_BLOCK_NO_NEXT;
    }

#ifndef R_OSPL_NDEBUG
    e=0;
#endif
#if  R_OSPL_IS_PREEMPTION || defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)
fin:
#endif
    self->TableFlags &= ~R_OSPL_TABLE_IS_IN_CRITICAL;

    if ( IS( was_all_enabled ) )
    {
        R_OSPL_EnableAllInterrupt();
    }

#if R_OSPL_IS_PREEMPTION
    if ( IS( is_lock ) )
    {
        R_OSPL_End_T_Lock();
    }
#endif
#ifndef R_OSPL_NDEBUG
    if ( e != 0 )
    {
        R_DebugBreak( NULL, e );
    }
#endif
}


/***********************************************************************
* Implement: R_OSPL_TABLE_GetIndex
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
*    This function has T-Lock area.
************************************************************************/
errnum_t  R_OSPL_TABLE_GetIndex( r_ospl_table_t* const  self,  const void* const  in_Key,
                                 int_fast32_t* const  out_Index,  r_ospl_if_not_t  in_TypeOfIfNot )
{
    errnum_t  e;
    bool_t    was_all_enabled = false;
#if R_OSPL_IS_PREEMPTION
    bool_t    is_lock = false;
#endif

#if defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)  /* IF_DQ */
    if ( self == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    if ( out_Index == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
#endif

#if R_OSPL_IS_PREEMPTION
    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_T_LOCK ) )
    {
        if ( R_OSPL_THREAD_GetCurrentId() != R_OSPL_THREAD_NULL )    /* If not interrrupt context */
        {
            e= R_OSPL_Start_T_Lock();
            if(e!=0)
            {
                goto fin;
            }
            is_lock = true;  /* T-Lock to "self" */
        }
    }
#endif

    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_I_LOCK ) )
    {
        was_all_enabled = R_OSPL_DisableAllInterrupt();
    }

#ifndef  R_OSPL_NDEBUG
    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_IS_IN_CRITICAL ) )
    {
        R_OSPL_RaiseUnrecoverable( E_STATE );
    }
    self->TableFlags |= R_OSPL_TABLE_IS_IN_CRITICAL;
#endif


    if ( (in_Key == self->KeyCache)  &&
            (in_TypeOfIfNot != R_OSPL_ALLOCATE_IF_EXIST_OR_IF_NOT)  &&
            (self->Count >= 1) )
    {
        if ( in_TypeOfIfNot != R_OSPL_OUTPUT_IF_NOT )
        {
            *out_Index = self->IndexCache;
        }
    }
    else
    {
        r_ospl_table_searched_t      searched;
        r_ospl_table_block_t* const  block_array = self->Area;
        int_fast32_t                 found_index;

        if ( self->Count == 0  &&  self->MaxCount >= 1  &&
                self->FirstFreeIndex == R_OSPL_TABLE_NOT_INITIALIZED )
        {
#if R_OSPL_IS_PREEMPTION
            if ( IS( is_lock ) )
            {
                R_OSPL_End_T_Lock();
                is_lock = false;
            }
#endif


            R_OSPL_TABLE_InitConst( self,  self->Area,
                                    self->MaxCount * sizeof( r_ospl_table_block_t ),  self->TableFlags );


#if R_OSPL_IS_PREEMPTION
            if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_T_LOCK ) )
            {
                if ( R_OSPL_THREAD_GetCurrentId() != R_OSPL_THREAD_NULL )    /* If not interrrupt context */
                {
                    e= R_OSPL_Start_T_Lock();
                    if(e!=0)
                    {
                        goto fin;
                    }
                    is_lock = true;  /* T-Lock to "self" */
                }
            }
#endif


            searched.SortedKeyIndex = 0;
            searched.IsFound = false;

            if ( in_TypeOfIfNot == R_OSPL_ALLOCATE_IF_EXIST_OR_IF_NOT )
            {
                self->TableFlags &= ~R_OSPL_TABLE_IS_KEY_SORTED;
            }
        }
        else if ( in_TypeOfIfNot == R_OSPL_ALLOCATE_IF_EXIST_OR_IF_NOT )
        {
            searched.SortedKeyIndex = self->Count;  /* Disable the following "so_index" loop */
            searched.IsFound = false;
            self->TableFlags &= ~R_OSPL_TABLE_IS_KEY_SORTED;
        }
        else
        {
            if ( IS_BIT_NOT_SET( self->TableFlags, R_OSPL_TABLE_IS_KEY_SORTED ) )
            {
                e=E_STATE;
                goto fin;
            }

            R_OSPL_TABLE_Search_Sub( self, in_Key, &searched );
        }

        if ( ! searched.IsFound )
        {
            if ( in_TypeOfIfNot == R_OSPL_ERROR_IF_NOT )
            {
                e = E_NOT_FOUND_SYMBOL;
                goto fin;
            }
            else if ( in_TypeOfIfNot == R_OSPL_DO_NOTHING_IF_NOT )
            {
                e = 0;
                goto fin;
            }
            else
            {
                int_fast32_t  so_index;  /* sorted_key_index */
                int_fast32_t  first_index;  /* FirstFreeIndex */

                if ( self->Count == self->MaxCount )
                {
                    e = E_FEW_ARRAY;
                    goto fin;
                }

                /* Insert and Set "block_array[ searched.SortedKeyIndex ].Key" */
                for ( so_index = self->Count - 1;
                        so_index >= searched.SortedKeyIndex;
                        so_index -= 1 )
                {
                    block_array[ so_index + 1 ].Key   = block_array[ so_index ].Key;
                    block_array[ so_index + 1 ].Index = block_array[ so_index ].Index;
                }
                block_array[ searched.SortedKeyIndex ].Key = in_Key;
                self->Count += 1;

                /* Set "block_array[ searched.SortedKeyIndex ].Index" */
                first_index = self->FirstFreeIndex;
                block_array[ searched.SortedKeyIndex ].Index = (int16_t) first_index;
                self->FirstFreeIndex = block_array[ first_index ].NextFreeIndex;
                block_array[ first_index ].NextFreeIndex = R_OSPL_TABLE_BLOCK_USED;

                /* For R_OSPL_OUTPUT_IF_NOT */
                in_TypeOfIfNot = R_OSPL_ALLOCATE_IF_NOT;
            }
        }

        self->KeyCache = in_Key;
        found_index = block_array[ searched.SortedKeyIndex ].Index;
        self->IndexCache = found_index;
        if ( in_TypeOfIfNot != R_OSPL_OUTPUT_IF_NOT )
        {
            *out_Index = found_index;
        }
    }

    e=0;
fin:
#ifndef  R_OSPL_NDEBUG
    self->TableFlags &= ~R_OSPL_TABLE_IS_IN_CRITICAL;
#endif

    if ( IS( was_all_enabled ) )
    {
        R_OSPL_EnableAllInterrupt();
    }

#if R_OSPL_IS_PREEMPTION
    if ( IS( is_lock ) )
    {
        R_OSPL_End_T_Lock();
    }
#endif
    return  e;
}


/***********************************************************************
* Implement: R_OSPL_TABLE_Free
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
*    This function has T-Lock area.
************************************************************************/
void  R_OSPL_TABLE_Free( r_ospl_table_t* const  self,  const void* const  in_Key )
{
    errnum_t  e;

    if ( IS_BIT_NOT_SET( self->TableFlags, R_OSPL_TABLE_IS_KEY_SORTED ) )
    {
        e=E_STATE;
        goto fin;
    }


    R_OSPL_TABLE_Free_Sub( self,  in_Key,  R_OSPL_NO_INDEX );


    e=0;
fin:
    if ( e != 0 )
    {
        R_OSPL_RaiseUnrecoverable( e );
    }
    return;
}


/***********************************************************************
* Implement: R_OSPL_TABLE_FreeByIndex
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
*    This function has T-Lock area.
************************************************************************/
void  R_OSPL_TABLE_FreeByIndex( r_ospl_table_t* const  self,  int_fast32_t const  in_Index )
{
    errnum_t  e;

    if ( in_Index < 0  ||  in_Index >= self->MaxCount )
    {
        e=E_OTHERS;
        goto fin;
    }
    /* This code can not in "R_OSPL_TABLE_Free_Sub". */
    /* Because case of "in_Index == R_OSPL_NO_INDEX". */


    R_OSPL_TABLE_Free_Sub( self,  NULL,  in_Index );


    e=0;
fin:
    if ( e != 0 )
    {
        R_OSPL_RaiseUnrecoverable( e );
    }
    return;
}


/***********************************************************************
* Implement: R_OSPL_TABLE_Free_Sub
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
*    This function has T-Lock area.
************************************************************************/
static void  R_OSPL_TABLE_Free_Sub( r_ospl_table_t* const  self,  const void* const  in_Key,  int_fast32_t  in_Index )
{
    errnum_t  e;
    bool_t    was_all_enabled = false;
#if R_OSPL_IS_PREEMPTION
    bool_t    is_lock = false;
#endif
    r_ospl_table_searched_t  searched;


#if  defined( R_OSPL_NDEBUG ) || defined(__QAC_ARM_H__)  /* IF_DQ */
    if ( self == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
#endif

#if R_OSPL_IS_PREEMPTION
    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_T_LOCK ) )
    {
        if ( R_OSPL_THREAD_GetCurrentId() != R_OSPL_THREAD_NULL )    /* If not interrrupt context */
        {
            e= R_OSPL_Start_T_Lock();
            if(e!=0)
            {
                goto fin;
            }
            is_lock = true;  /* T-Lock to "gs_global_error" */
        }
    }
#endif

    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_I_LOCK ) )
    {
        was_all_enabled = R_OSPL_DisableAllInterrupt();
    }

#ifndef  R_OSPL_NDEBUG
    if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_IS_IN_CRITICAL ) )
    {
        R_OSPL_RaiseUnrecoverable( E_STATE );
    }
    self->TableFlags |= R_OSPL_TABLE_IS_IN_CRITICAL;
#endif


    if ( in_Index == R_OSPL_NO_INDEX )
    {

        R_OSPL_TABLE_Search_Sub( self, in_Key, &searched );

    }
    else
    {
        r_ospl_table_block_t* const  block_array = self->Area;

        searched.IsFound = ( block_array[ in_Index ].NextFreeIndex == R_OSPL_TABLE_BLOCK_USED );
        if ( ! searched.IsFound )
        {
            e=E_NOT_FOUND_SYMBOL;
            goto fin;
        }

        if ( IS_BIT_SET( self->TableFlags, R_OSPL_TABLE_IS_KEY_SORTED ) )
        {
            e=E_STATE;
            goto fin;
        }
        /* Because (1) "block_array[ in_Index ].Index != in_Index". */
        /* Because (2) "block_array[ ].Index" is not sorted. */

        searched.SortedKeyIndex = self->Count - 1;
        block_array[ searched.SortedKeyIndex ].Index = (int16_t) in_Index;
    }

    if ( IS( searched.IsFound ) )
    {
        r_ospl_table_block_t* const  block_array = self->Area;
        int_fast32_t                 so_index;  /* sorted_key_index */
        int_fast32_t                 first_index;  /* FirstFreeIndex */
        int_fast32_t                 new_count;

        new_count = self->Count - 1;
        self->Count = new_count;

        /* Set "FirstFreeIndex", ".NextFreeIndex" */
        first_index = self->FirstFreeIndex;
        self->FirstFreeIndex = block_array[ searched.SortedKeyIndex ].Index;
        block_array[ self->FirstFreeIndex ].NextFreeIndex = (int16_t) first_index;

        /* Remove one "r_ospl_table_block_t" */
        /* Set "self->KeyCache" */
        if ( searched.SortedKeyIndex < new_count )
        {
            for ( so_index = searched.SortedKeyIndex;  so_index < new_count ;  so_index += 1 )
            {
                block_array[ so_index ].Key   = block_array[ so_index + 1 ].Key;
                block_array[ so_index ].Index = block_array[ so_index + 1 ].Index;
            }
            self->KeyCache   = block_array[ searched.SortedKeyIndex ].Key;
            self->IndexCache = block_array[ searched.SortedKeyIndex ].Index;
        }
        else if ( searched.SortedKeyIndex > 0 )    /* searched.SortedKeyIndex >= 1 */
        {
            self->KeyCache   = block_array[ searched.SortedKeyIndex - 1 ].Key;
            self->IndexCache = block_array[ searched.SortedKeyIndex - 1 ].Index;
        }
        else
        {
            self->KeyCache = NULL;
        }
    }

    e=0;
fin:
#ifndef  R_OSPL_NDEBUG
    self->TableFlags &= ~R_OSPL_TABLE_IS_IN_CRITICAL;
#endif

    if ( IS( was_all_enabled ) )
    {
        R_OSPL_EnableAllInterrupt();
    }

#if R_OSPL_IS_PREEMPTION
    if ( IS( is_lock ) )
    {
        R_OSPL_End_T_Lock();
    }
#endif

    if ( e != 0 )
    {
        R_OSPL_RaiseUnrecoverable( e );
    }
}


/***********************************************************************
* Function: R_OSPL_TABLE_Search_Sub
*    R_OSPL_TABLE_Search_Sub
*
* Arguments:
*    in_Key       - Key
*    out_Searched - out_Searched
*
* Return Value:
*    Block Index
*
* Description:
*    IF, ASSERT macro can not be called in this function because reentrant.
************************************************************************/
static void  R_OSPL_TABLE_Search_Sub( const r_ospl_table_t* const  self,
                                      const void* const  in_Key,  r_ospl_table_searched_t* const  out_Searched )
{
    int_fast32_t     left;
    int_fast32_t     right;
    int_fast32_t     middle;
    /* ->QAC 0306 : Sort by pointer value */
    uintptr_t const  target_key = (uintptr_t) in_Key;
    /* <-QAC 0306 */
    uintptr_t        middle_key;
    enum {           num_2 = 2 };  /* SEC M1.10.1, QAC-3132 */
    r_ospl_table_block_t*  array;

#if defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)  /* IF_DQ */
    if ( self == NULL )
    {
        goto fin;
    }
    if ( out_Searched == NULL )
    {
        goto fin;
    }
#endif

    array = self->Area;


    if ( self->Count == 0 )
    {
        out_Searched->SortedKeyIndex = 0;
        out_Searched->IsFound = false;
    }
    else
    {
        left = 0;
        right = self->Count - 1;

        while ( (right - left) >= num_2 )
        {
            middle = (int_fast32_t)( (uint_fast32_t)( right + left ) / num_2 );
            /* ->QAC 0306 : Sort by pointer value */
            middle_key = (uintptr_t) array[ middle ].Key;
            /* <-QAC 0306 */

            if ( target_key == middle_key )
            {
                out_Searched->SortedKeyIndex = middle;
                out_Searched->IsFound = true;
                goto fin;
            }
            else if ( target_key <  middle_key )
            {
                right = (int_fast32_t)( middle - 1 );
            }
            else
            {
                left = (int_fast32_t)( middle + 1 );
            }
        }

        /* ->QAC 0306 : Sort by pointer value */
        if ( target_key == (uintptr_t) array[ left ].Key )
        {
            out_Searched->SortedKeyIndex = left;
            out_Searched->IsFound = true;
        }
        else if ( target_key == (uintptr_t) array[ right ].Key )
        {
            out_Searched->SortedKeyIndex = right;
            out_Searched->IsFound = true;
        }
        else if ( target_key < (uintptr_t) array[ left ].Key )
        {
            out_Searched->SortedKeyIndex = left;
            out_Searched->IsFound = false;
        }
        else if ( target_key > (uintptr_t) array[ right ].Key )
        {
            out_Searched->SortedKeyIndex = right + 1;
            out_Searched->IsFound = false;
        }
        else
        {
            out_Searched->SortedKeyIndex = right;
            out_Searched->IsFound = false;
        }
        /* <-QAC 0306 */
    }

fin:
    return;
}


/***********************************************************************
* Implement: R_OSPL_TABLE_Print
************************************************************************/
#if R_OSPL_DEBUG_TOOL  &&  ! defined( R_OSPL_NDEBUG )
void  R_OSPL_TABLE_Print( r_ospl_table_t* const  self )
{
    int_fast32_t  so_index;  /* sorted_key_index */
    int_fast32_t  index;
    r_ospl_table_block_t* const  array = self->Area;

    printf( "R_OSPL_TABLE_Print: r_ospl_table_t 0x%08X\n", (uintptr_t) self );
    printf( "    .Count = %d\n", self->Count );
    for ( so_index = 0;  so_index < self->Count;  so_index += 1 )
    {
        printf( "    .Area[%d].Key, Index = 0x%08X, %d\n",
                so_index,  (uintptr_t) array[ so_index ].Key,  array[ so_index ].Index );
    }
    printf( "    .FirstFreeIndex = %d\n", self->FirstFreeIndex );
    for ( index = 0;  index < self->MaxCount;  index += 1 )
    {
        printf( "    .Area[%d].NextFreeIndex = %d\n", index,  array[ index ].NextFreeIndex );
    }
    printf( "        %d = R_OSPL_TABLE_BLOCK_NO_NEXT\n", R_OSPL_TABLE_BLOCK_NO_NEXT );
    printf( "        %d = R_OSPL_TABLE_BLOCK_USED\n", R_OSPL_TABLE_BLOCK_USED );
}
#endif


/***********************************************************************
* Implement: R_OSPL_CallInterruptCallback
************************************************************************/
void  R_OSPL_CallInterruptCallback( const r_ospl_caller_t* const  self,
                                    const r_ospl_interrupt_t* const  InterruptSource )
{
    errnum_t  e;
    r_ospl_async_t*  async;
#if ! R_OSPL_IS_PREEMPTION
    r_ospl_master_t* const  gs_master = R_OSPL_GetPrivateContext();
    /* GSCE can not allow an imported/exported variable. */
    r_ospl_thread_def_t*  current_thread = R_OSPL_THREAD_NULL;  /* R_OSPL_THREAD_NULL is for avoid QAC 3353 */
#endif
#if R_OSPL_ERROR_BREAK
    static int_fast32_t  gs_nested_interrupt_level = 0;
#endif
TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CallInterruptCallback()_E");


#if R_OSPL_ERROR_BREAK
    gs_nested_interrupt_level += 1;
#endif

#if ! R_OSPL_IS_PREEMPTION
#if  R_OSPL_STACK_CHECK_CODE
    R_OSPL_CHECK_STACK_OVERFLOW();  /* Check main stack */
#endif

    IF_DQ ( gs_master == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    current_thread = gs_master->CurrentThread;
    gs_master->CurrentThread = R_OSPL_THREAD_NULL;  /* NULL = Interrupt */
#endif

#if  R_OSPL_STACK_CHECK_CODE  &&  defined( GS_INTERRUPT_STACK_SIZE )
    {
        static bool_t  is_initialized = false;

        if ( ! is_initialized )
        {
            if ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL )
            {
                R_OSPL_SET_END_OF_STACK( R_OSPL_GET_STACK_POINTER() -
                                         ( GS_INTERRUPT_STACK_SIZE - GS_INTERRUPT_STACK_BUFFER_SIZE ) );

                R_OSPL_RESET_MIN_FREE_STACK_SIZE(); /* This fills sentinels in stack */
                is_initialized = true;
            }
        }

        R_OSPL_CHECK_STACK_OVERFLOW();
    }
#endif

#if ! R_OSPL_IS_PREEMPTION
#if R_OSPL_CPU_LOAD
    e= gs_master->IdleCallback( R_OSPL_INTERRUPT_START );
    IF(e)
    {
        goto fin;
    }
#endif
#endif

    IF_DQ( self == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    IF_DQ( self->Async == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    async = self->Async;

    ASSERT_D( ( async->Flags & R_F_OSPL_ASYNC_FLAGS_SENTINEL_MASK ) ==
              R_F_OSPL_ASYNC_FLAGS_SENTINEL_VALUE,  e=E_OTHERS; goto fin );
    /* If failed, memory area of "Async" variable was overwritten by other variable. */
    /* Reason of failed may be not disabled interrupt */
    /* or not called "*_I_LOCK_Unlock()" disabling interrupt by "*_I_LOCK_Disable()". */
TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CallInterruptCallback()_1");
    IF_DQ( self->PointerToState == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    IF_DQ( async->InterruptCallback == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    *self->PointerToState = self->StateValueOfOnInterrupting;


TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CallInterruptCallback()_2");
    e= async->InterruptCallback( InterruptSource, self );  /* Main of callback */
TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CallInterruptCallback()_3");


    if ( e != 0 )
    {
        if ( async->ReturnValue == 0 )
        {
            async->ReturnValue = e;
        }
        R_OSPL_CLEAR_ERROR();
    }

    e=0;
fin:
#if ! R_OSPL_IS_PREEMPTION
    IF_DQ ( gs_master == NULL ) {}
    else
    {
        gs_master->CurrentThread = current_thread;

#if R_OSPL_CPU_LOAD
        {
            errnum_t  ee;

#if  R_OSPL_STACK_CHECK_CODE
            R_OSPL_CHECK_STACK_OVERFLOW();
#endif

            ee= gs_master->IdleCallback( R_OSPL_INTERRUPT_END );
            e= R_OSPL_MergeErrNum( e, ee );
        }
#endif
    }
#endif


#if R_OSPL_ERROR_BREAK
    gs_nested_interrupt_level -= 1;

    if ( gs_nested_interrupt_level == 0 )
    {
#if ! R_OSPL_IS_PREEMPTION
        if ( gs_master != NULL )
        {
            current_thread = gs_master->CurrentThread;
            gs_master->CurrentThread = NULL;
        }
#endif


        R_DEBUG_BREAK_IF_ERROR();


#if ! R_OSPL_IS_PREEMPTION
        if ( gs_master != NULL )
        {
            gs_master->CurrentThread = current_thread;
        }
#endif
    }
#endif

#if  R_OSPL_STACK_CHECK_CODE
    R_OSPL_CHECK_STACK_OVERFLOW();
#endif
TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CallInterruptCallback()_X");
    R_UNREFERENCED_VARIABLE( e );
}


/***********************************************************************
* Implement: R_OSPL_SetErrNum
************************************************************************/
#if  R_OSPL_TLS_ERROR_CODE
void  R_OSPL_SetErrNum( errnum_t const  e )
{
    r_ospl_error_t*  err;

    if ( e != 0 )
    {

        err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                R_OSPL_ALLOCATE_IF_NOT );

        if ( err != NULL )
        {
            if ( err->ErrNum == 0 )
            {
                err->ErrNum = e;
            }

            R_OSPL_UnlockCurrentThreadError( &err );
        }
    }
}
#endif


/***********************************************************************
* Implement: R_OSPL_GetErrNum
************************************************************************/
#if  R_OSPL_TLS_ERROR_CODE
errnum_t  R_OSPL_GetErrNum(void)
{
    errnum_t         e;
    r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_ALLOCATE_IF_NOT );

    if ( err != NULL )
    {
        e = err->ErrNum;
    }
    else
    {
        e = E_NO_DEBUG_TLS;
    }

    R_OSPL_UnlockCurrentThreadError( &err );

    return  e;
}
#endif


/***********************************************************************
* Implement: R_OSPL_CLEAR_ERROR
************************************************************************/
void  R_OSPL_CLEAR_ERROR(void)
{
    r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_DO_NOTHING_IF_NOT );

    if ( err != NULL )
    {
        R_OSPL_CLEAR_ERROR_Sub( err );
    }

    R_OSPL_UnlockCurrentThreadError( &err );
}


/***********************************************************************
* Implement: R_OSPL_CLEAR_ERROR_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
void  R_OSPL_CLEAR_ERROR_Sub( r_ospl_error_t*  err )
{
#if R_OSPL_ERROR_BREAK
    err->IsError = false;
#endif

#if R_OSPL_TLS_ERROR_CODE
    err->ErrNum = 0;
#endif
}
#endif


/***********************************************************************
* Function: R_OSPL_OnRaisingError_Sub
*    Function part of error break
************************************************************************/
#if R_OSPL_ERROR_BREAK
static bool_t  R_OSPL_OnRaisingError_Sub( r_ospl_error_t*  in_Context,
        const char_t* const  in_FilePath,  int_fast32_t const  in_LineNum );  /* QAC-3450 */
static bool_t  R_OSPL_OnRaisingError_Sub( r_ospl_error_t*  in_Context,
        const char_t* const  in_FilePath,  int_fast32_t const  in_LineNum )
{
    bool_t           is_break = false;
    r_ospl_error_t*  err;


    if ( in_Context == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                R_OSPL_ALLOCATE_IF_NOT );
    }
    else
    {
        err = in_Context;
    }

    if ( err != NULL )
    {
        if ( IS( err->IsError ) )
        {
            is_break = false;
        }
        else
        {
            gs_global_error.RaisedGlobalErrorID ++;
            is_break = ( gs_global_error.RaisedGlobalErrorID ==
                         gs_global_error.BreakGlobalErrorID );

            err->IsError = true;
            err->ErrorID = gs_global_error.RaisedGlobalErrorID;
            err->FilePath = in_FilePath;
            err->LineNum = in_LineNum;
        }
    }

    if ( in_Context == NULL )
    {
        R_OSPL_UnlockCurrentThreadError( &err );
    }
    return  is_break;
}
#endif


/***********************************************************************
* Implement: R_OSPL_IF_Macro_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK
bool_t  R_OSPL_IF_Macro_Sub( bool_t const  in_Condition,  const char_t* const  in_File,
                             int_t const  in_Line )
{
#if  R_OSPL_STACK_CHECK_CODE
    R_OSPL_CHECK_STACK_OVERFLOW();
#endif

    if ( IS( in_Condition ) )
    {
        if ( R_OSPL_OnRaisingError_Sub( NULL, in_File, in_Line ) != 0 )
        {
            R_DebugBreak( in_File, in_Line );
        }
    }

    return  in_Condition;
}
#endif


/***********************************************************************
* Implement: R_OSPL_IF_C_Macro_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK
bool_t  R_OSPL_IF_C_Macro_Sub( bool_t const  in_Condition,  r_ospl_error_t*  in_Context,
                               const char_t* const  in_File,  int_t const  in_Line )
{
#if  R_OSPL_STACK_CHECK_CODE
#if  ! R_OSPL_IS_PREEMPTION
    R_OSPL_CHECK_STACK_OVERFLOW_SUB();
#else
    R_OSPL_CHECK_STACK_OVERFLOW_SUB( in_Context );
#endif
#endif

    if ( IS( in_Condition ) )
    {
        if ( R_OSPL_OnRaisingError_Sub( in_Context, in_File, in_Line ) != 0 )
        {
            R_DebugBreak( in_File, in_Line );
        }
    }
    return  (bool_t) in_Condition;
}
#endif


/***********************************************************************
* Function: R_OSPL_DebugBreakIfError
*    Function part of <R_DEBUG_BREAK_IF_ERROR>
************************************************************************/
#if R_OSPL_ERROR_BREAK
void  R_OSPL_DebugBreakIfError( const char_t* const  File,  int_t const  Line )
{
    r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_DO_NOTHING_IF_NOT );

    if ( err != NULL  &&  err->IsError )
    {
        /* See the value of "err->ErrorID" in watch window. */
        /* Call R_OSPL_SET_BREAK_ERROR_ID( N ); at main function. */
#if  R_OSPL_PRINTF == R_OSPL_PRINTF_ENABLED
        printf( "<ERROR error_ID=\"0x%X\" file=\"%s(%d)\"/>\n",
                err->ErrorID, err->FilePath, err->LineNum );
        printf( "Write \"R_OSPL_SET_BREAK_ERROR_ID( %d );\".\n",
                err->ErrorID );
#elif  R_OSPL_PRINTF == R_OSPL_PRINTF_TO_INT_LOG
        R_D_AddToIntLog( 0xBEBEBEBE );
        R_D_AddToIntLog( err->ErrorID );
        R_D_AddToIntLog( (uintptr_t) err->FilePath );
        R_D_AddToIntLog( err->LineNum );
#else
        /* Do Nothing */
#endif
        R_DebugBreak( File, Line );
    }

    R_OSPL_UnlockCurrentThreadError( &err );

    R_OSPL_CLEAR_ERROR();
}
#endif


/***********************************************************************
* Implement: R_OSPL_GET_ERROR_ID
************************************************************************/
#if R_OSPL_ERROR_BREAK
int_fast32_t  R_OSPL_GET_ERROR_ID(void)
{
    return  gs_global_error.RaisedGlobalErrorID;
}
#endif


/***********************************************************************
* Implement: R_OSPL_SET_BREAK_ERROR_ID
************************************************************************/
#if R_OSPL_ERROR_BREAK
void  R_OSPL_SET_BREAK_ERROR_ID( int_fast32_t const  ErrorID )
{
#ifndef R_OSPL_NDEBUG
    if ( gs_global_error.BreakGlobalErrorID != ErrorID )
    {
        printf( ">R_OSPL_SET_BREAK_ERROR_ID( %d );\n", ErrorID );
    }
#endif

    gs_global_error.BreakGlobalErrorID = ErrorID;
}
#endif


/***********************************************************************
* Implement: R_OSPL_GetCurrentThreadError
*
* Description:
*    There is T-Lock area in this function.
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
r_ospl_error_t*  R_OSPL_GetCurrentThreadError()
{
    r_ospl_error_t*  err;
    r_ospl_error_t*  err_copy = NULL;

    err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(), R_OSPL_ALLOCATE_IF_NOT );
    if ( err != NULL )
    {
        err_copy = err;
        R_OSPL_UnlockCurrentThreadError( &err );
    }

    return  err_copy;
}
#endif


/***********************************************************************
* Function: R_OSPL_GetCurrentThreadError_Sub
*    GetCurrentThreadError
*
* Arguments:
*    in_TypeOfIfNot - <r_ospl_if_not_t>
*
* Return Value:
*    Error information of current thread
*
* Description:
*    Don't call this in unlocked state.
*    There is NOT T-Lock area in this function.
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
r_ospl_error_t*  R_OSPL_GetCurrentThreadError_Sub( r_ospl_thread_id_t  in_Thread,
        r_ospl_if_not_t const  in_TypeOfIfNot )
{
    errnum_t            e;
    int_fast32_t        index;
    r_ospl_error_t*     err = NULL;


    if ( in_Thread == R_OSPL_THREAD_NULL )
    {
        err = &gs_error_for_interrupt;
        /* For small stack  */
    }
    else
    {

#if GS_LIST_UP_TLS_ATTACHED_THREAD
        index = R_OSPL_NO_INDEX;
        e= R_OSPL_TABLE_GetIndex( gs_global_error.ThreadIndexTable,
                                  (void*) in_Thread, &index, R_OSPL_DO_NOTHING_IF_NOT );
        IF_C ( e,  err )
        {
            goto fin;
        }
        /* It is not need to T-Lock, because not changed the table. */
        if ( index == R_OSPL_NO_INDEX )
        {
            printf( "TLS (Thread Local Storage) was attached with thread %d.\n", in_Thread );
        }
#endif


        index = R_OSPL_NO_INDEX;
        e= R_OSPL_TABLE_GetIndex( gs_global_error.ThreadIndexTable,
                                  (void*) in_Thread, &index, in_TypeOfIfNot );
        if ( e != 0 )
        {
            goto fin;
        }

        if ( index != R_OSPL_NO_INDEX )
        {
#if defined(R_OSPL_NDEBUG) || defined(__QAC_ARM_H__)  /* IF_DQ */
            if ( gs_global_error.ErrorArray == NULL )
            {
#ifndef R_OSPL_NDEBUG
                e=E_OTHERS;
#endif
                goto fin;
            }
            if ( index < -1 )
            {
#ifndef R_OSPL_NDEBUG
                e=E_OTHERS;
#endif
                goto fin;
            }
#endif

            err = &gs_global_error.ErrorArray[ index ];
        }
    }

#ifndef R_OSPL_NDEBUG
    e=0;
#endif
fin:
#ifndef R_OSPL_NDEBUG
    if ( e != 0 )
    {
        R_DebugBreak( NULL, e );
        /* If e == 2 (E_FEW_ARRAY), Set "R_OSPL_DEBUG_THREAD_COUNT" to bigger value. */
    }
#endif

    return  err;
}
#endif


/***********************************************************************
* Implement: R_OSPL_LockCurrentThreadError
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
r_ospl_error_t*  R_OSPL_LockCurrentThreadError(void)
{
    return  R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(), R_OSPL_ALLOCATE_IF_NOT );
}
#endif


/***********************************************************************
* Implement: R_OSPL_LockCurrentThreadError_Sub
*    GetCurrentThreadError and lock by T-Lock
*
* Arguments:
*    TypeOfIfNot - <r_ospl_if_not_t>
*
* Return Value:
*    Error information of current thread. NULL = Failed.
*
* Description:
*    See <R_OSPL_LockCurrentThreadError>.
************************************************************************/
r_ospl_error_t*  R_OSPL_LockCurrentThreadError_Sub( r_ospl_thread_id_t  in_Thread,
        r_ospl_if_not_t const  in_TypeOfIfNot )
{
    r_ospl_error_t*     err = NULL;
#if R_OSPL_IS_PREEMPTION
    errnum_t            e;
    bool_t              is_lock = false;
    bool_t              is_interrupt_context;
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_LockCurrentThreadError_Sub()_E");

#if R_OSPL_IS_PREEMPTION
#if  IS_ITRON_USED
    is_interrupt_context = sns_ctx();
    /* "in_Thread != R_OSPL_THREAD_NULL". */
    /* But "in_Thread" is return value of "R_OSPL_THREAD_GetCurrentId()". */
    /* After the interrupt was disabled by "get_imask", */
    /* "loc_mtx" in "R_OSPL_Start_T_Lock" returns an error. */
#else
    is_interrupt_context = ( in_Thread == R_OSPL_THREAD_NULL );
#endif

    if ( ! is_interrupt_context )
    {
        e= R_OSPL_Start_T_Lock();
        if(e!=0)
        {
            R_OSPL_RaiseUnrecoverable(e);
            goto fin;
        }
        is_lock = true;
        /* If double lock error was raised in "R_OSPL_CHECK_STACK_OVERFLOW" */
        /* replace "IF" macro in T-Lock area to "IF_C" macro and */
        /* replace "ASSERT_R" macro to "ASSERT_RC" macro. */
    }
#endif


#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
    err = R_OSPL_GetCurrentThreadError_Sub( in_Thread, in_TypeOfIfNot );
#else
    err = &gs_dummy_error;

    R_UNREFERENCED_VARIABLE_2( in_Thread, in_TypeOfIfNot );
#endif


#if R_OSPL_IS_PREEMPTION
    e=0;
fin:
    if ( IS( is_lock )  &&  err == NULL )
    {
        R_OSPL_End_T_Lock();
    }
#endif

	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_LockCurrentThreadError_Sub()_X");
	return err;
}


/***********************************************************************
* Implement: R_OSPL_UnlockCurrentThreadError
************************************************************************/
void  R_OSPL_UnlockCurrentThreadError( r_ospl_error_t** in_out_Error )
{
    r_ospl_error_t*  err = *in_out_Error;
#if R_OSPL_IS_PREEMPTION
    bool_t  is_interrupt_context;
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_UnlockCurrentThreadError_Sub()_E");
    if ( err != NULL )
    {
#if R_OSPL_IS_PREEMPTION
#if  IS_ITRON_USED
        is_interrupt_context = ( sns_ctx() );
        /* sns_ctx() returns interrupt context  or  disabled all interrupts */
#else
        is_interrupt_context = ( R_OSPL_THREAD_GetCurrentId() == NULL );
#endif

        if ( ! is_interrupt_context )
        {
            R_OSPL_End_T_Lock();
        }
#endif

        *in_out_Error = NULL;
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_UnlockCurrentThreadError_Sub()_X");
}


/***********************************************************************
* Implement: R_OSPL_SET_END_OF_STACK
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
void  R_OSPL_SET_END_OF_STACK( void* in_EndOfStackAddress )
{
    uint32_t*  end_of_stack = (uint32_t*)( (uintptr_t) in_EndOfStackAddress & ~( sizeof(uint32_t) - 1 ) );
    /* Alignment 4 by like R_Ceil_4u */

    if ( end_of_stack != NULL )
    {
        *end_of_stack = R_F_OSPL_STACK_CHECK_SENTINEL_VALUE;
    }

#if  ! R_OSPL_IS_PREEMPTION
    gs_global_error.EndOfStack = end_of_stack;
#else
    {
        r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                               R_OSPL_ALLOCATE_IF_NOT );

        if ( err != NULL )
        {
            err->EndOfStack = end_of_stack;
        }

        R_OSPL_UnlockCurrentThreadError( &err );
    }
#endif
}
#endif


/***********************************************************************
* Implement: R_OSPL_CHECK_STACK_OVERFLOW
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
void  R_OSPL_CHECK_STACK_OVERFLOW(void)
{
#if  ! R_OSPL_IS_PREEMPTION
    R_OSPL_CHECK_STACK_OVERFLOW_SUB();
#else
    r_ospl_error_t*  err = R_OSPL_GetCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_ALLOCATE_IF_NOT );

    R_OSPL_CHECK_STACK_OVERFLOW_SUB( err );
#endif
}
#endif


/***********************************************************************
* Implement: R_OSPL_GET_STACK_POINTER
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
uint8_t*  R_OSPL_GET_STACK_POINTER(void)
{

	uint32_t  array[1];

#ifdef  __CC_ARM
#pragma push
#pragma diag_suppress 1166  /* Warning of returning pointer to local variable */
#endif
#ifdef __ICCARM__
#pragma diag_suppress=Pe1056  /* Warning of returning pointer to local variable */
#endif
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wreturn-local-addr"
#endif

    return  (uint8_t*) &array[0];

#ifdef __GNUC__
#pragma GCC diagnostic warning "-Wreturn-local-addr"
#endif
#ifdef __ICCARM__
#pragma diag_default=Pe1056
#endif
#ifdef  __CC_ARM
#pragma pop
#endif

}
#endif  /*  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE */


/***********************************************************************
* Implement: R_OSPL_CHECK_STACK_OVERFLOW_SUB
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
#if  ! R_OSPL_IS_PREEMPTION
void  R_OSPL_CHECK_STACK_OVERFLOW_SUB(void)
#else
void  R_OSPL_CHECK_STACK_OVERFLOW_SUB( r_ospl_error_t*  err )
#endif
{
#if  R_OSPL_IS_PREEMPTION
    if ( err != NULL )
#endif
    {
        uint32_t*  end_of_stack;


#if !  R_OSPL_IS_PREEMPTION
        end_of_stack = gs_global_error.EndOfStack;
#else
        end_of_stack = err->EndOfStack;
#endif

        if ( end_of_stack != NULL )
        {
            if ( *end_of_stack != R_F_OSPL_STACK_CHECK_SENTINEL_VALUE )
            {
                R_OSPL_RaiseUnrecoverable( E_STACK_OVERFLOW );
            }
        }
    }
}
#endif


/***********************************************************************
* Implement: R_OSPL_RESET_MIN_FREE_STACK_SIZE
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
errnum_t  R_OSPL_RESET_MIN_FREE_STACK_SIZE(void)
{
    errnum_t  e;
#if  R_OSPL_IS_PREEMPTION
    r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_ALLOCATE_IF_NOT );
#else
    r_ospl_error_t*  err = NULL;
#endif


#if  R_OSPL_IS_PREEMPTION
    if ( err != NULL )
#endif
    {
        uint32_t*  pointer;
        uint8_t*   stack_pointer;
        uint32_t*  end_of_stack;


#if !  R_OSPL_IS_PREEMPTION
        end_of_stack = gs_global_error.EndOfStack;
#else
        end_of_stack = err->EndOfStack;
#endif


        ASSERT_RC( end_of_stack != NULL,  err,  e=E_OTHERS; goto fin );
        /* Error: Not called "R_OSPL_SET_END_OF_STACK" */

        stack_pointer = R_OSPL_GET_STACK_POINTER();
        stack_pointer = (uint8_t*)( ( (uintptr_t) stack_pointer - ( 2 * sizeof(*pointer) - 1u ) ) &
                                    ~( sizeof(*pointer) - 1 ) );  /* Alignment 4 by like R_Floor_4u( sp - 4 ) */

        for ( pointer = end_of_stack;  pointer <= (uint32_t*) stack_pointer;  pointer += 1 )
        {
            *pointer = R_F_OSPL_STACK_CHECK_SENTINEL_VALUE;
        }
    }

    e=0;
fin:
#if  R_OSPL_IS_PREEMPTION
    R_OSPL_UnlockCurrentThreadError( &err );
#endif
    return  e;
}
#endif


/***********************************************************************
* Implement: R_OSPL_GET_MIN_FREE_STACK_SIZE
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
size_t  R_OSPL_GET_MIN_FREE_STACK_SIZE(void)
{
    size_t  stack_size = 0;
#if  R_OSPL_IS_PREEMPTION
    r_ospl_error_t*  err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(),
                           R_OSPL_ALLOCATE_IF_NOT );
#endif


#if  R_OSPL_IS_PREEMPTION
    if ( err != NULL )
#endif
    {
        uint32_t*  end_of_stack;


#if !  R_OSPL_IS_PREEMPTION
        end_of_stack = gs_global_error.EndOfStack;
#else
        end_of_stack = err->EndOfStack;
#endif


        if ( end_of_stack != NULL )
        {
            uint32_t*  pointer = end_of_stack;

            while ( *pointer == R_F_OSPL_STACK_CHECK_SENTINEL_VALUE )
            {
                pointer += 1;
            }

            stack_size = (uint8_t*) pointer - (uint8_t*) end_of_stack;
        }
    }

#if  R_OSPL_IS_PREEMPTION
    R_OSPL_UnlockCurrentThreadError( &err );
#endif
    return  stack_size;
}
#endif


/***********************************************************************
* Function: R_OSPL_EVENT_GetEventObject_Sub
************************************************************************/
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
r_ospl_event_object_id_t  R_OSPL_EVENT_GetEventObject_Sub(
    r_ospl_thread_id_t const  in_ThreadId,
    r_ospl_event_flags_t const  in_SetFlags )
{
    r_ospl_event_object_id_t  event_object = R_OSPL_EVENT_OBJECT_NULL;
    r_ospl_error_t*           err = NULL;

    if ( in_ThreadId == R_OSPL_BY_ELEMENT_OBJECT )
    {
        event_object = (r_ospl_event_object_id_t) in_SetFlags;
    }
    else if ( IS_BIT_SET( in_SetFlags,  R_OSPL_EVENT_OBJECT_FLAG ) )
    {
        err = R_OSPL_GetCurrentThreadError_Sub( in_ThreadId,  R_OSPL_ALLOCATE_IF_NOT );
        ASSERT_R( err != NULL,  R_NOOP() );
        if ( err != NULL )
        {
            event_object = err->EnabledEventObject;
        }
    }

    return  event_object;
}
#endif


/***********************************************************************
* Implement: R_OSPL_EVENT_SetForDebug
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
void  R_OSPL_EVENT_SetForDebug( r_ospl_thread_id_t const  in_ThreadId,  bit_flags32_t const  in_SetFlags )
{
    uint32_t  bit_num;

    r_ospl_error_t*  err = NULL;


    /* Check "in_ThreadId" */
    ASSERT_R( in_ThreadId != R_OSPL_ByEventObject(),
              R_OSPL_RaiseUnrecoverable( E_OTHERS );  goto fin );
    /* Error: "R_OSPL_EVENT_Allocate" is not called yet. See <R_OSPL_IS_ASYNC_LIST>. */


    if ( IS_ANY_BITS_SET( in_SetFlags,  R_OSPL_EVENT_CHECK_BIT_MASK | R_OSPL_EVENT_CHECK_ORDER_MASK ) )
    {

        /* Check "check_bits_in_argument" */
        if ( IS_BIT_NOT_SET( in_SetFlags,  R_OSPL_EVENT_OBJECT_FLAG ) )
        {
            ASSERT_R( R_OSPL_IsSetBitsCount1( in_SetFlags & R_OSPL_EVENT_ALL_BITS ),
                      R_OSPL_BAD_EVENT_ERROR() );

            bit_num = R_OSPL_GET_FROM_32_BIT_REGISTER(
                          &in_SetFlags,  R_OSPL_EVENT_CHECK_BIT_MASK,  R_OSPL_EVENT_CHECK_BIT_SHIFT );
        }
        else    /* R_OSPL_EVENT_OBJECT_FLAG */
        {
            bit_num = R_OSPL_EVENT_OBJECT_FIRST_ID;
        }

        err = R_OSPL_LockCurrentThreadError_Sub( in_ThreadId, R_OSPL_ALLOCATE_IF_NOT );
        if ( err != NULL )
        {
            uint32_t  check_bits_in_argument =
                in_SetFlags & ( R_OSPL_EVENT_CHECK_BIT_MASK | R_OSPL_EVENT_CHECK_ORDER_MASK );

            uint32_t  check_bits_allocated =
                (bit_flags32_t) err->EventCheckBits[ bit_num ] << R_OSPL_EVENT_CHECK_BIT_SHIFT;

            ASSERT_R( check_bits_in_argument == check_bits_allocated  &&
                      check_bits_allocated != 0,
                      R_OSPL_RaiseUnrecoverable( E_OTHERS );  goto fin );
            /* Error: "R_OSPL_EVENT_Allocate" is not called yet or event flags are conflicted. */
        }
    }
    else
    {
        err = R_OSPL_LockCurrentThreadError_Sub( in_ThreadId, R_OSPL_ALLOCATE_IF_NOT );

        ASSERT_RC( IS_ALL_BITS_SET( err->EventStatus.UnexpectedEvents, in_SetFlags ),
                   err,  R_OSPL_BAD_EVENT_ERROR() );
        /* Error: Detected that "in_SetFlags" is not "Allocate" yet.See <R_OSPL_IS_ASYNC_LIST>. */

        err->EventStatus.UnexpectedEvents |= in_SetFlags;
    }

fin:
    R_OSPL_UnlockCurrentThreadError( &err );
}
#endif


/***********************************************************************
* Implement: R_OSPL_EVENT_Allocate
************************************************************************/
errnum_t  R_OSPL_EVENT_Allocate( volatile r_ospl_thread_id_t*  out_ThreadId,  r_ospl_thread_id_t  in_ThreadId,
                                 volatile bit_flags32_t*  out_SetFlag,  r_ospl_event_flags_t  in_SetFlag )
{
    errnum_t  e;
    bit_flags32_t       got_flags;
    r_ospl_error_t*     err = NULL;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Allocate()_E");

#if 1
	UNUSED_PARAM(got_flags);
#endif


// イベントフラグはいつでも使えるので、実質何もしない関数にしとくのが良い tokuyama@kmg

#if  R_OSPL_DETECT_BAD_EVENT  ||  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_EVENT_OBJECT_CODE  ||  R_OSPL_EVENT_WATCH
    r_ospl_thread_id_t  current_thread = R_OSPL_THREAD_GetCurrentId();
#endif
#if  R_OSPL_EVENT_OBJECT_CODE  ||  R_OSPL_EVENT_WATCH
    r_ospl_thread_id_t  by_object = R_OSPL_ByEventObject();
#endif
#if  R_OSPL_EVENT_OBJECT_CODE
    bool_t              is_set_enabled_event = false;
#endif
#if  R_OSPL_TLS_EVENT_CODE
    bool_t         is_set_flags = false;
    bit_flags32_t  old_flags = 0;  /* 0 : For compiler warning of "Used before set symbol" is safe by "is_set_flags". */
#endif
#if  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_DETECT_BAD_EVENT
    uint_fast32_t    bit_num = 0;  /* 0 : For compiler warning of "Used before set symbol" is safe by "check_bits". */
#endif
#if  R_OSPL_DETECT_BAD_EVENT
    bool_t                is_check_bits = false;
    uint32_t              check_bits = 0;
    r_ospl_event_flags_t  allocated_event_object = R_OSPL_EVENT_OBJECT_Cast( R_OSPL_EVENT_OBJECT_NULL );
#endif


#if  R_OSPL_EVENT_WATCH
    if ( in_ThreadId == by_object )
    {
        R_D_AddToIntLog( 0x7010E0B1 );  /* Event Object */
    }
    else
    {
        R_D_AddToIntLog( 0x70100000 + (int_fast32_t)( in_SetFlag & R_OSPL_EVENT_ALL_BITS ) );
    }
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) current_thread );
#endif


#if  R_OSPL_EVENT_OBJECT_CODE
    if ( in_ThreadId == by_object )
    {
#if  R_OSPL_DETECT_BAD_EVENT
        {
            int_fast32_t        event_object_index;
            const int_fast32_t  found = R_OSPL_NO_INDEX;

            event_object_index = found;
            e= R_OSPL_TABLE_GetIndex( R_OSPL_TABLE( gs_allocated_event_objects ),  (void*) in_SetFlag,
            &event_object_index,  R_OSPL_OUTPUT_IF_NOT );
            IF_C(e,err)
            {
                goto fin;
            }
            ASSERT_RC( event_object_index != found,  err,  e=E_ACCESS_DENIED; goto fin );
            /* Error: Event object is conflicted. */

            allocated_event_object = in_SetFlag;
        }
#endif

#if  R_OSPL_EVENT_OBJECT_CODE
        if ( err == NULL )
        {
            err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
        }
        if ( err != NULL )
        {
            ASSERT_RC( err->EnabledEventObject == R_OSPL_EVENT_OBJECT_NULL,
                       err,  e=E_ACCESS_DENIED; goto fin );

            is_set_enabled_event = true;

            err->EnabledEventObject = (r_ospl_event_object_id_t) in_SetFlag;
        }
#endif

        in_ThreadId = current_thread;
        in_SetFlag = R_OSPL_EVENT_OBJECT_FLAG;
    }
    else
    {
        ASSERT_DC( in_ThreadId == current_thread,  err,  e=E_OTHERS; goto fin );
    }
#endif


#if  R_OSPL_TLS_EVENT_CODE

    if ( err == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
    }
    if ( err != NULL )
    {

        if ( in_SetFlag == R_OSPL_UNUSED_FLAG )
        {
            bit_flags32_t  bits;

            ASSERT_RC( IS_ANY_BITS_NOT_SET( err->EventStatus.AllocatedEvents,
                                            R_OSPL_EVENT_DYNAMIC_BITS ),
                       err,  e = E_FEW_ARRAY;  goto fin );
            /* Error: All event flags are already allocated. */

            /* Search 0's bit. */
            bits = err->EventStatus.AllocatedEvents;  /* e.g. 0x00000000, 0x0000F0F0 */
            bits |= ~R_OSPL_EVENT_DYNAMIC_BITS;       /* e.g. 0xFFFF000F, 0xFFFFF0FF */
            bits = ~bits;                             /* e.g. 0x0000FFF0, 0x00000F00 */
            bit_num = R_OSPL_CountLeadingZeros( bits );  /* e.g.   16,   20 */
            bit_num = 31 - bit_num;                      /* e.g.   15,   11 */

            in_SetFlag = 1 << bit_num;
        }
        else
        {
            ASSERT_RC( R_OSPL_IsSetBitsCount1( in_SetFlag ),  err,  e=E_OTHERS; goto fin );

            ASSERT_RC( IS_BIT_NOT_SET( err->EventStatus.AllocatedEvents, in_SetFlag ),
                       err,  e = E_ACCESS_DENIED;  goto fin );
            /* Error: Event flags are conflicted. */
        }

        is_set_flags = true;
        old_flags = err->EventStatus.AllocatedEvents;

        err->EventStatus.AllocatedEvents |= in_SetFlag;
    }
#endif


#if  R_OSPL_DETECT_BAD_EVENT
    check_bits = 0;

    ASSERT_RC( R_OSPL_IsSetBitsCount1( in_SetFlag ),  err,  e=E_OTHERS; goto fin );


    /* Set at "R_OSPL_EVENT_CHECK_BIT_MASK" in "check_bits" */
    if ( IS_BIT_NOT_SET( in_SetFlag,  R_OSPL_EVENT_OBJECT_FLAG ) )
    {
        ASSERT_RC( IS_ALL_BITS_NOT_SET( in_SetFlag, ~ R_OSPL_EVENT_ALL_BITS ),
                   err,  e=E_OTHERS; goto fin );

        bit_num = 31 - R_OSPL_CountLeadingZeros( in_SetFlag );

        R_OSPL_SET_TO_32_BIT_REGISTER(
            &check_bits,  R_OSPL_EVENT_CHECK_BIT_MASK,  R_OSPL_EVENT_CHECK_BIT_SHIFT,
            bit_num );
    }
    else    /* If "in_SetFlag" has "R_OSPL_EVENT_OBJECT_FLAG" */
    {

        bit_num = R_OSPL_EVENT_OBJECT_FIRST_ID;

        R_OSPL_SET_TO_32_BIT_REGISTER(
            &check_bits,  R_OSPL_EVENT_CHECK_BIT_MASK,  R_OSPL_EVENT_CHECK_BIT_SHIFT,
            0 );
    }


    /* Set at "R_OSPL_EVENT_CHECK_ORDER_MASK" in "check_bits" */
    {
        bool_t  was_all_enabled = R_OSPL_DisableAllInterrupt();

        gs_global_error.PreviousEventNum += 1;

        if ( was_all_enabled )
        {
            R_OSPL_EnableAllInterrupt();
        }

        R_OSPL_SET_TO_32_BIT_REGISTER(
            &check_bits,  R_OSPL_EVENT_CHECK_ORDER_MASK,  R_OSPL_EVENT_CHECK_ORDER_SHIFT,
            gs_global_error.PreviousEventNum );
    }


    /* Add "check_bits" in "in_SetFlag" */
    if ( err == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
    }
    if ( err != NULL )
    {

        ASSERT_RC( err->EventCheckBits[ bit_num ] == 0,
                   err,  R_OSPL_RaiseUnrecoverable( E_OTHERS ) );
        /* Event flags are conflicted. */

        is_check_bits = true;

        err->EventCheckBits[ bit_num ] = (uint16_t)( check_bits >> R_OSPL_EVENT_CHECK_BIT_SHIFT );
        in_SetFlag |= check_bits;

        err->OnceAllocatedEventFlags |= ( 1 << bit_num );
    }
#endif


    R_OSPL_UnlockCurrentThreadError( &err );  /* For next "R_OSPL_EVENT_Wait" */
    err = NULL;

#if 0
    e= R_OSPL_EVENT_Wait( in_SetFlag, &got_flags, 0 );
    IF_C(e,err)
    {
        goto fin;   /* Clear an flag */
    }
#endif /* 0 */

#if  R_OSPL_DETECT_BAD_EVENT
    if ( IS_BIT_NOT_SET( got_flags,  R_OSPL_TIMEOUT ) )
    {
        if ( err == NULL )
        {
            err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
        }
        if ( err != NULL )
        {
            err->EventStatus.UnexpectedEvents |= got_flags;
        }
        ASSERT_RC( got_flags == 0u,  err,  e=E_ACCESS_DENIED; goto fin );
        /* Error: Event flags are already set or allocated. */
    }
#endif


    *out_ThreadId = in_ThreadId;
    *out_SetFlag = in_SetFlag;


    e=0;
#if 0
	fin:
#endif
#if  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_DETECT_BAD_EVENT  ||  R_OSPL_EVENT_OBJECT_CODE
    if ( e != 0 )    /* Rollback */
    {

#if  R_OSPL_DETECT_BAD_EVENT
        if ( IS( is_check_bits ) )
        {
            err->EventCheckBits[ bit_num ] = 0;
        }

        /* Call "R_OSPL_TABLE_Free" */
        if ( allocated_event_object != R_OSPL_EVENT_OBJECT_Cast( R_OSPL_EVENT_OBJECT_NULL ) )
        {
            R_OSPL_UnlockCurrentThreadError( &err );
            R_OSPL_TABLE_Free( R_OSPL_TABLE( gs_allocated_event_objects ),  (void*) allocated_event_object );
            err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
        }
#endif

#if  R_OSPL_EVENT_OBJECT_CODE
        if ( IS( is_set_enabled_event ) )
        {
            err->EnabledEventObject = R_OSPL_EVENT_OBJECT_NULL;
        }
#endif

#if  R_OSPL_TLS_EVENT_CODE
        if ( IS( is_set_flags ) )
        {
            err->EventStatus.AllocatedEvents = old_flags;
        }
#endif
    }
#endif
    R_OSPL_UnlockCurrentThreadError( &err );
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Allocate()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_EVENT_Free
************************************************************************/
errnum_t  R_OSPL_EVENT_Free( volatile r_ospl_thread_id_t*  in_out_ThreadId,
                             volatile r_ospl_event_flags_t*  in_out_SetFlag )
{
    errnum_t  e = 0;
    r_ospl_thread_id_t    current_thread = R_OSPL_THREAD_GetCurrentId();
    r_ospl_thread_id_t    in_ThreadId = *in_out_ThreadId;
    r_ospl_event_flags_t  in_SetFlag = *in_out_SetFlag;
TOPPERS_SYSLOG(LOG_INFO,"R_OSPL_EVENT_Free()_E");
#if  R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_EVENT_CODE
    r_ospl_error_t*       err = NULL;
#endif
#if  R_OSPL_DETECT_BAD_EVENT
    uint32_t   bit_num;
#endif
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    r_ospl_thread_id_t        by_object = R_OSPL_ByEventObject();
    r_ospl_event_object_id_t  event_object;
#endif

    if ( in_SetFlag == 0 )
    {
        e=0;
        goto fin;
    }


#if  R_OSPL_EVENT_WATCH
    R_D_AddToIntLog( 0x70F00000 + (int_fast32_t)( in_SetFlag & R_OSPL_EVENT_ALL_BITS ) );
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) current_thread );
#endif


    R_OSPL_EVENT_Clear( in_ThreadId, in_SetFlag );


#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    event_object = R_OSPL_EVENT_GetEventObject_Sub( in_ThreadId, in_SetFlag );
    if ( event_object != R_OSPL_EVENT_OBJECT_NULL  &&  in_ThreadId == by_object )
    {
        in_ThreadId = current_thread;
        in_SetFlag = R_OSPL_EVENT_OBJECT_FLAG;
    }
#endif


#if  R_OSPL_DETECT_BAD_EVENT
    if ( IS_BIT_NOT_SET( in_SetFlag,  R_OSPL_EVENT_OBJECT_FLAG ) )
    {
        bit_flags32_t  flag = in_SetFlag & R_OSPL_EVENT_ALL_BITS;

        ASSERT_RC( R_OSPL_IsSetBitsCount1( flag ),  err,  R_NOOP() );

        bit_num = 31 - R_OSPL_CountLeadingZeros( flag );
    }
    else    /* If "in_SetFlag" has "R_OSPL_EVENT_OBJECT_FLAG" */
    {

        bit_num = R_OSPL_EVENT_OBJECT_FIRST_ID;
    }

    if ( err == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
    }
    if ( err != NULL )
    {


        err->EventCheckBits[ bit_num ] = 0;


    }
#endif  /* R_OSPL_DETECT_BAD_EVENT */


#if  R_OSPL_TLS_EVENT_CODE
    if ( err == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
    }
    if ( err != NULL )
    {
        err->EventStatus.AllocatedEvents &= ~in_SetFlag;
    }
#endif


#if  R_OSPL_EVENT_OBJECT_CODE
#if  R_OSPL_DETECT_BAD_EVENT
    if ( event_object != R_OSPL_EVENT_OBJECT_NULL )
    {
        R_OSPL_UnlockCurrentThreadError( &err );

        R_OSPL_TABLE_Free( R_OSPL_TABLE( gs_allocated_event_objects ),  (void*) event_object );
    }
#endif

    if ( err == NULL )
    {
        err = R_OSPL_LockCurrentThreadError_Sub( current_thread,  R_OSPL_ALLOCATE_IF_NOT );
    }
    if ( IS_BIT_SET( in_SetFlag,  R_OSPL_EVENT_OBJECT_FLAG ) )
    {
        if ( err != NULL )
        {
            if ( err->EnabledEventObject != R_OSPL_EVENT_OBJECT_NULL )
            {
                in_ThreadId = by_object;
                in_SetFlag = (r_ospl_event_flags_t) event_object;
                *in_out_ThreadId = in_ThreadId;
                *in_out_SetFlag = in_SetFlag;
                err->EnabledEventObject = R_OSPL_EVENT_OBJECT_NULL;
            }
        }
    }
    else
    {
        if ( in_SetFlag != 0 )
        {
            ASSERT_RC( in_ThreadId == current_thread,  err,  e= R_OSPL_MergeErrNum( e, E_OTHERS ) );
        }
    }
#else
    if ( in_SetFlag != 0 )
    {
        ASSERT_RC( in_ThreadId == current_thread,  err,  e= R_OSPL_MergeErrNum( e, E_OTHERS ) );
    }
#endif

fin:
#if  R_OSPL_DETECT_BAD_EVENT  ||  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_EVENT_OBJECT_CODE
    R_OSPL_UnlockCurrentThreadError( &err );
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Free()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_EVENT_GetStatus
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
errnum_t  R_OSPL_EVENT_GetStatus( r_ospl_thread_id_t  in_Thread,  r_ospl_event_status_t** out_Status )
{
    errnum_t  e;
    r_ospl_error_t*  err;

    err = R_OSPL_GetCurrentThreadError_Sub( in_Thread,  R_OSPL_ALLOCATE_IF_NOT );
    IF ( err == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    *out_Status = &err->EventStatus;

    e=0;
fin:
    return  e;
}
#endif


/***********************************************************************
* Implement: R_OSPL_MODIFY_THREAD_LOCKED_COUNT
************************************************************************/
#if R_OSPL_ERROR_BREAK  &&  R_OSPL_IS_PREEMPTION
void  R_OSPL_MODIFY_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread,  int_fast32_t  in_Plus )
{
    r_ospl_error_t*  err;

    err = R_OSPL_LockCurrentThreadError_Sub( in_Thread, R_OSPL_ALLOCATE_IF_NOT );

    if ( err != NULL )
    {
        err->ThreadLockedCount += in_Plus;
    }

    R_OSPL_UnlockCurrentThreadError( &err );
}
#endif


/***********************************************************************
* Implement: R_OSPL_GET_THREAD_LOCKED_COUNT
************************************************************************/
#if R_OSPL_ERROR_BREAK  &&  R_OSPL_IS_PREEMPTION
int_fast32_t  R_OSPL_GET_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread )
{
    int_fast32_t  count;
    r_ospl_error_t*  err;

    err = R_OSPL_GetCurrentThreadError_Sub( in_Thread, R_OSPL_ALLOCATE_IF_NOT );

    if ( err != NULL )
    {
        count = err->ThreadLockedCount;
    }
    else
    {
        count = 0;
    }

    return  count;
}
#endif


