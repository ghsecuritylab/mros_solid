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
* File: r_ospl.h
*    OS Porting Layer API Functions. Main Header. For RZ/A1.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef R_OSPL_H
#define R_OSPL_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "Project_Config.h"
#include "platform.h"
#include "r_ospl_typedef.h"
#include "r_multi_compiler_typedef.h"
#include "locking.h"
#include "r_static_an_tag.h"
#include "r_ospl_debug.h"
#if ! R_OSPL_IS_PREEMPTION
#include "r_ospl_os_less.h"
#endif

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


/* Section: Version and initialize */
/***********************************************************************
* Function: R_OSPL_GetVersion
*    Returns version number of OSPL
*
* Arguments:
*    None
*
* Return Value:
*    Version number of OSPL
*
* Description:
*    Return value is same as "R_OSPL_VERSION" macro.
************************************************************************/
int32_t   R_OSPL_GetVersion(void);


/***********************************************************************
* Function: R_OSPL_IsPreemption
*    Returns whether the environment is supported preemption
*
* Arguments:
*    None
*
* Return Value:
*    Whether the environment is RTOS supported preemption
*
* Description:
*    Return value is same as "R_OSPL_IS_PREEMPTION" macro.
************************************************************************/
bool_t    R_OSPL_IsPreemption(void);


/* Section: Standard functions */
/***********************************************************************
* Function: R_NOOP
*    No operation from C++ specification
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    Compatible with __noop (MS C++). But our naming rule is not match.
************************************************************************/
#ifdef _SH
#pragma inline R_NOOP
#endif
INLINE void  R_NOOP(void) {}


/***********************************************************************
* Function: R_COUNT_OF
*    Returns element count of the array
*
* Arguments:
*    Array - An array
*
* Return Value:
*    Count of specified array's element
*
* Description:
*    Compatible with _countof (MS C++) and ARRAY_SIZE (Linux).
*    But our naming rule is not match.
*
* Example:
*    > uint32_t  array[10];
*    > R_COUNT_OF( array )  // = 10
*
* Example:
*    Array argument must not be specified the pointer using like array.
*    > uint32_t   array[10];
*    > func( array );
*    >
*    > void  func( uint32_t array[] )  // "array" is a pointer
*    > {
*    >     R_COUNT_OF( array )  // NG
*    > }
************************************************************************/
/* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#define  R_COUNT_OF( Array )  ( sizeof( Array ) / sizeof( *(Array) ) )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Macro: R_OSPL_ReturnFalse
*    No warning false constant.
*
* Description:
*    For SH, avoids warning of "C0004 (I) Constant as condition" in "IF_D".
*    For gcc, avoids warning of 'A' is static but used in inline function 'B'
*    which is not static [enabled by default].
************************************************************************/
#ifdef  _SH
#pragma inline  R_OSPL_ReturnFalse
STATIC_INLINE bool_t  R_OSPL_ReturnFalse(void)
{
    return false;
}
#else
#define  R_OSPL_ReturnFalse()  0
#endif


/* Section: Error handling and debugging (1) */


/***********************************************************************
* Macro: IF
*    Breaks and transits to error state, if condition expression is not 0
*
* Arguments:
*    Condition - Condition expression
*
* Return Value:
*    None
*
* Example:
*    > e= TestFunction(); IF(e){goto fin;}
*
* Description:
*    "IF" is as same as general "if", if "R_OSPL_ERROR_BREAK" macro was
*    defined to be 0. The following descriptions are available,
*    if "R_OSPL_ERROR_BREAK" macro was defined to be 1.
*
*    "IF" macro supports to find the code raising an error.
*
*    If the "if statement" that is frequently seen in guard condition and
*    after calling functions was changed to "IF" macro, the CPU breaks
*    at raising an error. Then the status (values of variables) can be
*    looked immediately and the code (call stack) can be looked. Thus,
*    debug work grows in efficiency.
*
*    "IF" macro promotes recognizing normal code and exceptional code.
*    Reading speed will grow up by skipping exceptional code.
*
*    Call "R_OSPL_SET_BREAK_ERROR_ID" function, if set to break at the code
*    raising an error.
*
*    Whether the state was error state or the error raised count is stored
*    in the thread local storage. In Release configuration, the variable
*    of error state and the error raised count is deleted. Manage the error
*    code using auto variable and so on at out of OSPL.
*
*    The error state is resolved by calling "R_OSPL_CLEAR_ERROR" function.
*    If "R_DEBUG_BREAK_IF_ERROR" macro was called with any error state,
*    the process breaks at the macro.
************************************************************************/
#if R_OSPL_ERROR_BREAK

/* ->MISRA 19.4 : Abnormal termination. Compliant with C language syntax. */ /* ->SEC M1.8.2 */
#define  IF( Condition ) \
		if ( IS( R_OSPL_IF_Macro_Sub( \
			IS( (int_fast32_t)( Condition ) ), __FILE__, __LINE__ ) ) )
/* (int_fast32_t) cast is for QAC warning of implicit cast unsigned to signed */
/* != 0 is for QAC warning of MISRA 13.2 Advice */
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */

#else  /* ! R_OSPL_ERROR_BREAK */

/* ->MISRA 19.4 : Abnormal termination. Compliant with C language syntax. */ /* ->SEC M1.8.2 */
#define  IF  if
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */
#endif


/***********************************************************************
* Macro: IF_D
*    It is same as "IF" (for Debug configuration only)
*
* Arguments:
*    Condition - Condition expression
*
* Return Value:
*    None
*
* Description:
*    In Release configuration, the result of condition expression is always "false".
*    The release configuration is the configuration defined "R_OSPL_NDEBUG".
************************************************************************/
/* ->MISRA 19.4 : Compliant with C language syntax. */ /* ->SEC M1.8.2 */
/* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
#ifndef R_OSPL_NDEBUG
#define  IF_D( Condition )  IF ( Condition )
#else
#define  IF_D( Condition )  if ( R_OSPL_ReturnFalse() )
#endif
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */


/***********************************************************************
* Macro: ASSERT_R
*    Assertion (Programming By Contract)
*
* Arguments:
*    Condition          - The condition expression expected true
*    goto_fin_Statement - The operation doing at condition is false
*
* Return Value:
*    None
*
* Description:
*    It is possible to write complex sentence divided by ";" in
*    "goto_fin_Statement" argument.
*
*    - Case of defined "R_OSPL_ERROR_BREAK" to be 0:
*    If the result of condition expression is 0(false), do "StatementsForError".
*    If operations did nothing, write "R_NOOP()" at "StatementsForError" argument.
*
*    - Case of defined "R_OSPL_ERROR_BREAK" to be 1:
*    If the result of condition expression is 0(false), the error state
*    will become active and the operation of "StatementForError" argument
*    will be done.
************************************************************************/
#ifndef  __cplusplus
#define  ASSERT_R( Condition, goto_fin_Statement ) \
		do{ IF(!(Condition)) { goto_fin_Statement; } } while(0)  /* do-while is CERT standard PRE10-C */
#else
#define  ASSERT_R( Condition, goto_fin_Statement ) \
		{ IF(!(Condition)) { goto_fin_Statement; } }  /* no C5236(I) */
#endif


/***********************************************************************
* Macro: ASSERT_D
*    Assertion (Programming By Contract) (for Debug configuration only)
*
* Arguments:
*    Condition          - The condition expression expected true
*    goto_fin_Statement - The operation doing at condition is false
*
* Return Value:
*    None
*
* Description:
*    This does nothing in Release configuration.
*    Release configuration is the configuration defined "R_OSPL_NDEBUG"
*    as same as standard library.
************************************************************************/
#ifndef R_OSPL_NDEBUG
#define  ASSERT_D  ASSERT_R
#else
/* ->MISRA 19.7 : Function's argument can not get "goto_fin_Statement" */ /* ->SEC M5.1.3 */
#define  ASSERT_D( Condition, goto_fin_Statement )  R_NOOP()
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */
#endif


/***********************************************************************
* Function: R_OSPL_IF_Macro_Sub
*    Sub routine of IF macro
*
* Arguments:
*    Condition - Condition in IF macro
*    File      - File name
*    Line      - Line number
*
* Return Value:
*    "Condition" argument
*
* Description:
*    This part is for compliant to MISRA 2004 - 19.7.
************************************************************************/
bool_t  R_OSPL_IF_Macro_Sub( bool_t const  Condition,  const char_t* const  File,
                             int_t const  Line );


/***********************************************************************
* Function: R_OSPL_IF_C_Macro_Sub
************************************************************************/
#if R_OSPL_ERROR_BREAK
bool_t  R_OSPL_IF_C_Macro_Sub( bool_t const  Condition,  r_ospl_error_t*  Context,
                               const char_t* const  File,  int_t const  Line );
#endif


/***********************************************************************
* Class: r_ospl_thread_id_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_THREAD_GetCurrentId
*    Get running thread ID (for OS less and OS-using environment)
*
* Arguments:
*    None
*
* Return Value:
*    The current running thread ID
*
* Description:
*    It is possible to use this function for both OS less and OS-using environment.
*    For OS less, returns thread ID passed to "R_OSPL_THREAD_SetCurrentId" function.
*    "NULL" is returned, if in interrupt context.
************************************************************************/
r_ospl_thread_id_t  R_OSPL_THREAD_GetCurrentId(void);


/***********************************************************************
* Function: R_OSPL_ByEventObject
************************************************************************/
r_ospl_thread_id_t  R_OSPL_ByEventObject(void);


/***********************************************************************
* Function: R_OSPL_EVENT_OBJECT_Cast
************************************************************************/
INLINE r_ospl_event_flags_t  R_OSPL_EVENT_OBJECT_Cast( r_ospl_event_object_id_t const  in_EventObjectId );


/***********************************************************************
* Function: R_OSPL_EVENT_Set
*    Set one or some bits to 1
*
* Arguments:
*    in_ThreadId - The thread ID attached the target event
*    in_SetFlags - The value of bit flags that target bit is 1
*
* Return Value:
*    None
*
* Description:
*    For OS less, there is the area disabled all interrupts.
*
*    For OS-using environment, the thread waiting in "R_OSPL_EVENT_Wait"
*    function might wake up soon.
*
*    Do nothing, when "ThreadId" = "NULL"
************************************************************************/
void  R_OSPL_EVENT_Set( r_ospl_thread_id_t const  in_ThreadId,  r_ospl_event_flags_t const  in_SetFlags );


/***********************************************************************
* Function: R_OSPL_EVENT_Clear
*    Set one or some bits to 0
*
* Arguments:
*    in_ThreadId    - The thread ID attached the target event
*    in_ClearFlags1 - The value of bit flags that clearing bit is 1
*
* Return Value:
*    None
*
* Description:
*    It is not necessary to call this function after called "R_OSPL_EVENT_Wait"
*    function.
*
*    The way that all bit flags is cleared is setting "R_OSPL_EVENT_ALL_BITS"
*    (=0x0000FFFF) at "ClearFlags1" argument.
*
*    When other thread was nofied by calling "R_OSPL_EVENT_Set", "R_OSPL_EVENT_Clear"
*    must not be called from caller (notifier) thread.
*
*    For OS less, there is the area disabled all interrupts.
*
*    Do nothing, when "ThreadId" = "NULL"
************************************************************************/
void  R_OSPL_EVENT_Clear( r_ospl_thread_id_t const  in_ThreadId,  r_ospl_event_flags_t const  in_ClearFlags1 );


/***********************************************************************
* Function: R_OSPL_EVENT_Get
*    Get 16bit flags value
*
* Arguments:
*    in_ThreadId - The thread ID attached the target event
*
* Return Value:
*    The value of 16bit flags
*
* Description:
*    This API cannot be used in newest specification.
*
*    In receiving the event, call "R_OSPL_EVENT_Wait" function instead of
*    "R_OSPL_EVENT_Get" function or call "R_OSPL_EVENT_Clear" function
*    passed the NOT operated value of flags got by "R_OSPL_EVENT_Get" function.
************************************************************************/
#if ( ( ! defined( osCMSIS )  ||  osCMSIS <= 0x10001 ) &&  R_OSPL_VERSION < 85 ) ||  R_OSPL_SUPPORT_EVENT_GET
bit_flags32_t  R_OSPL_EVENT_Get( r_ospl_thread_id_t const  in_ThreadId );
#endif


/***********************************************************************
* Function: R_OSPL_EVENT_Wait
*    Waits for setting the flags in 16bit and clear received flags
*
* Arguments:
*    in_WaigingFlags - The bit flags set to 1 waiting or "R_OSPL_ANY_FLAG"
*    out_GotFlags    - NULL is permitted. Output: 16 bit flags or "R_OSPL_TIMEOUT"
*    in_Timeout_msec - Time out (millisecond) or "R_OSPL_INFINITE"
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    Waits in this function until the flags become passed flags pattern
*    by "R_OSPL_EVENT_Set" function.
*
*    Check "r_ospl_async_t::ReturnValue", when the asynchronous operation
*    was ended.
************************************************************************/
errnum_t  R_OSPL_EVENT_Wait( bit_flags32_t const  in_WaigingFlags,  bit_flags32_t* const  out_GotFlags,
                             uint32_t const  in_Timeout_msec );
/* Unsigned flag (bit_flags32_t) is for QAC 4130 */


/***********************************************************************
* Function: R_OSPL_EVENT_Allocate
************************************************************************/
errnum_t  R_OSPL_EVENT_Allocate( volatile r_ospl_thread_id_t*  out_ThreadId,  r_ospl_thread_id_t  in_ThreadId,
                                 volatile bit_flags32_t*  out_SetFlag,  r_ospl_event_flags_t  in_SetFlag );


/***********************************************************************
* Function: R_OSPL_EVENT_Free
************************************************************************/
errnum_t  R_OSPL_EVENT_Free( volatile r_ospl_thread_id_t*  in_out_ThreadId,
                             volatile r_ospl_event_flags_t*  in_out_SetFlag );


/***********************************************************************
* Function: R_OSPL_EVENT_GetStatus
************************************************************************/
#if  R_OSPL_DETECT_BAD_EVENT
errnum_t  R_OSPL_EVENT_GetStatus( r_ospl_thread_id_t  in_ThreadId,  r_ospl_event_status_t** out_Status );
#endif


/***********************************************************************
* Class: r_ospl_flag32_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_FLAG32_InitConst
*    Clears all flags in 32bit to 0
*
* Arguments:
*    self - The value of 32bit flags
*
* Return Value:
*    None
*
* Description:
*    Operates following operation.
*    > volatile bit_flags32_t  self->flags;
*    > self->flags = 0;
************************************************************************/
void  R_OSPL_FLAG32_InitConst( volatile r_ospl_flag32_t* const  self );


/***********************************************************************
* Function: R_OSPL_FLAG32_Set
*    Set one or some bits to 1
*
* Arguments:
*    self        - The value of 32bit flags
*    in_SetFlags - The value of bit flags that target bit is 1
*
* Return Value:
*    None
*
* Description:
*    Operates following operation.
*    > volatile bit_flags32_t  self->Flags;
*    > bit_flags32_t           SetFlags;
*    > self->Flags |= SetFlags;
*    This function is not atomic because "|=" operator is "Read Modify Write" operation.
************************************************************************/
void  R_OSPL_FLAG32_Set( volatile r_ospl_flag32_t* const  self, bit_flags32_t const  in_SetFlags );


/***********************************************************************
* Function: R_OSPL_FLAG32_Clear
*    Set one or some bits to 0
*
* Arguments:
*    self           - The value of 32bit flags
*    in_ClearFlags1 - The value of bit flags that clearing bit is 1
*
* Return Value:
*    None
*
* Description:
*    Operates following operation.
*    > volatile bit_flags32_t  self->Flags;
*    > bit_flags32_t           ClearFlags1;
*    >
*    > self->Flags &= ~ClearFlags1;
*
*    Set "R_OSPL_FLAG32_ALL_BITS", if you wanted to clear all bits.
*
*    This function is not atomic because "&=" operator is "Read Modify Write" operation.
************************************************************************/
void  R_OSPL_FLAG32_Clear( volatile r_ospl_flag32_t* const  self, bit_flags32_t const  in_ClearFlags1 );


/***********************************************************************
* Function: R_OSPL_FLAG32_Get
*    Get 32bit flags value
*
* Arguments:
*    self - The value of 32bit flags
*
* Return Value:
*    The value of 32bit flags
*
* Description:
*    In receiving the event, call "R_OSPL_FLAG32_GetAndClear" function
*    instead of "R_OSPL_FLAG32_Get" function or call "R_OSPL_FLAG32_Clear"
*    function passed the NOT operated value of flags got by "R_OSPL_FLAG32_Get"
*    function.
*
*    > Operates following operation.
*    > volatile bit_flags32_t  self->Flags;
*    > bit_flags32_t           return_flags;
*    >
*    > return_flags = self->Flags;
*    >
*    > return  return_flags;
************************************************************************/
bit_flags32_t  R_OSPL_FLAG32_Get( volatile const r_ospl_flag32_t* const  self );


/***********************************************************************
* Function: R_OSPL_FLAG32_GetAndClear
*    Get 32bit flags value
*
* Arguments:
*    self - The value of 32bit flags
*
* Return Value:
*    The value of 32bit flags
*
* Description:
*    Operates following operation.
*    > volatile bit_flags32_t  self->Flags;
*    > bit_flags32_t           return_flags;
*    >
*    > return_flags = self->Flags;
*    > self->Flags = 0;
*    >
*    > return  return_flags;
*
*    This function is not atomic because the value might be set before clearing to 0.
************************************************************************/
bit_flags32_t  R_OSPL_FLAG32_GetAndClear( volatile r_ospl_flag32_t* const  self );


/***********************************************************************
* Class: r_ospl_queue_id_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_QUEUE_Create
*    Initializes a queue
*
* Arguments:
*    out_self    - Output: Address of initialized queue object
*    QueueDefine - Initial attributes of queue and work area
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    It is not possible to call this function from the library.
*    This function is called from porting layer of the driver and send
*    created queue to the driver.
*
*    OSPL does not have finalizing function (portabled with CMSIS).
*    An object specified "QueueDefine" argument can be specified to the
*    create function 1 times only. Some OS does not have this limitation.
*
*    The address of a variable as "r_ospl_queue_id_t" type is set at
*    "out_self" argument.
*    Internal variables of the queue are stored in the variable specified
*    with "QueueDefine" argument.
************************************************************************/
errnum_t  R_OSPL_QUEUE_Create( r_ospl_queue_id_t* out_self, r_ospl_queue_def_t* QueueDefine );


/***********************************************************************
* Function: R_OSPL_QUEUE_GetStatus
*    Gets status of the queue
*
* Arguments:
*    self       - A queue object
*    out_Status - Output: Pointer to the status structure
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    Got status are the information at calling moment.
*    If ohter threads were run, the status will be changed.
*    See "R_DRIVER_GetAsyncStatus" function about pointer type of
*    "out_Status" argument
************************************************************************/
errnum_t  R_OSPL_QUEUE_GetStatus( r_ospl_queue_id_t self,  const r_ospl_queue_status_t** out_Status );


/***********************************************************************
* Function: R_OSPL_QUEUE_Allocate
*    Allocates an element from the queue object
*
* Arguments:
*    self         - A queue object
*    out_Address  - Output: Address of allocated element
*    Timeout_msec - Timeout (msec) or R_OSPL_INFINITE
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    An error will be raised, if "Timeout_msec != 0" in interrupt context.
*    It becomes "*out_Address = NULL", when it was timeout and
*    "Timeout_msec = 0".
*    E_TIME_OUT error is raised, when it was timeout and "Timeout_msec != 0".
************************************************************************/
errnum_t  R_OSPL_QUEUE_Allocate( r_ospl_queue_id_t self,  void* out_Address,  uint32_t Timeout_msec );


/***********************************************************************
* Function: R_OSPL_QUEUE_Put
*    Sends the element to the queue
*
* Arguments:
*    self    - A queue object
*    Address - Address of element to put
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    It is correct, even if other thread put to the queue or get from
*    the queue from calling "R_OSPL_QUEUE_Allocate" to calling
*    "R_OSPL_QUEUE_Put".
*
*    The message put to the queue by this function receives the thread
*    calling "R_OSPL_QUEUE_Get" function.
************************************************************************/
errnum_t  R_OSPL_QUEUE_Put( r_ospl_queue_id_t self,  void* Address );


/***********************************************************************
* Function: R_OSPL_QUEUE_Get
*    Receives the element from the queue
*
* Arguments:
*    self         - A queue object
*    out_Address  - Output: Address of received element
*    Timeout_msec - Timeout (msec) or R_OSPL_INFINITE
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    Call "R_OSPL_QUEUE_Free" function after finishing to access to
*    the element. Don't access the memory area of the element after
*    calling "R_OSPL_QUEUE_Free".
*
*    "E_NOT_THREAD" error is raised, if "Timeout_msec = 0" was specified
*    from interrupt context. It is not possible to wait for put data to
*    the queue in interrupt context.
*
*    "*out_Address" is NULL and any errors are not raised, if it becomed
*    to timeout and "Timeout_msec = 0". "E_TIME_OUT" is raised,
*    if "Timeout_msec != 0".
*
*    Specify "Timeout_msec = 0", call the following functions by the following
*    order and use an event for preventing to block to receive other events
*    by the thread having waited for the queue.
*
*    Sending Side
*    - R_OSPL_QUEUE_Allocate
*    - R_OSPL_QUEUE_Put
*    - R_OSPL_EVENT_Set
*
*    Receiving Side
*    - R_OSPL_EVENT_Wait
*    - R_OSPL_QUEUE_Get
*    - R_OSPL_QUEUE_Free
*
*    In OS less environment, "R_OSPL_QUEUE_Get" supports pseudo multi
*    threading. See "R_OSPL_THREAD_GetIsWaiting" function.
************************************************************************/
errnum_t  R_OSPL_QUEUE_Get( r_ospl_queue_id_t self,  void* out_Address,  uint32_t Timeout_msec );


/***********************************************************************
* Function: R_OSPL_QUEUE_Free
*    Releases the element to the queue object
*
* Arguments:
*    self    - A queue object
*    Address - Address of received element
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    It is correct, even if other thread put to the queue or get from
*    the queue from calling "R_OSPL_QUEUE_Get" to calling "R_OSPL_QUEUE_Free".
************************************************************************/
errnum_t  R_OSPL_QUEUE_Free( r_ospl_queue_id_t self,  void* Address );


/***********************************************************************
* Function: R_OSPL_QUEUE_Print
*    Print status of the queue object
*
* Arguments:
*    self - A queue object
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
************************************************************************/
#ifndef  R_OSPL_NDEBUG
errnum_t  R_OSPL_QUEUE_Print( r_ospl_queue_id_t self );
#endif


/***********************************************************************
* Class: r_ospl_async_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_ASYNC_SetDefaultPreset
*    SetDefaultPreset
*
* Arguments:
*    in_out_Async - <r_ospl_async_t> structure.
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_ASYNC_SetDefaultPreset( r_ospl_async_t* const  in_out_Async );


/***********************************************************************
* Function: R_OSPL_ASYNC_CopyExceptAThread
*    CopyExceptAThread
*
* Arguments:
*    in_Source       - Source
*    out_Destination - Destination
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_ASYNC_CopyExceptAThread( const r_ospl_async_t* const  in_Source,
                                      r_ospl_async_t* const  out_Destination );


/***********************************************************************
* Class: r_ospl_caller_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_CallInterruptCallback
*    Calls the interrupt callback function. It is called from OS porting layer in the driver
*
* Arguments:
*    self               - The internal parameters about interrupt operations
*    in_InterruptSource - The source of the interrupt
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_CallInterruptCallback( const r_ospl_caller_t* const  self,
                                    const r_ospl_interrupt_t* const  in_InterruptSource );


/***********************************************************************
* Function: R_OSPL_CALLER_Initialize
*    Initialize <r_ospl_caller_t>.
*
* Arguments:
*    self      - The internal parameters about interrupt operations
*    in_Async  - <r_ospl_async_t>
*    in_PointerToState             - PointerToState
*    in_StateValueOfOnInterrupting - StateValueOfOnInterrupting
*    in_I_Lock       - I_Lock
*    in_I_LockVTable - I_LockVTable
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_CALLER_Initialize( r_ospl_caller_t* const  self,  r_ospl_async_t* const  in_Async,
                                volatile void* const  in_PointerToState,  int_t const  in_StateValueOfOnInterrupting,
                                void* const  in_I_Lock,  const r_ospl_i_lock_vtable_t* const  in_I_LockVTable );


/***********************************************************************
* Function: R_OSPL_CALLER_GetRootChannelNum
*    Get root channel number
*
* Arguments:
*    self  - The internal parameters about interrupt operations. <r_ospl_caller_t>.
*
* Return Value:
*    Root channel number
************************************************************************/
INLINE int_fast32_t  R_OSPL_CALLER_GetRootChannelNum( const r_ospl_caller_t* const  self );


/* Section: Interrupt */
/***********************************************************************
* Function: R_OSPL_OnInterruptForUnregistered
*    Interrupt callback function for unregisterd interrupt.
*
* Arguments:
*    int_sense  - (See INTC driver)
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_OnInterruptForUnregistered( uint32_t const  int_sense );


/***********************************************************************
* Function: R_OSPL_EnableAllInterrupt
*    Releases all disabled interrupts
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    Driver user should not call this function.
*    Call this function at the end of area of all interrupts disabled.
*    Do not release, if all interrupts was already disabled by caller function.
*    This function does not release disabled NMI.
************************************************************************/
void  R_OSPL_EnableAllInterrupt(void);


/***********************************************************************
* Function: R_OSPL_DisableAllInterrupt
*    Disables all interrupts
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    Driver user should not call this function.
*    Call this function at begin of area of all interrupts disabled.
*    This function does not disable NMI.
*
* Example:
*    > void  Func()
*    > {
*    >     bool_t  was_all_enabled = false;
*    >
*    >     was_all_enabled = R_OSPL_DisableAllInterrupt();
*    >
*    >     // All interrupt disabled
*    >
*    >     if ( was_all_enabled )
*    >         { R_OSPL_EnableAllInterrupt(); }
*    > }
************************************************************************/
bool_t  R_OSPL_DisableAllInterrupt(void);


/***********************************************************************
* Function: R_OSPL_SetInterruptPriority
*    Sets the priority of the interrupt line.
*
* Arguments:
*    in_IRQ_Num  - Interrupt request number
*    in_Priority - Priority. The less the prior.
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
************************************************************************/
errnum_t  R_OSPL_SetInterruptPriority( bsp_int_src_t const  in_IRQ_Num, int_fast32_t const  in_Priority );


/* Section: Locking channel */
/***********************************************************************
* Function: R_OSPL_LockChannel
*    Locks by channel number.
*
* Arguments:
*    in_ChannelNum       - Locking channel number or "R_OSPL_UNLOCKED_CHANNEL"
*    out_ChannelNum      - Output: Locked channel number, (in) NULL is permitted
*    in_HardwareIndexMin - Hardware index of channel number = 0
*    in_HardwareIndexMax - Hardware index of max channel number
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    This function is called from the internal of "R_DRIVER_Initialize"
*    function or "R_DRIVER_LockChannel" function.
*    This function calls "R_BSP_HardwareLock".
************************************************************************/
errnum_t  R_OSPL_LockChannel( int_fast32_t  in_ChannelNum,  int_fast32_t* out_ChannelNum,
                              mcu_lock_t  in_HardwareIndexMin,  mcu_lock_t  in_HardwareIndexMax );


/***********************************************************************
* Function: R_OSPL_UnlockChannel
*    Unlocks by channel number.
*
* Arguments:
*    in_ChannelNum       - Channel number
*    e                   - Raising error code, If there is no error, 0
*    in_HardwareIndexMin - Hardware index of channel number = 0
*    in_HardwareIndexMax - Hardware index of max channel number
*
* Return Value:
*    Error code.  If there is no error, the return value is 0
*
* Description:
*    This function is called from the internal of "R_DRIVER_Finalize"
*    function or "R_DRIVER_UnlockChannel" function.
*    This function calls "R_BSP_HardwareUnlock".
************************************************************************/
errnum_t  R_OSPL_UnlockChannel( int_fast32_t  in_ChannelNum,  errnum_t  e,
                                mcu_lock_t  in_HardwareIndexMin,  mcu_lock_t  in_HardwareIndexMax );


/***********************************************************************
* Class: r_ospl_c_lock_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_C_LOCK_InitConst
*    Initializes the C-lock object
*
* Arguments:
*    self - C-lock object
*
* Return Value:
*    None
*
* Description:
*    If *self is global variable or static variable initialized 0,
*    this function does not have to be called.
************************************************************************/
void  R_OSPL_C_LOCK_InitConst( r_ospl_c_lock_t* const  self );


/***********************************************************************
* Function: R_OSPL_C_LOCK_Lock
*    Locks the target, if lockable state.
*
* Arguments:
*    self - C-lock object
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    Even if lock owner called this function, if lock object was already
*    locked, E_ACCESS_DENIED error is raised.
*
*    "R_OSPL_C_LOCK_Lock" does not do exclusive control.
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Lock( r_ospl_c_lock_t* const  self );


/***********************************************************************
* Function: R_OSPL_C_LOCK_Unlock
*    Unlocks the target.
*
* Arguments:
*    self - C-lock object
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    If this function was called with unlocked object, this function
*    does nothing and raises "E_ACCESS_DENIED" error.
*
*    If self == NULL, this function does nothing and raises no error.
*    E_NOT_THREAD error is raised, if this function was called from the
*    interrupt context.
*
*    I-lock does not do in this function.
*
*    "R_OSPL_C_LOCK_Unlock" does not do exclusive control.
************************************************************************/
errnum_t  R_OSPL_C_LOCK_Unlock( r_ospl_c_lock_t* const  self );


/***********************************************************************
* Class: r_ospl_i_lock_vtable_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_I_LOCK_LockStub
*    Do nothing. This is registered to r_ospl_i_lock_vtable_t::Lock.
*
* Arguments:
*    self_ - I-lock object
*
* Return Value:
*    false
************************************************************************/
bool_t  R_OSPL_I_LOCK_LockStub( void* const  self_ );


/***********************************************************************
* Function: R_OSPL_I_LOCK_UnlockStub
*    Do nothing. This is registered to r_ospl_i_lock_vtable_t::Unlock.
*
* Arguments:
*    self_ - I-lock object
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_I_LOCK_UnlockStub( void* const  self_ );


/***********************************************************************
* Function: R_OSPL_I_LOCK_RequestFinalizeStub
*    Do nothing. This is registered to r_ospl_i_lock_vtable_t::RequestFinalize.
*
* Arguments:
*    self_ - I-lock object
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_I_LOCK_RequestFinalizeStub( void* const  self_ );


/* Section: Memory Operation */
/***********************************************************************
* Function: R_OSPL_MEMORY_Flush
*    Flushes cache memory
*
* Arguments:
*    in_FlushType - The operation of flush
*
* Return Value:
*    None
*
* Description:
*    Call the function of the driver after flushing input output buffer
*    in the cache memory, If the data area accessing by the hardware is
*    on cache and the driver did not manage the cache memory.
*    Whether the driver manages the cache memory is depend on the driver
*    specification.
************************************************************************/
void  R_OSPL_MEMORY_Flush( r_ospl_flush_t const  in_FlushType );


/***********************************************************************
* Function: R_OSPL_MEMORY_RangeFlush
*    Flushes cache memory with the range of virtual address.
*
* Arguments:
*    FlushType - The operation of flush
*
* Return Value:
*    None
*
* Description:
*    Align "StartAddress" argument and "Length" argument to cache line size.
*    If not aligned, E_OTHERS error is raised.
*    Refer to : R_OSPL_MEMORY_GetSpecification
*
*    If the data area written by the hardware and read from CPU was in cache
*    rea, when the hardware started without invalidate
*    ("R_OSPL_FLUSH_WRITEBACK_INVALIDATE" or "R_OSPL_FLUSH_INVALIDATE"),
*    invalidate the data area and read it after finished to write by hardware.
*    (If the driver does not manage the cache memory.)
************************************************************************/
errnum_t  R_OSPL_MEMORY_RangeFlush( r_ospl_flush_t const  FlushType,
                                    const void* const  StartAddress,  size_t const  Length );


/***********************************************************************
* Function: R_OSPL_MEMORY_GetSpecification
*    Gets the specification about memory and cache memory.
*
* Arguments:
*    out_MemorySpec - The specification about memory and cache memory
*
* Return Value:
*    None
************************************************************************/
void      R_OSPL_MEMORY_GetSpecification( r_ospl_memory_spec_t* const  out_MemorySpec );


/***********************************************************************
* Function: R_OSPL_MEMORY_Barrier
*    Set a memory barrier.
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    In ARM, This function calls DSB assembler operation.
*    This effects to L1 cache only.
************************************************************************/
void  R_OSPL_MEMORY_Barrier(void);


/***********************************************************************
* Function: R_OSPL_InstructionSyncBarrier
*    Set a instruction barrier.
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    In ARM, This function calls ISB assembler operation.
************************************************************************/
void  R_OSPL_InstructionSyncBarrier(void);


/***********************************************************************
* Function: R_OSPL_ToPhysicalAddress
*    Changes to physical address
*
* Arguments:
*    in_Address          - Virtual address
*    out_PhysicalAddress - Output: Physical address
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    This function must be modified by MMU setting.
************************************************************************/
errnum_t  R_OSPL_ToPhysicalAddress( const volatile void* const  in_Address,  uintptr_t* const  out_PhysicalAddress );


/***********************************************************************
* Function: R_OSPL_ToCachedAddress
*    Checks the address in the L1 cache area or Changes to the address in the L1 cache area.
*
* Arguments:
*    in_Address        - Virtual address
*    out_CachedAddress - Output: Virtual address for cached area
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    This function must be modified by MMU setting.
*    If MMU had a static flat mapping, this function only checks
*    in the L1 cache area.
*    If "E_ACCESS_DENIED" error was raised, you may know the variable by
*    looking at value of "Address" argument and map file.
************************************************************************/
errnum_t  R_OSPL_ToCachedAddress( const volatile void* const  in_Address,  void* const  out_CachedAddress );


/***********************************************************************
* Function: R_OSPL_ToUncachedAddress
*    Checks the address in L1 uncached area or Changes to the address in the L1 uncached area.
*
* Arguments:
*    in_Address          - Virtual address
*    out_UncachedAddress - Output: Virtual address for uncached area
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    This function must be modified by MMU setting.
*    If MMU had a static flat mapping, this function only checks
*    in the uncached area.
*    If "E_ACCESS_DENIED" error was raised, you may know the variable by
*    looking at value of "Address" argument and map file.
************************************************************************/
errnum_t  R_OSPL_ToUncachedAddress( const volatile void* const  in_Address,  void* const  out_UncachedAddress );


/***********************************************************************
* Function: R_OSPL_MEMORY_GetLevelOfFlush
*    Gets the level of cache for flushing the memory indicated by the address.
*
* Arguments:
*    in_Address - The address in flushing memory
*    out_Level  - Output: 0=Not need to flush, 1=L1 cache only, 2=both of L1 and L2 cache
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_MEMORY_GetLevelOfFlush( const void*  in_Address,  int_fast32_t* out_Level );


/***********************************************************************
* Function: R_OSPL_MEMORY_GetMaxLevelOfFlush
*    Gets maximum level of cache for flushing the memory.
*
* Arguments:
*    None
*
* Return Value:
*    0=Not need to flush, 1=L1 cache only, 2=both of L1 and L2 cache.
************************************************************************/
int_fast32_t  R_OSPL_MEMORY_GetMaxLevelOfFlush(void);


/***********************************************************************
* Function: R_OSPL_AXI_Get2ndCacheAttribute
*    Get 2nd cache attribute of AXI bus for peripheral (not CPU) from physical address.
*
* Arguments:
*    in_PhysicalAddress - The physical address in the memory area
*    out_CacheAttribute - Output: Cache_attribute, AWCACHE[3:0], ARCACHE[3:0]
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_AXI_Get2ndCacheAttribute( uintptr_t const  in_PhysicalAddress,
        r_ospl_axi_cache_attribute_t* const  out_CacheAttribute );


/***********************************************************************
* Function: R_OSPL_AXI_GetProtection
*    Gets protection attribute of AXI bus from the address
*
* Arguments:
*    in_PhysicalAddress - The physical address in the memory area
*    out_CacheAttribute - Output: The protection attribute of AXI bus AWPROT[2:0], ARPROT[2:0]
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_AXI_GetProtection( uintptr_t const  in_PhysicalAddress,
                                    r_ospl_axi_protection_t* const  out_protection );


/* Section: Timer */
/***********************************************************************
* Function: R_OSPL_Delay
*    Waits for a while until passed time
*
* Arguments:
*    in_DelayTime_msec - Time of waiting (millisecond)
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    Maximum value is "R_OSPL_MAX_TIME_OUT" (=65533).
************************************************************************/
errnum_t  R_OSPL_Delay( uint32_t const  in_DelayTime_msec );


/***********************************************************************
* Function: R_OSPL_FTIMER_InitializeIfNot
*    Set up the free running timer
*
* Arguments:
*    out_Specification - NULL is permitted. Output: The precision of the free run timer
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
*
* Description:
*    The free running timer does not stop.
*
*    If the counter of the free running timer was overflow, the counter returns to 0.
*    Even in interrupt handler, the counter does count up.
*    OSPL free running timer does not use any interrupt.
*
*    Using timer can be selected by "R_OSPL_FTIMER_IS" macro.
*
*    If the free running timer was already set up, this function does not set up it,
*    outputs to "out_Specification" argument and does not raise any error.
*
*    When OSPL API function with timeout or "R_OSPL_Delay" function was called,
*    "R_OSPL_FTIMER_InitializeIfNot" function is callbacked from these functions.
*
*    There is all interrupt disabled area inside.
************************************************************************/
errnum_t  R_OSPL_FTIMER_InitializeIfNot( r_ospl_ftimer_spec_t* const  out_Specification );


/***********************************************************************
* Function: R_OSPL_FTIMER_GetSpecification
*    Gets the specification of free running timer.
*
* Arguments:
*    out_Specification - Output: The precision of the free run timer
*
* Return Value:
*    None
************************************************************************/
void      R_OSPL_FTIMER_GetSpecification( r_ospl_ftimer_spec_t* const  out_Specification );


/***********************************************************************
* Function: R_OSPL_FTIMER_Get
*    Get current time of free running timer.
*
* Arguments:
*    None
*
* Return Value:
*    The current clock count of free run timer
*
* Description:
*    Call "R_OSPL_FTIMER_InitializeIfNot" function before calling this function.
*    Call "R_OSPL_FTIMER_IsPast" function, when it is determined whether time passed.
*
* Example:
*    > errnum_t              e;
*    > r_ospl_ftimer_spec_t  ts;
*    > uint32_t              start;
*    > uint32_t              end;
*    >
*    > e= R_OSPL_FTIMER_InitializeIfNot( &ts ); IF(e){goto fin;}
*    > start = R_OSPL_FTIMER_Get();
*    >
*    > // The section of measuring
*    >
*    > end = R_OSPL_FTIMER_Get();
*    > printf( "%d msec\n", R_OSPL_FTIMER_CountToTime(
*    >         &ts, end - start ) );
************************************************************************/
uint32_t  R_OSPL_FTIMER_Get(void);


/***********************************************************************
* Function: R_OSPL_FTIMER_IsPast
*    Returns whether specified time was passed
*
* Arguments:
*    ts            - Precision of the free running timer
*    in_Now        - Count of current time
*    in_TargetTime - Count of target time
*    out_IsPast    - Output: Whether the target time was past or not
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_FTIMER_IsPast( const r_ospl_ftimer_spec_t* const  ts,
                                uint32_t const  in_Now,  uint32_t const  in_TargetTime,  bool_t* const  out_IsPast );


/***********************************************************************
* Function: R_OSPL_FTIMER_TimeToCount
*    Change from mili-second unit to free running timer unit
*
* Arguments:
*    ts      - Precision of the free running timer
*    in_msec - The value of mili-second unit
*
* Return Value:
*    The value of free running timer unit
*
* Description:
*    The fractional part is been round up. (For waiting time must be more
*    than specified time.)
*
*    This function calculates like the following formula.
*    > ( msec * ts->msec_Denominator + ts->msec_Numerator - 1 ) / ts->msec_Numerator
*
*    Attention: If "ts->msec_Denominator" was more than "ts->msec_Numerator",
*    take care of overflow.
************************************************************************/
INLINE uint32_t  R_OSPL_FTIMER_TimeToCount( const r_ospl_ftimer_spec_t* const  ts,
        uint32_t const  in_msec );


/***********************************************************************
* Function: R_OSPL_FTIMER_CountToTime
*    Change from free running timer unit to mili-second unit
*
* Arguments:
*    ts       - Precision of the free running timer
*    in_Count - The value of free running timer unit
*
* Return Value:
*    The value of mili-second unit
*
* Description:
*    The fractional part is been round down. (Because overflow does not
*    occur, when "Count = r_ospl_ftimer_spec_t::MaxCount" )
*
*    This function calculates like the following formula.
*    > ( Count * ts->msec_Numerator ) / ts->msec_Denominator
************************************************************************/
INLINE uint32_t  R_OSPL_FTIMER_CountToTime( const r_ospl_ftimer_spec_t* const  ts,
        uint32_t const  in_Count );


/***********************************************************************
* Class: r_ospl_table_t
************************************************************************/

/***********************************************************************
* Function: R_OSPL_TABLE_InitConst
*    Initializes an index table
*
* Arguments:
*    self            - Index table object
*    in_Area         - First address of the index table
*    in_AreaByteSize - Size of the index table. See <R_OSPL_TABLE_SIZE>
*    in_Flags        - 0 or R_OSPL_T_LOCK or R_OSPL_I_LOCK
*
* Return Value:
*    None
************************************************************************/
void  R_OSPL_TABLE_InitConst( r_ospl_table_t* const  self,
                              void* const  in_Area,  size_t const  in_AreaByteSize,  bit_flags_fast32_t const  in_Flags );


/***********************************************************************
* Function: R_OSPL_TABLE_GetIndex
*    Returns index from related key
*
* Arguments:
*    self           - Index table object
*    in_Key         - Key number
*    out_Index      - Output: Related index
*    in_TypeOfIfNot - Behavior when key was not registerd. See <r_ospl_if_not_t>
*
* Return Value:
*    Error code.  If there is no error, the return value is 0.
************************************************************************/
errnum_t  R_OSPL_TABLE_GetIndex( r_ospl_table_t* const  self, const void* const  in_Key,
                                 int_fast32_t* const  out_Index,  r_ospl_if_not_t  in_TypeOfIfNot );


/***********************************************************************
* Function: R_OSPL_TABLE_Free
*    Separates relationship of specified key and related index
*
* Arguments:
*    self        - Index table object
*    in_Key      - Key number
*
* Return Value:
*    None
*
* Description:
*    Error is not raised, even if specified key was already separated.
************************************************************************/
void  R_OSPL_TABLE_Free( r_ospl_table_t* const  self, const void* const  in_Key );


/***********************************************************************
* Function: R_OSPL_TABLE_FreeByIndex
************************************************************************/
void  R_OSPL_TABLE_FreeByIndex( r_ospl_table_t* const  self,  int_fast32_t const  in_Key );


/***********************************************************************
* Function: R_OSPL_TABLE_Print
*    Print status of specified index table object (for debug)
*
* Arguments:
*    self        - Index table object
*
* Return Value:
*    None
************************************************************************/
#if R_OSPL_DEBUG_TOOL
void  R_OSPL_TABLE_Print( r_ospl_table_t* const  self );
#endif


/* Section: Bit flags */
/***********************************************************************
* Function: IS_BIT_SET
*    Evaluate whether any passed bits are 1 or not (bit_flags_fast32_t)
*
* Arguments:
*    in_Variable   - The value of target bit flags
*    in_ConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the any passed bit are 1
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_BIT_SET( in_Variable, in_ConstValue ) \
	( BIT_And_Sub( in_Variable, in_ConstValue ) != 0u )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: IS_ANY_BITS_SET
*    Evaluate whether any passed bits are 1 or not
*
* Arguments:
*    in_Variable     - The value of target bit flags
*    in_OrConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the any passed bit are 1
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_ANY_BITS_SET( in_Variable, in_OrConstValue ) \
	( BIT_And_Sub( in_Variable, in_OrConstValue ) != 0u )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: IS_ALL_BITS_SET
*    Evaluate whether all passed bits are 1 or not
*
* Arguments:
*    in_Variable     - The value of target bit flags
*    in_OrConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the all passed bit are 1
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_ALL_BITS_SET( in_Variable, in_OrConstValue ) \
	( BIT_And_Sub( in_Variable, in_OrConstValue ) == (in_OrConstValue) )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: IS_BIT_NOT_SET
*    Evaluate whether the passed bit is 0 or not
*
* Arguments:
*    in_Variable   - The value of target bit flags
*    in_ConstValue - The value that investigating bit is 1
*
* Return Value:
*    Whether the passed bit is 0
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_BIT_NOT_SET( in_Variable, in_ConstValue ) \
	( BIT_And_Sub( in_Variable, in_ConstValue ) == 0u )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: IS_ANY_BITS_NOT_SET
*    Evaluate whether any passed bits are 0 or not
*
* Arguments:
*    in_Variable     - The value of target bit flags
*    in_OrConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the any passed bit are 0
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_ANY_BITS_NOT_SET( in_Variable, in_OrConstValue ) \
	( BIT_And_Sub( in_Variable, in_OrConstValue ) != (in_OrConstValue) )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: IS_ALL_BITS_NOT_SET
*    Evaluate whether all passed bits are 0 or not
*
* Arguments:
*    in_Variable     - The value of target bit flags
*    in_OrConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the all passed bit are 0
************************************************************************/
/* ->MISRA 19.7 : For return _Bool type */ /* ->SEC M5.1.3 */
#define  IS_ALL_BITS_NOT_SET( in_Variable, in_OrConstValue ) \
	( BIT_And_Sub( in_Variable, in_OrConstValue ) == 0u )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Function: BIT_And_Sub
*    Sub routine of bitwise operation
*
* Arguments:
*    in_Variable   - The value of target bit flags
*    in_ConstValue - The value that investigating bits are 1
*
* Return Value:
*    Whether the all passed bit are 0
*
* Description:
*    This part is for compliant to MISRA 2004 - 19.7.
************************************************************************/
INLINE uint_fast32_t  BIT_And_Sub( bit_flags_fast32_t const  in_Variable,
                                   bit_flags_fast32_t const  in_ConstValue );


/***********************************************************************
* About: IS_BIT_SET__Warning
*
* - This is for QAC-3344 warning : MISRA 13.2 Advice : Tests of a value against
*   zero should be made explicit, unless the operand is effectively Boolean.
* - This is for QAC-1253 warning : SEC M1.2.2 : A "U" suffix shall be applied
*   to all constants of unsigned type.
************************************************************************/


/***********************************************************************
* Function: R_OSPL_CountLeadingZeros
*    Counts bit 0 count from MSB.
*
* Arguments:
*    in_Variable - The value of target bit flags
*
* Return Value:
*    bit 0 count from MSB.
************************************************************************/
#if defined( __CC_ARM )
#define  R_OSPL_CountLeadingZeros( in_Variable ) \
	( (int_fast32_t) __clz( in_Variable ) )

#elif defined( __GNUC__ )
#define  R_OSPL_CountLeadingZeros( in_Variable ) \
	__builtin_clz( in_Variable )

#elif defined( __ICCARM__ )
#define  R_OSPL_CountLeadingZeros( in_Variable ) \
	( (int_fast32_t) __CLZ( in_Variable ) )
#else
int_fast32_t  R_OSPL_CountLeadingZeros( bit_flags32_t  in_BitFlags );
#endif


/***********************************************************************
* Function: R_OSPL_IsSetBitsCount1
*    Whther is set bits count 1. e.g 0b0001, 0b0010, ...
*
* Arguments:
*    in_Value - The value of target bit flags
*
* Return Value:
*    Whther is set bits count 1.
************************************************************************/
INLINE bool_t    R_OSPL_IsSetBitsCount1( uint32_t in_Value );


/* Section: Error handling and debugging (2) */
/***********************************************************************
* Function: R_DEBUG_BREAK
*    Breaks here
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    Does break by calling "R_DebugBreak" function.
*    This macro is not influenced the setting of "R_OSPL_ERROR_BREAK" macro.
************************************************************************/
#define  R_DEBUG_BREAK() \
	R_DebugBreak(__FILE__,__LINE__)


/***********************************************************************
* Function: R_DebugBreak
*    The function callbacked from OSPL for breaking
*
* Arguments:
*    in_File - File name of breaking
*    in_Line - Line number of breaking
*
* Return Value:
*    None
*
* Description:
*    Set a break point at this function.
*    In Release configuration, "File = NULL, Line = 0".
*    If "File = NULL", "Line" argument is error code.
*    This function can be customized by application developer.
************************************************************************/
void  R_DebugBreak( const char_t* const  in_File,  int_fast32_t const  in_Line );


/***********************************************************************
* Function: R_DEBUG_BREAK_IF_ERROR
*    Breaks here, if it is error state
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    This function does nothing, if "R_OSPL_ERROR_BREAK" macro was defined
*    to be 0. The following descriptions are available, if "R_OSPL_ERROR_BREAK"
*    macro was defined to be 1.
*
*    Checks the error state of the current thread.
*    Call this macro from the last of each thread.
*    Does break by calling "R_DebugBreak" function.
*
*    If an error was raised, this function calls "printf" with following message.
*    Set "error_ID" to "R_OSPL_SET_BREAK_ERROR_ID"
*    > <ERROR error_ID="0x1" file="../src/api.c(336)"/>
*    > Write "R_OSPL_SET_BREAK_ERROR_ID( 1 );".
************************************************************************/
#define  R_DEBUG_BREAK_IF_ERROR()  R_OSPL_DebugBreakIfError(__FILE__,__LINE__)

#if R_OSPL_ERROR_BREAK
void  R_OSPL_DebugBreakIfError( const char_t* const  in_File,  int_fast32_t const  in_Line );
#else
#ifdef _SH
#pragma inline R_OSPL_DebugBreakIfError
#endif
INLINE void  R_OSPL_DebugBreakIfError( const char_t* const  in_File,  int_fast32_t const  in_Line )
{
    R_UNREFERENCED_VARIABLE_2( in_File,  in_Line );
}
/* The inline body function can not be linked for gcc, if "R_DEBUG_BREAK_IF_ERROR" was inline. */
#endif


/***********************************************************************
* Function: R_OSPL_RaiseUnrecoverable
*    Raises the error of system unrecoverable
*
* Arguments:
*    e - Error code
*
* Return Value:
*    None
*
* Description:
*    The error of system unrecoverable is the error of impossible to
*    self-recover by process or main system. Example, the heap area was
*    broken or there are not any responses from hardware. This error can
*    be recoverable by OS or the system controller(e.g. Software reset)
*
*    Example, when an error of recovery process was raised,
*    "R_OSPL_RaiseUnrecoverable" function must be called.
*
*    "R_OSPL_RaiseUnrecoverable" function can be customized by the
*    application. By default, it calls "R_DebugBreak" function and falls
*    into the infinite loop.
************************************************************************/
void  R_OSPL_RaiseUnrecoverable( errnum_t const  e );


/***********************************************************************
* Function: R_OSPL_MergeErrNum
*    Merge the error code raised in the finalizing operation
*
* Arguments:
*    in_CurrentError - Current error code
*    in_AppendError  - New append error code
*
* Return Value:
*    Merged error code
*
* Description:
*    When the state was error state, if other new error was raised,
*    new error code is ignored.
*    - If "CurrentError != 0", this function returns "CurrentError" argument.
*    - If "CurrentError == 0", this function returns "AppendError" argument.
*
*    This function can be modify by user.
*
* Example:
*    > ee= Sample();
*    > e= R_OSPL_MergeErrNum( e, ee );
*    > return  e;
************************************************************************/
INLINE errnum_t  R_OSPL_MergeErrNum( errnum_t const  in_CurrentError,  errnum_t const  in_AppendError );


/***********************************************************************
* Function: R_OSPL_SetErrNum
*    Sets an error code to TLS (Thread Local Storage).
*
* Arguments:
*    e - Raising error code
*
* Return Value:
*    None
*
* Description:
*    Usually error code is returned. If API function cannot return any
*    error code, API function can have the specification of setting error
*    code by "R_OSPL_SetErrNum".
*
*    There is this function, if "R_OSPL_TLS_ERROR_CODE" macro was defined
*    to be 1.
*    This function does nothing, if any error code was stored already in TLS.
*    The state does not change to error state, if "R_OSPL_SetErrNum" function
*    was called only. See "R_OSPL_GET_ERROR_ID".
************************************************************************/
#if R_OSPL_TLS_ERROR_CODE
void  R_OSPL_SetErrNum( errnum_t const  e );
#endif


/***********************************************************************
* Function: R_OSPL_GetErrNum
*    Returns the error code from TLS (Thread Local Storage).
*
* Arguments:
*    None
*
* Return Value:
*    Error code
*
* Description:
*    Usually error code is returned. If API function cannot return any
*    error code, API function may have the specification of getting error
*    code by "R_OSPL_GetErrNum".
*
*    There is this function, if "R_OSPL_TLS_ERROR_CODE" macro was defined
*    to be 1. This function returns 0 after called "R_OSPL_CLEAR_ERROR"
*    function.
************************************************************************/
#if R_OSPL_TLS_ERROR_CODE
errnum_t  R_OSPL_GetErrNum(void);
#endif


/***********************************************************************
* Function: R_OSPL_CLEAR_ERROR
*    Clears the error state
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    This function does nothing, if "R_OSPL_ERROR_BREAK" macro and
*    "R_OSPL_TLS_ERROR_CODE" macro were defined to be 0. The following
*    descriptions are available, if "R_OSPL_ERROR_BREAK" macro was
*    defined to be 1.
*
*    Whether the state is the error state is stored in thread local
*    storage. "R_OSPL_GetErrNum" function returns 0 after called this
*    function.
*
*    If the error state was not cleared, the following descriptions were caused.
*    - Breaks at "R_DEBUG_BREAK_IF_ERROR" macro
*    - "R_OSPL_SET_BREAK_ERROR_ID" function behaves not expected behavior
*       because the count of error is not counted up.
************************************************************************/
void  R_OSPL_CLEAR_ERROR(void);


/***********************************************************************
* Function: R_OSPL_GET_ERROR_ID
*    Returns the number of current error
*
* Arguments:
*    None
*
* Return Value:
*    The number of current error
*
* Description:
*    This function does nothing, if "R_OSPL_ERROR_BREAK" macro was defined
*    to be 0. The following descriptions are available, if "R_OSPL_ERROR_BREAK"
*    macro was defined to be 1.
*
*    This function returns 0, if any errors were not raised.
*
*    This function returns 1, if first error was raised.
*
*    After that, this function returns 2, if second error was raised after
*    calling "R_OSPL_CLEAR_ERROR" function.
*    This function does not return 0 after that the error was cleared by
*    calling "R_OSPL_CLEAR_ERROR".
*    The number of current error is running number in the whole of system
*    (all threads).
*
*    Error is raised by following macros.
*    > IF, IF_D, ASSERT_R, ASSERT_D
*    The process breaks at a moment of error raised, if the number of current
*    error was set to "R_OSPL_SET_BREAK_ERROR_ID" macro.
************************************************************************/
#if R_OSPL_ERROR_BREAK
int_fast32_t  R_OSPL_GET_ERROR_ID(void);
#else
#ifdef _SH
#pragma inline R_OSPL_GET_ERROR_ID
#endif
INLINE int_fast32_t  R_OSPL_GET_ERROR_ID(void)
{
    return -1;
}
#endif


/***********************************************************************
* Function: R_OSPL_SET_BREAK_ERROR_ID
*    Register to break at raising error at the moment
*
* Arguments:
*    ID - Breaking number of error
*
* Return Value:
*    None
*
* Description:
*    This function does nothing, if "R_OSPL_ERROR_BREAK" macro was defined
*    to be 0. The following descriptions are available, if "R_OSPL_ERROR_BREAK"
*    macro was defined to be 1.
*
*    Set a break point at "R_DebugBreak" function, when the process breaks
*    at the error raised code.
*
*    The number of "ErrorID" argument can be known by "R_DEBUG_BREAK_IF_ERROR"
*    macro or "R_OSPL_GET_ERROR_ID" macro.
*    In multi-threading environment, the number of "ErrorID" argument is the
*    number of raised errors in all threads.
*
*    The following code breaks at first error.
*    > R_OSPL_SET_BREAK_ERROR_ID( 1 );
*
*    The following code breaks at next error after resuming from meny errors.
*    > R_OSPL_SET_BREAK_ERROR_ID( R_OSPL_GET_ERROR_ID() + 1 );
************************************************************************/
#if R_OSPL_ERROR_BREAK
void  R_OSPL_SET_BREAK_ERROR_ID( int_fast32_t const  ID );
#else
#ifdef _SH
#pragma inline R_OSPL_SET_BREAK_ERROR_ID
#endif
INLINE void  R_OSPL_SET_BREAK_ERROR_ID( int_fast32_t const ID )
{
    R_UNREFERENCED_VARIABLE( ID );
}
#endif


/***********************************************************************
* Function: R_OSPL_GET_STACK_POINTER
*    Get value of stack pointer.
*
* Arguments:
*    None
*
* Return Value:
*    Value of stack pointer
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
uint8_t*  R_OSPL_GET_STACK_POINTER(void);
#endif


/***********************************************************************
* Function: R_OSPL_SET_END_OF_STACK
*    Set address of end of stack for current thread.
*
* Arguments:
*    in_EndOfStackAddress - End of stack area. This address is in the stack.
*
* Return Value:
*    None
*
* Description:
*    "EndOfStackAddress" argument is specified with (start address of stack area) +/-
*    (stack size of thread - 1).
*    This function writes sentinel's value "R_F_OSPL_STACK_CHECK_SENTINEL_VALUE"
*    in the end of stack area.
*    Stack check does in "IF" macro or "R_OSPL_CHECK_STACK_OVERFLOW" function
*    by not overwritten sentinel's value.
*    Stack check is disabled until calling this function.
*
*    This function can not be called, if "R_OSPL_STACK_CHECK_CODE" macro was defined
*    to be 0. It is available, if "R_OSPL_STACK_CHECK_CODE" macro was defined to be 1.
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
void  R_OSPL_SET_END_OF_STACK( void* in_EndOfStackAddress );
#endif


/***********************************************************************
* Function: R_OSPL_CHECK_STACK_OVERFLOW
*    Check whether stack overflow was occured.
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* Description:
*    Stack check does by not overwritten sentinel's value.
*    This function is called from "IF" macro, "ASSERT_R" macro and so on.
*    If stack overflow was occured, this function calls "R_OSPL_RaiseUnrecoverable".
*
*    This function can not be called, if "R_OSPL_STACK_CHECK_CODE" macro was defined
*    to be 0. It is available, if "R_OSPL_STACK_CHECK_CODE" macro was defined to be 1.
*
*    This function does nothing, if "R_OSPL_SET_END_OF_STACK" was not called.
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
void  R_OSPL_CHECK_STACK_OVERFLOW(void);
#endif


/***********************************************************************
* Function: R_OSPL_RESET_MIN_FREE_STACK_SIZE
*    Reset minimum free stack size of current thread by filling sentinel to stack area.
*
* Arguments:
*    None
*
* Return Value:
*    Error Code. 0=No Error.
*
* Description:
*    This function fills "R_F_OSPL_STACK_CHECK_SENTINEL_VALUE" from end of stack
*    specified by "R_OSPL_SET_END_OF_STACK" to current stack pointer.
*
*    This function can not be called, if "R_OSPL_STACK_CHECK_CODE" macro was defined
*    to be 0. It is available, if "R_OSPL_STACK_CHECK_CODE" macro was defined to be 1.
*
*    This function raises an error, if "R_OSPL_SET_END_OF_STACK" was not called.
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
errnum_t  R_OSPL_RESET_MIN_FREE_STACK_SIZE(void);
#endif


/***********************************************************************
* Function: R_OSPL_GET_MIN_FREE_STACK_SIZE
*    Count minimum free stack size of current thread by reading stack area.
*
* Arguments:
*    None
*
* Return Value:
*    Minimum free stack size of current thread.
*
* Description:
*    This function can not be called, if "R_OSPL_STACK_CHECK_CODE" macro was defined
*    to be 0. It is available, if "R_OSPL_STACK_CHECK_CODE" macro was defined to be 1.
*
*    This function returns 0, if "R_OSPL_SET_END_OF_STACK" was not called.
************************************************************************/
#if  R_OSPL_ERROR_BREAK  &&  R_OSPL_STACK_CHECK_CODE
size_t  R_OSPL_GET_MIN_FREE_STACK_SIZE(void);
#endif


/***********************************************************************
* Function: R_OSPL_GetCurrentThreadError
*    Returns debbug information of current thread.
*
* Arguments:
*    None
*
* Return Value:
*    Debbug information of current thread.
************************************************************************/
#if R_OSPL_ERROR_BREAK
r_ospl_error_t*  R_OSPL_GetCurrentThreadError(void);
#endif


/***********************************************************************
* Function: R_OSPL_LockCurrentThreadError
*    Lock (T-Lock) and Returns debug information of current thread.
*
* Arguments:
*    None
*
* Return Value:
*    Debbug information of current thread.
*
* Description:
*    This function locks by T-Lock.
*    If NULL was returned, This function does not lock.
*    "R_OSPL_UnlockCurrentThreadError" unlocks.
*    Use "IF_C" and "ASSERT_RC" macro instead of "IF", "ASSERT_R" macro,
*    for locking the debug information.
************************************************************************/
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
r_ospl_error_t*  R_OSPL_LockCurrentThreadError(void);
#endif


/***********************************************************************
* Function: R_OSPL_UnlockCurrentThreadError
*    Unlock (T-Lock) debbug information of current thread.
*
* Arguments:
*    *in_out_Error - (in) Locked error or NULL. (out) NULL
*
* Return Value:
*    None
*
* Description:
*    This unlocks, only if locked error was specified.
************************************************************************/
void  R_OSPL_UnlockCurrentThreadError( r_ospl_error_t** in_out_Error );


/***********************************************************************
* Function: R_OSPL_MODIFY_THREAD_LOCKED_COUNT
*    Modifies count of objects that current thread has locked.
*
* Arguments:
*    Plus - The value of adding to the counter.
*
* Return Value:
*    None
*
* Description:
*    The counter is subtracted, if this argument was minus.
*
*    Drivers calls this function.
*    This function is not called from OSPL.
*    This function does nothing, if "R_OSPL_ERROR_BREAK" macro is 0.
************************************************************************/
#if R_OSPL_ERROR_BREAK
#if R_OSPL_IS_PREEMPTION
void         R_OSPL_MODIFY_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread,  int_fast32_t in_Plus );
#else
INLINE void  R_OSPL_MODIFY_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread,  int_fast32_t in_Plus )
{
    R_UNREFERENCED_VARIABLE_2( in_Thread, in_Plus );
}
#endif
#else
INLINE void  R_OSPL_MODIFY_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread,  int_fast32_t in_Plus )
{
    R_UNREFERENCED_VARIABLE_2( in_Thread, in_Plus );
}
#endif


/***********************************************************************
* Function: R_OSPL_GET_THREAD_LOCKED_COUNT
*    Returns count of objects that current thread has locked.
*
* Arguments:
*    None
*
* Return Value:
*    Count of objects that current thread has locked
*
* Description:
*    This function returns 0, if "R_OSPL_ERROR_BREAK" macro is 0.
************************************************************************/
#if R_OSPL_ERROR_BREAK
#if R_OSPL_IS_PREEMPTION
int_fast32_t  R_OSPL_GET_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread );
#else
INLINE int_fast32_t  R_OSPL_GET_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread )
{
    R_UNREFERENCED_VARIABLE( in_Thread );
    return 0;
}
#endif
#else
INLINE int_fast32_t  R_OSPL_GET_THREAD_LOCKED_COUNT( r_ospl_thread_id_t  in_Thread )
{
    R_UNREFERENCED_VARIABLE( in_Thread );
    return 0;
}
#endif


/* Section: Accessing to register bit field */
/***********************************************************************
* Function: R_OSPL_SET_TO_32_BIT_REGISTER
*    Reads modifies writes for bit field of 32bit register.
*
* Arguments:
*    in_out_Register - Address of accessing register
*    in_Mask         - Mask of accessing bit field
*    in_Shift        - Shift count. Lowest bit number
*    in_Value        - Writing value before shift to the bit field
*
* Return Value:
*    None
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at left of "=" and right of "=". But this is not depend on compiler spcifications. */
/* ->SEC M1.2.2(QAC-1259) */
/*    If "Value" is signed, this is depend on CPU bit width. This expects 32bit CPU. But driver code is no problem. */

#define  R_OSPL_SET_TO_32_BIT_REGISTER( in_out_Register, in_Mask, in_Shift, in_Value ) \
	( *(volatile uint32_t*)(in_out_Register) = (uint32_t)( \
		( ((uint32_t) *(volatile uint32_t*)(in_out_Register)) & \
		~(in_Mask) ) | ( (in_Mask) & ( ( (uint_fast32_t)(in_Value) << (in_Shift) ) & (in_Mask) ) ) ) )
/* This code is optimized well. */

/* <-SEC M1.2.2(QAC-1259) */
/* <-SEC R3.6.2(QAC-3345) */

#else

INLINE void  R_OSPL_SET_TO_32_BIT_REGISTER( volatile uint32_t* const  in_Register,
        uint32_t const  in_Mask,  int_fast32_t const  in_Shift,  uint32_t const  in_Value );

#endif


/***********************************************************************
* Function: R_OSPL_SET_TO_16_BIT_REGISTER
*    Reads modifies writes for bit field of 16bit register.
*
* Arguments:
*    in_out_Register - Address of accessing register
*    in_Mask         - Mask of accessing bit field
*    in_Shift        - Shift count. Lowest bit number
*    in_Value        - Writing value before shift to the bit field
*
* Return Value:
*    None
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at left of "=" and right of "=". But this is not depend on compiler spcifications. */
/* ->SEC M1.2.2(QAC-1259) */
/*    If "Value" is signed, this is depend on CPU bit width. This expects 32bit CPU. But driver code is no problem. */

#define  R_OSPL_SET_TO_16_BIT_REGISTER( in_out_Register, in_Mask, in_Shift, in_Value ) \
	( *(volatile uint16_t*)(in_out_Register) = (uint16_t)( \
		( ((uint16_t) *(volatile uint16_t*)(in_out_Register)) & \
		~(in_Mask) ) | ( (in_Mask) & ( ( (uint_fast16_t)(in_Value) << (in_Shift) ) & (in_Mask) ) ) ) )
/* This code is optimized well. */


/* <-SEC M1.2.2(QAC-1259) */
/* <-SEC R3.6.2(QAC-3345) */

#else

INLINE void  R_OSPL_SET_TO_16_BIT_REGISTER( volatile uint16_t* const  in_Register,
        uint16_t const  in_Mask,  int_fast32_t const  in_Shift,  uint16_t const  in_Value );

#endif


/***********************************************************************
* Function: R_OSPL_SET_TO_8_BIT_REGISTER
*    Reads modifies writes for bit field of 8bit register.
*
* Arguments:
*    in_out_Register - Address of accessing register
*    in_Mask         - Mask of accessing bit field
*    in_Shift        - Shift count. Lowest bit number
*    in_Value        - Writing value before shift to the bit field
*
* Return Value:
*    None
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at left of "=" and right of "=". But this is not depend on compiler spcifications. */
/* ->SEC M1.2.2(QAC-1259) */
/*    If "Value" is signed, this is depend on CPU bit width. This expects 32bit CPU. But driver code is no problem. */


#define  R_OSPL_SET_TO_8_BIT_REGISTER( in_out_Register, in_Mask, in_Shift, in_Value ) \
	( *(volatile uint8_t*)(in_out_Register) = (uint8_t)( \
		( ((uint8_t) *(volatile uint8_t*)(in_out_Register)) & \
		~(in_Mask) ) | ( (in_Mask) & ( ( (uint_fast8_t)(in_Value) << (in_Shift) ) & (in_Mask) ) ) ) )
/* This code is optimized well. */

/* <-SEC M1.2.2(QAC-1259) */
/* <-SEC R3.6.2(QAC-3345) */

#else

INLINE void  R_OSPL_SET_TO_8_BIT_REGISTER( volatile uint8_t* const  in_Register,
        uint8_t const  in_Mask,  int_fast32_t const  in_Shift,  uint8_t const  in_Value );

#endif


/***********************************************************************
* Function: R_OSPL_GET_FROM_32_BIT_REGISTER
*    Reads for bit field of 32bit register.
*
* Arguments:
*    in_RegisterValueAddress - Address of accessing register
*    in_Mask                 - Mask of accessing bit field
*    in_Shift                - Shift count. Lowest bit number
*
* Return Value:
*    Read value after shift
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at &(get address), cast and *(memory load). But this is not double volatile access. */
/*    RegisterValueAddress is for avoid QAC-0310,QAC-3345 by cast code at caller. */

#define  R_OSPL_GET_FROM_32_BIT_REGISTER( in_RegisterValueAddress, in_Mask, in_Shift ) \
	( (uint32_t)( ( (uint32_t)*(volatile const uint32_t*) (in_RegisterValueAddress) \
		&  (uint_fast32_t)(in_Mask) ) >> (in_Shift) ) )
/* This code is optimized well. */

/* <-SEC R3.6.2(QAC-3345) */

#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

/* This inline functions is not expanded on __CC_ARM 5.15 */
INLINE uint32_t  R_OSPL_GET_FROM_32_BIT_REGISTER( volatile const uint32_t* const  in_RegisterAddress,
        uint32_t const  in_Mask,  int_fast32_t const  in_Shift );

#endif


/***********************************************************************
* Function: R_OSPL_GET_FROM_16_BIT_REGISTER
*    Reads for bit field of 16bit register.
*
* Arguments:
*    in_RegisterValueAddress - Address of accessing register
*    in_Mask                 - Mask of accessing bit field
*    in_Shift                - Shift count. Lowest bit number
*
* Return Value:
*    Read value after shift
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at &(get address), cast and *(memory load). But this is not double volatile access. */
/*    RegisterValueAddress is for avoid QAC-0310,QAC-3345 by cast code at caller. */

#define  R_OSPL_GET_FROM_16_BIT_REGISTER( in_RegisterValueAddress, in_Mask, in_Shift ) \
	( (uint16_t)( ( (uint_fast32_t)*(volatile const uint16_t*) (in_RegisterValueAddress) \
		&  (uint_fast16_t)(in_Mask) ) >> (in_Shift) ) )
/* This code is optimized well. */

/* <-SEC R3.6.2(QAC-3345) */

#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

/* This inline functions is not expanded on __CC_ARM 5.15 */
INLINE uint16_t  R_OSPL_GET_FROM_16_BIT_REGISTER( volatile const uint16_t* const  RegisterAddress,
        uint16_t const  Mask,  int_fast32_t const  Shift );

#endif


/***********************************************************************
* Function: R_OSPL_GET_FROM_8_BIT_REGISTER
*    Reads for bit field of 8bit register.
*
* Arguments:
*    in_RegisterValueAddress - Address of accessing register
*    in_Mask                 - Mask of accessing bit field
*    in_Shift                - Shift count. Lowest bit number
*
* Return Value:
*    Read value after shift
************************************************************************/
#if R_OSPL_BIT_FIELD_ACCESS_MACRO

/* ->SEC R3.6.2(QAC-3345) */
/*    Volatile access at &(get address), cast and *(memory load). But this is not double volatile access. */
/*    RegisterValueAddress is for avoid QAC-0310,QAC-3345 by cast code at caller. */

#define  R_OSPL_GET_FROM_8_BIT_REGISTER( in_RegisterValueAddress, in_Mask, in_Shift ) \
	( (uint8_t)( ( (uint_fast32_t)*(volatile const uint8_t*) (in_RegisterValueAddress) \
		&  (uint_fast8_t)(in_Mask) ) >> (in_Shift) ) )
/* This code is optimized well. */

/* <-SEC R3.6.2(QAC-3345) */

#else  /* __QAC_ARM_H__ */  /* This code must be tested defined "__QAC_ARM_H__" */

/* This inline functions is not expanded on __CC_ARM 5.15 */
INLINE uint8_t  R_OSPL_GET_FROM_8_BIT_REGISTER( volatile const uint8_t* const  in_RegisterAddress,
        uint8_t const  in_Mask,  int_fast32_t const  in_Shift );

#endif


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

/* Inline Functions */
#include  "r_ospl_inline.h"

#endif /* R_OSPL_H */
