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
* File: r_ospl_typedef.h
*    OS Porting Layer. Data types.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef R_OSPL_TYPEDEF_H
#define R_OSPL_TYPEDEF_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#if  defined( __GNUC__ )  ||  defined( _SH )
#include  <stdint.h>
#include  <stdbool.h>
#endif
#include  "Project_Config.h"
#include  "r_ospl_typedef_1.h"
#include  "r_typedefs.h"
#include  "locking_typedef.h"
#include  "mcu_interrupts_typedef.h"
#include  "r_ospl_os_less_typedef.h"  /* Common API */
#if  IS_ITRON_USED
//#include  "kernel.h"
//#include  "itron.h"
#include "t_stddef.h"
#elif  IS_RTX_USED
#include  "r_ospl_RTX_typedef.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

typedef struct st_r_ospl_async_t          r_ospl_async_t;
typedef struct st_r_ospl_async_status_t   r_ospl_async_status_t;
typedef struct st_r_ospl_flag32_t         r_ospl_flag32_t;
typedef struct st_r_ospl_interrupt_t      r_ospl_interrupt_t;
typedef struct st_r_ospl_irq_mask_t       r_ospl_irq_mask_t;
typedef struct st_r_ospl_memory_spec_t    r_ospl_memory_spec_t;
typedef struct st_r_ospl_ftimer_spec_t    r_ospl_ftimer_spec_t;
typedef struct st_r_ospl_table_t          r_ospl_table_t;
typedef struct st_r_ospl_i_lock_vtable_t  r_ospl_i_lock_vtable_t;
typedef struct st_r_ospl_caller_t         r_ospl_caller_t;


/***********************************************************************
* Structure: r_ospl_thread_id_t
*    Pointer to a thread.
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION
typedef r_ospl_thread_def_t*  r_ospl_thread_id_t;
#else
#if  IS_RTX_USED
typedef void*  r_ospl_thread_id_t;
#define  R_OSPL_THREAD_INTERRUPT  ((void*) 0 )
#elif  IS_ITRON_USED
typedef  ID  r_ospl_thread_id_t;
#else
#error
#endif
#endif


/***********************************************************************
* Structure: r_ospl_event_object_id_t
*    Pointer to an event object.
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION
typedef r_ospl_event_object_def_t*  r_ospl_event_object_id_t;
#else
#if  IS_RTX_USED
typedef void*  r_ospl_event_object_id_t;
#elif  IS_ITRON_USED
typedef  ID  r_ospl_event_object_id_t;
#else
#error
#endif
#endif


/* Section: Global */
/***********************************************************************
* Function: r_ospl_callback_t
*    The function type of interrupt callback
*
* Arguments:
*    InterruptSource - Source of interrupt
*    Caller          - Driver's internal parameters about interrupt operations
*
* Return Value:
*    Error code.  If there is no error, the return value is 0. The value set to "r_ospl_async_t::ReturnValue"
*
* Description:
*    This is type of the interrupt callback function running in the
*    interrupt context and called from the interrupt handler.
*
*    It is possible to replace to application defined interrupt callback function
*    by setting to "r_ospl_async_t::InterruptCallback". But it is usually not
*    necessary to replace.
*
*    As interrupt callback function, the default interrupt callback function
*    provided from the driver is used. It is unusual to use application defined
*    interrupt callback function.
*    Write the response code of the interrupt (event driven code) next to the
*    code calling <R_OSPL_EVENT_Wait> function
*
*    Whether the asynchronous operation was ended is possible to know whether
*    the variable of "r_ospl_async_state_t" type referred from
*    <R_DRIVER_GetAsyncStatus> function is set to <R_OSPL_RUNNABLE> value.
*
*    It is not necessary to write the code of interrupt return (IRET) in
*    the interrupt callback function. The interrupt handlers calling interrupt
*    callback function calls IRET, if necessary.
*
*    It is not possible to divide interrupt callback functions by the kind of
*    interrupt. Alternatively, it is possible to write operations in interrupt
*    handlers calling interrupt callback function in porting layer under the driver.
*    There are interrupt handlers by each interrupt numbers.
*
*    It is possible to signal any event from application defined interrupt
*    callback function. But it is necessary to do following operations:
************************************************************************/
typedef errnum_t  (* r_ospl_callback_t )( const r_ospl_interrupt_t* InterruptSource, const r_ospl_caller_t* Caller );


/***********************************************************************
* Enumeration: r_ospl_async_state_t
*    Asynchronous State
*
*    : R_OSPL_UNINITIALIZED - 0
*    : R_OSPL_RUNNABLE      - 1, Runnable or Waitable
*    : R_OSPL_RUNNING       - 2, Running  or Waiting
*    : R_OSPL_INTERRUPTING  - 3
*    : R_OSPL_INTERRUPTED   - 4
************************************************************************/
typedef enum
{
    R_OSPL_UNINITIALIZED = 0,
    R_OSPL_RUNNABLE,
    R_OSPL_RUNNING,
    R_OSPL_INTERRUPTING,
    R_OSPL_INTERRUPTED
} r_ospl_async_state_t;


/***********************************************************************
* Enumeration: r_ospl_async_type_t
*    Asynchronous State
*
*    : R_OSPL_ASYNC_TYPE_NORMAL   - 1
*    : R_OSPL_ASYNC_TYPE_FINALIZE - 2, Asynchronous finalizing
************************************************************************/
typedef enum
{
    R_OSPL_ASYNC_TYPE_NORMAL   = 1,
    R_OSPL_ASYNC_TYPE_FINALIZE = 2
} r_ospl_async_type_t;


/***********************************************************************
* Constant: R_OSPL_EVENT_BIT_COUNT
************************************************************************/
#if R_OSPL_DETECT_BAD_EVENT
#define  R_OSPL_EVENT_BIT_COUNT  16
#endif


/***********************************************************************
* Constant: R_OSPL_EVENT_OBJECT_FIRST_ID
************************************************************************/
#if R_OSPL_DETECT_BAD_EVENT
enum { R_OSPL_EVENT_OBJECT_FIRST_ID = R_OSPL_EVENT_BIT_COUNT };
#endif


/***********************************************************************
* Constant: R_OSPL_EVENT_OBJECT_COUNT
************************************************************************/
#if R_OSPL_DETECT_BAD_EVENT
#define  R_OSPL_EVENT_OBJECT_COUNT  1
#endif


/***********************************************************************
* Type: r_ospl_event_flags_t
*    Event Flag's Value or Event Object's ID.
************************************************************************/
typedef  uint32_t  r_ospl_event_flags_t;


/***********************************************************************
* Enumeration: r_ospl_table_flags_t
*    Size of this type is same as <bit_flags_fast32_t>.
*
*    : R_OSPL_TABLE_T_LOCK         - 0x0001. Enables T-Lock
*    : R_OSPL_TABLE_I_LOCK         - 0x0002. Enables I-Lock
*    : R_OSPL_TABLE_IS_IN_CRITICAL - 0x0004. Internal bit
*    : R_OSPL_TABLE_IS_KEY_SORTED  - 0x0008. Internal bit
************************************************************************/
typedef  bit_flags_fast32_t  r_ospl_table_flags_t;
#define  R_OSPL_TABLE_T_LOCK          0x0001u
#define  R_OSPL_TABLE_I_LOCK          0x0002u
#define  R_OSPL_TABLE_IS_IN_CRITICAL  0x0004u
#define  R_OSPL_TABLE_IS_KEY_SORTED   0x0008u


/***********************************************************************
* Structure: r_ospl_table_block_t
*    Block of <r_ospl_table_t>
************************************************************************/
typedef struct st_r_ospl_table_block_t  r_ospl_table_block_t;
struct st_r_ospl_table_block_t
{

    /* Variable: Key
    	this[ SortedKeyIndex ].Key ... */
    const void*  Key;

    /* Variable: Index */
    int16_t  Index;

    /* Variable: NextFreeIndex
    	this[ Index ].NextFreeIndex  : Index is not same as SortedKeyIndex
    	or R_OSPL_TABLE_BLOCK_USED */
    int16_t  NextFreeIndex;
};
enum
{
    R_OSPL_TABLE_BLOCK_USED      = -1,
    R_OSPL_TABLE_BLOCK_NO_NEXT   = -2
};


/***********************************************************************
* Structure: r_ospl_table_t
*    Index table
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
#if  ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_table_t
{
    void*         Area;
    int_fast32_t  Count;
    int_fast32_t  MaxCount;
    const void*   KeyCache;
    int_fast32_t  IndexCache;  /* Not SortedKeyIndex */
    int32_t       FirstFreeIndex;

    volatile r_ospl_table_flags_t  TableFlags;
};
#endif


/***********************************************************************
* Constant: R_OSPL_TABLE_NOT_INITIALIZED
************************************************************************/
enum { R_OSPL_TABLE_NOT_INITIALIZED = -2 };


/***********************************************************************
* Macro: R_OSPL_TABLE_DEF
*    Defines attributes and work area of <r_ospl_table_t>.
************************************************************************/
#if  ! R_OSPL_LIBRARY_MAKING
#define  R_OSPL_TABLE_DEF( Name, MaxCount, TableFlags ) \
		uint8_t             Name##_OSPL_DEF_MemoryArray[ (MaxCount) * sizeof(r_ospl_table_block_t) ]; \
		r_ospl_table_t      Name##_OSPL_DEF = { \
			Name##_OSPL_DEF_MemoryArray,  /* .Area */ \
			0,         /* .Count */ \
			MaxCount,  /* .MaxCount */ \
			NULL,      /* .KeyCache */ \
			0,         /* .IndexCache */ \
			R_OSPL_TABLE_NOT_INITIALIZED,  /* .FirstFreeIndex */ \
			TableFlags /* .TableFlags */ \
		}
#endif


/***********************************************************************
* Macro: R_OSPL_TABLE
*    Returns initial attributes and work area of queue.
*
* Arguments:
*    Name     - Table's name. Do not bracket by ""
*
* Description:
*    It is not possible to use this macro in the library.
*    If implement of OSPL was changed, the library must be recompiled.
************************************************************************/
#if  ! R_OSPL_LIBRARY_MAKING
#define  R_OSPL_TABLE( Name )  (&Name##_OSPL_DEF)
#endif


/***********************************************************************
* Structure: r_ospl_block_t
*    Block in queue
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION
typedef struct st_r_ospl_block_t  r_ospl_block_t;
#if ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_block_t
{
    r_ospl_block_t*  Next;
};
#endif
#endif


/***********************************************************************
* Structure: r_ospl_queue_status_t
************************************************************************/
typedef struct st_r_ospl_queue_status_t  r_ospl_queue_status_t;
struct st_r_ospl_queue_status_t
{

    /* Variable: UsedCount */
    int_fast32_t  UsedCount;

    /* Variable: MaxCount */
    int_fast32_t  MaxCount;
};


/***********************************************************************
* Structure: r_ospl_queue_def_t
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION

typedef struct st_r_ospl_queue_def_t  r_ospl_queue_def_t;
#if ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_queue_def_t
{
    uint8_t*               MemoryArray;  /* [index_of_block][ElementSize] */
    size_t                 ElementSize;  /*(byte)*/
    r_ospl_block_t*        BlockArray;   /* [index_of_block] */
    r_ospl_block_t*        NotUsedBlocksList;
    r_ospl_block_t*        PutBlocksFirst; /* Get last */
    r_ospl_block_t*        PutBlocksLast;  /* Get first, "Next" is last2 (for first) */
    r_ospl_queue_status_t  PublicStatus;
};
#endif

#else
#if  IS_RTX_USED

typedef struct st_r_ospl_queue_def_t  r_ospl_queue_def_t;
#if ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_queue_def_t
{
    osMailQId                       MailQId;
    r_ospl_queue_status_t           PublicStatus;
    const r_ospl_rtx_osMailQDef_t*  MailQDef;
};
#endif

#elif  IS_ITRON_USED

typedef struct st_r_ospl_queue_def_t  r_ospl_queue_def_t;
#if ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_queue_def_t
{
    uint8_t*               MemoryArrayForPool;   /* TSZ_MPF */
    size_t                 ElementSize;  /*(byte)*/
    ID                     MemoryPool;
#if  R_OSPL_QUEUE_IMPLEMENT == R_OSPL_QUEUE_IMPLEMENT_DATA_QUEUE
    ID                     DataQueue;
#elif  R_OSPL_QUEUE_IMPLEMENT == R_OSPL_QUEUE_IMPLEMENT_MAIL_BOX
    ID                     MailBox;
#endif
    /* HiOS is is not supported receiving from data queue in interrupt. */
    r_ospl_queue_status_t  PublicStatus;
};
#endif

#else
#error
#endif
#endif


/***********************************************************************
* Structure: r_ospl_queue_id_t
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION

typedef struct st_r_ospl_queue_def_t*  r_ospl_queue_id_t;

#else

typedef r_ospl_queue_def_t*  r_ospl_queue_id_t;

#endif


/* Section: Global */
/***********************************************************************
* Macro: R_OSPL_QUEUE_DEF
*    Defines attributes and work area of queue.
*
* Arguments:
*    Name     - Thread name. Do not bracket by ""
*    MaxCount - Max count of elements in the queue
*    Type     - Output
*
* Description:
*    It is not possible to use this macro in the library.
*    If implement of OSPL was changed, the library must be recompiled.
************************************************************************/
#if  ! R_OSPL_LIBRARY_MAKING
#if  ! R_OSPL_IS_PREEMPTION

#define  R_OSPL_QUEUE_DEF( Name, MaxCount, Type ) \
	uint8_t             Name##_OSPL_DEF_MemoryArray[ MaxCount * sizeof(Type) ]; \
	r_ospl_block_t      Name##_OSPL_DEF_BlockArray[ MaxCount ]; \
	r_ospl_queue_def_t  Name##_OSPL_DEF = { \
		Name##_OSPL_DEF_MemoryArray, sizeof(Type), \
		Name##_OSPL_DEF_BlockArray, NULL, NULL, NULL, { 0, MaxCount } }

#else
#if  IS_RTX_USED

#define  R_OSPL_QUEUE_DEF( Name, MaxCount, Type ) \
	r_ospl_rtx_osMailQDef( Name, MaxCount, Type ); \
	r_ospl_queue_def_t  Name##_OSPL_DEF = { NULL, 0, (MaxCount), osMailQ( Name ) }

#elif  IS_ITRON_USED
#if  R_OSPL_QUEUE_IMPLEMENT == R_OSPL_QUEUE_IMPLEMENT_DATA_QUEUE

#define  R_OSPL_QUEUE_DEF( Name, MaxCount, Type ) \
	uint8_t             Name##_OSPL_DEF_MemoryArrayForPool[ TSZ_MPF( MaxCount, sizeof(Type) ) ]; \
	r_ospl_queue_def_t  Name##_OSPL_DEF = { \
		Name##_OSPL_DEF_MemoryArrayForPool, \
		sizeof(Type), 0, 0, { 0, MaxCount } }
/* HiOS is not supported TSZ_DTQ */

#else

#define  R_OSPL_QUEUE_DEF( Name, MaxCount, Type ) \
	uint8_t             Name##_OSPL_DEF_MemoryArrayForPool[ TSZ_MPF( MaxCount, sizeof(Type) + sizeof(T_MSG) ) ]; \
	r_ospl_queue_def_t  Name##_OSPL_DEF = { \
		Name##_OSPL_DEF_MemoryArrayForPool, \
		sizeof(Type), 0, 0, { 0, MaxCount } }
/* HiOS is not supported  TSZ_MPRIHD */

#endif

#else
#error
#endif

#endif
#endif


/***********************************************************************
* Macro: R_OSPL_QUEUE
*    Returns initial attributes of queue and work area.
*
* Arguments:
*    Name     - Queue's name. Do not bracket by ""
*
* Description:
*    It is not possible to use this macro in the library.
*    If implement of OSPL was changed, the library must be recompiled.
************************************************************************/
#if  ! R_OSPL_LIBRARY_MAKING
#define  R_OSPL_QUEUE( Name )  (&Name##_OSPL_DEF)
#endif


/***********************************************************************
* Enumeration: r_ospl_flush_t
*    Flush operations
*
*    : R_OSPL_FLUSH_INVALIDATE               - 0
*    : R_OSPL_FLUSH_WRITEBACK_INVALIDATE     - 2
*    : R_OSPL_FLUSH_WRITEBACK_INVALIDATE_2ND - 8
************************************************************************/
typedef bit_flags_fast32_t  r_ospl_flush_t;
#define  /*<uint_fast32_t>*/  R_OSPL_FLUSH_INVALIDATE                0u
#define  /*<uint_fast32_t>*/  R_OSPL_FLUSH_WRITEBACK_INVALIDATE      2u
#define  /*<uint_fast32_t>*/  R_OSPL_FLUSH_WRITEBACK_INVALIDATE_2ND  8u


/***********************************************************************
* Structure: r_ospl_flag32_t
*    This is the type of flags having 32bit
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
struct st_r_ospl_flag32_t
{
    volatile uint32_t  Flags;
};


/***********************************************************************
* Structure: r_ospl_interrupt_t
*    Structure related to interrupt source. e.g. interrupt number
************************************************************************/
struct st_r_ospl_interrupt_t
{

    /* Variable: IRQ_Num
    	<bsp_int_src_t> */
    bsp_int_src_t  IRQ_Num;

    /* Variable: ChannelNum */
    int_fast32_t   ChannelNum;

    /* Variable: Type */
    int_fast32_t   Type;

    /* Variable: Delegate */
    void*  Delegate;
};


/***********************************************************************
* Structure: r_ospl_async_status_t
*    Structure of driver's status and interrupt status defined by OSPL
************************************************************************/
struct st_r_ospl_async_status_t
{

    /* Variable: State
    	<r_ospl_async_state_t> */
    volatile r_ospl_async_state_t  State;

    /* Variable: IsEnabledInterrupt */
    volatile bool_t  IsEnabledInterrupt;

    /* Variable: InterruptEnables */
    volatile r_ospl_flag32_t  InterruptEnables;

    /* Variable: InterruptFlags */
    volatile r_ospl_flag32_t  InterruptFlags;

    /* Variable: CancelFlags
    	r_ospl_flag32_t < <r_ospl_cancel_flag_t> > */
    volatile r_ospl_flag32_t  CancelFlags;  /*<r_ospl_cancel_flag_t>*/
#if  R_OSPL_IS_PREEMPTION
    union
    {

        /* Variable: LockOwner.Thread */
        volatile r_ospl_thread_id_t  Thread;

        /* Variable: LockOwner.Context */
        volatile void*  Context;
    } LockOwner;
#endif
};


/***********************************************************************
* Structure: r_ospl_async_t
*    Setting of notifications
************************************************************************/
struct st_r_ospl_async_t
{

    /* Variable: Flags */
    bit_flags_fast32_t  Flags;

    /* Variable: Delegate */
    void*  Delegate;

    /* Variable: A_Thread */
    r_ospl_thread_id_t  A_Thread;

    /* Variable: A_EventValue */
    r_ospl_event_flags_t  A_EventValue;  /* QAC 4130 */

    /* Variable: I_Thread */
    r_ospl_thread_id_t  I_Thread;

    /* Variable: I_EventValue */
    r_ospl_event_flags_t  I_EventValue;  /* QAC 4130 */

    /* Variable: InterruptCallback */
    r_ospl_callback_t  InterruptCallback;

    /* Variable: ReturnValue */
    errnum_t  ReturnValue;
};

enum
{
    /* Constant: R_F_OSPL_A_Thread */
    R_F_OSPL_A_Thread          = 0x0001,

    /* Constant: R_F_OSPL_A_EventValue */
    R_F_OSPL_A_EventValue      = 0x0002,

    /* Constant: R_F_OSPL_I_Thread */
    R_F_OSPL_I_Thread          = 0x0004,

    /* Constant: R_F_OSPL_I_EventValue */
    R_F_OSPL_I_EventValue      = 0x0008,

    /* Constant: R_F_OSPL_InterruptCallback */
    R_F_OSPL_InterruptCallback = 0x0010,

    /* Constant: R_F_OSPL_Delegate */
    R_F_OSPL_Delegate          = 0x0080,


    /* Constant: R_F_OSPL_AsynchronousPreset */
    R_F_OSPL_AsynchronousPreset = 0x0100,

    /* Constant: R_F_OSPL_MaskOfPreset */
    R_F_OSPL_MaskOfPreset       = 0x0F00
};


/* Section: Global */
/***********************************************************************
* Macros: r_ospl_internal_sentinel
*
*    : R_F_OSPL_ASYNC_FLAGS_SENTINEL_MASK  - 0x7FFFFF40
*    : R_F_OSPL_ASYNC_FLAGS_SENTINEL_VALUE - 0x4A5C0000
*    : R_F_OSPL_STACK_CHECK_SENTINEL_VALUE - 0x57AC512E
************************************************************************/
#ifndef R_OSPL_NDEBUG
enum
{
    R_F_OSPL_ASYNC_FLAGS_SENTINEL_MASK  = 0x7FFFFF40,
    R_F_OSPL_ASYNC_FLAGS_SENTINEL_VALUE = 0x4A5C0000,
    R_F_OSPL_STACK_CHECK_SENTINEL_VALUE = 0x57AC512E
};
#endif


/***********************************************************************
* Structure: r_ospl_caller_t
*    Context of interrupt callback function caller
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
struct st_r_ospl_caller_t
{
    r_ospl_async_t*                Async;
    volatile int_fast32_t*         PointerToState;
    int_fast32_t                   StateValueOfOnInterrupting;
    void*                          I_Lock;
    const r_ospl_i_lock_vtable_t*  I_LockVTable;
};


/***********************************************************************
* Structure: r_ospl_i_lock_vtable_t
*    I-Lock V-Table
************************************************************************/
typedef bool_t (* r_ospl_i_lock_lock_func_t )( void* const  self_ );
typedef void   (* r_ospl_i_lock_unlock_func_t )( void* const  self_ );
typedef void   (* r_ospl_i_lock_get_rf_func_t )( void* const  self_ );    /* rf = RequestFinalize */
typedef bool_t (* r_ospl_i_lock_get_rcn_func_t )( const void* const  self_ );   /* rcn = RootChannelNum */

struct st_r_ospl_i_lock_vtable_t
{

    /* Function: Lock */
    bool_t  (* Lock )( void* const  self_ );

    /* Function: Unlock */
    void    (* Unlock )( void* const  self_ );

    /* Function: RequestFinalize */
    void    (* RequestFinalize )( void* const  self_ );

    /* Function: GetRootChannelNum */
    int_fast32_t  (* GetRootChannelNum )( const void* const  self_ );
};


/***********************************************************************
* Structure: r_ospl_memory_spec_t
*    Memory specification
************************************************************************/
struct st_r_ospl_memory_spec_t
{

    /* Variable: CacheLineSize
    	(byte) */
    uint_fast32_t  CacheLineSize;
};


/***********************************************************************
* Structure: r_ospl_ftimer_spec_t
*    Free run timer specification
************************************************************************/
struct st_r_ospl_ftimer_spec_t
{

    /* Variable: msec_Numerator */
    uint32_t  msec_Numerator;

    /* Variable: msec_Denominator */
    uint32_t  msec_Denominator;

    /* Variable: MaxCount */
    uint32_t  MaxCount;

    /* Variable: ExtensionOfCount */
    uint32_t  ExtensionOfCount;
};


/* Section: Global */
/***********************************************************************
* Constant: R_OSPL_THREAD_NULL
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION
#define  R_OSPL_THREAD_NULL       NULL
#else
#if  IS_RTX_USED
#define  R_OSPL_THREAD_NULL       NULL
#elif  IS_ITRON_USED
#define  R_OSPL_THREAD_NULL       ((ID) 0 )
#else
#error
#endif
#endif


/* Section: Global */
/***********************************************************************
* Constant: R_OSPL_EVENT_OBJECT_NULL
************************************************************************/
#if  ! R_OSPL_IS_PREEMPTION
#define  R_OSPL_EVENT_OBJECT_NULL       NULL
#else
#if  IS_RTX_USED
#define  R_OSPL_EVENT_OBJECT_NULL       NULL
#elif  IS_ITRON_USED
#define  R_OSPL_EVENT_OBJECT_NULL       ((ID) 0 )
#else
#error
#endif
#endif


/***********************************************************************
* Enumeration: r_ospl_if_not_t
*    Operation if not exists
*
*    : R_OSPL_ERROR_IF_NOT                - 0
*    : R_OSPL_ALLOCATE_IF_NOT             - 1
*    : R_OSPL_DO_NOTHING_IF_NOT           - 2
*    : R_OSPL_OUTPUT_IF_NOT               - 3
*    : R_OSPL_ALLOCATE_IF_EXIST_OR_IF_NOT - 4
************************************************************************/
typedef enum
{
    R_OSPL_ERROR_IF_NOT                = 0,
    R_OSPL_ALLOCATE_IF_NOT             = 1,
    R_OSPL_DO_NOTHING_IF_NOT           = 2,
    R_OSPL_OUTPUT_IF_NOT               = 3,
    R_OSPL_ALLOCATE_IF_EXIST_OR_IF_NOT = 4
} r_ospl_if_not_t;


/***********************************************************************
* Structure: r_ospl_event_status_t
************************************************************************/
#if  R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_DETECT_BAD_EVENT
typedef  struct st_r_ospl_event_status_t  r_ospl_event_status_t;
struct st_r_ospl_event_status_t
{
#if  R_OSPL_TLS_EVENT_CODE
    bit_flags32_t  AllocatedEvents;
#endif

#if  R_OSPL_DETECT_BAD_EVENT
    bit_flags32_t  UnexpectedEvents;
#endif
};
#endif


/***********************************************************************
* Structure: r_ospl_error_t
*    Error status and thread local storage.
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
typedef  struct st_r_ospl_error_t  r_ospl_error_t;
#if ! R_OSPL_LIBRARY_MAKING
struct st_r_ospl_error_t
{
#if  ! R_OSPL_ERROR_BREAK  &&  ! R_OSPL_TLS_ERROR_CODE  &&  ! R_OSPL_STACK_CHECK_CODE
    uint8_t  Dummy;  /* For T-Lock by "R_OSPL_LockCurrentThreadError_Sub" */
#endif
#if R_OSPL_ERROR_BREAK
    bool_t         IsError;
    int_fast32_t   ErrorID;
    const char_t*  FilePath;
#if  ! R_OSPL_IS_PREEMPTION
    int_fast32_t  LineNum;
#else
    int16_t  LineNum;
    int16_t  ThreadLockedCount;
#endif
#endif
#if R_OSPL_TLS_ERROR_CODE
    errnum_t       ErrNum;
#endif
#if R_OSPL_STACK_CHECK_CODE  &&  R_OSPL_IS_PREEMPTION
    uint32_t*      EndOfStack;
#endif
#if R_OSPL_TLS_EVENT_CODE  ||  R_OSPL_DETECT_BAD_EVENT
    r_ospl_event_status_t  EventStatus;
#endif
#if R_OSPL_DETECT_BAD_EVENT
    uint16_t       EventCheckBits[ R_OSPL_EVENT_BIT_COUNT + R_OSPL_EVENT_OBJECT_COUNT ];
    bit_flags32_t  OnceAllocatedEventFlags;
#endif
#if  R_OSPL_EVENT_OBJECT_CODE
    r_ospl_event_object_id_t  EnabledEventObject;  /* R_OSPL_EVENT_OBJECT_NULL = Disabled */
#endif
};
#endif


/***********************************************************************
* Structure: r_ospl_global_error_t
*    Error status of global
*
* Description:
*    Member variables should not be accessed.
************************************************************************/
#if ! R_OSPL_LIBRARY_MAKING
#if R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE
typedef  struct st_r_ospl_global_error_t  r_ospl_global_error_t;
struct st_r_ospl_global_error_t
{
    r_ospl_table_t*  ThreadIndexTable;
    r_ospl_error_t*  ErrorArray;

#if R_OSPL_ERROR_BREAK
    int_fast32_t     RaisedGlobalErrorID;
    int_fast32_t     BreakGlobalErrorID;
#endif
#if R_OSPL_DETECT_BAD_EVENT
    uint32_t         PreviousEventNum;
#endif
#if R_OSPL_STACK_CHECK_CODE  &&  ! R_OSPL_IS_PREEMPTION
    uint32_t*        EndOfStack;  /* It is shared by main and interrupt. */
#endif
};
#endif
#endif


/* Section: Global */
/***********************************************************************
* Enumeration: r_ospl_axi_cache_attribute_t
*    Cache attribute on AXI bus.
*
*    : R_OSPL_AXI_CACHE_ZERO     - Not AXI
*    : R_OSPL_AXI_STRONGLY       - Strongly order access
*    : R_OSPL_AXI_DEVICE         - DEVICE
*    : R_OSPL_AXI_UNCACHED       - Normal access (Out of order) uncached
*    : R_OSPL_AXI_WRITE_BACK_W   - Write back, allocate on write
*    : R_OSPL_AXI_WRITE_BACK     - Write back, allocate on both read and write
************************************************************************/
typedef enum
{
    R_OSPL_AXI_CACHE_ZERO     =  0,
    R_OSPL_AXI_STRONGLY       =  0,
    R_OSPL_AXI_DEVICE         =  1,
    R_OSPL_AXI_UNCACHED       =  3,
    R_OSPL_AXI_WRITE_BACK_W   = 11,
    R_OSPL_AXI_WRITE_BACK     = 15
} r_ospl_axi_cache_attribute_t;


/***********************************************************************
* Enumeration: r_ospl_axi_protection_t
*    Protection on AXI bus.
*
*    : R_OSPL_AXI_PROTECTION_ZERO - Not AXI
*    : R_OSPL_AXI_SECURE          - TrustZone secure acccess
*    : R_OSPL_AXI_NON_SECURE      - TrustZone non-secure acccess
************************************************************************/
typedef enum
{
    R_OSPL_AXI_PROTECTION_ZERO = 0,
    R_OSPL_AXI_SECURE          = 0,
    R_OSPL_AXI_NON_SECURE      = 2
} r_ospl_axi_protection_t;


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Enumeration: Event_Bit_Name
*
*    : R_OSPL_ANY_FLAG          - 0x00000000
*    : R_OSPL_A_FLAG            - 0x00000001
*    : R_OSPL_I_FLAG            - 0x00000002
*    : R_OSPL_FINAL_A_FLAG      - 0x00000004
*    : R_OSPL_EVENT_OBJECT_FLAG - 0x20000000
*
* Description:
*    R_OSPL_EVENT_OBJECT_FLAG is event flag's value instead of event object.
*    See <R_OSPL_EVENT_Allocate>.
************************************************************************/
#define R_OSPL_ANY_FLAG           0x00000000u
#define R_OSPL_A_FLAG             0x00000001u
#define R_OSPL_I_FLAG             0x00000002u
#define R_OSPL_FINAL_A_FLAG       0x00000004u
#define R_OSPL_EVENT_OBJECT_FLAG  0x20000000u


/***********************************************************************
* Type: r_ospl_cancel_flag_t
*    Bit flags of <r_ospl_cancel_bit_t>
************************************************************************/
typedef int_t  r_ospl_cancel_flag_t;


/***********************************************************************
* Enumeration: r_ospl_cancel_bit_t
*
*    : R_OSPL_CANNEL_REQUEST   - 0x0001
*    : R_OSPL_CANNELING        - 0x0002
*    : R_OSPL_CANNELED         - 0x0004
*    : R_OSPL_FINALIZE_REQUEST - 0x0010
*    : R_OSPL_FINALIZING       - 0x0020
*    : R_OSPL_FINALIZED        - 0x0040
************************************************************************/
#define R_OSPL_CANNEL_REQUEST     0x00000001u
#define R_OSPL_CANNELING          0x00000002u
#define R_OSPL_CANNELED           0x00000004u
#define R_OSPL_FINALIZE_REQUEST   0x00000010u
#define R_OSPL_FINALIZING         0x00000020u
#define R_OSPL_FINALIZED          0x00000040u


/***********************************************************************
* Constant: R_OSPL_EVENT_ALL_BITS
*    All event flag's bits.
************************************************************************/
#define R_OSPL_EVENT_ALL_BITS  0x0000FFFFu


/***********************************************************************
* Constant: R_OSPL_EVENT_DYNAMIC_BITS
*    Dynamic allocatable event flag's bits by <R_OSPL_UNUSED_FLAG>.
************************************************************************/
#define  R_OSPL_EVENT_DYNAMIC_BITS  0x0000FFF0u


/***********************************************************************
* Constant: R_OSPL_EVENT_ALL_BITS_COUNT
************************************************************************/
#define  R_OSPL_EVENT_ALL_BITS_COUNT  16


/***********************************************************************
* Constant: R_OSPL_EVENT_DYNAMIC_BITS_COUNT
*    Maximum allcatable count by <R_OSPL_UNUSED_FLAG>.
************************************************************************/
#define  R_OSPL_EVENT_DYNAMIC_BITS_COUNT  12


/***********************************************************************
* Constant: R_OSPL_UNUSED_FLAG
************************************************************************/
#if  R_OSPL_TLS_EVENT_CODE
#define R_OSPL_UNUSED_FLAG  0x10000000u
#endif


/***********************************************************************
* Constants: OSPL_Others
*
*    : R_OSPL_INFINITE         - One of time out value
*    : R_OSPL_FLAG32_ALL_BITS  - R_OSPL_FLAG32_ALL_BITS
*    : R_OSPL_TIMEOUT          - Raised time out
*    : R_OSPL_MAX_TIME_OUT     - Max value of time out
*    : R_OSPL_NO_INDEX         - R_OSPL_NO_INDEX
*    : R_OSPL_UNLOCKED_CHANNEL - R_OSPL_UNLOCKED_CHANNEL
************************************************************************/
#define R_OSPL_INFINITE           0xFFFFFFFFu
#define R_OSPL_FLAG32_ALL_BITS    0xFFFFFFFFu
#define R_OSPL_TIMEOUT            0x40000000u
enum {  R_OSPL_MAX_TIME_OUT     = 65533 };
enum {  R_OSPL_NO_INDEX         = -1         };
enum {  R_OSPL_UNLOCKED_CHANNEL = 0x00000FEE };


/***********************************************************************
* Macro: R_OSPL_TABLE_SIZE
*    Calculates the size of <r_ospl_table_t> type index table
*
* Arguments:
*    MaxCount - Max index count
*
* Return Value:
*    Table size
************************************************************************/
/* ->MISRA 19.7 : Array count must const */ /* ->SEC M5.1.3 */
#define  R_OSPL_TABLE_1_SIZE  8  /* sizeof(r_ospl_table_block_t) */
#define  R_OSPL_TABLE_SIZE( MaxCount ) \
	( (MaxCount) * R_OSPL_TABLE_1_SIZE )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Macro: R_OSPL_DEBUG_WORK_SIZE
*    Calculates the size of debug work area
*
* Arguments:
*    ThreadMaxCount - Max thread count using error breaking system of OSPL
*
* Return Value:
*    Size of debug work area
************************************************************************/
/* ->MISRA 19.7 : Array count must const */ /* ->SEC M5.1.3 */
#define  R_OSPL_DEBUG_WORK_1_SIZE  32  /* sizeof(r_ospl_table_block_t) + sizeof(r_ospl_error_t) */
#define  R_OSPL_DEBUG_WORK_SIZE( ThreadMaxCount ) \
	( (ThreadMaxCount) * R_OSPL_DEBUG_WORK_1_SIZE )
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */


/***********************************************************************
* Enumeration: errnum_t
*    Error code defined by OSPL
*
*    : E_OTHERS             - 0x01
*    : E_FEW_ARRAY          - 0x02
*    : E_FEW_MEMORY         - 0x03
*    : E_FIFO_OVER          - 0x04
*    : E_NOT_FOUND_SYMBOL   - 0x05
*    : E_NO_NEXT            - 0x06
*    : E_ACCESS_DENIED      - 0x07
*    : E_NOT_IMPLEMENT_YET  - 0x09
*    : E_ERRNO              - 0x0E
*    : E_LIMITATION         - 0x0F
*    : E_STATE              - 0x10
*    : E_NOT_THREAD         - 0x11
*    : E_PATH_NOT_FOUND     - 0x12
*    : E_BAD_COMMAND_ID     - 0x16
*    : E_TIME_OUT           - 0x17
*    : E_STACK_OVERFLOW     - 0x1C
*    : E_NO_DEBUG_TLS       - 0x1D
*    : E_EXIT_TEST          - 0x1E
************************************************************************/
#define  E_CATEGORY_MASK   0xFFFFFFE0u  /* E_CATEGORY_* */
enum {   E_OFFSET_MASK   = 0x0000001F };

#ifndef  E_CATEGORY_COMMON  /* Overwritable */
#define  E_CATEGORY_COMMON  E_CATEGORY_COMMON
enum { E_CATEGORY_COMMON = 0x00000000 };  /* 0x01, 0x02 .. 0x1F : Reseved */
#endif

enum { E_OTHERS             = E_CATEGORY_COMMON | 0x01 }; /*  1 */
enum { E_FEW_ARRAY          = E_CATEGORY_COMMON | 0x02 }; /*  2 */
enum { E_FEW_MEMORY         = E_CATEGORY_COMMON | 0x03 }; /*  3 */
enum { E_FIFO_OVER          = E_CATEGORY_COMMON | 0x04 }; /*  4 */
enum { E_NOT_FOUND_SYMBOL   = E_CATEGORY_COMMON | 0x05 }; /*  5 */
enum { E_NO_NEXT            = E_CATEGORY_COMMON | 0x06 }; /*  6 */
enum { E_ACCESS_DENIED      = E_CATEGORY_COMMON | 0x07 }; /*  7 */
enum { E_NOT_IMPLEMENT_YET  = E_CATEGORY_COMMON | 0x09 }; /*  9 */
enum { E_ERRNO              = E_CATEGORY_COMMON | 0x0E }; /* 14 */
enum { E_LIMITATION         = E_CATEGORY_COMMON | 0x0F }; /* 15 */
enum { E_STATE              = E_CATEGORY_COMMON | 0x10 }; /* 16 */
enum { E_NOT_THREAD         = E_CATEGORY_COMMON | 0x11 }; /* 17 */
enum { E_PATH_NOT_FOUND     = E_CATEGORY_COMMON | 0x12 }; /* 18 */
enum { E_BAD_COMMAND_ID     = E_CATEGORY_COMMON | 0x16 }; /* 22 */
enum { E_TIME_OUT           = E_CATEGORY_COMMON | 0x17 }; /* 23 */
enum { E_STACK_OVERFLOW     = E_CATEGORY_COMMON | 0x1C }; /* 28 */
enum { E_NO_DEBUG_TLS       = E_CATEGORY_COMMON | 0x1D }; /* 29 */
enum { E_EXIT_TEST          = E_CATEGORY_COMMON | 0x1E }; /* 30 */


/***********************************************************************
* Constant: nullptr
*    The value as C++11 std::nullptr_t. But not check to integer type
************************************************************************/
#define  nullptr  0


/***********************************************************************
* Macro: R_JOIN_SYMBOL_FOR_ASSERT
*    Sub routine of <R_STATIC_ASSERT>
*
* - This macro extends "x", "y"
* - This code is referenced by CERT secure coding standard PRE05-C
************************************************************************/
/* ->MISRA 19.7 : Extend macro arguments */ /* ->SEC M5.1.3 */
//#define R_JOIN_SYMBOL_FOR_ASSERT(x, y) R_JOIN_SYMBOL_FOR_ASSERT_SUB(x, y)
#define R_JOIN_SYMBOL_FOR_ASSERT(x, y)						   // Ç∆ÇËÇ†Ç¶Ç∏ÅAÇ¬Ç‘ÇµÇ∆Ç≠ tokuyama@kmg 2016.09.15
/* <-MISRA 19.7 */											   /* <-SEC M5.1.3 */
/* ->MISRA 19.13 : This is used only assertion in compiling */ /* ->SEC M5.1.2 (1) */
//#define R_JOIN_SYMBOL_FOR_ASSERT_SUB(x, y) x##y
#define R_JOIN_SYMBOL_FOR_ASSERT_SUB(x, y)  // Ç∆ÇËÇ†Ç¶Ç∏ÅAÇ¬Ç‘ÇµÇ∆Ç≠ tokuyama@kmg 2016.09.15
/* <-MISRA 19.13 */ /* <-SEC M5.1.2 (1) */


/***********************************************************************
* Macro: R_STATIC_ASSERT
*    "static_assert" for in function
*
* Description:
*    Compatible with static_assert (C++0x). But our naming rule is not match.
*    This code is referenced by CERT secure coding standard DCL03-C.
*    This macro raised QAC warning 3205. This is no problem.
************************************************************************/
/* ->SEC M1.1.1 */
//#define  R_STATIC_ASSERT( ConstantExpression, StringLiteral ) \
//	do { typedef char_t R_JOIN_SYMBOL_FOR_ASSERT( assertion_failed_t_, __LINE__ ) \
//		[(ConstantExpression) ? 1 : -1]; } while(0)
#define R_STATIC_ASSERT(ConstantExpression, StringLiteral) {} //Ç∆ÇËÇ†Ç¶Ç∏Ç¬Ç‘Ç∑ tokuyama@kmg 2016.09.15
/* If "ConstantExpression" is false, illegal array size error will be raised. */
/* <-SEC M1.1.1 */


/***********************************************************************
* Macro: R_STATIC_ASSERT_GLOBAL
*    "static_assert" for in global scope
************************************************************************/
/* ->SEC M1.1.1 */
/* ->MISRA 19.4 : There is not if statement in global */ /* ->SEC M1.8.2 */
/* ->MISRA 19.7 : Cannot function */ /* ->SEC M5.1.3 */
//#define R_STATIC_ASSERT_GLOBAL(ConstantExpression, StringLiteral)          \
//	typedef char_t R_JOIN_SYMBOL_FOR_ASSERT(assertion_failed_t_, __LINE__) \
//		[(ConstantExpression) ? 1 : -1]
#define R_STATIC_ASSERT_GLOBAL(ConstantExpression, StringLiteral) //Ç∆ÇËÇ†Ç¶Ç∏Ç¬Ç‘ÇµÇƒÇ®Ç≠ tokuyama@kmg 2016.09.15
/* If "ConstantExpression" is false, illegal array size error will be raised. */
/* <-MISRA 19.7 */ /* <-SEC M5.1.3 */
/* <-MISRA 19.4 */ /* <-SEC M1.8.2 */
/* <-SEC M1.1.1 */


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

#endif /* R_OSPL_TYPEDEF_H */
