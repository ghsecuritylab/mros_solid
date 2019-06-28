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
* File: r_ospl_RTX.c
*    OS Porting Layer API for RTX
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "r_ospl_os_less_private.h"
#include  "r_ospl_private.h"
//#include  "pl310.h"  /* 2nd cache */ //tokuyama@kmg 2016.09.13
#if R_OSPL_IS_PREEMPTION
//#include  "cmsis_os.h"
//#include  "gic.h"
#include  "r_ospl_RTOS_private.h"
#include "kernel.h" // tokuyama@kmg 2016.09.13
#endif
#include <syslog.h>
#ifdef DEBUG_OSPL
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

#include "solid_mem.h"

#define	UNUSED_PARAM(param)             (void)(param)

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Macro: OS_ERROR_SIGNAL
*    CMSIS-RTOS defined immediate value
************************************************************************/
#define  OS_ERROR_SIGNAL  0x80000000


/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/


/***********************************************************************
* Implement: R_OSPL_InitializeIfNot
************************************************************************/
errnum_t  R_OSPL_InitializeIfNot(void)
{
    return  0;
}


/***********************************************************************
* Implement: R_OSPL_Initialize
************************************************************************/
errnum_t  R_OSPL_Initialize( const void* const  in_NullConfig )
{
    R_UNREFERENCED_VARIABLE( in_NullConfig );
    return  0;
}


/***********************************************************************
* Implement: R_OSPL_THREAD_GetCurrentId
************************************************************************/
r_ospl_thread_id_t  R_OSPL_THREAD_GetCurrentId(void)
{
// Toppers システムコールに変更 tokuyama@kmg 2016.09.08

    ID taskid; // tokuyama@kmg 2016.09.08 
    ER ercd; // tokuyama@kmg 2016.09.08
	ercd = get_tid(&taskid); // tokuyama@kmg 2016.09.08
    return taskid; // tokuyama@kmg 2016.09.08
//    return  osThreadGetId(); // tokuyama@kmg 2016.09.08
}


/***********************************************************************
* Implement: R_OSPL_EVENT_Set
************************************************************************/
void  R_OSPL_EVENT_Set( r_ospl_thread_id_t const  in_ThreadId,  bit_flags32_t const  in_SetFlags )
{
    ER ercd;
#if 0
    int32_t  ret;

	#if R_OSPL_EVENT_OBJECT_CODE
    r_ospl_event_object_id_t  event_object;
#endif
#endif
TOPPERS_SYSLOG(LOG_INFO,"R_OSPL_EVENT_Set()_E");

    if ( in_ThreadId == R_OSPL_THREAD_NULL )
    {
        goto fin;
    }

#if 0
#if  R_OSPL_EVENT_WATCH
    R_D_AddToIntLog( 0x70200000 + (int_fast32_t)( in_SetFlags & R_OSPL_EVENT_ALL_BITS ) );
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) R_OSPL_THREAD_GetCurrentId() );
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) in_ThreadId );
#endif
#if  R_OSPL_DETECT_BAD_EVENT
    R_OSPL_EVENT_SetForDebug( in_ThreadId, in_SetFlags );
#endif

#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    event_object = R_OSPL_EVENT_GetEventObject_Sub( in_ThreadId, in_SetFlags );
    if ( event_object == R_OSPL_EVENT_OBJECT_NULL )  /* If Event Flag */
#endif
#endif
    {
// Set_flg() に置き換え。 tokuyama@kmg 2016.09.08
		ercd = set_flg(in_ThreadId, in_SetFlags); /* set_flg() と iset_flg() は同じ関数 */
//        ret = osSignalSet( (osThreadId) in_ThreadId,
//                           (int32_t)( in_SetFlags & R_OSPL_EVENT_ALL_BITS ) );
#if 0
        ASSERT_D( (ret & OS_ERROR_SIGNAL) == 0,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( ret );  /* for Release configuration */
#endif
    }
#if 0
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    else    /* If Event Object */
    {
        osSemaphoreId  object = (osSemaphoreId) event_object;
        osStatus       es;

// ねぐっちゃっていい tokuyama@kmg 2016.09.08
        es= osSemaphoreRelease( object );
        ASSERT_D( es == osOK,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( es );  /* for Release configuration */
    }
#endif
#endif

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Set()_X");
	return;
}


/***********************************************************************
* Implement: R_OSPL_EVENT_Clear
************************************************************************/
// クリアするビットのパターンが OSPL と Toppersでは、逆の仕様なので要注意
void  R_OSPL_EVENT_Clear( r_ospl_thread_id_t const  in_ThreadId,  bit_flags32_t const  in_ClearFlags1 )
{
    ER ercd;
    FLGPTN clrptn;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Clear()_E");

#if 1
	UNUSED_PARAM(clrptn);
#endif

#if 0
    int32_t  ret;

	#if R_OSPL_EVENT_OBJECT_CODE
    r_ospl_event_object_id_t  event_object;
#endif
#endif

    if ( in_ThreadId == R_OSPL_THREAD_NULL )
    {
        goto fin;
    }
#if 0
#if  R_OSPL_EVENT_WATCH
    R_D_AddToIntLog( 0x70C00000 + (int_fast32_t)( in_ClearFlags1 & R_OSPL_EVENT_ALL_BITS ) );
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) R_OSPL_THREAD_GetCurrentId() );
    R_D_AddToIntLog( (int_fast32_t) (uintptr_t) in_ThreadId );
#endif
#endif
#if 0
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    event_object = R_OSPL_EVENT_GetEventObject_Sub( in_ThreadId, in_ClearFlags1 );
    if ( event_object == R_OSPL_EVENT_OBJECT_NULL )  /* If Event Flag */
#endif
#endif
    {
        if ( in_ClearFlags1 != 0 )    /* For avoiding error in osSignalClear */
        {
            // Toppers の clr_flg() に置き換え tokuyama@kmg 2016.09.08
//            ret = osSignalClear( (osThreadId) in_ThreadId,
//                                 (int32_t)( in_ClearFlags1 & R_OSPL_EVENT_ALL_BITS ) );
			ercd = clr_flg(in_ThreadId, (~in_ClearFlags1));

            /* "& R_OSPL_EVENT_ALL_BITS" is for avoiding error in osSignalClear */
#if 0
            ASSERT_D( (ret & OS_ERROR_SIGNAL) == 0,  R_NOOP() );
            R_UNREFERENCED_VARIABLE( ret );  /* for Release configuration */
#endif
        }
    }
#if 0
#if  R_OSPL_EVENT_OBJECT_CODE  &&  ( R_OSPL_ERROR_BREAK  ||  R_OSPL_TLS_ERROR_CODE )
    else    /* If Event Object */
    {
        osSemaphoreId  object = (osSemaphoreId) event_object;
        int32_t        ei;

// 放置で問題ないtokuyama@kmg 2016.09.08
        ei= osSemaphoreWait( object, 0 );
        ASSERT_D( ei != -1,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( ei );  /* for Release configuration */
    }
#endif
#endif

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Clear()_X");
	return;
}


/***********************************************************************
* Implement: R_OSPL_EVENT_Get
************************************************************************/
#if 0
#if ( ( ! defined( osCMSIS )  ||  osCMSIS <= 0x10001 ) &&  R_OSPL_VERSION < 85 ) ||  R_OSPL_SUPPORT_EVENT_GET
// おそらく、特定の OS 向けで、これは放置でかまわない tokuyama@kmg 2016.09.08
bit_flags32_t  R_OSPL_EVENT_Get( r_ospl_thread_id_t const  in_ThreadId )
{
    int32_t  ret;

    if ( in_ThreadId == R_OSPL_THREAD_NULL )
    {
        ret = 0;
    }
    else
    {

        ret = osSignalGet( (osThreadId) in_ThreadId );
        ASSERT_D( (ret & OS_ERROR_SIGNAL) == 0,  R_NOOP() );
    }

    return  (bit_flags32_t) ret;
}
#endif
#endif

/***********************************************************************
* Implement: R_OSPL_EVENT_Wait
************************************************************************/
errnum_t  R_OSPL_EVENT_Wait( bit_flags32_t const  in_WaitingFlags,  bit_flags32_t* const  out_GotFlags,
                             uint32_t const  in_Timeout_msec )
{
    errnum_t  e;
#if 0
    osEvent   event;
#if  R_OSPL_EVENT_OBJECT_CODE  ||  R_OSPL_EVENT_WATCH  ||  R_OSPL_DETECT_BAD_EVENT
    r_ospl_thread_id_t const  current_thread = (r_ospl_thread_id_t) R_OSPL_THREAD_GetCurrentId();
#endif

#if  R_OSPL_EVENT_OBJECT_CODE
    r_ospl_event_object_id_t    event_object;
#endif
#if  R_OSPL_DETECT_BAD_EVENT
    r_ospl_event_status_t*  status = NULL;
    r_ospl_error_t*         err = NULL;
#endif
#endif
//    bit_flags32_t  got_flags = 0x00000000u;
    FLGPTN got_flags = 0x00000000u;
    TMO micro_sec;
    ER ercd;
    ID tskid;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Wait()_E");

#if 0
    R_STATIC_ASSERT( R_OSPL_INFINITE == TO_UNSIGNED( osWaitForever ), "" );

#if  R_OSPL_EVENT_WATCH
    if ( in_Timeout_msec > 0 )
    {
        R_D_AddToIntLog( 0x70BE0000 + (int_fast32_t) in_WaigingFlags );
        R_D_AddToIntLog( (int_fast32_t) (uintptr_t) current_thread );

        {
            static int tc;
            if ( R_D_Counter( &tc, 0, NULL ) )
            {
                R_DEBUG_BREAK();
            }
        }
    }
#endif

#if  R_OSPL_EVENT_OBJECT_CODE
    event_object = R_OSPL_EVENT_GetEventObject_Sub( current_thread,  R_OSPL_EVENT_OBJECT_FLAG );

    if ( event_object != R_OSPL_EVENT_OBJECT_NULL )
    {
        ASSERT_D( in_WaigingFlags == R_OSPL_ANY_FLAG  ||
                  IS_BIT_SET( in_WaigingFlags, R_OSPL_EVENT_OBJECT_FLAG ),
                  R_NOOP() );
    }
#endif

#if  R_OSPL_DETECT_BAD_EVENT
    e= R_OSPL_EVENT_GetStatus( current_thread,  &status );
    IF(e!=0)
    {
        goto fin;
    }
    if ( in_Timeout_msec != 0 )
    {
        ASSERT_R( status->AllocatedEvents != 0u,  R_OSPL_BAD_EVENT_ERROR() );
        /* Error: Any events are not allocated. */
    }
#endif


#if  R_OSPL_EVENT_OBJECT_CODE
    if ( event_object == R_OSPL_EVENT_OBJECT_NULL )  /* If Event Flag */
#endif
#endif 
    {
// Signal? twai_flg で置き換える tokuyama@kmg 2016.09.08
// ER ercd = twai_flg(ID lfgid, FLGPTN waiptn, MODE wfmode,
//                    FLGPTN * p_flgptn, TMO tmout)
// 待ち時間は mille sec -> micro sec 変換
// OSPL では、Signal はタスクに従属している資源なので、（暫定） task_id = flag_id固定
// 自タスク ID を取得してから呼び出し
	if (in_Timeout_msec != R_OSPL_INFINITE)
		{
			micro_sec = in_Timeout_msec * 1000u;
		}
	else
		{
			micro_sec = TMO_FEVR;
		}
        ercd = get_tid(&tskid); // 暫定仕様なのでエラーハンドリングはしない

        ercd = twai_flg(tskid, in_WaitingFlags, TWF_ANDW, &got_flags, micro_sec);

//        event = osSignalWait( (int32_t)( in_WaitingFlags & R_OSPL_EVENT_ALL_BITS ), in_Timeout_msec );

//        if ( (event.status == osOK) || (event.status == osEventTimeout) )
        if((ercd == E_OK) || (ercd == E_TMOUT))
        {
            got_flags = R_OSPL_TIMEOUT;
//            IF ( event.status == osEventTimeout )
            if (ercd == E_TMOUT)
            {
                e=E_TIME_OUT;
                goto fin;
            }
        }
        else
        {
//            ASSERT_R( event.status == osEventSignal,  e=E_OTHERS;  goto fin );
//            got_flags = (bit_flags32_t) event.value.signals;
                e = E_OTHERS;
        }
		// 待ちの解除後はフラグがクリアされる仕様なので、clr_flgも呼ばんとあかん
		R_OSPL_EVENT_Clear(tskid, R_OSPL_EVENT_ALL_BITS);
//		ercd = clr_flg(tskid, 0);
	}
#if 0
#if  R_OSPL_EVENT_OBJECT_CODE
    else    /* If Event Object */
    {
// Semaphore 利用のコード? tokuyama@kmg 2016.09.08
        osSemaphoreId  object = (osSemaphoreId) event_object;
        int32_t        ei;


        ei= osSemaphoreWait( object,  in_Timeout_msec );
        IF ( ei == -1 )
        {
            e=E_OTHERS;
            goto fin;
        }


        if ( ei == 0 )
        {
            got_flags = R_OSPL_TIMEOUT;
        }
        else
        {
            got_flags = R_OSPL_EVENT_OBJECT_FLAG;
        }
    }
#endif


#if  R_OSPL_DETECT_BAD_EVENT
    {
        uint32_t  got_flags_2 = got_flags & ( R_OSPL_EVENT_OBJECT_FLAG | R_OSPL_EVENT_ALL_BITS );

        ASSERT_R( IS_ALL_BITS_SET( status->AllocatedEvents,  got_flags_2 ),  R_OSPL_BAD_EVENT_ERROR() );

        err = R_OSPL_LockCurrentThreadError_Sub( current_thread, R_OSPL_ALLOCATE_IF_NOT );
        err->EventStatus.UnexpectedEvents |= got_flags_2;
    }
#endif
#endif 
    e=0;
fin:
#if 0
#if  R_OSPL_DETECT_BAD_EVENT
    R_OSPL_UnlockCurrentThreadError( &err );
#endif
#if  R_OSPL_EVENT_WATCH
    if ( in_Timeout_msec > 0 )
    {
        R_D_AddToIntLog( 0x70AF0000 );
        R_D_AddToIntLog( (int_fast32_t) (uintptr_t) current_thread );
        R_D_AddToIntLog( (int_fast32_t) got_flags );
    }
#endif
#endif
    if ( out_GotFlags != NULL )
    {
        *out_GotFlags = got_flags;
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_EVENT_Wait()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_MEMORY_Flush
キャッシュのフラッシュ SOLID に機能があるか...　// tokuyama@kmg 2016.09.08
************************************************************************/
// SOLID_MEM_CACHE_Flush() で置き換えられる？ //tokuyama@kmg 2016.09.14
void  R_OSPL_MEMORY_Flush( r_ospl_flush_t const  in_FlushType )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_MEMORY_Flush()_E");

#if 0 //tokuyama@kmg 2016.09.14
	if ( in_FlushType == R_OSPL_FLUSH_WRITEBACK_INVALIDATE )
    {
#if 0
        printf( "L1Flush\n" );
#endif

#if IS_RTX_USED
        __v7_clean_inv_dcache_all();
#else
#error
#endif
    }
    else if ( in_FlushType == R_OSPL_FLUSH_WRITEBACK_INVALIDATE_2ND )
    {
#if 0
        printf( "PL310Flush\n" );
#endif

        PL310_CleanInvAllByWay();
    }
    else
    {
        ASSERT_D( false,  R_NOOP() );
    }
#else
#if 0 
	SOLID_MEM_CACHE_Flush();
#endif
#endif /* 0 */
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_MEMORY_Flush()_X");
}


/***********************************************************************
* Function: R_OSPL_Is1bitOnly_Fast32_Sub
*    R_OSPL_Is1bitOnly_Fast32_Sub
************************************************************************/
#ifndef  R_OSPL_NDEBUG
static bool_t  R_OSPL_Is1bitOnly_Fast32_Sub( uint_fast32_t in_Value )
{
    if ( (in_Value & 0x0000FFFFu) == 0 )
    {
        in_Value >>= 16;
    }
    if ( (in_Value & 0x000000FFu) == 0 )
    {
        in_Value >>=  8;
    }
    if ( (in_Value & 0x0000000Fu) == 0 )
    {
        in_Value >>=  4;
    }
    if ( (in_Value & 0x00000003u) == 0 )
    {
        in_Value >>=  2;
    }
    if ( (in_Value & 0x00000001u) == 0 )
    {
        in_Value >>=  1;
    }
    return  ( in_Value == 1 );
}
#endif


/***********************************************************************
* Implement: R_OSPL_MEMORY_RangeFlush
領域限定でキャッシュをフラッシュ。　そもそも機能ああるか？ tokuyama@kmg 2016.09.08
************************************************************************/
// SOLID_MEM_CACHE_Flush() で置き換えられる？ //tokuyama@kmg 2016.09.14
errnum_t  R_OSPL_MEMORY_RangeFlush( r_ospl_flush_t const  in_FlushType,
                                    const void* const  in_StartAddress,  size_t const  in_Length )
{
    errnum_t   e;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CACHE_Flush()_E");

//　エラーチェックはしないで、呼び出してみる... tokuyama@kmg 2016.09.14
#if 0 // tokuyama@kmg 2016.09.14
	size_t     cache_line_size;
    size_t     cache_line_mask;
    uintptr_t  start;
    uintptr_t  over;

    ASSERT_R( in_FlushType == R_OSPL_FLUSH_INVALIDATE,  e=E_BAD_COMMAND_ID; goto fin );

    cache_line_size = R_OSPL_MEMORY_GetCacheLineSize();
    cache_line_mask = cache_line_size - 1u;
    ASSERT_D( R_OSPL_Is1bitOnly_Fast32_Sub( cache_line_size ), e=E_OTHERS; goto fin );

    /* ->MISRA 11.3 */ /* ->SEC R2.7.1 */
    ASSERT_R( ( (uintptr_t) in_StartAddress & cache_line_mask ) == 0u, e=E_OTHERS; goto fin );
    ASSERT_R( ( in_Length & cache_line_mask ) == 0u, e=E_OTHERS; goto fin );

    start = (uintptr_t) in_StartAddress;
    over  = ( (uintptr_t) in_StartAddress + in_Length ) - 1u;
    /* <-MISRA 11.3 */ /* <-SEC R2.7.1 */

    R_OSPL_MEMORY_RangeFlush_Sub( start, over, cache_line_size );
#else
	SOLID_MEM_CACHE_Flush ((SOLID_ADDRESS)in_StartAddress, in_Length);	 // void 型なので、エラーコードは e=0 を返しておく。
#endif /* 0 */

    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_CACHE_Flush()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_MEMORY_GetSpecification
************************************************************************/
void  R_OSPL_MEMORY_GetSpecification( r_ospl_memory_spec_t* const  out_MemorySpec )
{
    IF_DQ( out_MemorySpec == NULL )
    {
        goto fin;
    }
#if 0 // たぶんこれも参照されない関数 tokuyama@kmg 2016.09.14
    out_MemorySpec->CacheLineSize = R_OSPL_MEMORY_GetCacheLineSize();
#endif /* 0 */

fin:
    return;
}


/***********************************************************************
* Implement: R_OSPL_Delay 
************************************************************************/
// dly_tsk()に置き換え済 tokuyama@kmg 2016.09.08
errnum_t  R_OSPL_Delay( uint32_t const  in_DelayTime_msec )
{
    errnum_t  e;
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_Delay()_E");

#if 0
    osStatus  rs;
    bool_t const    is_overflow = ( in_DelayTime_msec > R_OSPL_MAX_TIME_OUT );
#else
    ER rs;
#endif
    uint32_t const  delay_parameter = in_DelayTime_msec + 1u;
    RELTIM microsec;

#if 0
    ASSERT_D( ! is_overflow, R_NOOP() );
#endif
    /* RTX 5.16: If delay_parameter = 100000, "osDelay" waits 65534 */

// dly_tsk()に置き換え tokuyama@kmg 2016.09.08
/* R_OSPL_Delay() の引数は milli sec。 dly_tsk() の引数は μ sec なので、変換必要。*/
/* それ以外の面倒なエラー判定は、dly_tsk に任せて、ここでは判定しない。*/
//    rs= osDelay( delay_parameter );
    microsec = delay_parameter * 1000u;
    rs = dly_tsk (microsec);
#if 0
    IF ( rs == osErrorISR )
    {
        e=E_NOT_THREAD;
        R_OSPL_RaiseUnrecoverable( e );
        goto fin;
    }
    IF ( IS( is_overflow ) )
    {
        e=E_TIME_OUT;
        goto fin;
    }
#endif

   if (
//        (rs != osOK) &&           /* for delay_parameter == 0 */
//        (rs != osEventTimeout)   /* for delay_parameter != 0 */
            (rs != E_OK)
        )
    {
        e=E_OTHERS;
        goto fin;
    }

    e=0;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_OSPL_Delay()_X");
	return e;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_Create
************************************************************************/
errnum_t  R_OSPL_QUEUE_Create( r_ospl_queue_id_t* out_self, r_ospl_queue_def_t* in_QueueDefine )
{
    errnum_t  e;
#if 0 // tokuyama@kmg 2016.09.14
    r_ospl_queue_def_t*  self = in_QueueDefine;

    self->MailQId = osMailCreate( (osMailQDef_t*) in_QueueDefine->MailQDef, NULL );
    IF ( self->MailQId == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }
    self->PublicStatus.UsedCount = 0;

    *out_self = self;
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_Create() called.");
#endif /* 0 */
    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
#endif
	return e;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_GetStatus
************************************************************************/
errnum_t  R_OSPL_QUEUE_GetStatus( r_ospl_queue_id_t  self,  const r_ospl_queue_status_t** out_Status )
{
#if 0 // tokuyama@kmg 2016.09.14
    *out_Status = &self->PublicStatus;
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_GetStatus() called.");
#endif /* 0 */

    return  0;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_Allocate
************************************************************************/
errnum_t  R_OSPL_QUEUE_Allocate( r_ospl_queue_id_t  self,  void* out_Address,  uint32_t in_Timeout_msec )
{
    errnum_t  e;
#if 0 //tokuyama@kmg 2016.09.14
    void*     address;
    bool_t    was_all_enabled = false;

    address = osMailAlloc( self->MailQId, in_Timeout_msec );
    *(void**) out_Address = address;
    IF ( address == NULL  &&  in_Timeout_msec > 0 )
    {
        if ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL )
        {
            e=E_NOT_THREAD;
        }
        else
        {
            e=E_TIME_OUT;
        }
        goto fin;
    }

    was_all_enabled = R_OSPL_DisableAllInterrupt();
    self->PublicStatus.UsedCount += 1;
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_Allocate() called.");
#endif /* 0 */

    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
    if ( was_all_enabled )
    {
        R_OSPL_EnableAllInterrupt();
    }
#endif /* 0 */
    return  e;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_Put
************************************************************************/
errnum_t  R_OSPL_QUEUE_Put( r_ospl_queue_id_t  self,  void* in_Address )
{
    errnum_t  e;
#if 0 //tokuyama@kmg 2016.09.14
    osStatus  status;

    status = osMailPut( self->MailQId, in_Address );
    IF ( status != osOK )
    {
        e=E_OTHERS;
        goto fin;
    }
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_Put() called.");
#endif /* 0 */

    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
#endif
	return e;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_Get
************************************************************************/
errnum_t  R_OSPL_QUEUE_Get( r_ospl_queue_id_t  self,  void* out_Address,  uint32_t in_Timeout_msec )
{
    errnum_t  e;
#if 0 //tokuyama@kmg 2016.09.14
    osEvent   event;

    event = osMailGet( self->MailQId, in_Timeout_msec );

    if ( event.status != osOK )
    {
        IF ( event.status != osEventMail )
        {
            if ( event.status == osEventTimeout )
            {
                e = E_TIME_OUT;
            }
            else if ( event.status == osErrorParameter )
            {
                if ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL )
                {
                    e = E_NOT_THREAD;
                }
                else
                {
                    e = E_OTHERS;
                }
            }
            else
            {
                e = E_OTHERS;
            }
            goto fin;
        }
    }

    *(void**) out_Address = event.value.p;
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_Get() called.");
#endif /* 0 */

    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
#endif
	return e;
}


/***********************************************************************
* Implement: R_OSPL_QUEUE_Free
************************************************************************/
errnum_t  R_OSPL_QUEUE_Free( r_ospl_queue_id_t  self,  void* in_Address )
{
    errnum_t  e;
#if 0 // tokuyama@kmg 2016.09.14
    osStatus  status;
    bool_t    was_all_enabled = false;

    status = osMailFree( self->MailQId, in_Address );
    IF ( status != osOK )
    {
        e=E_OTHERS;
        goto fin;
    }

    was_all_enabled = R_OSPL_DisableAllInterrupt();
    self->PublicStatus.UsedCount -= 1;
#else
    TOPPERS_SYSLOG(LOG_INFO, "***** R_OSPL_QUQUE_Free() called.");
#endif /* 0 */

    e=0;
#if 0 //tokuyama@kmg 2016.09.14
fin:
    if ( was_all_enabled )
    {
        R_OSPL_EnableAllInterrupt();
    }
#endif
    return  e;
}


