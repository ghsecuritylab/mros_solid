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
* File: locking.c
*    Lock related FIT BSP. For RTX, RZ/A1 BSP, OS less.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "r_ospl_private.h"
#if R_OSPL_IS_PREEMPTION
#include  "r_ospl_RTOS_private.h"
#include "kernel.h"
//#include "itron.h"
#endif
#if IS_RZ_A1_BSP_USED
#include  "dma_if.h"  /* R_DMA_Alloc */
#endif


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/
int osMutexCreate();

/******************************************************************************
Private global variables and functions
******************************************************************************/

/***********************************************************************
* Variable: gs_ospl_mutex
************************************************************************/
#if R_OSPL_IS_PREEMPTION
// TOPPERS の Mutex に置き換える（計画) tokuyama@kmg 2016.09.13
static ID  gs_ospl_mutex;  /* gs_OSPL_Mutex */ // tokuyama@kmg 2016.09.13
#endif


/***********************************************************************
* Variable: g_bsp_Locks
************************************************************************/
BSP_CFG_USER_LOCKING_TYPE  g_bsp_Locks[ BSP_NUM_LOCKS ];


/***********************************************************************
* Implement: R_BSP_HardwareLock
************************************************************************/
bool_t  R_BSP_HardwareLock( mcu_lock_t const  in_HardwareIndex )
{
#if BSP_CFG_USER_LOCKING_ENABLED
    return  BSP_CFG_USER_LOCKING_HW_LOCK_FUNCTION( in_HardwareIndex );
#else
    bool_t  is_success;


#if IS_RZ_A1_BSP_USED
    if ( in_HardwareIndex >= BSP_LOCK_DMAC0  &&  in_HardwareIndex <= BSP_LOCK_DMAC15 )
    {
        int_fast32_t  channel_of_DMAC = in_HardwareIndex - BSP_LOCK_DMAC0;

        channel_of_DMAC = R_DMA_Alloc( channel_of_DMAC, NULL );
        IF ( channel_of_DMAC == -1 )
        {
            is_success = false;
            goto fin;
        }

        is_success = true;
        goto fin;
    }
#endif

    IF_D ( R_CUT_IF_ALWAYS_FALSE( in_HardwareIndex < 0u  ||)  in_HardwareIndex >= R_COUNT_OF( g_bsp_Locks ) )
    {
        is_success = false;
        goto fin;
    }

    is_success = R_BSP_SoftwareLock( &g_bsp_Locks[ in_HardwareIndex ] );
    IF ( ! is_success )
    {
        goto fin;
    }

    is_success = true;
fin:
    return  is_success;
#endif
}


/***********************************************************************
* Implement: R_OSPL_LockUnlockedChannel
************************************************************************/
#if ! BSP_CFG_USER_LOCKING_ENABLED
errnum_t  R_OSPL_LockUnlockedChannel( int_fast32_t* out_ChannelNum,
                                      mcu_lock_t  in_HardwareIndexMin,  mcu_lock_t  in_HardwareIndexMax )
{
    errnum_t         e;
    mcu_lock_t       hardware_index;
    r_ospl_error_t*  err = NULL;


    err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(), R_OSPL_ALLOCATE_IF_NOT );

#if R_OSPL_IS_PREEMPTION
    if ( R_OSPL_THREAD_GetCurrentId() == R_OSPL_THREAD_NULL )    /* If interrrupt context */
    {
        e = E_NOT_THREAD;
        goto fin;
    }
#endif


#if IS_RZ_A1_BSP_USED
    if ( in_HardwareIndexMin == BSP_LOCK_DMAC0 )
    {
        int_fast32_t  channel_of_DMAC;

        channel_of_DMAC = R_DMA_Alloc( DMA_ALLOC_CH, NULL );
        IF ( channel_of_DMAC == -1 )
        {
            e = E_FEW_ARRAY;
            goto fin;
        }

        *out_ChannelNum = channel_of_DMAC;
        e = 0;
        goto fin;
    }
#endif


    for ( hardware_index = in_HardwareIndexMin;  hardware_index <= in_HardwareIndexMax;
            hardware_index += 1 )
    {
        r_ospl_c_lock_t*  lock = &g_bsp_Locks[ hardware_index ];

        if ( ! lock->IsLocked )
        {
            lock->IsLocked = true;
            break;
        }
    }
    IF_C ( hardware_index > in_HardwareIndexMax,  err )
    {
        e=E_FEW_ARRAY;
        goto fin;
    }


    *out_ChannelNum = hardware_index - in_HardwareIndexMin;


    e=0;
fin:
    R_OSPL_UnlockCurrentThreadError( &err );
    return  e;
}
#endif


/***********************************************************************
* Implement: R_BSP_HardwareUnlock
************************************************************************/
bool_t  R_BSP_HardwareUnlock( mcu_lock_t const  in_HardwareIndex )
{
#if BSP_CFG_USER_LOCKING_ENABLED
    return  BSP_CFG_USER_LOCKING_HW_UNLOCK_FUNCTION( in_HardwareIndex );
#else
    bool_t  is_success;


#if IS_RZ_A1_BSP_USED
    if ( in_HardwareIndex >= BSP_LOCK_DMAC0  &&  in_HardwareIndex <= BSP_LOCK_DMAC15 )
    {
        int_fast32_t  channel_of_DMAC = in_HardwareIndex - BSP_LOCK_DMAC0;
        int_fast32_t  err2;

        err2 = R_DMA_Free( channel_of_DMAC, NULL );
        IF ( err2 != ESUCCESS )
        {
            is_success = false;
            goto fin;
        }

        is_success = true;
        goto fin;
    }
#endif


    IF_D ( R_CUT_IF_ALWAYS_FALSE( in_HardwareIndex < 0u  ||)  in_HardwareIndex >= R_COUNT_OF( g_bsp_Locks ) )
    {
        is_success = false;
        R_OSPL_RaiseUnrecoverable( E_FEW_ARRAY );
        goto fin;
    }

    is_success = R_BSP_SoftwareUnlock( &g_bsp_Locks[ in_HardwareIndex ] );
    IF ( ! is_success )
    {
        goto fin;
    }

    is_success = true;
fin:
    return  is_success;
#endif
}


/***********************************************************************
* Implement: R_BSP_SoftwareLock
************************************************************************/
bool_t  R_BSP_SoftwareLock( BSP_CFG_USER_LOCKING_TYPE* const  in_LockObject )
{
#if BSP_CFG_USER_LOCKING_ENABLED
    return  BSP_CFG_USER_LOCKING_SW_LOCK_FUNCTION( in_LockObject );
#else
    errnum_t  e;
    r_ospl_error_t*  err = NULL;

    err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(), R_OSPL_ALLOCATE_IF_NOT );


    e= R_OSPL_C_LOCK_Lock_Sub( in_LockObject, err );
    if ( e == E_ACCESS_DENIED )
    {
        R_OSPL_CLEAR_ERROR_Sub( err );
        goto fin;
    }
    IF(e)
    {
        goto fin;
    }

    e=0;
fin:
    R_OSPL_UnlockCurrentThreadError( &err );
    return  (bool_t)( e == 0 );
#endif
}


/***********************************************************************
* Implement: R_BSP_SoftwareUnlock
************************************************************************/
bool_t  R_BSP_SoftwareUnlock( BSP_CFG_USER_LOCKING_TYPE* const  in_LockObject )
{
#if BSP_CFG_USER_LOCKING_ENABLED
    return  BSP_CFG_USER_LOCKING_SW_UNLOCK_FUNCTION( in_LockObject );
#else
    errnum_t  e;
    r_ospl_error_t*  err = NULL;

    err = R_OSPL_LockCurrentThreadError_Sub( R_OSPL_THREAD_GetCurrentId(), R_OSPL_ALLOCATE_IF_NOT );


    e= R_OSPL_C_LOCK_Unlock_Sub( in_LockObject, err );
    if ( e == E_ACCESS_DENIED )
    {
        R_OSPL_CLEAR_ERROR_Sub( err );
        goto fin;
    }
    IF(e)
    {
        goto fin;
    }

    e=0;
fin:
    R_OSPL_UnlockCurrentThreadError( &err );
    return  (bool_t)( e == 0 );
#endif
}


/***********************************************************************
* Implement: R_OSPL_Start_T_Lock
*    The function callbacked from OSPL internal, when T-Lock started
*
* Arguments:
*    None
*
* Return Value:
*    Error Code. 0=No Error.
*
* InternalDescription:
*    See. <R_OSPL_LockCurrentThreadError_Sub>.
************************************************************************/
#if R_OSPL_IS_PREEMPTION
errnum_t  R_OSPL_Start_T_Lock(void)
{
    errnum_t  e;
    ER ercd;  // tokuyama@kmg 2016.09.13

//    static osMutexDef( gs_ospl_mutex ); // tokuyama@kmg 2016.09.13 kernel_cfg.c で固定確保なので、動的生成はしない。


#if  GS_DEBUG_T_LOCK
    R_D_AddToIntLog( 0xDDDD7101 );
    R_D_AddToIntLog( (uintptr_t) R_OSPL_THREAD_GetCurrentId() );
#endif


    if ( gs_ospl_mutex <= 0 )
    {
// TOPPERS の Mutex に置き換える tokuyama@kmg 2016.09.13
// Semaphore も静的生成なので、ここは常に正常終了でリターン。 tokuyama@kmg 2016.09.08
// 関数戻り値の定義が判っていない。 tokuyama@kmg 2016.09.08
		gs_ospl_mutex = osMutexCreate(); //tokuyama@kmg 直値書かないよう sample-jpeg-display.c に関数を用意（そちらでは直値なんだけど...)
//		gs_ospl_mutex = osMutexCreate(osMutex(gs_ospl_mutex));
//        if ( gs_ospl_mutex == NULL )
//        {
//            e=E_OTHERS;
            //goto fin;
        //}
	}

// TOPPERS の Mutex に置き換える tokuyama@kmg 2016.09.08
#if 0
	es= osMutexWait( gs_ospl_mutex, TO_UNSIGNED( osWaitForever ) );
    if ( es == osErrorISR )
    {
        es = osOK;
    }
    if ( es != osOK )
    {
        e=E_OTHERS;
        goto fin;
    }
#else
	ercd = tloc_mtx (gs_ospl_mutex, TMO_FEVR);
	if (ercd == E_CTX)
	{
		ercd = E_OK;
	}
	if (ercd != E_OK)
	{
		e = E_OTHERS;
		goto fin;
	}
#endif /* 0 */

e=0;
fin:
#if  GS_DEBUG_T_LOCK
    R_D_AddToIntLog( 0xDDDD7102 );
    R_D_AddToIntLog( (uintptr_t) R_OSPL_THREAD_GetCurrentId() );
#endif

    return  e;
}
#endif


/***********************************************************************
* Implement: R_OSPL_End_T_Lock
*    The function callbacked from OSPL internal, when T-Lock ended
*
* Arguments:
*    None
*
* Return Value:
*    None
*
* InternalDescription:
*    See. <R_OSPL_UnlockCurrentThreadError>.
************************************************************************/
#if R_OSPL_IS_PREEMPTION
void  R_OSPL_End_T_Lock(void)
{
#if  GS_DEBUG_T_LOCK
    R_D_AddToIntLog( 0xDDDD710F );
    R_D_AddToIntLog( (uintptr_t) R_OSPL_THREAD_GetCurrentId() );
#endif


	// TOPPERS の Mutex に置き換える tokuyama@kmg 2016.09.13
#if 0 // tokuyama@kmg 2016.09.13
	if (gs_ospl_mutex != NULL)
    {
		osStatus  rs;

        rs= osMutexRelease( gs_ospl_mutex );
        if ( rs == osErrorISR )
        {
            rs = osOK;
        }
        ASSERT_R( rs == osOK,  R_OSPL_RaiseUnrecoverable( E_OTHERS ) );
	}
#else
		ER ercd;
	if (gs_ospl_mutex <= 0)
    {

		ercd = unl_mtx ( gs_ospl_mutex);
		if ( ercd == E_CTX)
			{
				ercd = E_OK;
			}
	}
#endif /* 0 */
		
}
#endif


/***********************************************************************
* Implement: R_OSPL_ByEventObject
************************************************************************/
#if  R_OSPL_EVENT_OBJECT_CODE  ||  R_OSPL_DETECT_BAD_EVENT
r_ospl_thread_id_t  R_OSPL_ByEventObject(void)
{
    return  R_OSPL_BY_ELEMENT_OBJECT;
}
#endif


