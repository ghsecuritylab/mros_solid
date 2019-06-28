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
/******************************************************************************
* File: jcu_api.c
*    JPEG Codec Unit (JCU) driver API. Main Code.
*
* - $Module: JCU $ $PublicVersion: 1.03 $ (=JCU_VERSION)
* - $Rev: 38 $
* - $Date:: 2014-03-18 16:14:45 +0900#$
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include <stdint.h>
#include    <string.h>
#include    "r_typedefs.h"
#include    "iodefine.h"
#include    "r_ospl.h"
#include    "r_jcu_api.h"
#include    "r_jcu_local.h"
#include    "r_jcu_pl.h"

#include <syslog.h>
#ifdef DEBUG_JCU
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: R_JCU_EVENT_A
************************************************************************/
#define  R_JCU_EVENT_A    R_OSPL_A_FLAG


/***********************************************************************
* Define: R_JCU_EVENT_TERMINATE
************************************************************************/
#define  R_JCU_EVENT_TERMINATE  R_OSPL_FINAL_A_FLAG


/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/
/* Section: Internal Global */

/***********************************************************************
* Variable: gs_jcu_internal_information
************************************************************************/
static jcu_internal_information_t  gs_jcu_internal_information;


/***********************************************************************
* Variable: gs_jcu_i_lock
************************************************************************/
static jcu_i_lock_t  gs_jcu_i_lock;


/******************************************************************************
Private global variables and functions
******************************************************************************/

static jcu_errorcode_t  R_JCU_StartAsync_Sub( r_ospl_async_t* const  async );
static jcu_errorcode_t  R_JCU_TerminateAsync_Sub( r_ospl_async_t* const  async );


/***********************************************************************
* Class: jcu_i_lock_t
************************************************************************/
static void    R_JCU_I_LOCK_Reset( jcu_i_lock_t* const  self );
static bool_t  R_JCU_I_LOCK_Lock( void* const  self_ );
static void    R_JCU_I_LOCK_Unlock( void* const  self_ );
static void    R_JCU_I_LOCK_RequestFinalize( void* const  self_ );
static int_fast32_t  R_JCU_I_LOCK_GetRootChannelNum( const void* const  self_ );


/* Section: Global */
/***********************************************************************
* Function: R_JCU_Initialize
*    Initialize the driver
*
* Arguments:
*    NullConfig - (in) NULL
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_Initialize( void* const  NullConfig )
/* <-SEC M1.1.1 */
{
    errnum_t          e;
    jcu_errorcode_t   returnValue;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO,"R_JCU_Initialize()_E");

    static const r_ospl_i_lock_vtable_t  gs_I_LockVTable =
    {
        &( R_JCU_I_LOCK_Lock ),  /* MISRA 16.9 */
        &( R_JCU_I_LOCK_Unlock ),
        &( R_JCU_I_LOCK_RequestFinalize ),
        &( R_JCU_I_LOCK_GetRootChannelNum )
    };

    R_UNREFERENCED_VARIABLE( NullConfig );
    R_IT_WILL_BE_NOT_CONST( NullConfig );

    if ( self->I_Lock == NULL )
    {
        jcu_i_lock_t* const  i_lock = &gs_jcu_i_lock;

        self->Is_I_LockMaster = R_JCU_I_LOCK_Replace(
                                    i_lock,  &gs_I_LockVTable );
    }

    if ( IS( self->Is_I_LockMaster ) )
    {
        R_JCU_I_LOCK_Reset( self->I_Lock );
    }
    R_OSPL_FLAG32_InitConst( &self->AsyncStatus.InterruptFlags );
    R_OSPL_FLAG32_InitConst( &self->AsyncStatus.InterruptEnables );
    R_OSPL_FLAG32_InitConst( &self->AsyncStatus.CancelFlags );
    self->AsyncStatus.IsEnabledInterrupt = false;

    /* Error check */
    if (self->AsyncStatus.Status != JCU_STATUS_UNDEF)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    /* Run the User-defined function if user set the function */
    e= R_JCU_OnInitialize();
    IF ( e != 0 )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    }

    /* Initialize the variable */
    self->AsyncStatus.IsPaused = false;
    self->AsyncStatus.SubStatusFlags = 0;
    self->IsCountMode = false;
    self->ErrorFilter = JCU_INT_ERROR_ALL;
    self->AsyncForFinalize = NULL;

    /* Set the register */
    returnValue = JCU_SetRegisterForInitialize();

    if (returnValue == JCU_ERROR_OK)
    {
        /* Set the updated status */
        self->AsyncStatus.Status = JCU_STATUS_INIT;

        /* Initialize the status for the encode */
        self->Codec = JCU_CODEC_NOT_SELECTED;
    } /* end if */

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Initialize()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_Terminate
*    Terminate the driver function
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_Terminate(void)
/* <-SEC M1.1.1 */
{
    errnum_t         ee;
    jcu_errorcode_t  ej;
    r_ospl_async_t   async;
    bit_flags32_t    got_flags;

    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Terminate()_E");
    async.A_EventValue = 0;

    if ( self->AsyncStatus.Status != JCU_STATUS_UNDEF )
    {
        errnum_t  e;

        async.Flags = R_F_OSPL_A_Thread | R_F_OSPL_A_EventValue;
        e= R_OSPL_EVENT_Allocate(
               &async.A_Thread,  R_OSPL_THREAD_GetCurrentId(),
               &async.A_EventValue,  R_JCU_EVENT_TERMINATE );
        IF(e!=0)
        {
            ej=JCU_ERROR_PARAM;
            goto fin;
        }

        ej= R_JCU_TerminateAsync_Sub( &async );
        IF( ej != JCU_ERROR_OK )
        {
            goto fin;
        }

        e= R_OSPL_EVENT_Wait( async.A_EventValue, &got_flags, R_OSPL_INFINITE );
        IF(e!=0)
        {
            ej=JCU_ERROR_PARAM;
            goto fin;
        }

        e= async.ReturnValue;
        IF(e!=0)
        {
            ej=e;
            goto fin;
        }
    }

    ej = JCU_ERROR_OK;
fin:
    ee= R_OSPL_EVENT_Free( &async.A_Thread,  &async.A_EventValue );
    if ( ee != 0 )
    {
        ej= R_OSPL_MergeErrNum( ej, JCU_ERROR_PARAM );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Terminate()_X");
	return ej;
}


/***********************************************************************
* Function: R_JCU_TerminateAsync
*    Terminate the driver function
*
* Arguments:
*    async - <r_ospl_async_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_TerminateAsync( r_ospl_async_t* const  async )
/* <-SEC M1.1.1 */
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateAsync()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateAsync()_X");
	return R_JCU_TerminateAsync_Sub(async);
}


/***********************************************************************
* Function: R_JCU_TerminateAsync_Sub
*    Terminate the driver function
*
* Arguments:
*    async - <r_ospl_async_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
static jcu_errorcode_t R_JCU_TerminateAsync_Sub( r_ospl_async_t* const  async )
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue = JCU_ERROR_OK;
    bool_t            was_enabled = false;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateAsync_Sub()_E");
    IF_DQ( async == NULL )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    }

    R_JCU_SetDefaultAsync( async, R_OSPL_ASYNC_TYPE_FINALIZE );
    async->ReturnValue = 0;

    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

    /* Check the status */
    if ( self->AsyncStatus.Status == JCU_STATUS_UNDEF )
    {
        returnValue = JCU_ERROR_OK;
        goto fin;
    } /* end if */

    if ( self->AsyncStatus.Status == JCU_STATUS_RUN )
    {
        self->AsyncForFinalize = async;

        R_OSPL_FLAG32_Set( &self->AsyncStatus.CancelFlags, R_OSPL_FINALIZE_REQUEST );

        /* "R_JCU_OnInterrupted" will be called */
    }
    else
    {
        returnValue = R_JCU_TerminateStep2();
        R_OSPL_EVENT_Set( async->A_Thread, async->A_EventValue );
    }

fin:
    /* Finalize I-Lock */
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );

        if ( IS( self->Is_I_LockMaster ) )
        {
            if ( IS( gs_jcu_i_lock.IsRequestedFinalize ) )
            {
                R_JCU_I_LOCK_Finalize();
            }
        }
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateAsync_Sub()_X"); 
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_TerminateStep2
*    Terminate the driver function
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
jcu_errorcode_t  R_JCU_TerminateStep2(void)
{
    errnum_t         ee;
    jcu_errorcode_t  returnValue = JCU_ERROR_OK;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;

    R_AVOID_UNSAFE_ALWAYS_WARNING( returnValue );
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateStep2()_E");

    /* Set the updated status */
    R_OSPL_FLAG32_Clear( &self->AsyncStatus.CancelFlags, R_OSPL_FLAG32_ALL_BITS );
    self->AsyncStatus.Status = JCU_STATUS_UNDEF;
    self->I_LockVTable->RequestFinalize( self->I_Lock );


    /* Run the additional function selected by parameter */
    ee= R_JCU_OnFinalize( returnValue );
    IF ( (ee != 0) && (returnValue == JCU_ERROR_OK) )
    {
        returnValue = JCU_ERROR_PARAM;
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_TerminateStep2()_X");
    return  returnValue;
}


/***********************************************************************
* Function: R_JCU_SelectCodec
*    Select Codec Type
*
* Arguments:
*    codec - <jcu_codec_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SelectCodec(
    const jcu_codec_t codec)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled;  /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SelectCodec()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF ((self->AsyncStatus.Status == JCU_STATUS_UNDEF)
        || (self->AsyncStatus.Status == JCU_STATUS_RUN))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckSelectCodec(codec);
    IF (returnValue != JCU_ERROR_OK)
    {
        goto fin;
    }
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Clear the count mode flag */
    self->IsCountMode = false;

    /* Register set */
    JCU_SetRegisterForCodec(codec);

    /* Set the updated status */
    self->AsyncStatus.Status = JCU_STATUS_SELECTED;

    /* Set codec type to internal information variable */
    switch (codec)
    {
    case JCU_ENCODE:
        self->Codec = JCU_STATUS_ENCODE;
        break;
    case JCU_DECODE:
        self->Codec = JCU_STATUS_DECODE;
        break;
        /* ->QAC 2018 : For MISRA 15.3, SEC R3.5.2 */
    default:
        /* <-QAC 2018 */
        R_NOOP();  /* NOT REACHED */
        break;
    } /* end switch */

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SelectCodec()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_Start
*    Start decoding or encoding. Synchronized.
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_Start(
    void)
/* <-SEC M1.1.1 */
{
    errnum_t         e;
    errnum_t         ee;
    jcu_errorcode_t  ej;
    r_ospl_async_t   async;
    bit_flags32_t    got_flags;

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Start()_E");
    async.A_EventValue = 0;  /* For fin block */

    async.Flags = R_F_OSPL_A_Thread | R_F_OSPL_A_EventValue;
    e= R_OSPL_EVENT_Allocate(
           &async.A_Thread,  R_OSPL_THREAD_GetCurrentId(),
           &async.A_EventValue,  R_JCU_EVENT_A );
    IF(e!=0)
    {
        ej=JCU_ERROR_PARAM;
        goto fin;
    }

    ej= R_JCU_StartAsync_Sub( &async );
    IF( ej != JCU_ERROR_OK )
    {
        goto fin;
    }

    e= R_OSPL_EVENT_Wait( async.A_EventValue, &got_flags, R_OSPL_INFINITE );
    IF(e!=0)
    {
        ej=JCU_ERROR_PARAM;
        goto fin;
    }

    e= async.ReturnValue;
    IF(e!=0)
    {
        ej=e;
        goto fin;
    }

    ej = JCU_ERROR_OK;
fin:
    ee= R_OSPL_EVENT_Free( &async.A_Thread,  &async.A_EventValue );
    if ( ee != 0 )
    {
        ej= R_OSPL_MergeErrNum( ej, JCU_ERROR_PARAM );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Start()_X");
	return ej;
}


/***********************************************************************
* Function: R_JCU_StartAsync
*    Start decoding or encoding. Asynchronized.
*
* Arguments:
*    async - <r_ospl_async_t>.
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t  R_JCU_StartAsync( r_ospl_async_t* const  async )
/* <-SEC M1.1.1 */
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_StartAsyc()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_StartAsync()_X");
	return R_JCU_StartAsync_Sub(async);
}


/***********************************************************************
* Function: R_JCU_StartAsync_Sub
*    Start decoding or encoding. Asynchronized.
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
static jcu_errorcode_t  R_JCU_StartAsync_Sub( r_ospl_async_t* const  async )
/* <-SEC M1.1.1 */
{
    errnum_t          e;
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Start_Sub()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (self->AsyncStatus.Status != JCU_STATUS_READY)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckStart();
    IF(returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */

    IF ( async == NULL )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    R_JCU_SetDefaultAsync( async, R_OSPL_ASYNC_TYPE_NORMAL );
    async->ReturnValue = 0;

    /* Attach "Async" to interrupt */
    /* ->MISRA 11.4 : Not too big "enum" is same bit count as "int" */ /* ->SEC R2.7.1 */
    R_OSPL_CALLER_Initialize( &self->InterruptCallbackCaller,
                              async, (int_fast32_t*)&self->AsyncStatus.Status, JCU_STATUS_INTERRUPTING,
                              self->I_Lock, self->I_LockVTable );
    /* <-MISRA 11.4 */ /* <-SEC R2.7.1 */
    e= R_JCU_SetInterruptCallbackCaller( &self->InterruptCallbackCaller );
    IF ( e != 0 )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    }
    R_OSPL_FLAG32_Set( &self->AsyncStatus.InterruptEnables, JCU_INTERRUPT_LINE_ALL );

    /* Clear event flags */
    R_OSPL_EVENT_Clear( async->A_Thread, async->A_EventValue );
    R_OSPL_EVENT_Clear( async->I_Thread, async->I_EventValue );

    /* Set the updated status when register update is finished */
    /* Change the status */
    self->AsyncStatus.Status = JCU_STATUS_RUN;

    /* Register set */
    JCU_SetRegisterForStart( self->ErrorFilter );

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Start_Sub()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetCountMode
*    Set the count mode (separate operating) parameter
*
* Arguments:
*    buffer - <jcu_count_mode_param_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SetCountMode(
    const jcu_count_mode_param_t  *const buffer)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetCountMode()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_SELECTED)
          || (self->AsyncStatus.Status == JCU_STATUS_READY)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF(buffer == NULL)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckSetCountMode(buffer);
    IF(returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Set the count mode flag */
    if ((buffer->inputBuffer.isEnable != false) || (buffer->outputBuffer.isEnable != false))
    {
        self->IsCountMode = true;
    }
    else
    {
        self->IsCountMode = false;
    } /* end if */

    /* Register set */
    JCU_SetRegisterForSetCountMode(buffer);
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetCountMode()_X");
    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
    return returnValue;
}


/***********************************************************************
* Function: R_JCU_Continue
*    Restart the processing caused by count mode. Synchronized.
*
* Arguments:
*    type - The target to continue
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_Continue(
    const jcu_continue_type_t  type)
/* <-SEC M1.1.1 */
{
    errnum_t         e;
    errnum_t         ee;
    jcu_errorcode_t  ej;
    r_ospl_async_t   async;
    bit_flags32_t    got_flags;

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Continue()_E");
    async.A_EventValue = 0;  /* For fin block */

    async.Flags = R_F_OSPL_A_Thread | R_F_OSPL_A_EventValue;
    e= R_OSPL_EVENT_Allocate(
           &async.A_Thread,  R_OSPL_THREAD_GetCurrentId(),
           &async.A_EventValue,  R_JCU_EVENT_A );
    IF(e!=0)
    {
        ej=JCU_ERROR_PARAM;
        goto fin;
    }

    ej= R_JCU_ContinueAsync( type, &async );
    IF( ej != JCU_ERROR_OK )
    {
        goto fin;
    }

    e= R_OSPL_EVENT_Wait( async.A_EventValue, &got_flags, R_OSPL_INFINITE );
    IF(e!=0)
    {
        ej=JCU_ERROR_PARAM;
        goto fin;
    }

    e= async.ReturnValue;
    IF(e!=0)
    {
        ej=e;
        goto fin;
    }

    ej = JCU_ERROR_OK;
fin:
    ee= R_OSPL_EVENT_Free( &async.A_Thread,  &async.A_EventValue );
    if ( ee != 0 )
    {
        ej= R_OSPL_MergeErrNum( ej, JCU_ERROR_PARAM );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Continue()_X");
	return ej;
}


/***********************************************************************
* Function: R_JCU_ContinueAsync
*    Restart the processing caused by count mode. Asynchronized.
*
* Arguments:
*    type - The target to continue
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_ContinueAsync(
    const jcu_continue_type_t  type,
    r_ospl_async_t* const  async)
/* <-SEC M1.1.1 */
{
    errnum_t          e;
    jcu_errorcode_t   returnValue = JCU_ERROR_PARAM;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    bit_flags32_t     mask;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
TOPPERS_SYSLOG(LOG_INFO, "R_JCU_ContinueAsync()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (self->AsyncStatus.Status != JCU_STATUS_READY)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (self->AsyncStatus.IsPaused == false)
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    IF ( async == NULL )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Clear the sub status flag */
    switch ( type )
    {
    case JCU_INPUT_BUFFER:
        mask = ( ( JINF_MASK | DINLF_MASK ) << JCU_SHIFT_JINTS1 );
        break;
    case JCU_OUTPUT_BUFFER:
        mask = ( ( DOUTLF_MASK | JOUTF_MASK ) << JCU_SHIFT_JINTS1 );
        break;
    default:
        ASSERT_R( type == JCU_IMAGE_INFO, returnValue=JCU_ERROR_PARAM; goto fin );
        mask = INS3_MASK;
        break;
    }
    ASSERT_R( IS_ANY_BITS_SET( self->AsyncStatus.SubStatusFlags, mask ), returnValue=JCU_ERROR_PARAM; goto fin );
    self->AsyncStatus.SubStatusFlags &= ~mask;


    R_JCU_SetDefaultAsync( async, R_OSPL_ASYNC_TYPE_NORMAL );
    async->ReturnValue = 0;

    /* Clear event flags */
    R_OSPL_EVENT_Clear( async->A_Thread, async->A_EventValue );
    R_OSPL_EVENT_Clear( async->I_Thread, async->I_EventValue );

    /* JCU will restart */
    if( self->AsyncStatus.SubStatusFlags == 0 )
    {
        /* Attach "Async" to interrupt */
        /* ->MISRA 11.4 : Not too big "enum" is same bit count as "int" */ /* ->SEC R2.7.1 */
        R_OSPL_CALLER_Initialize( &self->InterruptCallbackCaller,
                                  async, (int_fast32_t*)&self->AsyncStatus.Status, JCU_STATUS_INTERRUPTING,
                                  self->I_Lock, self->I_LockVTable );
        /* <-MISRA 11.4 */ /* <-SEC R2.7.1 */
        e= R_JCU_SetInterruptCallbackCaller( &self->InterruptCallbackCaller );
        IF ( e != 0 )
        {
            returnValue = JCU_ERROR_PARAM;
            goto fin;
        }
        R_OSPL_FLAG32_Set( &self->AsyncStatus.InterruptEnables, JCU_INTERRUPT_LINE_ALL );

        /* Change the status */
        self->AsyncStatus.Status = JCU_STATUS_RUN;
        self->AsyncStatus.IsPaused = false;

        /* Register set */
        JCU_SetRegisterForContinue( type );
    }

    /* JCU will not restart */
    else
    {
        /* Register set */
        JCU_SetRegisterForContinue( type );

        /* Change the status */
        self->AsyncStatus.Status = JCU_STATUS_READY;

        /* Set event flags */
        R_OSPL_EVENT_Set( async->A_Thread, async->A_EventValue );
        R_OSPL_EVENT_Set( async->I_Thread, async->I_EventValue );
    }


    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_ContinueAsync()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_GetAsyncStatus
*    R_JCU_GetAsyncStatus.
*
* Arguments:
*    None
*
* Return Value:
*    None
************************************************************************/
void  R_JCU_GetAsyncStatus( const jcu_async_status_t** const  out_Status )
{
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetAsyncStatus()_E");
    IF_DQ( out_Status == NULL )
    {
        goto fin;
    }

    *out_Status = &self->AsyncStatus;

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetAsyncStatus()_X");
	return;
}


/***********************************************************************
* Function: R_JCU_SetDecodeParam
*    Set parameter for decode.
*
* Arguments:
*    decode - Select the encode parameter for decoding. <jcu_decode_param_t>
*    buffer - Buffer settings for decode. <jcu_buffer_param_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SetDecodeParam(
    const jcu_decode_param_t   *const decode,
    const jcu_buffer_param_t   *const buffer)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetDecodeParam()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF(self->Codec != JCU_STATUS_DECODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF((decode == NULL) || (buffer == NULL))
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckSetDecodeParam(decode, buffer);
    IF(returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */

#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_SetRegisterForSetDecodePrm(decode, buffer);

    /* Set the updated status */
    self->AsyncStatus.Status = JCU_STATUS_READY;

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetDecodeParam()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetPauseForImageInfo
*    Whether stop or not stop for R_JCU_GetImageInfo()
*
* Arguments:
*    is_pause - Whether stop or not stop
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
jcu_errorcode_t R_JCU_SetPauseForImageInfo( const bool_t is_pause )
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetPauseForImageInfo()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF(self->Codec != JCU_STATUS_DECODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_SetRegisterForSetPause( is_pause, self->ErrorFilter );

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetPauseForImageInfo()_E");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_GetImageInfo
*    GetImageInfo
*
* Arguments:
*    buffer - Image information. <jcu_image_info_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_GetImageInfo(
    jcu_image_info_t   *const buffer)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetImageInfo()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (self->AsyncStatus.Status != JCU_STATUS_READY)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (self->Codec != JCU_STATUS_DECODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (buffer == NULL)
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckGetImageInfo(buffer);
    if(returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_GetRegisterForGetImageInfo(buffer);

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetImageInfo()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetErrorFilter
*    SetErrorFilter
*
* Arguments:
*    filter - enable bit of error. <jcu_int_detail_errors_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
jcu_errorcode_t R_JCU_SetErrorFilter(jcu_int_detail_errors_t filter)
{
    jcu_errorcode_t   returnValue;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetErrorFilter()_E");

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_INIT)
          || (self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    IF ( IS_ANY_BITS_SET( filter, (bit_flags_fast32_t) ~JCU_INT_ERROR_ALL ) )
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    self->ErrorFilter = filter;

    returnValue = JCU_ERROR_OK;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetErrorFilter()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetQuantizationTable
*    SetQuantizationTable
*
* Arguments:
*    tableNo - The table number to set the value. <jcu_table_no_t>
*    table   - The body of the table to set.
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SetQuantizationTable(
    const jcu_table_no_t   tableNo,
    const uint8_t         *const table)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetQuantizationTable()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (self->Codec != JCU_STATUS_ENCODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (table == NULL)
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckSetQuantizationTbl(tableNo);
    IF (returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_SetRegisterForSetQtTable(tableNo, table);

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetQuantizationTable()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetHuffmanTable
*    Set the Huffman table
*
* Arguments:
*    tableNo - The table number to set the value. <jcu_table_no_t>
*    type    - The type of Huffman table type (AC or DC). <jcu_huff_t>
*    table   - The body of the table to set
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SetHuffmanTable(
    const jcu_table_no_t   tableNo,
    const jcu_huff_t       type,
    const uint8_t         *const table)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetHuffmanTable()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (self->Codec != JCU_STATUS_ENCODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (table == NULL)
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckSetHuffmanTable(tableNo, type);
    IF (returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_SetRegisterForSetHuffTbl(tableNo, type, table);

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetHuffmanTable()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_SetEncodeParam
*    Set the parameter fo encoding
*
* Arguments:
*    encode - Select the encode parameter for encoding. <jcu_encode_param_t>
*    buffer - Select the buffer settings for encoding. <jcu_buffer_param_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_SetEncodeParam(
    const jcu_encode_param_t   *const encode,
    const jcu_buffer_param_t   *const buffer)
/* <-SEC M1.1.1 */
{
    jcu_errorcode_t   returnValue;
    bool_t            was_enabled; /* = false; */ /* QAC 3197 */
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetEncodeParam()_E");
    was_enabled = self->I_LockVTable->Lock( self->I_Lock );

#ifdef JCU_PARAMETER_CHECK
    IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          || (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF (self->Codec != JCU_STATUS_ENCODE)
    {
        returnValue = JCU_ERROR_STATUS;
        goto fin;
    } /* end if */

    IF ((encode == NULL) || (buffer == NULL))
    {
        returnValue = JCU_ERROR_PARAM;
        goto fin;
    } /* end if */

    returnValue = JCU_ParaCheckEncodeParam(encode, buffer);
    IF (returnValue != JCU_ERROR_OK)
    {
        goto fin;
    } /* end if */
#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */

    /* Register set */
    JCU_SetRegisterForSetEncodePrm(encode, buffer);

    /* Set the updated status */
    self->AsyncStatus.Status = JCU_STATUS_READY;

    returnValue = JCU_ERROR_OK;
fin:
    if ( IS( was_enabled ) )
    {
        self->I_LockVTable->Unlock( self->I_Lock );
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetEncodeParam()_X");
	return returnValue;
}


/***********************************************************************
* Function: R_JCU_GetEncodedSize
*    GetEncodedSize
*
* Arguments:
*    out_Size - EncodedSize
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
/* ->SEC M1.1.1 API Function */
jcu_errorcode_t R_JCU_GetEncodedSize(
    size_t *const out_Size)
/* <-SEC M1.1.1 */
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetEncodedSize()_E");
	JCU_GetEncodedSize(out_Size);
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetEncodedSize()_X");
	return JCU_ERROR_OK;
}


/***********************************************************************
* Function: R_JCU_Set2ndCacheAttribute
*    Set AXI bus A*CACHE bits for 2nd cache to JCU.
*
* Arguments:
*    read_cache_attribute  - <r_ospl_axi_cache_attribute_t>
*    write_cache_attribute - <r_ospl_axi_cache_attribute_t>
*
* Return Value:
*    <jcu_errorcode_t> type.
************************************************************************/
jcu_errorcode_t  R_JCU_Set2ndCacheAttribute(
    r_ospl_axi_cache_attribute_t const  read_cache_attribute,
    r_ospl_axi_cache_attribute_t const  write_cache_attribute )
{
    //static const uint32_t  mask_JCU = 0xFFFF0000u;
    jcu_errorcode_t    returnValue;
    //bool_t             was_all_enabled = false;
    //uint32_t           new_value;
    //volatile uint32_t  value;
    //jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Set2ndCacheAttribute()_E");

//#ifdef JCU_PARAMETER_CHECK
    //IF (!((self->AsyncStatus.Status == JCU_STATUS_READY)
          //|| (self->AsyncStatus.Status == JCU_STATUS_SELECTED)))
    //{
        //returnValue = JCU_ERROR_STATUS;
        //goto fin;
    //} /* end if */
//#endif  /* #Ifdef JCU_PARAMETER_CHECK_ */


    //new_value = ( read_cache_attribute << 24 ) | ( write_cache_attribute << 16 );

    //was_all_enabled = R_OSPL_DisableAllInterrupt();

    //value = INB.AXIBUSCTL0;
    //value = new_value | ( value & ~mask_JCU );  /* Mutual Exclusion from Ether */
    //INB.AXIBUSCTL0 = value;

    returnValue = JCU_ERROR_OK;
//fin:
    //if ( was_all_enabled )
    //{
        //R_OSPL_EnableAllInterrupt();
    //}
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_Set2ndCacheAttribute()_X");
    return  returnValue;
}


/***********************************************************************
* Function: R_JCU_EnableInterrupt
*    EnableInterrupt
*
* Arguments:
*    None
*
* Return Value:
*    None
************************************************************************/
void  R_JCU_EnableInterrupt(void)
{
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_EnableInterrupt()_E");
	self->AsyncStatus.IsEnabledInterrupt = true;
    R_JCU_OnEnableInterrupt( self->AsyncStatus.InterruptEnables.Flags );
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_EnableInterrupt()_X");
}


/***********************************************************************
* Function: R_JCU_DisableInterrupt
*    DisableInterrupt
*
* Arguments:
*    None
*
* Return Value:
*    Was interrupt enabled
************************************************************************/
bool_t  R_JCU_DisableInterrupt(void)
{
    bool_t    was_interrupted;
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_DisableInterrupt()_E");
    was_interrupted = self->AsyncStatus.IsEnabledInterrupt;

    R_JCU_OnDisableInterrupt( self->AsyncStatus.InterruptEnables.Flags );
    self->AsyncStatus.IsEnabledInterrupt = false;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_DisableInterrupt()_X");
    return  was_interrupted;
}


/***********************************************************************
* Function: R_JCU_GetInternalInformation
*    R_JCU_GetInternalInformation
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_internal_information_t> type.
************************************************************************/
jcu_internal_information_t*  R_JCU_GetInternalInformation(void)
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetInternalInformation()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetInternalInformation()_X");

	return  &gs_jcu_internal_information;
}


/***********************************************************************
* Function: R_JCU_GetILockObject
*    R_JCU_GetILockObject
*
* Arguments:
*    None
*
* Return Value:
*    <jcu_i_lock_t> type.
************************************************************************/
jcu_i_lock_t*  R_JCU_GetILockObject(void)
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetILockObject()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_GetILockObject()_X");
	return &gs_jcu_i_lock;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_Replace
*    Replace associated I-Lock object
*
* Arguments:
*    I_Lock       - I-Lock object
*    I_LockVTable - V-Table
*
* Return Value:
*    Whether success to replace
************************************************************************/
bool_t  R_JCU_I_LOCK_Replace( void* const  I_Lock,  const r_ospl_i_lock_vtable_t* const  I_LockVTable )
{
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
    bool_t  ret = false;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Replace()_E");
    ASSERT_R( self->AsyncStatus.Status == JCU_STATUS_UNDEF,  ret = false; goto fin );

    if ( I_Lock != NULL )
    {
        if ( self->I_Lock == NULL )
        {
            self->I_Lock = I_Lock;
            self->I_LockVTable = I_LockVTable;
            ret = true;
        }
    }
    else
    {
        self->I_Lock = NULL;
    }

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Replace()_X");
	return ret;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_Finalize
*    Finalize the I-Lock object
*
* Arguments:
*    None
*
* Return Value:
*    None
************************************************************************/
void  R_JCU_I_LOCK_Finalize(void)
{
    jcu_internal_information_t* const  self = &gs_jcu_internal_information;
    bool_t  b;

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Finalize()_E");
    ASSERT_D( self->Is_I_LockMaster, R_NOOP() );

    R_JCU_I_LOCK_Reset( self->I_Lock );
    b= R_JCU_I_LOCK_Replace( NULL, NULL );
    R_UNREFERENCED_VARIABLE( b );  /* QAC 3200 : This is not error information */
    self->Is_I_LockMaster = false;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Finalize()_X");
}


/***********************************************************************
* Function: R_JCU_I_LOCK_Reset
*    Reset the I-Lock object
*
* Arguments:
*    self - I-Lock object
*
* Return Value:
*    None
************************************************************************/
static void   R_JCU_I_LOCK_Reset( jcu_i_lock_t* const  self )
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Reset()_E");
	IF_DQ(self == NULL)
    {
        goto fin;
    }

    self->IsLock = false;
    R_JCU_OnDisableInterrupt( JCU_INTERRUPT_LINE_ALL );
    JCU_ClearInterruptFlag();
    self->IsRequestedFinalize = false;

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Reset()_X");
	return;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_Lock
*    Lock the I-Lock object
*
* Arguments:
*    self_ - I-Lock object
*
* Return Value:
*    Was interrupt enabled
************************************************************************/
static bool_t  R_JCU_I_LOCK_Lock( void* const  self_ )
{
    bool_t  is_locked;
    bool_t  was_all_enabled; /* = false; */ /* QAC 3197 */
    bool_t  b;
    jcu_i_lock_t* const  self = (jcu_i_lock_t*) self_;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Lock()_E");
    IF_DQ( self == NULL )
    {
        is_locked = true;
        goto fin;
    }

    was_all_enabled = R_OSPL_DisableAllInterrupt();

    is_locked = self->IsLock;
    if ( ! is_locked )
    {
        b= R_JCU_DisableInterrupt();
        R_UNREFERENCED_VARIABLE( b );  /* QAC 3200 : This is not error information */
        self->IsLock = true;
    }

    if ( IS( was_all_enabled ) )
    {
        R_OSPL_EnableAllInterrupt();
    }

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Lock()_X");
	return !is_locked;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_Unlock
*    Unlock the I-Lock object
*
* Arguments:
*    self_ - I-Lock object
*
* Return Value:
*    None
************************************************************************/
static void  R_JCU_I_LOCK_Unlock( void* const  self_ )
{
    bool_t  was_all_enabled; /* = false; */ /* QAC 3197 */
    jcu_i_lock_t* const  self = (jcu_i_lock_t*) self_;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Unlock()_E");
    IF_DQ( self == NULL )
    {
        goto fin;
    }

    was_all_enabled = R_OSPL_DisableAllInterrupt();

    R_JCU_EnableInterrupt();
    self->IsLock = false;

    if ( IS( was_all_enabled ) )
    {
        R_OSPL_EnableAllInterrupt();
    }

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_Unlock()_X");
	return;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_RequestFinalize
*    Request to finalize the I-Lock object
*
* Arguments:
*    self_ - I-Lock object
*
* Return Value:
*    None
************************************************************************/
static void   R_JCU_I_LOCK_RequestFinalize( void* const  self_ )
{
    jcu_i_lock_t* const  self = (jcu_i_lock_t*) self_;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_RequestFinalize()_E");
    IF_DQ( self == NULL )
    {
        goto fin;
    }

    self->IsRequestedFinalize = true;

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_RequestFinalize()_X");
	return;
}


/***********************************************************************
* Function: R_JCU_I_LOCK_GetRootChannelNum
*    Get root channel number of I-Lock object
*
* Arguments:
*    self_ - I-Lock object
*
* Return Value:
*    A channel number
************************************************************************/
static int_fast32_t  R_JCU_I_LOCK_GetRootChannelNum( const void* const  self_ )
{
    R_UNREFERENCED_VARIABLE( self_ );
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_GetRootChannelNum()_E");
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_I_LOCK_GetRootChannelNum()_X");

	return  0;
}
