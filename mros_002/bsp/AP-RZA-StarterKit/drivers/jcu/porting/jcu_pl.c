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
/*******************************************************************************
* File: jcu_pl.c
*    JPEG Codec Unit (JCU) Sample Driver. OS Porting Layer. For RZ/A1.
*
* - $Module: JCU $ $PublicVersion: 1.03 $ (=JCU_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-02-26 13:18:53 +0900#$
******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "iodefine.h"
#include  "r_jcu_api.h"
#include  "r_jcu_pl.h"
#include <kernel.h>
#include <syslog.h>
#ifdef DEBUG_JCU
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Structure: jcu_pl_t
************************************************************************/
typedef struct st_jcu_pl_t  jcu_pl_t;
struct st_jcu_pl_t
{
    const r_ospl_caller_t*     InterruptCallbackCaller;
    const jcu_async_status_t*  Status;
};


/***********************************************************************
* Variable: gs_jcu_pl
************************************************************************/
static jcu_pl_t  gs_jcu_pl;


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Constant: JCU_INT_PRIORITY
************************************************************************/
enum { JCU_INT_PRIORITY = 2 };


/***********************************************************************
* Constant: CPG_JCU_CLOCK_POWER_OFF
************************************************************************/
#define  CPG_JCU_CLOCK_POWER_OFF  0x00000002u


/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
#if ! R_OSPL_IS_PREEMPTION
static void  JCU_IRQ_JEDI_Handler( uint32_t const  int_sense );
static void  JCU_IRQ_JDTI_Handler( uint32_t const  int_sense );
#else
void  JCU_IRQ_JEDI_Handler(void); // tokuyama@kmg 2016.09.15 static だと困る
void JCU_IRQ_JDTI_Handler(void); // tokuyama@kmg 2016.09.15 static だと困る
#endif


/***********************************************************************
* Variable: gs_jedi_interrupt_context
*    JEDI interrupt context
************************************************************************/
static const r_ospl_interrupt_t  gs_jedi_interrupt_context = { BSP_INT_SRC_JEDI, 0 };


/***********************************************************************
* Variable: gs_jdti_interrupt_context
*    JDTI interrupt context
************************************************************************/
static const r_ospl_interrupt_t  gs_jdti_interrupt_context = { BSP_INT_SRC_JDTI, 0 };


/***********************************************************************
* Function: R_JCU_SetDefaultAsync
*    SetDefaultAsync
*
* Arguments:
*    ref_Async    - <r_ospl_async_t>.
*    in_AsyncType - <r_ospl_async_type_t>.
*
* Return Value:
*    None.
************************************************************************/
void  R_JCU_SetDefaultAsync( r_ospl_async_t* const  ref_Async,  r_ospl_async_type_t  in_AsyncType )
{
	TOPPERS_SYSLOG(LOG_INFO,"R_JCU_SetDefaultAsync()_E");

	IF_DQ( ref_Async == NULL )
    {
        goto fin;
    }

    R_OSPL_ASYNC_SetDefaultPreset( ref_Async );

    if ( IS_BIT_NOT_SET( ref_Async->Flags, R_F_OSPL_A_Thread ) )
    {
        ref_Async->A_Thread = R_OSPL_THREAD_NULL;
    }

    if ( IS_BIT_NOT_SET( ref_Async->Flags, R_F_OSPL_A_EventValue ) )
    {
        if ( in_AsyncType == R_OSPL_ASYNC_TYPE_NORMAL )
        {
            ref_Async->A_EventValue = R_OSPL_A_FLAG;
        }
        else
        {
            ref_Async->A_EventValue = R_OSPL_FINAL_A_FLAG;
        }
    }
    else
    {
        ASSERT_D( IS_BIT_SET( ref_Async->Flags, R_F_OSPL_A_Thread ), R_NOOP() );
    }

    if ( IS_BIT_NOT_SET( ref_Async->Flags, R_F_OSPL_I_Thread ) )
    {
        ref_Async->I_Thread = R_OSPL_THREAD_NULL;
    }

    if ( IS_BIT_NOT_SET( ref_Async->Flags, R_F_OSPL_I_EventValue ) )
    {
        if ( ref_Async->I_Thread == R_OSPL_THREAD_NULL )
        {
            ref_Async->I_EventValue = 0;
        }
        else
        {
            ref_Async->I_EventValue = R_OSPL_I_FLAG;
        }
    }
    else
    {
        ASSERT_D( IS_BIT_SET( ref_Async->Flags, R_F_OSPL_I_Thread ), R_NOOP() );
    }

    if ( IS_BIT_NOT_SET( ref_Async->Flags, R_F_OSPL_InterruptCallback ) )
    {
        ref_Async->InterruptCallback = &( R_JCU_OnInterruptDefault );    /* MISRA 16.9 */
    }

    ref_Async->Flags = R_F_OSPL_A_Thread | R_F_OSPL_A_EventValue |
                       R_F_OSPL_I_Thread | R_F_OSPL_I_EventValue |
                       R_F_OSPL_InterruptCallback | R_F_OSPL_Delegate;
fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetDefaultAsync()_X");

	return;
}


/***********************************************************************
* Function: R_JCU_OnInitialize
*    OnInitialize
*
* Arguments:
*    None.
*
* Return Value:
*    Error code, 0=No error.
************************************************************************/
errnum_t  R_JCU_OnInitialize(void)
{
    errnum_t         e;
//    bsp_int_err_t    eb; //tokuyama@kmg
    jcu_pl_t* const  self = &gs_jcu_pl;
//    bsp_int_src_t const  num_of_JEDI_IRQ = gs_jedi_interrupt_context.IRQ_Num; // tokuyama@kmg
//    bsp_int_src_t const  num_of_JDTI_IRQ = gs_jdti_interrupt_context.IRQ_Num; // tokuyama@kmg
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnInitialize()_E");
    self->InterruptCallbackCaller = NULL;
    self->Status = NULL;

#if 0 // 割り込みの設定は、TOPPERSの方法に変更 tokuyama@kmg 2016.09.15
    /* Register "JEDI" */
    eb= R_BSP_InterruptWrite( num_of_JEDI_IRQ, &( JCU_IRQ_JEDI_Handler ) );  /* MISRA 16.9 */
    IF ( eb != 0 )
    {
        e=E_OTHERS;
        goto fin;
    }

    e= R_OSPL_SetInterruptPriority( num_of_JEDI_IRQ, JCU_INT_PRIORITY );
    IF ( e != 0 )
    {
        goto fin;
    }


    /* Register "JDTI" */
    eb= R_BSP_InterruptWrite( num_of_JDTI_IRQ, &( JCU_IRQ_JDTI_Handler ) );  /* MISRA 16.9 */
    IF ( eb != 0 )
    {
        e=E_OTHERS;
        goto fin;
    }

    e= R_OSPL_SetInterruptPriority( num_of_JDTI_IRQ, JCU_INT_PRIORITY );
    IF ( e != 0 )
    {
        goto fin;
    }
#endif /* 0 */

    /* start to suuply the clock for JCU */
    {
        /* ->QAC 0306 */
        uint32_t          cpg_reg;
        volatile uint8_t  dummy;

        cpg_reg = (uint32_t)(CPG.STBCR6) & ~CPG_JCU_CLOCK_POWER_OFF;
        CPG.STBCR6 =(uint8_t)cpg_reg;
        dummy = CPG.STBCR6;  /* Dummy read for operation completely */
    } /* <-QAC 0306 */

    e=0;
#if 0
fin:
#endif
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnInitialize()_X");

	return e;
}


/***********************************************************************
* Function: R_JCU_OnFinalize
*    OnFinalize
*
* Arguments:
*    None.
*
* Return Value:
*    Error code or e, 0 = successful and input e=0.
************************************************************************/
/* ->QAC 3227 : "e" is usually changed in finalize function. */
errnum_t  R_JCU_OnFinalize( errnum_t e )
/* <-QAC 3227 */
{
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnFinalize()_E");

	/* stop to suuply the clock for JCU */
    {
        /* ->QAC 0306 */
        uint32_t          cpg_reg;
        volatile uint8_t  dummy;

        cpg_reg = (uint32_t)(CPG.STBCR6) | CPG_JCU_CLOCK_POWER_OFF;
        CPG.STBCR6 = (uint8_t)cpg_reg;
        dummy = CPG.STBCR6;  /* Dummy read for operation completely */
    } /* <-QAC 0306 */

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnFinalize()_X");

	return e;
}


/***********************************************************************
* Function: R_JCU_SetInterruptCallbackCaller
*    SetInterruptCallbackCaller
*
* Arguments:
*    in_Caller - <r_ospl_caller_t>.
*
* Return Value:
*    Error Code. 0=No Error.
************************************************************************/
errnum_t  R_JCU_SetInterruptCallbackCaller( const r_ospl_caller_t* const  in_Caller )
{
    jcu_pl_t* const  self = &gs_jcu_pl;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetInterruptCallbackCaller()_E");
    self->InterruptCallbackCaller = in_Caller;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_SetInterruptCallbackCaller()_X");
    return  0;
}


/***********************************************************************
* Function: R_JCU_OnEnableInterrupt
*    OnEnableInterrupt
*
* Arguments:
*    in_Enables - <jcu_interrupt_lines_t>.
*
* Return Value:
*    None.
************************************************************************/
void  R_JCU_OnEnableInterrupt( jcu_interrupt_lines_t const  in_Enables )
{
    bsp_int_err_t  eb;

	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnEnableInterrupt()_E");
    if ( IS_BIT_SET( in_Enables, JCU_INTERRUPT_LINE_JEDI ) )
    {
        bsp_int_src_t const  num_of_IRQ = gs_jedi_interrupt_context.IRQ_Num;

        eb= R_BSP_InterruptControl( num_of_IRQ, BSP_INT_CMD_INTERRUPT_ENABLE, FIT_NO_PTR );
        ASSERT_D( eb == 0,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( eb ); /* for Release configuration */
    }

    if ( IS_BIT_SET( in_Enables, JCU_INTERRUPT_LINE_JDTI ) )
    {
        bsp_int_src_t const  num_of_IRQ = gs_jdti_interrupt_context.IRQ_Num;

        eb= R_BSP_InterruptControl( num_of_IRQ, BSP_INT_CMD_INTERRUPT_ENABLE, FIT_NO_PTR );
        ASSERT_D( eb == 0,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( eb ); /* for Release configuration */
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnEnableInterrupt()_X");

}

/***********************************************************************
* Function: R_JCU_OnDisableInterrupt
*    OnDisableInterrupt
*
* Arguments:
*    in_Disables1 - Set disabling bit to 1. <jcu_interrupt_lines_t>.
*
* Return Value:
*    None.
************************************************************************/
void  R_JCU_OnDisableInterrupt( jcu_interrupt_lines_t const  in_Disables1 )
{
    bsp_int_err_t  eb;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnDisableInterrupt()_E");
    if ( IS_BIT_SET( in_Disables1, JCU_INTERRUPT_LINE_JEDI ) )
    {
        bsp_int_src_t const  num_of_IRQ = gs_jedi_interrupt_context.IRQ_Num;

        eb= R_BSP_InterruptControl( num_of_IRQ, BSP_INT_CMD_INTERRUPT_DISABLE, FIT_NO_PTR );
        ASSERT_D( eb == 0,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( eb ); /* for Release configuration */
    }

    if ( IS_BIT_SET( in_Disables1, JCU_INTERRUPT_LINE_JDTI ) )
    {
        bsp_int_src_t const  num_of_IRQ = gs_jdti_interrupt_context.IRQ_Num;

        eb= R_BSP_InterruptControl( num_of_IRQ, BSP_INT_CMD_INTERRUPT_DISABLE, FIT_NO_PTR );
        ASSERT_D( eb == 0,  R_NOOP() );
        R_UNREFERENCED_VARIABLE( eb ); /* for Release configuration */
    }
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnDisableInterrupt()_X");
}

/***********************************************************************
* Function: R_JCU_OnInterruptDefault
*    OnInterruptDefault
*
* Arguments:
*    in_InterruptSource - <r_ospl_interrupt_t>.
*    in_Caller          - <r_ospl_caller_t>.
*
* Return Value:
*    Error Code. 0=No Error.
************************************************************************/
errnum_t  R_JCU_OnInterruptDefault( const r_ospl_interrupt_t* const  in_InterruptSource,
                                    const r_ospl_caller_t* const  in_Caller )
{
    errnum_t  e;
    errnum_t  ee;
    r_ospl_async_t*  async;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnInterruptDefault()_E");
    e = 0;
    R_AVOID_UNSAFE_ALWAYS_WARNING( e );

    IF_DQ( in_Caller == NULL )
    {
        goto fin;
    }

    ee= R_JCU_OnInterrupting( in_InterruptSource );
    IF ( (ee != 0) && (e == 0) )
    {
        e = ee;
    }

    async = in_Caller->Async;

//
	//tokuyama@kmg CPU をunlock. もっと早くlock解除しても良いかもしれないけど、最悪ここで解除しないと set_flag() が失敗する
//
    unl_cpu();

    if ( async->I_Thread == R_OSPL_THREAD_NULL )
    {
        ee= R_JCU_OnInterrupted();
        IF ( (ee != 0) && (e == 0) )
        {
            e = ee;
        }
    }
    else
    {
        R_OSPL_EVENT_Set( async->I_Thread, async->I_EventValue );
    }

fin:
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_OnInterruptDefault()_X");

	return e;
}


/***********************************************************************
* Function: JCU_IRQ_JEDI_Handler
*    JEDI (JCU Encode Decode Interrupt) interrupt handler.
*
* Arguments:
*    int_sense - Ignored.
*
* Return Value:
*    None.
************************************************************************/
#if ! R_OSPL_IS_PREEMPTION
static void  JCU_IRQ_JEDI_Handler( uint32_t const  int_sense )
{
    jcu_pl_t* const                  self = &gs_jcu_pl;
    const r_ospl_interrupt_t* const  i_context = &gs_jedi_interrupt_context;

    R_OSPL_CallInterruptCallback( self->InterruptCallbackCaller, i_context );

    R_UNREFERENCED_VARIABLE( int_sense );
}
#else
void  JCU_IRQ_JEDI_Handler(void)
{
	// tokuyama@kmg 2016.09.14 TOPPERS 形式で割り込み登録するのを忘れない
    jcu_pl_t* const                  self = &gs_jcu_pl;
	const r_ospl_interrupt_t *const i_context = &gs_jedi_interrupt_context;
//	const r_ospl_interrupt_t *const i_context = &gs_jdti_interrupt_context;
	TOPPERS_SYSLOG(LOG_INFO, "JCU_IRQ_JEDI_Handler()_E");
	R_OSPL_CallInterruptCallback(self->InterruptCallbackCaller, i_context);

    TOPPERS_SYSLOG(LOG_INFO, "JCU_IRQ_JEDI_Handler()_X");
//    GIC_EndInterrupt( BSP_INT_SRC_JEDI );//tokuyama@kmg 2016.09.15 後始末は Toppers 任せでいいのか？
}
#endif


/***********************************************************************
* Function: JCU_IRQ_JDTI_Handler
*    JDTI (JCU Data Transfer Interrupt) interrupt handler.
*
* Arguments:
*    int_sense - Ignored.
*
* Return Value:
*    None.
************************************************************************/
#if ! R_OSPL_IS_PREEMPTION
static void  JCU_IRQ_JDTI_Handler( uint32_t const  int_sense )
{
    jcu_pl_t* const                  self = &gs_jcu_pl;
    const r_ospl_interrupt_t* const  i_context = &gs_jdti_interrupt_context;

    R_UNREFERENCED_VARIABLE( int_sense );

    R_OSPL_CallInterruptCallback( self->InterruptCallbackCaller, i_context );
}
#else
// tokuyama@kmg 2016.09.14 TOPPERS の形式に割り込みの登録にするのは忘れない。
void JCU_IRQ_JDTI_Handler(void)
{
    jcu_pl_t* const                  self = &gs_jcu_pl;
    const r_ospl_interrupt_t* const  i_context = &gs_jdti_interrupt_context;
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_JDTI_Handler()_E");
    R_OSPL_CallInterruptCallback( self->InterruptCallbackCaller, i_context );
	TOPPERS_SYSLOG(LOG_INFO, "R_JCU_JDTI_Handler()_X");

	//    GIC_EndInterrupt( BSP_INT_SRC_JDTI ); //tokuyama@kmg 2016.09.15 後始末は Toppers 任せでいいのか？
}
#endif
