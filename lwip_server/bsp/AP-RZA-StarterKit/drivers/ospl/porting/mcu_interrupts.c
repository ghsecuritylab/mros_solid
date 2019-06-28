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
* File: mcu_interrupts.c
*    Interrupt related FIT BSP. For RTX.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "mcu_interrupts.h"

#include "kernel.h"
//#include "itron.h"

/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Macro: GS_INTERRUPT_DEBUG
*    0 or 1
************************************************************************/
#define  GS_INTERRUPT_DEBUG  0


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


/***********************************************************************
* Implement: R_BSP_InterruptWrite
************************************************************************/
// Interrupt handler の設定は、kernel_cfg.c で static に行う。 tokuyama@kmg 2016.09.08
// 成功したことにして、何もせずにリターンしたらよいかな...。 tokuyama@kmg 2016.09.08

bsp_int_err_t  R_BSP_InterruptWrite( bsp_int_src_t const  in_IRQ_Num,  bsp_int_cb_t const  in_Callback )
{
    bsp_int_err_t  eb;
#if 0
    uint32_t       er;
    IRQn_Type      irq;  /* QAC 3441 */

#if  GS_INTERRUPT_DEBUG
    R_D_AddToIntLog( 0x71000000 + in_IRQ_Num );
#endif

    if ( in_Callback == FIT_NO_FUNC )
    {
        irq = R_CAST_bsp_int_src_t_to_IRQn_Type( in_IRQ_Num );
        er= InterruptHandlerUnregister( irq );
        IF ( er != 0u )
        {
            eb=BSP_INT_ERR_INVALID_ARG;
            goto fin;
        }
    }
    else
    {
        irq = R_CAST_bsp_int_src_t_to_IRQn_Type( in_IRQ_Num );  /* QAC 3441 */
        er= InterruptHandlerRegister( irq, in_Callback );
        IF ( er != 0u )
        {
            eb=BSP_INT_ERR_INVALID_ARG;
            goto fin;
        }
    }
#endif 
    eb = BSP_INT_SUCCESS;
#if 0 //tokuyama@kmg 2016.09.14
fin:
#endif
	return eb;
}


/***********************************************************************
* Implement: R_BSP_InterruptRead
************************************************************************/
bsp_int_err_t  R_BSP_InterruptRead( bsp_int_src_t const  in_IRQ_Num,  bsp_int_cb_t* const  out_Callback )
{
    R_UNREFERENCED_VARIABLE_2( in_IRQ_Num, out_Callback );
    return  BSP_INT_ERR_UNSUPPORTED;
}


/***********************************************************************
* Implement: R_OSPL_OnInterruptForUnregistered
************************************************************************/
void  R_OSPL_OnInterruptForUnregistered( uint32_t const  int_sense )
{
    R_UNREFERENCED_VARIABLE( int_sense );
    R_OSPL_RaiseUnrecoverable( E_OTHERS );
}


/***********************************************************************
* Implement: R_BSP_InterruptControl
************************************************************************/
// 割り込み禁止、割り込み許可の実装 ena_int() dis_int() に置き換え tokuyama@kmg 2016.09.08
bsp_int_err_t  R_BSP_InterruptControl( bsp_int_src_t const  in_IRQ_Num,  bsp_int_cmd_t const  in_Command,
                                       void* const  in_NotUsed )
{
    bsp_int_err_t  eb;
    IRQn_Type      irq;  /* QAC 3441 */

    R_UNREFERENCED_VARIABLE( in_NotUsed );
    R_IT_WILL_BE_NOT_CONST( in_NotUsed );

    switch ( in_Command )
    {
    case  BSP_INT_CMD_INTERRUPT_ENABLE:

#if  GS_INTERRUPT_DEBUG
        R_D_AddToIntLog( 0x71100000 + in_IRQ_Num );
#endif

        irq = R_CAST_bsp_int_src_t_to_IRQn_Type( in_IRQ_Num );
//        GIC_EnableIRQ( irq ); // tokuyama@kmg 2016.09.08
        ena_int(irq); // tokuyama@kmg 2016.09.08
        break;

    default:
        IF_DS( in_Command != BSP_INT_CMD_INTERRUPT_DISABLE )
        {
            eb=BSP_INT_ERR_INVALID_ARG;
            goto fin;
        }

#if  GS_INTERRUPT_DEBUG
        R_D_AddToIntLog( 0x71200000 + in_IRQ_Num );
#endif

        irq = R_CAST_bsp_int_src_t_to_IRQn_Type( in_IRQ_Num );
//        GIC_DisableIRQ( irq ); // tokuyama@kmg 2016.09.08
        dis_int(irq); // tokuyama@kmg 2016.09.08
        break;
    }

    eb = BSP_INT_SUCCESS;
#if !defined(NDEBUG) || defined(__QAC_ARM_H__)
fin:
#endif
    return  eb;
}


/***********************************************************************
* Implement: R_OSPL_SetInterruptPriority
************************************************************************/
// 割り込み優先度は、kernel_cfg.c で静的設定なので、null関数
errnum_t  R_OSPL_SetInterruptPriority( bsp_int_src_t const  in_IRQ_Num,  int_fast32_t const  in_Priority )
{
//    GIC_SetPriority( in_IRQ_Num, (uint32_t) in_Priority ); // tokuyama@kmg 2016.09.08
    return  0;
}
