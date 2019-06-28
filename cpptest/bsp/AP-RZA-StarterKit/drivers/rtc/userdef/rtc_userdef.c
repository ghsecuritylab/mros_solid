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
* Copyright (C) 2012 - 2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name   : rtc_userdef.c
* $Rev: 1131 $
* $Date:: 2014-08-07 15:23:35 +0900#$
* Description : Realtime Clock(RTC) driver (User define function)
*******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_typedefs.h"
#include "dev_drv.h"                /* Device Driver common header */
#include "devdrv_rtc.h"             /* RTC Driver header */
#include "iodefine.h"
#include "rza_io_regrw.h"

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


/******************************************************************************
Private global variables and functions
******************************************************************************/


/******************************************************************************
* Function Name: Userdef_RTC_Init
* Description  : This is a user-defined function. RTC should be initialized.
*              : In the sample code, this function stops the time count operation 
*              : and disables the carry interrupt, alarm interrupt, and periodic 
*              : interrupt after the RTC module standby has been cancelled. 
*              : Selects 32.768kHz from RTC_X1 as an operation clock and sets 
*              : RTC to operate the on-chip crystal oscillator. The RESET bit 
*              : in the RTC control register 2 (RCR2) is also set.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RTC_Init(void)
{
    volatile uint8_t dummy_buf;

    /* ==== Module standby clear ==== */
    /* ---- Supply clock to the RTC ---- */
    RZA_IO_RegWrite_8(&CPG.STBCR6, 0, CPG_STBCR6_MSTP60_SHIFT, CPG_STBCR6_MSTP60);
    dummy_buf = CPG.STBCR6;     /* Dummy read */

    /* ==== RTC initial setting ==== */
    /* Stop time counter */
    RZA_IO_RegWrite_8(&RTC.RCR2, 0, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;

    /* Set to avoid generation of carry interrupt */
    RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_CIE_SHIFT, RTC_RCR1_CIE);
    /* Clear carry flag */
    RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_CF_SHIFT, RTC_RCR1_CF);
    /* Set to avoid generation of alarm interrupt */
    RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_AIE_SHIFT, RTC_RCR1_AIE);
    /* Clear alarm flag */
    RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_AF_SHIFT, RTC_RCR1_AF);
    /* Set to avoid generation of periodic interrupts */
    RZA_IO_RegWrite_8(&RTC.RCR2, 0, RTC_RCR2_PES_SHIFT, RTC_RCR2_PES);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;
	/* Clear periodic interrupt flag */
    RZA_IO_RegWrite_8(&RTC.RCR2, 0, RTC_RCR2_PEF_SHIFT, RTC_RCR2_PEF);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;

    /* Select 32.768kHz from RTC_X1 as operation clock */
    RZA_IO_RegWrite_8(&RTC.RCR5, 0, RTC_RCR5_RCKSEL_SHIFT, RTC_RCR5_RCKSEL);
    /* Run on-chip crystal oscillator, enable RTC_X1 clock input */
    RZA_IO_RegWrite_8(&RTC.RCR2, 1, RTC_RCR2_RTCEN_SHIFT, RTC_RCR2_RTCEN);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;

    /* Reset divider circuits */
    RZA_IO_RegWrite_8(&RTC.RCR2, 1, RTC_RCR2_RESET_SHIFT, RTC_RCR2_RESET);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;
}


/* End of File */

