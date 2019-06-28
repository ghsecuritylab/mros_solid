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
* File Name   : rtc.c
* $Rev: 1258 $
* $Date:: 2014-09-05 11:38:03 +0900#$
* Description : Realtime Clock(RTC) driver
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
#define RTC_READ_RPEAT_COUNT    (2)     /* Repeat count of time counter acquisition processing */

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


/******************************************************************************
Private global variables and functions
******************************************************************************/
static int32_t  RTC_CheckArgTimeEnable(rtc_time_t * time);
static int32_t  RTC_CheckArgTimeValue(rtc_time_t * time);
static int32_t  RTC_CheckArgAlarmEnbValue(rtc_alarm_enb_t * alarm_enb);
static void     RTC_ConvTimeToRegValue(rtc_time_t * time, rtc_time_t * reg_value);
static void     RTC_ConvRegValueToTime(rtc_time_t * reg_value, rtc_time_t * time);
static uint16_t RTC_ConvToBcd(uint16_t value);
static uint16_t RTC_ConvFromBcd(uint16_t bcd);

struct st_rtc *rtcreg;

/******************************************************************************
* Function Name: R_RTC_Init
* Description  : Initializes the RTC.
*              : Calls the user-defined function Userdef_RTC_Init and initializes
*              : the RTC by Userdef_RTC_Init.
* Arguments    : none
* Return Value : none
******************************************************************************/
void R_RTC_Init(void)
{
    /* ==== RTC initialization ==== */
    rtcreg = (struct st_rtc *)g_RTCRegs.addr;

    Userdef_RTC_Init();
}

/******************************************************************************
* Function Name: R_RTC_Open
* Description  : Starts the RTC time count operation.
*              : RTC starts the time count according to the current time of
*              : time counter.
* Arguments    : none
* Return Value : none
******************************************************************************/
void R_RTC_Open(void)
{
    volatile uint8_t dummy_buf;
    (void)dummy_buf;
    /* ==== Start time counting RTC ==== */
    RZA_IO_RegWrite_8(&RTC.RCR2, 1, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
    dummy_buf = RTC.RCR2;
    dummy_buf = RTC.RCR2;
}

/******************************************************************************
* Function Name: R_RTC_Close
* Description  : Stops the RTC time count operation.
* Arguments    : none
* Return Value : none
******************************************************************************/
void R_RTC_Close(void)
{
    volatile uint8_t dummy_buf;
    (void)dummy_buf;
    /* ==== Stop time counting RTC ==== */
    RZA_IO_RegWrite_8(&RTC.RCR2, 0, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
    dummy_buf = RTC.RCR2;
    dummy_buf = RTC.RCR2;
}

/******************************************************************************
* Function Name: R_RTC_SetCnt
* Description  : Sets the time specified by the argument time to RTC time counter.
*              : Sets the RESET bit in the control register 2 (RCR2). If RTC_ENABLE
*              : is specified for the argument time->second.enable, performs BCD-coding
*              : for the time of the argument time->second.value and writes it
*              : to the RTC second counter (RSECCNT). If RTC_DISABLE is specified
*              : for the argument time->second.enable, writing is not performed.
*              : Otherwise, any other member of the time should also be written
*              : to the respective RTC time counters. Because writing to the time
*              : counter is disabled while the RTC is in time count operation,
*              : suspends the time count operation at the beginning of this function
*              : and restarts at the end of this function.
* Arguments    : rtc_time_t * time : Time
*              :                   :   time->second.value  : Second      (0 to 59)
*              :                   :   time->minute.value  : Minute      (0 to 59)
*              :                   :   time->hour.value    : Hour        (0 to 23)
*              :                   :   time->week.value    : Day of week (0 to 6)
*              :                   :   time->day.value     : Day         (1 to 31)
*              :                   :   time->month.value   : Month       (1 to 12)
*              :                   :   time->year.value    : Year        (0 to 9999)
*              :                   : Specification for setting object
*              :                   : (RTC_ENABLE : Do set, RTC_DISABLE : Do NOT set)
*              :                   :   time->second.enable : Second counter setting
*              :                   :   time->minute.enable : Minute counter setting
*              :                   :   time->hour.enable   : Hour counter setting
*              :                   :   time->week.enable   : Day of week counter setting
*              :                   :   time->day.enable    : Day counter setting
*              :                   :   time->month.enable  : Month counter setting
*              :                   :   time->year.enable   : Year counter setting
* Return Value : DEVDRV_SUCCESS : Success in setting value to RTC time counter
*              : DEVDRV_ERROR   : Failure in setting value to RTC time counter
******************************************************************************/
int32_t R_RTC_SetCnt(rtc_time_t * time)
{
    rtc_time_t   bcd_value;
    uint8_t      start;
    int32_t      ret;
    volatile uint8_t dummy_buf;
    (void)dummy_buf;

    /* ==== Argument check ==== */
    ret = RTC_CheckArgTimeEnable(time);
    if (DEVDRV_SUCCESS == ret)
    {
        ret = RTC_CheckArgTimeValue(time);
    }
    if (DEVDRV_ERROR == ret)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Perform BCD-coding for the time specified by the argument ==== */
    RTC_ConvTimeToRegValue(time, &bcd_value);   /* Time -> Time counter value */

    /* ==== Stop time counting ==== */
    start = RZA_IO_RegRead_8(&RTC.RCR2, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
    if (1 == start)
    {
        RZA_IO_RegWrite_8(&RTC.RCR2, 0, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
		dummy_buf = RTC.RCR2;
		dummy_buf = RTC.RCR2;
	}

    /* ==== Reset ==== */
    RZA_IO_RegWrite_8(&RTC.RCR2, 1, RTC_RCR2_RESET_SHIFT, RTC_RCR2_RESET);
	dummy_buf = RTC.RCR2;
	dummy_buf = RTC.RCR2;

    /* ==== Time counter settings ==== */
    if (RTC_ENABLE == time->second.enable)
    {
        RTC.RSECCNT = bcd_value.second.value;   /* RSECCNT - Second counter */
    }
    if (RTC_ENABLE == time->minute.enable)
    {
        RTC.RMINCNT = bcd_value.minute.value;   /* RMINCNT - Minute counter */
    }
    if (RTC_ENABLE == time->hour.enable)
    {
        RTC.RHRCNT  = bcd_value.hour.value;     /* RHRCNT  - Hour counter   */
    }
    if (RTC_ENABLE == time->week.enable)
    {
        RTC.RWKCNT  = bcd_value.week.value;     /* RWKCNT  - Day of week counter */
    }
    if (RTC_ENABLE == time->day.enable)
    {
        RTC.RDAYCNT = bcd_value.day.value;      /* RDAYCNT - Day counter    */
    }
    if (RTC_ENABLE == time->month.enable)
    {
        RTC.RMONCNT = bcd_value.month.value;    /* RMONCNT - Month counter  */
    }
    if (RTC_ENABLE == time->year.enable)
    {
        RTC.RYRCNT  = bcd_value.year.value;     /* RYRCNT  - Year counter   */
    }

    /* ==== Start time counting ==== */
    if (1 == start)
    {
        RZA_IO_RegWrite_8(&RTC.RCR2, 1, RTC_RCR2_START_SHIFT, RTC_RCR2_START);
		dummy_buf = RTC.RCR2;
		dummy_buf = RTC.RCR2;
	}

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RTC_GetCnt
* Description  : Obtains the time from the RTC time counter and stores it in
*              : the area specified by the argument time.
*              : If RTC_ENABLE is specified for the argument time->second.enable,
*              : converts the BCD-coded time read from the RTC second counter (RSECCNT)
*              : into integer value and stores it in the argument time->second.value.
*              : If RTC_DISABLE is specified for the argument time->second.enable,
*              : readout is not performed. Otherwise, any other member of the
*              : time should also be readout from the respective RTC time counters.
*              : During the readout processing from the RTC time counter, the
*              : carry flag (CF) of the control register 1 (RCR1) is cleared to
*              : "0", the count value from the time counter is read, and the carry
*              : flag is verified. If the carry flag is set when reading out from
*              : the time counter, the readout is determined to be invalid and
*              : the readout processing from the time counter is re-executed.
*              : If the carry flag is not set even after the readout processing
*              : has been executed twice, this function returns DEVDRV_ERROR.
* Arguments    : rtc_time_t * time : Storage area for obtained time
*              :                   :   time->second.value  : Second      (0 to 59)
*              :                   :   time->minute.value  : Minute      (0 to 59)
*              :                   :   time->hour.value    : Hour        (0 to 23)
*              :                   :   time->week.value    : Day of week (0 to 6)
*              :                   :   time->day.value     : Day         (1 to 31)
*              :                   :   time->month.value   : Month       (1 to 12)
*              :                   :   time->year.value    : Year        (0 to 9999)
*              :                   : Specification for obtaining object
*              :                   : (RTC_ENABLE : Do obtain, RTC_DISABLE : Do NOT obtain)
*              :                   :   time->second.enable : Second counter obtaining
*              :                   :   time->minute.enable : Minute counter obtaining
*              :                   :   time->hour.enable   : Hour counter obtaining
*              :                   :   time->week.enable   : Day of week counter obtaining
*              :                   :   time->day.enable    : Day counter obtaining
*              :                   :   time->month.enable  : Month counter obtaining
*              :                   :   time->year.enable   : Year counter obtaining
* Return Value : DEVDRV_SUCCESS : Success in obtaining value from RTC time counter
*              : DEVDRV_ERROR   : Failure in obtaining value from RTC time counter
******************************************************************************/
int32_t R_RTC_GetCnt(rtc_time_t * time)
{
    rtc_time_t        bcd_value;
    uint32_t          read_cnt;
    int32_t           ret;
    volatile uint8_t  dummy_buf_8b;

    /* ==== Argument check ==== */
    ret = RTC_CheckArgTimeEnable(time);
    if (DEVDRV_ERROR == ret)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    ret = DEVDRV_ERROR;

    for (read_cnt = 0; read_cnt < RTC_READ_RPEAT_COUNT; read_cnt++)
    {
        /* ==== Clear carry flag (Alarm flag retains values) ==== */
        dummy_buf_8b = RTC.RCR1;
        RTC.RCR1 = (dummy_buf_8b & (~0x80)) | 0x01; /* CF=0, AF=1 */
        dummy_buf_8b = RTC.RCR1;    /* Dummy read */

        /* ==== Readout time counter values ==== */
        /* ---- Second ---- */
        if (RTC_ENABLE == time->second.enable)
        {
            bcd_value.second.value = RTC.RSECCNT;   /* RSECCNT - Second counter      */
        }
        /* ---- Minute ---- */
        if (RTC_ENABLE == time->minute.enable)
        {
            bcd_value.minute.value = RTC.RMINCNT;   /* RMINCNT - Minute counter      */
        }
        /* ---- Hour ---- */
        if (RTC_ENABLE == time->hour.enable)
        {
            bcd_value.hour.value = RTC.RHRCNT;      /* RHRCNT  - Hour counter        */
        }
        /* ---- Day of week ---- */
        if (RTC_ENABLE == time->week.enable)
        {
            bcd_value.week.value = RTC.RWKCNT;      /* RWKCNT  - Day of week counter */
        }
        /* ---- Day ---- */
        if (RTC_ENABLE == time->day.enable)
        {
            bcd_value.day.value = RTC.RDAYCNT;      /* RDAYCNT - Day counter         */
        }
        /* ---- Month ---- */
        if (RTC_ENABLE == time->month.enable)
        {
            bcd_value.month.value = RTC.RMONCNT;    /* RMONCNT - Month counter       */
        }
        /* ---- Year ---- */
        if (RTC_ENABLE == time->year.enable)
        {
            bcd_value.year.value = RTC.RYRCNT;      /* RYRCNT  - Year counter        */
        }

        /* ==== Read carry flag ==== */
        /* If carry flag is "0", readout time counter value is determined to be valid */
        if (0 == RZA_IO_RegRead_8(&RTC.RCR1, RTC_RCR1_CF_SHIFT, RTC_RCR1_CF))
        {
            ret = DEVDRV_SUCCESS;
            break;
        }
    }

    if (DEVDRV_SUCCESS == ret)
    {
        /* ---- Convert the read BCD-coded time into integer value ---- */
        /* ---- and store it in the area specified by the argument ---- */
        RTC_ConvRegValueToTime(&bcd_value, time);   /* Time counter value -> Time */
    }

    return ret;
}

/******************************************************************************
* Function Name: R_RTC_SetAlarm
* Description  : Sets the alarm time specified by the argument time to the RTC
*              : alarm register. Also, specifies the time (second, minute, hour,
*              : day of week, day, month, and year) for alarm to become active
*              : using member of the argument alarm_enb.
*              : If this function is called in the state of the alarm interrupt
*              : enable, disables the alarm interrupt. If RTC_ENABLE is specified
*              : for the argument time->second.enable, performs BCD-coding for
*              : the alarm time of the argument time->second.value and writes
*              : it to the RTC second alarm register (RSECAR). If RTC_DISABLE
*              : is specified for the argument time->second.enable, writing is
*              : not performed. Otherwise, any other member of the alarm timer
*              : should also be written to the respective RTC alarm registers.
*              : Writes the value of the argument alarm_enb->second to the ENB
*              : bit in the RTC second alarm register (RSECAR). Other member of
*              : the setting information of activating of the alarm time is also
*              : written to the ENB bits in the RTC alarm registers.
*              : Clears the alarm flag (AF bit in the Control Register 1 (RCR1))
*              : to "0". If this function is called in the state of the alarm
*              : interrupt enable, enables the alarm interrupt.
*              : When the time of the alarm register in which "1" has been set
*              : to the ENB bit matches the time counter, "1" is set to the alarm
*              : flag. That means the alarm flag informs that the current time
*              : matches the alarm time.
* Arguments    : rtc_time_t * time : Alarm time
*              :                   :   time->second.value  : Second      (0 to 59)
*              :                   :   time->minute.value  : Minute      (0 to 59)
*              :                   :   time->hour.value    : Hour        (0 to 23)
*              :                   :   time->week.value    : Day of week (0 to 6)
*              :                   :   time->day.value     : Day         (1 to 31)
*              :                   :   time->month.value   : Month       (1 to 12)
*              :                   :   time->year.value    : Year        (0 to 9999)
*              :                   : Specification for setting object
*              :                   : (RTC_ENABLE : Do set, RTC_DISABLE : Do NOT set)
*              :                   :   time->second.enable : Second alarm setting
*              :                   :   time->minute.enable : Minute alarm setting
*              :                   :   time->hour.enable   : Hour alarm setting
*              :                   :   time->week.enable   : Day of week alarm setting
*              :                   :   time->day.enable    : Day alarm setting
*              :                   :   time->month.enable  : Month alarm setting
*              :                   :   time->year.enable   : Year alarm setting
*              : rtc_alarm_enb_t * alarm_enb
*              :                   : Setting information of activating of the
*              :                   : alarm time
*              :                   : (0: Activate alarm time, 1: Deactivate alarm time)
*              :                   :   alarm_enb->second : Second      (0 or 1)
*              :                   :   alarm_enb->minute : Minute      (0 or 1)
*              :                   :   alarm_enb->hour   : Hour        (0 or 1)
*              :                   :   alarm_enb->week   : Day of week (0 or 1)
*              :                   :   alarm_enb->day    : Day         (0 or 1)
*              :                   :   alarm_enb->month  : Month       (0 or 1)
*              :                   :   alarm_enb->year   : Year        (0 or 1)
* Return Value : DEVDRV_SUCCESS : Success in setting value to RTC alarm register
*              : DEVDRV_ERROR   : Failure in setting value to RTC alarm register
******************************************************************************/
int32_t R_RTC_SetAlarm(rtc_time_t * time, rtc_alarm_enb_t * alarm_enb)
{
    rtc_time_t       bcd_value;
    uint8_t          aie;
    volatile uint8_t dummy_buf_8b;
    int32_t          ret;
    (void)dummy_buf_8b;

    /* ==== Argument check ==== */
    ret = RTC_CheckArgTimeEnable(time);
    if (DEVDRV_SUCCESS == ret)
    {
        ret = RTC_CheckArgTimeValue(time);

        if (DEVDRV_SUCCESS == ret)
        {
            ret = RTC_CheckArgAlarmEnbValue(alarm_enb);
        }
    }
    if (DEVDRV_ERROR == ret)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Perform BCD-coding for the alarm time specified by the argument ==== */
    RTC_ConvTimeToRegValue(time, &bcd_value);   /* Alarm time -> Alarm register value */

    /* ==== Disables the alarm interrupt ==== */
    aie = RZA_IO_RegRead_8(&RTC.RCR1, RTC_RCR1_AIE_SHIFT, RTC_RCR1_AIE);
    if (1 == aie)
    {
        RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_AIE_SHIFT, RTC_RCR1_AIE);
    }

    /* ==== Alarm register setting ==== */
    /* ---- Second ---- */
    if (RTC_ENABLE == time->second.enable)
    {
        /* RSECAR - Second alarm */
        RZA_IO_RegWrite_8(&RTC.RSECAR, bcd_value.second.value, RTC_RSECAR_SEC_SHIFT, RTC_RSECAR_SEC);
    }
    /* ---- Minute ---- */
    if (RTC_ENABLE == time->minute.enable)
    {
        /* RMINAR - Minute alarm */
        RZA_IO_RegWrite_8(&RTC.RMINAR, bcd_value.minute.value, RTC_RMINAR_MIN_SHIFT, RTC_RMINAR_MIN);
    }
    /* ---- Hour ---- */
    if (RTC_ENABLE == time->hour.enable)
    {
        /* RHRAR  - Hour alarm */
        RZA_IO_RegWrite_8(&RTC.RHRAR, bcd_value.hour.value, RTC_RHRAR_HR_SHIFT, RTC_RHRAR_HR);
    }
    /* ---- Day of week ---- */
    if (RTC_ENABLE == time->week.enable)
    {
        /* RWKAR  - Day of week alarm */
        RZA_IO_RegWrite_8(&RTC.RWKAR, bcd_value.week.value, RTC_RWKAR_WK_SHIFT, RTC_RWKAR_WK);
    }
    /* ---- Day ---- */
    if (RTC_ENABLE == time->day.enable)
    {
        /* RDAYAR - Day alarm */
        RZA_IO_RegWrite_8(&RTC.RDAYAR, bcd_value.day.value, RTC_RDAYAR_DAY_SHIFT, RTC_RDAYAR_DAY);
    }
    /* ---- Month ---- */
    if (RTC_ENABLE == time->month.enable)
    {
        /* RMONAR - Month alarm */
        RZA_IO_RegWrite_8(&RTC.RMONAR, bcd_value.month.value, RTC_RMONAR_MON_SHIFT, RTC_RMONAR_MON);
    }
    /* ---- Year ---- */
    if (RTC_ENABLE == time->year.enable)
    {
        /* RYRAR  - Year alarm */
        RZA_IO_RegWrite_16(&RTC.RYRAR, bcd_value.year.value, RTC_RYRAR_YR_SHIFT, RTC_RYRAR_YR);
    }

    /* ==== ENB bit setting ==== */
    /* ---- Second ---- */
    RZA_IO_RegWrite_8(&RTC.RSECAR, alarm_enb->second, RTC_RSECAR_ENB_SHIFT, RTC_RSECAR_ENB);
    /* ---- Minute ---- */
    RZA_IO_RegWrite_8(&RTC.RMINAR, alarm_enb->minute, RTC_RMINAR_ENB_SHIFT, RTC_RMINAR_ENB);
    /* ---- Hour ---- */
    RZA_IO_RegWrite_8(&RTC.RHRAR, alarm_enb->hour, RTC_RHRAR_ENB_SHIFT, RTC_RHRAR_ENB);
    /* ---- Day of week ---- */
    RZA_IO_RegWrite_8(&RTC.RWKAR, alarm_enb->week, RTC_RWKAR_ENB_SHIFT, RTC_RWKAR_ENB);
    /* ---- Day ---- */
    RZA_IO_RegWrite_8(&RTC.RDAYAR, alarm_enb->day, RTC_RDAYAR_ENB_SHIFT, RTC_RDAYAR_ENB);
    /* ---- Month ---- */
    RZA_IO_RegWrite_8(&RTC.RMONAR, alarm_enb->month, RTC_RMONAR_ENB_SHIFT, RTC_RMONAR_ENB);
    /* ---- Year ---- */
    RZA_IO_RegWrite_8(&RTC.RCR3, alarm_enb->year, RTC_RCR3_ENB_SHIFT, RTC_RCR3_ENB);

    /* ==== Clear alarm flag ==== */
    RZA_IO_RegWrite_8(&RTC.RCR1, 0, RTC_RCR1_AF_SHIFT, RTC_RCR1_AF);
    dummy_buf_8b = RTC.RCR1;    /* Dummy read */

    /* ==== Enables the alarm interrupt ==== */
    if (1 == aie)
    {
        RZA_IO_RegWrite_8(&RTC.RCR1, 1, RTC_RCR1_AIE_SHIFT, RTC_RCR1_AIE);
    }

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RTC_GetAlarm
* Description  : Obtains the alarm time from the RTC alarm register and stores
*              : it in the area specified by the argument time.
*              : If RTC_ENABLE is specified for the argument time->second.enable,
*              : converts the BCD-coded alarm time read from the RTC second alarm
*              : register (RSECAR) into integer value and stores it in the argument
*              : time->second.value. If RTC_DISABLE is specified for the argument
*              : time->second.enable, any readout is not performed. Any other
*              : member of the alarm time is read out from the respective RTC
*              : alarm registers.
*              : Reads the setting information of activating of the alarm time
*              : from the ENB bit in the second alarm register (RSECAR) and
*              : stores it in the argument alarm_enb->second. Other member of
*              : the setting information of activating of the alarm time is
*              : also read from the ENB bits in the RTC alarm registers.
* Arguments    : rtc_time_t * time : Storage area for obtained alarm time
*              :                   :   time->second.value  : Second      (0 to 59)
*              :                   :   time->minute.value  : Minute      (0 to 59)
*              :                   :   time->hour.value    : Hour        (0 to 23)
*              :                   :   time->week.value    : Day of week (0 to 6)
*              :                   :   time->day.value     : Day         (1 to 31)
*              :                   :   time->month.value   : Month       (1 to 12)
*              :                   :   time->year.value    : Year        (0 to 9999)
*              :                   : Specification for obtaining object
*              :                   : (RTC_ENABLE : Do obtain, RTC_DISABLE : Do NOT obtain)
*              :                   :   time->second.enable : Second alarm obtaining
*              :                   :   time->minute.enable : Minute alarm obtaining
*              :                   :   time->hour.enable   : Hour alarm obtaining
*              :                   :   time->week.enable   : Day of week alarm obtaining
*              :                   :   time->day.enable    : Day alarm obtaining
*              :                   :   time->month.enable  : Month alarm obtaining
*              :                   :   time->year.enable   : Year alarm obtaining
* Arguments    : rtc_alarm_enb_t * alarm_enb
*              :                   : Storage area for obtained setting information
*              :                   : of activating of the alarm time
*              :                   : (0: Activate alarm time, 1: Deactivate alarm time)
*              :                   :   alarm_enb->second : Second      (0 or 1)
*              :                   :   alarm_enb->minute : Minute      (0 or 1)
*              :                   :   alarm_enb->hour   : Hour        (0 or 1)
*              :                   :   alarm_enb->week   : Day of week (0 or 1)
*              :                   :   alarm_enb->day    : Day         (0 or 1)
*              :                   :   alarm_enb->month  : Month       (0 or 1)
*              :                   :   alarm_enb->year   : Year        (0 or 1)
* Return Value : DEVDRV_SUCCESS : Success in obtaining value from RTC alarm register
*              : DEVDRV_ERROR   : Failure in obtaining value from RTC alarm register
******************************************************************************/
int32_t R_RTC_GetAlarm(rtc_time_t * time, rtc_alarm_enb_t * alarm_enb)
{
    rtc_time_t   bcd_value;
    int32_t      ret;

    /* ==== Argument check ==== */
    ret = RTC_CheckArgTimeEnable(time);
    if (DEVDRV_ERROR == ret)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Readout alarm register value ==== */
    /* ---- Second ---- */
    if (RTC_ENABLE == time->second.enable)
    {
        /* RSECAR - Second alarm */
        bcd_value.second.value = RZA_IO_RegRead_8(&RTC.RSECAR, RTC_RSECAR_SEC_SHIFT, RTC_RSECAR_SEC);
    }
    /* ---- Minute ---- */
    if (RTC_ENABLE == time->minute.enable)
    {
        /* RMINAR - Minute alarm */
        bcd_value.minute.value = RZA_IO_RegRead_8(&RTC.RMINAR, RTC_RMINAR_MIN_SHIFT, RTC_RMINAR_MIN);
    }
    /* ---- Hour ---- */
    if (RTC_ENABLE == time->hour.enable)
    {
        /* RHRAR  - Hour alarm */
        bcd_value.hour.value = RZA_IO_RegRead_8(&RTC.RHRAR, RTC_RHRAR_HR_SHIFT, RTC_RHRAR_HR);
    }
    /* ---- Day of week ---- */
    if (RTC_ENABLE == time->week.enable)
    {
        /* RWKAR  - Day of week alarm */
        bcd_value.week.value = RZA_IO_RegRead_8(&RTC.RWKAR, RTC_RWKAR_WK_SHIFT, RTC_RWKAR_WK);
    }
    /* ---- Day ---- */
    if (RTC_ENABLE == time->day.enable)
    {
        /* RDAYAR - Day alarm */
        bcd_value.day.value = RZA_IO_RegRead_8(&RTC.RDAYAR, RTC_RDAYAR_DAY_SHIFT, RTC_RDAYAR_DAY);
    }
    /* ---- Month ---- */
    if (RTC_ENABLE == time->month.enable)
    {
        /* RMONAR - Month alarm */
        bcd_value.month.value = RZA_IO_RegRead_8(&RTC.RMONAR, RTC_RMONAR_MON_SHIFT, RTC_RMONAR_MON);
    }
    /* ---- Year ---- */
    if (RTC_ENABLE == time->year.enable)
    {
        /* RYRAR  - Year alarm */
        bcd_value.year.value = RZA_IO_RegRead_16(&RTC.RYRAR, RTC_RYRAR_YR_SHIFT, RTC_RYRAR_YR);
    }

    /* ---- Convert the read BCD-coded alarm time into integer value ---- */
    /* ---- and store it in the area specified by the argument       ---- */
    RTC_ConvRegValueToTime(&bcd_value, time);   /* Alarm register value -> Alarm time */

    /* ==== Readout ENB bit ==== */
    /* ---- Second ---- */
    alarm_enb->second = RZA_IO_RegRead_8(&RTC.RSECAR, RTC_RSECAR_ENB_SHIFT, RTC_RSECAR_ENB);
    /* ---- Minute ---- */
    alarm_enb->minute = RZA_IO_RegRead_8(&RTC.RMINAR, RTC_RMINAR_ENB_SHIFT, RTC_RMINAR_ENB);
    /* ---- Hour ---- */
    alarm_enb->hour = RZA_IO_RegRead_8(&RTC.RHRAR, RTC_RHRAR_ENB_SHIFT, RTC_RHRAR_ENB);
    /* ---- Day of week ---- */
    alarm_enb->week = RZA_IO_RegRead_8(&RTC.RWKAR, RTC_RWKAR_ENB_SHIFT, RTC_RWKAR_ENB);
    /* ---- Day ---- */
    alarm_enb->day = RZA_IO_RegRead_8(&RTC.RDAYAR, RTC_RDAYAR_ENB_SHIFT, RTC_RDAYAR_ENB);
    /* ---- Month ---- */
    alarm_enb->month = RZA_IO_RegRead_8(&RTC.RMONAR, RTC_RMONAR_ENB_SHIFT, RTC_RMONAR_ENB);
    /* ---- Year ---- */
    alarm_enb->year = RZA_IO_RegRead_8(&RTC.RCR3, RTC_RCR3_ENB_SHIFT, RTC_RCR3_ENB);

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: RTC_CheckArgTimeEnable
* Description  : Checks the range of the argument time and returns the result.
*              : In this function, the range of the member enable of time structure
*              : should be checked.
* Arguments    : rtc_time_t * time : Time
* Return Value : DEVDRV_SUCCESS : Argument within the range
*              : DEVDRV_ERROR   : Argument outside the range
******************************************************************************/
static int32_t RTC_CheckArgTimeEnable(rtc_time_t * time)
{
    /* ==== Check range ==== */
    /* ---- Second ---- */
    if (time->second.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Minute ---- */
    if (time->minute.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Hour ---- */
    if (time->hour.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Day of week ---- */
    if (time->week.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Day ---- */
    if (time->day.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Month ---- */
    if (time->month.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Year ---- */
    if (time->year.enable > RTC_ENABLE)
    {
        return DEVDRV_ERROR;
    }

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: RTC_CheckArgTimeValue
* Description  : Checks the range of the argument time and returns the result.
*              : In this function, the range of the member value of time structure
*              : should be check.
* Arguments    : rtc_time_t * time : Time
* Return Value : DEVDRV_SUCCESS : Argument within the range
*              : DEVDRV_ERROR   : Argument outside the range
******************************************************************************/
static int32_t RTC_CheckArgTimeValue(rtc_time_t * time)
{
    /* ==== Check range ==== */
    /* ---- Second ---- */
    if (RTC_ENABLE == time->second.enable)
    {
        if (time->second.value > 59)
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Minute ---- */
    if (RTC_ENABLE == time->minute.enable)
    {
        if (time->minute.value > 59)
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Hour ---- */
    if (RTC_ENABLE == time->hour.enable)
    {
        if (time->hour.value > 23)
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Day of week ---- */
    if (RTC_ENABLE == time->week.enable)
    {
        if (time->week.value > 6)
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Day ---- */
    if (RTC_ENABLE == time->day.enable)
    {
        if ((time->day.value < 1) || (time->day.value > 31))
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Month ---- */
    if (RTC_ENABLE == time->month.enable)
    {
        if ((time->month.value < 1) || (time->month.value > 12))
        {
            return DEVDRV_ERROR;
        }
    }
    /* ---- Year ---- */
    if (RTC_ENABLE == time->year.enable)
    {
        if (time->year.value > 9999)
        {
            return DEVDRV_ERROR;
        }
    }

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: RTC_CheckArgAlarmEnbValue
* Description  : Checks the range of the argument alarm_enb and returns the result.
* Arguments    : rtc_alarm_enb_t * alarm_enb : Alarm-specified information
* Return Value : DEVDRV_SUCCESS : Argument within the range
*              : DEVDRV_ERROR   : Argument outside the range
******************************************************************************/
static int32_t RTC_CheckArgAlarmEnbValue(rtc_alarm_enb_t * alarm_enb)
{
    /* ==== Check range ==== */
    /* ---- Second ---- */
    if (alarm_enb->second > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Minute ---- */
    if (alarm_enb->minute > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Hour ---- */
    if (alarm_enb->hour > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Day of week ---- */
    if (alarm_enb->week > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Day ---- */
    if (alarm_enb->day > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Month ---- */
    if (alarm_enb->month > 1)
    {
        return DEVDRV_ERROR;
    }
    /* ---- Year ---- */
    if (alarm_enb->year > 1)
    {
        return DEVDRV_ERROR;
    }

    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: RTC_ConvTimeToRegValue
* Description  : Performs BCD-coding for the time specified by the argument time
*              : and writes it to the area specified by the argument reg_value.
*              : However, The conversion is performed only for the time if "1"
*              : is specified for the member enable of the argument time.
* Arguments    : rtc_time_t * time      : Time (0 to 9999)
*              : rtc_time_t * reg_value : Register value (BCD)
* Return Value : none
******************************************************************************/
static void RTC_ConvTimeToRegValue(rtc_time_t * time, rtc_time_t * reg_value)
{
    /* ==== Convert from time to register value ==== */
    /* ---- Second ---- */
    if (RTC_ENABLE == time->second.enable)
    {
        reg_value->second.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->second.value));
    }
    /* ---- Minute ---- */
    if (RTC_ENABLE == time->minute.enable)
    {
        reg_value->minute.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->minute.value));
    }
    /* ---- Hour ---- */
    if (RTC_ENABLE == time->hour.enable)
    {
        reg_value->hour.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->hour.value));
    }
    /* ---- Day of week ---- */
    if (RTC_ENABLE == time->week.enable)
    {
        reg_value->week.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->week.value));
    }
    /* ---- Day ---- */
    if (RTC_ENABLE == time->day.enable)
    {
        reg_value->day.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->day.value));
    }
    /* ---- Month ---- */
    if (RTC_ENABLE == time->month.enable)
    {
        reg_value->month.value = (uint8_t)RTC_ConvToBcd((uint16_t)(time->month.value));
    }
    /* ---- Year ---- */
    if (RTC_ENABLE == time->year.enable)
    {
        reg_value->year.value = RTC_ConvToBcd(time->year.value);
    }
}

/******************************************************************************
* Function Name: RTC_ConvRegValueToTime
* Description  : Converts the BCD-coded time specified by the argument reg_value
*              : into integer value and writes it to the area specified by the
*              : argument time. However, this conversion is performed only for
*              : the time if "1" is specified for the member enable of the argument
*              : time.
* Arguments    : rtc_time_t * reg_value : Register value (BCD)
*              : rtc_time_t * time      : Time (0 to 9999)
* Return Value : none
******************************************************************************/
static void RTC_ConvRegValueToTime(rtc_time_t * reg_value, rtc_time_t * time)
{
    /* ==== Convert from register value to time ==== */
    /* ---- Second ---- */
    if (RTC_ENABLE == time->second.enable)
    {
        time->second.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->second.value));
    }
    /* ---- Minute ---- */
    if (RTC_ENABLE == time->minute.enable)
    {
        time->minute.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->minute.value));
    }
    /* ---- Hour ---- */
    if (RTC_ENABLE == time->hour.enable)
    {
        time->hour.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->hour.value));
    }
    /* ---- Day of week ---- */
    if (RTC_ENABLE == time->week.enable)
    {
        time->week.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->week.value));
    }
    /* ---- Day ---- */
    if (RTC_ENABLE == time->day.enable)
    {
        time->day.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->day.value));
    }
    /* ---- Month ---- */
    if (RTC_ENABLE == time->month.enable)
    {
        time->month.value = (uint8_t)RTC_ConvFromBcd((uint16_t)(reg_value->month.value));
    }
    /* ---- Year ---- */
    if (RTC_ENABLE == time->year.enable)
    {
        time->year.value = RTC_ConvFromBcd(reg_value->year.value);
    }
}

/******************************************************************************
* Function Name: RTC_ConvToBcd
* Description  : Performs BCD-coding for the value specified by the argument value
*              : and returns.
* Arguments    : uint16_t value : Value (0 to 9999)
* Return Value : uint16_t : BCD-coded value
******************************************************************************/
static uint16_t RTC_ConvToBcd(uint16_t value)
{
    uint16_t bcd;

    bcd = 0;

    /* ==== BCD-coding ==== */
    bcd += (value / 1000) * 0x1000;
    bcd += ((value / 100) % 10) * 0x100;
    bcd += ((value / 10) % 10) * 0x10;
    bcd += value % 10;

    return bcd;
}

/******************************************************************************
* Function Name: RTC_ConvFromBcd
* Description  : Converts the BCD-coded value specified by the argument bcd into
*              : integer value and returns.
* Arguments    : uint16_t bcd : Value (BCD)
* Return Value : uint16_t : Integer value
******************************************************************************/
static uint16_t RTC_ConvFromBcd(uint16_t bcd)
{
    uint16_t value;

    value = 0;

    /* ==== Convert BCD-coded value into integer value ==== */
    value += (bcd / 0x1000) * 1000;
    value += ((bcd / 0x100) % 0x10) * 100;
    value += ((bcd / 0x10) % 0x10) * 10;
    value += bcd % 0x10;

    return value;
}


/* End of File */

