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
/******************************************************************************
* File Name    : devdrv_rtc.h
* $Rev: 1131 $
* $Date:: 2014-08-07 15:23:35 +0900#$
* Description  : Realtime Clock(RTC) driver header
******************************************************************************/
#ifndef DEVDRV_RTC_H
#define DEVDRV_RTC_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/


/******************************************************************************
Typedef definitions
******************************************************************************/
/* ==== Type declaration for time setting ==== */
/* ---- Type declaration for registers with 8-bit width ---- */
typedef struct rtc_8
{
    uint8_t value;      /* Setting values for time counter or alarm register  */
    uint8_t enable;     /* Specification for setting or acquisition           */
                        /* (RTC_DISABLE : Disable setting or acquisition      */
                        /*  RTC_ENABLE  : Enable setting or acquisition)      */
} rtc_8_t;

/* ---- Type declaration for registers with 16-bit width (For year counter and year alarm registers) ---- */
typedef struct rtc_16
{
    uint16_t value;     /* Setting values for year counter or year alarm register */
    uint8_t  enable;    /* Specification for setting or acquisition               */
                        /* (RTC_DISABLE : Disable setting or acquisition          */
                        /*  RTC_ENABLE  : Enable setting or acquisition)          */
} rtc_16_t;

/* ==== Structure declaration for time setting ==== */
typedef struct rtc_time
{                       /*             value       enable                      */
    rtc_8_t  second;    /* Second      (0 to 59)   (RTC_DISABLE or RTC_ENABLE) */
    rtc_8_t  minute;    /* Minute      (0 to 59)   (RTC_DISABLE or RTC_ENABLE) */
    rtc_8_t  hour;      /* Hour        (0 to 23)   (RTC_DISABLE or RTC_ENABLE) */
    rtc_8_t  week;      /* Day of week (0 to 6)    (RTC_DISABLE or RTC_ENABLE) */
    rtc_8_t  day;       /* Day         (1 to 31)   (RTC_DISABLE or RTC_ENABLE) */
    rtc_8_t  month;     /* Month       (1 to 12)   (RTC_DISABLE or RTC_ENABLE) */
    rtc_16_t year;      /* Year        (0 to 9999) (RTC_DISABLE or RTC_ENABLE) */
} rtc_time_t;

/* ==== Structure declaration for alarm-specified information (ENB bit) ==== */
typedef struct rtc_alarm_enb
{
    uint8_t second;     /* Second      (0 or 1) */
    uint8_t minute;     /* Minute      (0 or 1) */
    uint8_t hour;       /* Hour        (0 or 1) */
    uint8_t week;       /* Day of week (0 or 1) */
    uint8_t day;        /* Day         (0 or 1) */
    uint8_t month;      /* Month       (0 or 1) */
    uint8_t year;       /* Year        (0 or 1) */
} rtc_alarm_enb_t;

/* ==== Day of week definitions ==== */
typedef enum rtc_week
{
    RTC_WK_SUNDAY,      /* Sunday    */
    RTC_WK_MONDAY,      /* Monday    */
    RTC_WK_TUESDAY,     /* Tuesday   */
    RTC_WK_WEDNESDAY,   /* Wednesday */
    RTC_WK_THURSDAY,    /* Thursday  */
    RTC_WK_FRIDAY,      /* Friday    */
    RTC_WK_SATURDAY,    /* Saturday  */
    RTC_WK_TOTAL
} rtc_week_t;

/******************************************************************************
Macro definitions
******************************************************************************/
/* Enable or Disable setting and reading time counter (Second, minute, hour, day of week, day, month, and year) */
#define RTC_ENABLE          (1)     /* Enable  */
#define RTC_DISABLE         (0)     /* Disable */

/******************************************************************************
Variable Externs
******************************************************************************/


/******************************************************************************
Functions Prototypes
******************************************************************************/
/* ==== API functions ==== */
void    R_RTC_Init(void);
void    R_RTC_Open(void);
void    R_RTC_Close(void);
int32_t R_RTC_SetCnt(rtc_time_t * time);
int32_t R_RTC_GetCnt(rtc_time_t * time);
int32_t R_RTC_SetAlarm(rtc_time_t * time, rtc_alarm_enb_t * alarm_enb);
int32_t R_RTC_GetAlarm(rtc_time_t * time, rtc_alarm_enb_t * alarm_enb);

/* ==== User-defined functions ==== */
void Userdef_RTC_Init(void);


#endif  /* DEVDRV_RTC_H */

/* End of File */
