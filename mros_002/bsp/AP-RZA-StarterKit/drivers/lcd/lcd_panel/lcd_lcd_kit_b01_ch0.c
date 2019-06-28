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
/**************************************************************************//**
* @file         lcd_lcd_kit_b01_ch0.c
* @version      1.00
* $Rev: 199 $
* $Date:: 2014-05-23 16:33:52 +0900#$
* @brief        LCD panel for vdc5 channel 0 function
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include    <string.h>

#include    "r_typedefs.h"
#include    "r_vdc5.h"

#include    "iodefine.h"
#include    "lcd_panel.h"
#include    "dev_drv.h"
#include    "devdrv_riic.h"
#include <syslog.h>
#ifdef DEBUG_VDC5
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

#if     (LCD_VDC5_CH0_PANEL==2)

/******************************************************************************
Macro definitions
******************************************************************************/
#if 0 //tokuyama@kmg
/* Port 11 */
#define LCD_PORT11_5TH (0x90FFu)
/* Port 10 */
#define LCD_PORT10_5TH (0xFFC0u)

#define FH_1_2_CYCLE   (2u)

#define     GPIO_P1_1_TO_0          (0x0003u)
#define     LCD_PANEL_RIIC_CH       ((uint32_t)DEVDRV_CH_0)
#define     WAIT_COUNT              ((uint32_t)0x10000u)
#else /* 0 */
/* Port 4 bit: 0, 7, 8-15 */
#define LCD_PORT4_1ST (0x00FFu)
/* Port 3 bit: 0-7 */
#define LCD_PORT3_1ST (0xFF87u)

#define FH_1_2_CYCLE   (2u)

#define     GPIO_P1_3_TO_2          (0x000Cu)
#define     LCD_PANEL_RIIC_CH       ((uint32_t)DEVDRV_CH_1)
#define     WAIT_COUNT              ((uint32_t)0x10000u)
#endif /* 0 */

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
static void set_BackLight_using_iic(void);


/**************************************************************************//**
 * @brief       Backlight setting using RIIC
 * @param[in]   void
 * @retval      None
 *****************************************************************************/
static void set_BackLight_using_iic (void)
{
    int32_t             ret;
    uint8_t             buff[2u];
    volatile uint32_t   wait;
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_E");
    ret = R_RIIC_Init(LCD_PANEL_RIIC_CH);
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_Init() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }
    /* Wait for more than 500 usec to wait for the completion of R8C reset. */
    for (wait = 0; wait < WAIT_COUNT; wait++)
    {
    }
    ret = R_RIIC_SendCond(LCD_PANEL_RIIC_CH, RIIC_TX_MODE_START);
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_SendCnd() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }

    /* Transmit slave address */
    ret = R_RIIC_WriteSlaveAddr(
            LCD_PANEL_RIIC_CH,
            (uint16_t)(LCD_KIT_RIIC_SLAVE_ADDR_R8C | LCD_KIT_RIIC_RW_BIT_WRITE),
            RIIC_TEND_WAIT_TRANSMIT,
            0u);
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_WriteSlaveAddr() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }

    /* Transmit data */
    buff[0] = (uint8_t)LCD_KIT_B01_R8C_CMD_BL;      /* Backlight */
    buff[1] = LCD_KIT_B01_BACKLIGHT;
    ret = R_RIIC_Write(LCD_PANEL_RIIC_CH, buff, (uint32_t)(sizeof buff / (sizeof (uint8_t))));
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_Write() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }

    ret = R_RIIC_SendCond(LCD_PANEL_RIIC_CH, RIIC_TX_MODE_STOP);
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_SendCond() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }
    ret = R_RIIC_DetectStop(LCD_PANEL_RIIC_CH);
    if (DEVDRV_SUCCESS != ret)
    {
TOPPERS_SYSLOG(LOG_INFO, "ERROR* R_RIIC_DetectStop() = %d", ret);
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
        return;
    }
TOPPERS_SYSLOG(LOG_INFO, "set_BackLight_using_iic()_X");
}   /* End of function set_BackLight_using_iic() */

/**************************************************************************//**
 * @brief       LCD panel I/O port setup (VDC5 channel 0)
 * @param[in]   void
 * @retval      None
******************************************************************************/
void GRAPHICS_SetLcdPanel_Ch0 (void)
{
    volatile uint32_t   dummy_read;
    uint32_t            reg_data;

TOPPERS_SYSLOG(LOG_INFO, "GRAPHICS_SetLcdPanel_Ch0()_E");
#if 0 //tokuyama@kmg
    /*  LCD-KIT-B01 (RGB666)
                        : LCD0_CLK              ... P11_15, 5th alternative function
            DE          : LCD0_TCON2            ... P11_12, 5th alternative function
        LCD0_DATA
            R[5:0]      : LCD0_DATA[17:12]      ... P10_6 ~ P10_11, 5th alternative function
            G[5:0]      : LCD0_DATA[11:8]       ... P10_12 ~ P10_15, 5th alternative function
                        : LCD0_DATA[7:6]        ... P11_0 ~ P11_1, 5th alternative function
            B[5:0]      : LCD0_DATA[5:0]        ... P11_2 ~ P11_7, 5th alternative function
    */

    /* Port 11 */
    reg_data        = (uint32_t)GPIO.PMC11 & (uint32_t)~LCD_PORT11_5TH;
    GPIO.PMC11      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC11;
    dummy_read      = reg_data;
    /* PFCAE11, PFCE11, PFC11 ... 5th alternative function
       PIPC11, PMC11
       b15      : P11_15
       b12      : P11_12
       b7:b0    : P11_7 ~ P11_0 */
    reg_data        = (uint32_t)GPIO.PFCAE11 | (uint32_t)LCD_PORT11_5TH;
    GPIO.PFCAE11    = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE11 & (uint32_t)~LCD_PORT11_5TH;
    GPIO.PFCE11     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC11 & (uint32_t)~LCD_PORT11_5TH;
    GPIO.PFC11      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC11 | (uint32_t)LCD_PORT11_5TH;
    GPIO.PIPC11     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC11 | (uint32_t)LCD_PORT11_5TH;
    GPIO.PMC11      = (uint16_t)reg_data;

    /* Port 10 */
    reg_data        = (uint32_t)GPIO.PMC10 & (uint32_t)~LCD_PORT10_5TH;
    GPIO.PMC10      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC10;
    dummy_read      = reg_data;
    /* PFCAE10, PFCE10, PFC10 ... 5th alternative function
       PIPC10, PMC10
       b15:b6   : P10_15 ~ P10_6 */
    reg_data        = (uint32_t)GPIO.PFCAE10 | (uint32_t)LCD_PORT10_5TH;
    GPIO.PFCAE10    = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE10 & (uint32_t)~LCD_PORT10_5TH;
    GPIO.PFCE10     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC10 & (uint32_t)~LCD_PORT10_5TH;
    GPIO.PFC10      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC10 | (uint32_t)LCD_PORT10_5TH;
    GPIO.PIPC10     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC10 | (uint32_t)LCD_PORT10_5TH;
    GPIO.PMC10      = (uint16_t)reg_data;
#else
    /*  LCD-KIT-B01 (RGB666)
                        : LCD0_CLK              ... P3_0, 1st alternative function
            DE          : LCD0_TCON6            ... P3_7, 1st alternative function
        LCD0_DATA
            R[5:0]      : LCD0_DATA[17:12]      ... P4_3 ~ P4_7, 1st alternative function
            G[5:0]      : LCD0_DATA[11:9]       ... P4_0 ~ P4_2, 1st alternative function
                        : LCD0_DATA[8:6]        ... P3_13 ~ P3_15, 1st alternative function
            B[5:0]      : LCD0_DATA[5:0]        ... P3_8 ~ P3_12, 1st alternative function
    */
    /* Port 3 */
    reg_data        = (uint32_t)GPIO.PMC3 | (uint32_t)LCD_PORT3_1ST;
    GPIO.PMC3      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC3;
    dummy_read      = reg_data;
    /* PFCAE3, PFCE3, PFC3 ... 1st alternative function
       PIPC3, PMC3
       b15      : P3_0
       b12      : P3_7
       b7:b0    : P3_14 ~ P3_8 */
    reg_data        = (uint32_t)GPIO.PFCAE3 & (uint32_t)~LCD_PORT3_1ST;
    GPIO.PFCAE3    = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE3 & (uint32_t)~LCD_PORT3_1ST;
    GPIO.PFCE3     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC3 & (uint32_t)~LCD_PORT3_1ST;
    GPIO.PFC3      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC3 | (uint32_t)LCD_PORT3_1ST;
    GPIO.PIPC3     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC3 | (uint32_t)LCD_PORT3_1ST;
    GPIO.PMC3      = (uint16_t)reg_data;

    /* Port 4 */
    reg_data        = (uint32_t)GPIO.PMC4 | (uint32_t)LCD_PORT4_1ST;
    GPIO.PMC4      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC4;
    dummy_read      = reg_data;
    /* PFCAE4, PFCE4, PFC4 ... 1st alternative function
       PIPC4, PMC4
       b15:b6   : P4_7 ~ P4_0 */
    reg_data        = (uint32_t)GPIO.PFCAE4 & (uint32_t)~LCD_PORT4_1ST;
    GPIO.PFCAE4    = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE4 & (uint32_t)~LCD_PORT4_1ST;
    GPIO.PFCE4     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC4 & (uint32_t)~LCD_PORT4_1ST;
    GPIO.PFC4      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC4 | (uint32_t)LCD_PORT4_1ST;
    GPIO.PIPC4     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC4 | (uint32_t)LCD_PORT4_1ST;
    GPIO.PMC4      = (uint16_t)reg_data;
#endif /* 0 */

#if 0 //tokuyama@kmg
    /* Backlight LED
        I2C channel 0
        I2C slave address: b'100 0010
        Command: 0x03
        Data: 0x00 (default) - 0xFF */
    /* RIIC channel 0 */
    /* Port 1 */
    reg_data        = (uint32_t)GPIO.PIBC1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PIBC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PBDC1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PBDC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PM1 | (uint32_t)GPIO_P1_1_TO_0;
    GPIO.PM1        = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PMC1       = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1;
    dummy_read      = reg_data;
    reg_data        = (uint32_t)GPIO.PIPC1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PIPC1      = (uint16_t)reg_data;
    /* PBDC1
       PFCAE1, PFCE1, PFC1 ... 1st alternative function
       PIPC1, PMC1
       b1:b0    : P1_1, P1_0 */
    reg_data        = (uint32_t)GPIO.PBDC1 | (uint32_t)GPIO_P1_1_TO_0;
    GPIO.PBDC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCAE1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PFCAE1     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PFCE1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC1 & (uint32_t)~GPIO_P1_1_TO_0;
    GPIO.PFC1       = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC1 | (uint32_t)GPIO_P1_1_TO_0;
    GPIO.PIPC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1 | (uint32_t)GPIO_P1_1_TO_0;
    GPIO.PMC1       = (uint16_t)reg_data;
#else
    /* Backlight LED
        I2C channel 1 //tokuyama@kmg
        I2C slave address: b'100 0010
        Command: 0x03
        Data: 0x00 (default) - 0xFF */ // 0x64 以上はすべて 100%
    /* RIIC channel 0 */
    /* Port 1 */
    reg_data        = (uint32_t)GPIO.PIBC1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PIBC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PBDC1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PBDC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PM1 | (uint32_t)GPIO_P1_3_TO_2;
    GPIO.PM1        = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PMC1       = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1;
    dummy_read      = reg_data;
    reg_data        = (uint32_t)GPIO.PIPC1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PIPC1      = (uint16_t)reg_data;
    /* PBDC1
       PFCAE1, PFCE1, PFC1 ... 1st alternative function
       PIPC1, PMC1
       b1:b0    : P1_3, P1_2 */
    reg_data        = (uint32_t)GPIO.PBDC1 | (uint32_t)GPIO_P1_3_TO_2;
    GPIO.PBDC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCAE1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PFCAE1     = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFCE1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PFCE1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PFC1 & (uint32_t)~GPIO_P1_3_TO_2;
    GPIO.PFC1       = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PIPC1 | (uint32_t)GPIO_P1_3_TO_2;
    GPIO.PIPC1      = (uint16_t)reg_data;
    reg_data        = (uint32_t)GPIO.PMC1 | (uint32_t)GPIO_P1_3_TO_2;
    GPIO.PMC1       = (uint16_t)reg_data;
#endif /* 0 */
    set_BackLight_using_iic();
TOPPERS_SYSLOG(LOG_INFO, "GRAPHICS_SetLcdPanel_Ch0()_X");

}   /* End of function GRAPHICS_SetLcdPanel_Ch0() */

/**************************************************************************//**
 * @brief       LCD TCON setup parameter acquisition processing (VDC5 channel 0)
 * @param[out]  outctrl     : Address of the area for storing the LCD TCON timing setup data table
 * @retval      None
******************************************************************************/
void GRAPHICS_SetLcdTconSettings_Ch0 (const vdc5_lcd_tcon_timing_t * * const outctrl)
{
    /* LCD-KIT-B01 (RGB666), WVGA 800x480 */
    /* TCON timing setting, VE */
    static const vdc5_lcd_tcon_timing_t lcd_tcon_timing_VE =
    {
        (uint16_t)(LCD_CH0_DISP_VS * FH_1_2_CYCLE),   /* Signal pulse start position */
        (uint16_t)(LCD_CH0_DISP_VW * FH_1_2_CYCLE),   /* Pulse width */
        VDC5_LCD_TCON_POLMD_NORMAL,
        VDC5_LCD_TCON_REFSEL_HSYNC,
        VDC5_SIG_POL_NOT_INVERTED,          /* Polarity inversion control of signal */
        VDC5_LCD_TCON_PIN_NON,              /* Output pin for LCD driving signal */
        LCD_CH0_OUT_EDGE                    /* Output phase control of signal */
    };
    /* TCON timing setting, HE */
    static const vdc5_lcd_tcon_timing_t lcd_tcon_timing_HE =
    {
        (uint16_t)LCD_CH0_DISP_HS,          /* Signal pulse start position */
        (uint16_t)LCD_CH0_DISP_HW,          /* Pulse width */
        VDC5_LCD_TCON_POLMD_NORMAL,
        VDC5_LCD_TCON_REFSEL_HSYNC,         /* Signal operating reference select */
        VDC5_SIG_POL_NOT_INVERTED,          /* Polarity inversion control of signal */
        VDC5_LCD_TCON_PIN_NON,              /* Output pin for LCD driving signal */
        LCD_CH0_OUT_EDGE                    /* Output phase control of signal */
    };
    /* TCON timing setting, DE */
    static const vdc5_lcd_tcon_timing_t lcd_tcon_timing_DE =
    {
        (uint16_t)0,
        (uint16_t)0,
        VDC5_LCD_TCON_POLMD_NORMAL,
        VDC5_LCD_TCON_REFSEL_HSYNC,
        VDC5_SIG_POL_NOT_INVERTED,          /* Polarity inversion control of signal */
        VDC5_LCD_TCON_PIN_6,                /* Output pin for LCD driving signal */ //tokuyama@kmg
        LCD_CH0_OUT_EDGE                    /* Output phase control of signal */
    };
TOPPERS_SYSLOG(LOG_INFO, "GRAPHICS_SetLcdTconSettings_Ch0()_E");

    if (outctrl != NULL)
    {
        outctrl[VDC5_LCD_TCONSIG_STVA_VS]   = NULL;                 /* STVA/VS: Vsync */
        outctrl[VDC5_LCD_TCONSIG_STVB_VE]   = &lcd_tcon_timing_VE;  /* STVB/VE: Not used */
        outctrl[VDC5_LCD_TCONSIG_STH_SP_HS] = NULL;                 /* STH/SP/HS: Hsync */
        outctrl[VDC5_LCD_TCONSIG_STB_LP_HE] = &lcd_tcon_timing_HE;  /* STB/LP/HE: Not used */
        outctrl[VDC5_LCD_TCONSIG_CPV_GCK]   = NULL;                 /* CPV/GCK: Not used */
        outctrl[VDC5_LCD_TCONSIG_POLA]      = NULL;                 /* POLA: Not used */
        outctrl[VDC5_LCD_TCONSIG_POLB]      = NULL;                 /* POLB: Not used */
        outctrl[VDC5_LCD_TCONSIG_DE]        = &lcd_tcon_timing_DE;  /* DE: Not used */
    }
TOPPERS_SYSLOG(LOG_INFO, "GRAPHICS_SetLcdTconSettings_Ch0()_X");
}   /* End of function GRAPHICS_SetLcdTconSettings_Ch0() */

#endif  /* LCD_VDC5_CH0_PANEL==LCD_CH0_PANEL_LCD_KIT_B01 */

