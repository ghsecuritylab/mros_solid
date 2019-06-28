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
* Copyright (C) 2012 - 2013 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name   : riic_userdef.c
* $Rev: 692 $
* $Date:: 2014-01-30 17:14:18 +0900#$
* Description : RIIC driver (User define function)
*******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_typedefs.h"
#include "dev_drv.h"                /* Device Driver common header */
#include "devdrv_riic.h"            /* RIIC Driver header */
//#include "devdrv_intc.h"            /* INTC Driver Header */ tokuyama@kmg 2016.09.23 ���荞�݊֌W Toppers�Ή�
#include "iodefine.h"
#include "rza_io_regrw.h"
#include <kernel.h>
#include <syslog.h>
#include "solid_intc.h"
//#define DEBUG_RIIC
#ifdef DEBUG_RIIC
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

/******************************************************************************
Typedef definitions
******************************************************************************/
typedef struct
{
    uint16_t    intiicri_id;
    void     (* intiicri_func)(uint32_t int_sense);
    uint16_t    intiicti_id;
    void     (* intiicti_func)(uint32_t int_sense);
    uint16_t    intiictei_id;
    void     (* intiictei_func)(uint32_t int_sense);
} userdef_riic_int_t;

typedef struct
{
    volatile uint8_t  * reg_riicncr1;
    volatile uint8_t  * reg_riicnmr1;
    volatile uint8_t  * reg_riicnbrl;
    volatile uint8_t  * reg_riicnbrh;
    volatile uint8_t  * reg_riicnmr3;
    volatile uint8_t  * reg_riicnfer;
    volatile uint8_t  * reg_riicnier;
    volatile uint8_t  * reg_riicncr2;
    volatile uint8_t  * reg_riicnsr2;
} userdef_riic_reg_t;


/******************************************************************************
Macro definitions
******************************************************************************/
/* ==== Transfer speed setting (CKS bit, BRL bit, and BRH bit) ==== */
/* CPU internal clock (I(clock)) : 400MHz, Internal bus clock (B(clock)) : 133.33MHz  */
/* Peripheral clock1 (P1(clock)) : 66.67MHz, Peripheral clock0 (P0(clock)) : 33.33MHz */
/* RIIC digital noise filter : Used (4 stages)                                        */
/* Setting for the RIIC transfer at the speed of 100kbps (Duty cycle: 0.524)          */
/* when the RIIC internal reference clock (IIC(clock)) is P0(clock)/8 with            */
/* the conditionsabove.                                                               */
#define RIIC_CKS    RIIC_CKS_DIVISION_8 /* I2C clock source : P0(clock)/8 */
#define RIIC_BRL    (14)                /* SCL Low width                  */
#define RIIC_BRH    (16)                /* SCL High width                 */


#define RIIC_INTIICRI_PRI   ((uint8_t)9)    /* INTIICRI interrupt priority    */
#define RIIC_INTIICTI_PRI   ((uint8_t)9)    /* INTIICTI interrupt priority    */
#define RIIC_INTIICTEI_PRI  ((uint8_t)9)    /* INTIICTEI interrupt priority   */
#define RIIC_CHANNEL_NUM    (4u)            /* Number of RIIC channels        */


// �Ƃ肠���� devdrv_intc.h �������������Ă��܂����B tokuyama@kmg 2016.09.23
#define INTC_ID_INTIICTEI0 (189)  /* I2C interface                      */
#define INTC_ID_INTIICRI0 (190)   /*                                    */
#define INTC_ID_INTIICTI0 (191)   /*                                    */
#define INTC_ID_INTIICSPI0 (192)  /*                                    */
#define INTC_ID_INTIICSTI0 (193)  /*                                    */
#define INTC_ID_INTIICNAKI0 (194) /*                                    */
#define INTC_ID_INTIICALI0 (195)  /*                                    */
#define INTC_ID_INTIICTMOI0 (196) /*                                    */
#define INTC_ID_INTIICTEI1 (197)  /*                                    */
#define INTC_ID_INTIICRI1 (198)   /*                                    */
#define INTC_ID_INTIICTI1 (199)   /*                                    */
#define INTC_ID_INTIICSPI1 (200)  /*                                    */
#define INTC_ID_INTIICSTI1 (201)  /*                                    */
#define INTC_ID_INTIICNAKI1 (202) /*                                    */
#define INTC_ID_INTIICALI1 (203)  /*                                    */
#define INTC_ID_INTIICTMOI1 (204) /*                                    */
#define INTC_ID_INTIICTEI2 (205)  /*                                    */
#define INTC_ID_INTIICRI2 (206)   /*                                    */
#define INTC_ID_INTIICTI2 (207)   /*                                    */
#define INTC_ID_INTIICSPI2 (208)  /*                                    */
#define INTC_ID_INTIICSTI2 (209)  /*                                    */
#define INTC_ID_INTIICNAKI2 (210) /*                                    */
#define INTC_ID_INTIICALI2 (211)  /*                                    */
#define INTC_ID_INTIICTMOI2 (212) /*                                    */
#define INTC_ID_INTIICTEI3 (213)  /*                                    */
#define INTC_ID_INTIICRI3 (214)   /*                                    */
#define INTC_ID_INTIICTI3 (215)   /*                                    */
#define INTC_ID_INTIICSPI3 (216)  /*                                    */
#define INTC_ID_INTIICSTI3 (217)  /*                                    */
#define INTC_ID_INTIICNAKI3 (218) /*                                    */
#define INTC_ID_INTIICALI3 (219)  /*                                    */
#define INTC_ID_INTIICTMOI3 (220) /*                                    */


/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


/******************************************************************************
Private global variables and functions
******************************************************************************/
static void Ri0_Interrupt(uint32_t int_sense);
static void Ti0_Interrupt(uint32_t int_sense);
static void Tei0_Interrupt(uint32_t int_sense);
static void Ri1_Interrupt(uint32_t int_sense);
static void Ti1_Interrupt(uint32_t int_sense);
static void Tei1_Interrupt(uint32_t int_sense);
static void Ri2_Interrupt(uint32_t int_sense);
static void Ti2_Interrupt(uint32_t int_sense);
static void Tei2_Interrupt(uint32_t int_sense);
static void Ri3_Interrupt(uint32_t int_sense);
static void Ti3_Interrupt(uint32_t int_sense);
static void Tei3_Interrupt(uint32_t int_sense);

static void set_Interrupt(devdrv_ch_t ch);
static void set_riic_init(devdrv_ch_t ch);

static volatile uint8_t riic_receive_full_flg[RIIC_CHANNEL_NUM];    /* Receive data full flag   */
static volatile uint8_t riic_transmit_empty_flg[RIIC_CHANNEL_NUM];  /* Transmit data empty flag */
static volatile uint8_t riic_transmit_end_flg[RIIC_CHANNEL_NUM];    /* Transmit end flag        */

static const userdef_riic_int_t userdef_riic_int[RIIC_CHANNEL_NUM] =
{
    {INTC_ID_INTIICRI0, Ri0_Interrupt, INTC_ID_INTIICTI0, Ti0_Interrupt, INTC_ID_INTIICTEI0, Tei0_Interrupt},
    {INTC_ID_INTIICRI1, Ri1_Interrupt, INTC_ID_INTIICTI1, Ti1_Interrupt, INTC_ID_INTIICTEI1, Tei1_Interrupt},
    {INTC_ID_INTIICRI2, Ri2_Interrupt, INTC_ID_INTIICTI2, Ti2_Interrupt, INTC_ID_INTIICTEI2, Tei2_Interrupt},
    {INTC_ID_INTIICRI3, Ri3_Interrupt, INTC_ID_INTIICTI3, Ti3_Interrupt, INTC_ID_INTIICTEI3, Tei3_Interrupt}
};
#if 0
static const userdef_riic_reg_t userdef_riic_reg[RIIC_CHANNEL_NUM] =
{
    {
        &(RIIC0.RIICnCR1.UINT8[0]), &(RIIC0.RIICnMR1.UINT8[0]), &(RIIC0.RIICnBRL.UINT8[0]),
        &(RIIC0.RIICnBRH.UINT8[0]), &(RIIC0.RIICnMR3.UINT8[0]), &(RIIC0.RIICnFER.UINT8[0]),
        &(RIIC0.RIICnIER.UINT8[0]), &(RIIC0.RIICnCR2.UINT8[0]), &(RIIC0.RIICnSR2.UINT8[0])
    },
    {
        &(RIIC1.RIICnCR1.UINT8[0]), &(RIIC1.RIICnMR1.UINT8[0]), &(RIIC1.RIICnBRL.UINT8[0]),
        &(RIIC1.RIICnBRH.UINT8[0]), &(RIIC1.RIICnMR3.UINT8[0]), &(RIIC1.RIICnFER.UINT8[0]),
        &(RIIC1.RIICnIER.UINT8[0]), &(RIIC1.RIICnCR2.UINT8[0]), &(RIIC1.RIICnSR2.UINT8[0])
    },
    {
        &(RIIC2.RIICnCR1.UINT8[0]), &(RIIC2.RIICnMR1.UINT8[0]), &(RIIC2.RIICnBRL.UINT8[0]),
        &(RIIC2.RIICnBRH.UINT8[0]), &(RIIC2.RIICnMR3.UINT8[0]), &(RIIC2.RIICnFER.UINT8[0]),
        &(RIIC2.RIICnIER.UINT8[0]), &(RIIC2.RIICnCR2.UINT8[0]), &(RIIC2.RIICnSR2.UINT8[0])
    },
    {
        &(RIIC3.RIICnCR1.UINT8[0]), &(RIIC3.RIICnMR1.UINT8[0]), &(RIIC3.RIICnBRL.UINT8[0]),
        &(RIIC3.RIICnBRH.UINT8[0]), &(RIIC3.RIICnMR3.UINT8[0]), &(RIIC3.RIICnFER.UINT8[0]),
        &(RIIC3.RIICnIER.UINT8[0]), &(RIIC3.RIICnCR2.UINT8[0]), &(RIIC3.RIICnSR2.UINT8[0])
    }
};
#else
static userdef_riic_reg_t userdef_riic_reg[RIIC_CHANNEL_NUM];

static void initialize_userdef_riic0_regaddr_table()
{
	userdef_riic_reg[0].reg_riicncr1 = &(RIIC0.RIICnCR1.UINT8[0]);
	userdef_riic_reg[0].reg_riicnmr1 = &(RIIC0.RIICnMR1.UINT8[0]);
	userdef_riic_reg[0].reg_riicnbrl = &(RIIC0.RIICnBRL.UINT8[0]);
	userdef_riic_reg[0].reg_riicnbrh = &(RIIC0.RIICnBRH.UINT8[0]);
	userdef_riic_reg[0].reg_riicnmr3 = &(RIIC0.RIICnMR3.UINT8[0]);
	userdef_riic_reg[0].reg_riicnfer = &(RIIC0.RIICnFER.UINT8[0]);
	userdef_riic_reg[0].reg_riicnier = &(RIIC0.RIICnIER.UINT8[0]);
	userdef_riic_reg[0].reg_riicncr2 = &(RIIC0.RIICnCR2.UINT8[0]);
	userdef_riic_reg[0].reg_riicnsr2 = &(RIIC0.RIICnSR2.UINT8[0]);
}

static void initialize_userdef_riic1_regaddr_table()
{
	userdef_riic_reg[1].reg_riicncr1 = &(RIIC1.RIICnCR1.UINT8[0]);
	userdef_riic_reg[1].reg_riicnmr1 = &(RIIC1.RIICnMR1.UINT8[0]);
	userdef_riic_reg[1].reg_riicnbrl = &(RIIC1.RIICnBRL.UINT8[0]);
	userdef_riic_reg[1].reg_riicnbrh = &(RIIC1.RIICnBRH.UINT8[0]);
	userdef_riic_reg[1].reg_riicnmr3 = &(RIIC1.RIICnMR3.UINT8[0]);
	userdef_riic_reg[1].reg_riicnfer = &(RIIC1.RIICnFER.UINT8[0]);
	userdef_riic_reg[1].reg_riicnier = &(RIIC1.RIICnIER.UINT8[0]);
	userdef_riic_reg[1].reg_riicncr2 = &(RIIC1.RIICnCR2.UINT8[0]);
	userdef_riic_reg[1].reg_riicnsr2 = &(RIIC1.RIICnSR2.UINT8[0]);
}

static void initialize_userdef_riic2_regaddr_table()
{
	userdef_riic_reg[2].reg_riicncr1 = &(RIIC2.RIICnCR1.UINT8[0]);
	userdef_riic_reg[2].reg_riicnmr1 = &(RIIC2.RIICnMR1.UINT8[0]);
	userdef_riic_reg[2].reg_riicnbrl = &(RIIC2.RIICnBRL.UINT8[0]);
	userdef_riic_reg[2].reg_riicnbrh = &(RIIC2.RIICnBRH.UINT8[0]);
	userdef_riic_reg[2].reg_riicnmr3 = &(RIIC2.RIICnMR3.UINT8[0]);
	userdef_riic_reg[2].reg_riicnfer = &(RIIC2.RIICnFER.UINT8[0]);
	userdef_riic_reg[2].reg_riicnier = &(RIIC2.RIICnIER.UINT8[0]);
	userdef_riic_reg[2].reg_riicncr2 = &(RIIC2.RIICnCR2.UINT8[0]);
	userdef_riic_reg[2].reg_riicnsr2 = &(RIIC2.RIICnSR2.UINT8[0]);
}

static void initialize_userdef_riic3_regaddr_table()
{
	userdef_riic_reg[3].reg_riicncr1 = &(RIIC3.RIICnCR1.UINT8[0]);
	userdef_riic_reg[3].reg_riicnmr1 = &(RIIC3.RIICnMR1.UINT8[0]);
	userdef_riic_reg[3].reg_riicnbrl = &(RIIC3.RIICnBRL.UINT8[0]);
	userdef_riic_reg[3].reg_riicnbrh = &(RIIC3.RIICnBRH.UINT8[0]);
	userdef_riic_reg[3].reg_riicnmr3 = &(RIIC3.RIICnMR3.UINT8[0]);
	userdef_riic_reg[3].reg_riicnfer = &(RIIC3.RIICnFER.UINT8[0]);
	userdef_riic_reg[3].reg_riicnier = &(RIIC3.RIICnIER.UINT8[0]);
	userdef_riic_reg[3].reg_riicncr2 = &(RIIC3.RIICnCR2.UINT8[0]);
	userdef_riic_reg[3].reg_riicnsr2 = &(RIIC3.RIICnSR2.UINT8[0]);
}
#endif

/******************************************************************************
* Function Name: Userdef_RIIC0_Init
* Description  : The RIIC initial setting and the setting for the interrupts 
*              : (receive data full interrupt, transmit data empty interrupt, and 
*              : transmit end interrupt) are required when the RIIC channel 0 is used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_Init(void)
{
    volatile uint8_t dummy_buf;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_Init()_E");

	// first of all regster address table needs to be initlialized
	initialize_userdef_riic0_regaddr_table();

    /* ==== Module standby clear ==== */
    /* ---- Supply clock to the RIIC(channel 0) ---- */
    RZA_IO_RegWrite_8(&CPG.STBCR9, 0, CPG_STBCR9_MSTP97_SHIFT, CPG_STBCR9_MSTP97);
    dummy_buf = CPG.STBCR9;     /* Dummy read */

    /* ==== RIIC initial setting ==== */
    set_riic_init(DEVDRV_CH_0);

    /* ==== Interrupt setting ==== */
    set_Interrupt(DEVDRV_CH_0);
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_Init()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_Init
* Description  : The RIIC initial setting and the setting for the interrupts 
*              : (receive data full interrupt, transmit data empty interrupt, and 
*              : transmit end interrupt) are required when the RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_Init(void)
{
    volatile uint8_t dummy_buf;
	TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_Init()_E");

	/* first of all regster address table needs to be initlialized */
	initialize_userdef_riic1_regaddr_table();

	/* ==== Module standby clear ==== */
    /* ---- Supply clock to the RIIC(channel 1) ---- */
    RZA_IO_RegWrite_8(&CPG.STBCR9, 0, CPG_STBCR9_MSTP96_SHIFT, CPG_STBCR9_MSTP96);
    dummy_buf = CPG.STBCR9;     /* Dummy read */

    /* ==== RIIC initial setting ==== */
    set_riic_init(DEVDRV_CH_1);

    /* ==== Interrupt setting ==== */
    set_Interrupt(DEVDRV_CH_1);
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_Init()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_Init
* Description  : The RIIC initial setting and the setting for the interrupts 
*              : (receive data full interrupt, transmit data empty interrupt, and 
*              : transmit end interrupt) are required when the RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_Init(void)
{
    volatile uint8_t dummy_buf;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_Init()_E");

	// first of all regster address table needs to be initlialized
	initialize_userdef_riic2_regaddr_table();

	/* ==== Module standby clear ==== */
    /* ---- Supply clock to the RIIC(channel 2) ---- */
    RZA_IO_RegWrite_8(&CPG.STBCR9, 0, CPG_STBCR9_MSTP95_SHIFT, CPG_STBCR9_MSTP95);
    dummy_buf = CPG.STBCR9;     /* Dummy read */

    /* ==== RIIC initial setting ==== */
    set_riic_init(DEVDRV_CH_2);

    /* ==== Interrupt setting ==== */
    set_Interrupt(DEVDRV_CH_2);
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_Init()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_Init
* Description  : The RIIC initial setting and the setting for the interrupts 
*              : (receive data full interrupt, transmit data empty interrupt, and 
*              : transmit end interrupt) are required when the RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_Init(void)
{
    volatile uint8_t dummy_buf;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_Init()_E");

	// first of all regster address table needs to be initlialized
	initialize_userdef_riic3_regaddr_table();

	/* ==== Module standby clear ==== */
    /* ---- Supply clock to the RIIC(channel 3) ---- */
    RZA_IO_RegWrite_8(&CPG.STBCR9, 0, CPG_STBCR9_MSTP94_SHIFT, CPG_STBCR9_MSTP94);
    dummy_buf = CPG.STBCR9;     /* Dummy read */

    /* ==== RIIC initial setting ==== */
    set_riic_init(DEVDRV_CH_3);

    /* ==== Interrupt setting ==== */
    set_Interrupt(DEVDRV_CH_3);
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_Init()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_InitReceiveFull
* Description  : Execute this function to initialize the RIIC channel 0 receive
*              : data full notification information when the RIIC channel 0 is used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_InitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_0] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_InitReceiveFull
* Description  : Execute this function to initialize the RIIC channel 1 receive
*              : data full notification information when the RIIC channel 1 is used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_InitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_1] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_InitReceiveFull
* Description  : Execute this function to initialize the RIIC channel 2 receive
*              : data full notification information when the RIIC channel 2 is  used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_InitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_2] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_InitReceiveFull
* Description  : Execute this function to initialize the RIIC channel 3 receive
*              : data full notification information when the RIIC channel 3 isused. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_InitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_3] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_InitTransmitEmpty
* Description  : Execute this function to initialize the RIIC channel 0 transmit
*              : data empty notification information when the RIIC channel 0 is used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_InitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_0] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_InitTransmitEmpty
* Description  : Execute this function to initialize the RIIC channel 1 transmit
*              : data empty notification information when the RIIC channel 1 isused. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_InitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_1] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_InitTransmitEmpty
* Description  : Execute this function to initialize the RIIC channel 2 transmit
*              : data empty notification information when the RIIC channel 2 isused. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_InitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_2] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_InitTransmitEmpty
* Description  : Execute this function to initialize the RIIC channel 3 transmit
*              : data empty notification information when the RIIC channel 3 is used. 
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_InitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_3] = DEVDRV_FLAG_OFF;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_InitTransmitEnd
* Description  : When the RIIC channel 0 is used, execute this function to 
*              : initialize the RIIC channel 0 transmit end notification 
*              : information if the argument mode specifies RIIC_TEND_WAIT_TRANSMIT, 
*              : or initialize the RIIC channel 0 receive data full notification 
*              : information if the argument mode specifies RIIC_TEND_WAIT_RECEIVE.
* Arguments    : uint32_t mode : Continuation of transmission mode or 
*              :               : transition to reception mode
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_InitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitTransmitEnd()_E");
    /* ==== Continue transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        riic_transmit_end_flg[DEVDRV_CH_0] = DEVDRV_FLAG_OFF;
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        riic_receive_full_flg[DEVDRV_CH_0] = DEVDRV_FLAG_OFF;
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_InitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_InitTransmitEnd
* Description  : When the RIIC channel 1 is used, execute this function to 
*              : initialize the RIIC channel 1 transmit end notification 
*              : information if the argument mode specifies RIIC_TEND_WAIT_TRANSMIT, 
*              : or initialize the RIIC channel 1 receive data full notification 
*              : information if the argument mode specifies RIIC_TEND_WAIT_RECEIVE.
* Arguments    : uint32_t mode : Continuation of transmission mode or 
*              :               : transition to reception mode
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_InitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitTransmitEnd()_E");
    /* ==== Continue transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        riic_transmit_end_flg[DEVDRV_CH_1] = DEVDRV_FLAG_OFF;
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        riic_receive_full_flg[DEVDRV_CH_1] = DEVDRV_FLAG_OFF;
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_InitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_InitTransmitEnd
* Description  : Execute this function to initialize the RIIC channel 2 
*              : transmit end notification information.
* Arguments    : uint32_t mode : Continuation of transmission mode or 
*              :               : transition to reception mode
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_InitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitTransmitEnd()_E");
    /* ==== Continue transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        riic_transmit_end_flg[DEVDRV_CH_2] = DEVDRV_FLAG_OFF;
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        riic_receive_full_flg[DEVDRV_CH_2] = DEVDRV_FLAG_OFF;
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_InitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_InitTransmitEnd
* Description  : When the RIIC channel 1 is used, execute this function to 
*              : initialize the RIIC channel 3 transmit end notification 
*              : information if the argument mode specifies RIIC_TEND_WAIT_TRANSMIT, 
*              : or initialize the RIIC channel 3 receive data full notification 
*              : information if the said argument specifies RIIC_TEND_WAIT_RECEIVE.
* Arguments    : uint32_t mode : Continuation of transmission mode or 
*              :               : transition to reception mode
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_InitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitTransmitEnd()_E");
    /* ==== Continue transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        riic_transmit_end_flg[DEVDRV_CH_3] = DEVDRV_FLAG_OFF;
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        riic_receive_full_flg[DEVDRV_CH_3] = DEVDRV_FLAG_OFF;
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_InitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_SetReceiveFull
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 0 receive data full notification information when the 
*              : RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_SetReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_0] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_SetReceiveFull
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 1 receive data full notification information when the 
*              : RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_SetReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_1] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_SetReceiveFull
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 2 receive data full notification information when the 
*              : RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_SetReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_2] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_SetReceiveFull
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 3 receive data full notification information when the 
*              : RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_SetReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetReceiveFull()_E");
    riic_receive_full_flg[DEVDRV_CH_3] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_SetTransmitEmpty
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 0 transmit data empty notification information when the 
*              : RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_SetTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_0] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_SetTransmitEmpty
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 1 transmit data full notification information when the 
*              : RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_SetTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_1] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_SetTransmitEmpty
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 2 transmit data empty notification information when 
*              : the RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_SetTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_2] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_SetTransmitEmpty
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 3 transmit data empty notification information when the 
*              : RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_SetTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetTransmitEmpty()_E");
    riic_transmit_empty_flg[DEVDRV_CH_3] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_SetTransmitEnd
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 0 transmit end notification information when the 
*              : RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_SetTransmitEnd(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetTransmitEnd()_E");
    riic_transmit_end_flg[DEVDRV_CH_0] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_SetTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_SetTransmitEnd
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 1 transmit end notification information when the 
*              : RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_SetTransmitEnd(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetTransmitEnd()_E");
    riic_transmit_end_flg[DEVDRV_CH_1] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_SetTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_SetTransmitEnd
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 2 transmit end notification information when the 
*              : RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_SetTransmitEnd(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetTransmitEnd()_E");
    riic_transmit_end_flg[DEVDRV_CH_2] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_SetTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_SetTransmitEnd
* Description  : Execute this function to satisfy the conditions for the RIIC
*              : channel 3 transmit end notification information when the 
*              : RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_SetTransmitEnd(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetTransmitEnd()_E");
    riic_transmit_end_flg[DEVDRV_CH_3] = DEVDRV_FLAG_ON;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_SetTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_WaitReceiveFull
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 0 receive data full notification information are 
*              : satisfied when the RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_WaitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitReceiveFull()_E");
    while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_0])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_WaitReceiveFull
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 1 receive data full notification information are 
*              : satisfied when the RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_WaitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitReceiveFull()_E");
    while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_1])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_WaitReceiveFull
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 2 receive data full notification information are 
*              : satisfied when the RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_WaitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitReceiveFull()_E");
    while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_2])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_WaitReceiveFull
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 3 receive data full notification information are 
*              : satisfied when the RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_WaitReceiveFull(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitReceiveFull()_E");
    while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_3])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitReceiveFull()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_WaitTransmitEmpty
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 0 transmit data empty notification information are 
*              : satisfied when the RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_WaitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitTransmitEmpty()_E");
    while (DEVDRV_FLAG_OFF == riic_transmit_empty_flg[DEVDRV_CH_0])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_WaitTransmitEmpty
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 1 transmit data empty notification information are 
*              : satisfied when the RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_WaitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitTransmitEmpty()_E");
    while (DEVDRV_FLAG_OFF == riic_transmit_empty_flg[DEVDRV_CH_1])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_WaitTransmitEmpty
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 2 transmit data empty notification information are 
*              : satisfied when the RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_WaitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitTransmitEmpty()_E");
    while (DEVDRV_FLAG_OFF == riic_transmit_empty_flg[DEVDRV_CH_2])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_WaitTransmitEmpty
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 3 transmit data empty notification information are 
*              : satisfied when the RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_WaitTransmitEmpty(void)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitTransmitEmpty()_E");
    while (DEVDRV_FLAG_OFF == riic_transmit_empty_flg[DEVDRV_CH_3])
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitTransmitEmpty()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_WaitTransmitEnd
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 0 transmit end notification information are 
*              : satisfied when the RIIC channel 0 is used.
* Arguments    : uint32_t mode : Selection for wait operation after transmission
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_WaitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitTransmitEnd()_E");
    /* ==== Continuation of transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        while (DEVDRV_FLAG_OFF == riic_transmit_end_flg[DEVDRV_CH_0])
        {
            /* Wait */
        }
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_0])
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_WaitTransmitEnd
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 1 transmit end notification information are 
*              : satisfied when the RIIC channel 1 is used.
* Arguments    : uint32_t mode : Selection for wait operation after transmission
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_WaitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitTransmitEnd()_E");
    /* ==== Continuation of transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        while (DEVDRV_FLAG_OFF == riic_transmit_end_flg[DEVDRV_CH_1])
        {
            /* Wait */
        }
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_1])
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_WaitTransmitEnd
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 2 transmit end notification information are 
*              : satisfied when the RIIC channel 2 is used.
* Arguments    : uint32_t mode : Selection for wait operation after transmission
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_WaitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitTransmitEnd()_E");
    /* ==== Continuation of transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        while (DEVDRV_FLAG_OFF == riic_transmit_end_flg[DEVDRV_CH_2])
        {
            /* Wait */
        }
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_2])
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_WaitTransmitEnd
* Description  : Execute this function to wait until the conditions for the 
*              : RIIC channel 3 transmit end notification information are 
*              : satisfied when the RIIC channel 3 is used.
* Arguments    : uint32_t mode : Selection for wait operation after transmission
*              :               :   RIIC_TEND_WAIT_TRANSMIT : Continuation of transmission mode
*              :               :   RIIC_TEND_WAIT_RECEIVE  : Transition to reception mode
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_WaitTransmitEnd(uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitTransmitEnd()_E");
    /* ==== Continuation of transmission mode ==== */
    if (RIIC_TEND_WAIT_TRANSMIT == mode)
    {
        while (DEVDRV_FLAG_OFF == riic_transmit_end_flg[DEVDRV_CH_3])
        {
            /* Wait */
        }
    }
    /* ==== Transit to reception mode ==== */
    else
    {
        while (DEVDRV_FLAG_OFF == riic_receive_full_flg[DEVDRV_CH_3])
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitTransmitEnd()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_WaitBusMastership
* Description  : Execute this function to wait the RIIC channel 0 bus free and 
*              : bus busy when the RIIC channel 0 is used.
* Arguments    : uint32_t mode
*              :            : Mode selection for bus free or bus busy
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_FREE : Wait for bus free
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_BUSY : Wait for bus busy
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_WaitBusMastership(uint32_t mode)
{
    volatile uint8_t  * riicncr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitBusMastership()_E");

    riicncr2 = userdef_riic_reg[DEVDRV_CH_0].reg_riicncr2;
    /* ==== Bus free wait mode ==== */
    if (RIIC_BUS_MASTERSHIP_WAIT_FREE == mode)
    {
        while (1 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
    /* ==== Bus busy wait mode ==== */
    else
    {
        while (0 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitBusMastership()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_WaitBusMastership
* Description  : Execute this function to wait the RIIC channel 1 bus free and 
*              : bus busy when the RIIC channel 1 is used.
* Arguments    : uint32_t mode
*              :            : Mode selection for bus free or bus busy
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_FREE : Wait for bus free
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_BUSY : Wait for bus busy
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_WaitBusMastership(uint32_t mode)
{
    volatile uint8_t  * riicncr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitBusMastership()_E");

    riicncr2 = userdef_riic_reg[DEVDRV_CH_1].reg_riicncr2;
    /* ==== Bus free wait mode ==== */
    if (RIIC_BUS_MASTERSHIP_WAIT_FREE == mode)
    {
        while (1 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
    /* ==== Bus busy wait mode ==== */
    else
    {
        while (0 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitBusMastership()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_WaitBusMastership
* Description  : Execute this function to wait the RIIC channel 2 bus free and 
*              : bus busy when the RIIC channel 2 is used.
* Arguments    : uint32_t mode
*              :            : Mode selection for bus free or bus busy
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_FREE : Wait for bus free
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_BUSY : Wait for bus busy
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_WaitBusMastership(uint32_t mode)
{
    volatile uint8_t  * riicncr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitBusMastership()_E");

    riicncr2 = userdef_riic_reg[DEVDRV_CH_2].reg_riicncr2;
    /* ==== Bus free wait mode ==== */
    if (RIIC_BUS_MASTERSHIP_WAIT_FREE == mode)
    {
        while (1 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
    /* ==== Bus busy wait mode ==== */
    else
    {
        while (0 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitBusMastership()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_WaitBusMastership
* Description  : Execute this function to wait the RIIC channel 3 bus free and 
*              : bus busy when the RIIC channel 3 is used.
* Arguments    : uint32_t mode
*              :            : Mode selection for bus free or bus busy
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_FREE : Wait for bus free
*              :            :   RIIC_BUS_MASTERSHIP_WAIT_BUSY : Wait for bus busy
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_WaitBusMastership(uint32_t mode)
{
    volatile uint8_t  * riicncr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitBusMastership()_E");

    riicncr2 = userdef_riic_reg[DEVDRV_CH_3].reg_riicncr2;
    /* ==== Bus free wait mode ==== */
    if (RIIC_BUS_MASTERSHIP_WAIT_FREE == mode)
    {
        while (1 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
    /* ==== Bus busy wait mode ==== */
    else
    {
        while (0 == RZA_IO_RegRead_8(riicncr2, RIICn_RIICnCR2_BBSY_SHIFT, RIICn_RIICnCR2_BBSY))
        {
            /* Wait */
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitBusMastership()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC0_WaitStop
* Description  : Execute this function to wait until the RIIC channel 0 stop 
*              : condition is detected when the RIIC channel 0 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC0_WaitStop(void)
{
    volatile uint8_t  * riicnsr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitStop()_E");

    riicnsr2 = userdef_riic_reg[DEVDRV_CH_0].reg_riicnsr2;
    /* === Wait for stop condition detection ==== */
    while (0 == RZA_IO_RegRead_8(riicnsr2, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP))
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC0_WaitStop()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC1_WaitStop
* Description  : Execute this function to wait until the RIIC channel 1 stop 
*              : condition is detected when the RIIC channel 1 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC1_WaitStop(void)
{
    volatile uint8_t  * riicnsr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitStop()_E");

    riicnsr2 = userdef_riic_reg[DEVDRV_CH_1].reg_riicnsr2;
    /* === Wait for stop condition detection ==== */
    while (0 == RZA_IO_RegRead_8(riicnsr2, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP))
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC1_WaitStop()_X");
}

/******************************************************************************
* Function Name: Userdef_RIIC2_WaitStop
* Description  : Execute this function to wait until the RIIC channel 2 stop 
*              : condition is detected when the RIIC channel 2 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC2_WaitStop(void)
{
    volatile uint8_t  * riicnsr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitStop()_E");

    riicnsr2 = userdef_riic_reg[DEVDRV_CH_2].reg_riicnsr2;
    /* === Wait for stop condition detection ==== */
    while (0 == RZA_IO_RegRead_8(riicnsr2, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP))
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC2_WaitStop()_E");
}

/******************************************************************************
* Function Name: Userdef_RIIC3_WaitStop
* Description  : Execute this function to wait until the RIIC channel 3 stop 
*              : condition is detected  when the RIIC channel 3 is used.
* Arguments    : none
* Return Value : none
******************************************************************************/
void Userdef_RIIC3_WaitStop(void)
{
    volatile uint8_t  * riicnsr2;
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitStop()_E");

    riicnsr2 = userdef_riic_reg[DEVDRV_CH_3].reg_riicnsr2;
    /* === Wait for stop condition detection ==== */
    while (0 == RZA_IO_RegRead_8(riicnsr2, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP))
    {
        /* Wait */
    }
TOPPERS_SYSLOG(LOG_INFO, "Userdef_RIIC3_WaitStop()_X");
}

/******************************************************************************
Additional functions
******************************************************************************/
/******************************************************************************
* Description  : An interrupt hander processing to be executed when the RIIC
*              : channel 0 receive data full interrupt is accepted.
*              : The API function R_RIIC_RiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used.)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ri0_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ri0_Interrupt()_E");
    R_RIIC_RiInterrupt(DEVDRV_CH_0);
TOPPERS_SYSLOG(LOG_INFO, "Ri0_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC
*              : channel 0 transmit data empty interrupt is accepted.
*              : The API function R_RIIC_TiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ti0_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ti0_Interrupt()_E");
    R_RIIC_TiInterrupt(DEVDRV_CH_0);
TOPPERS_SYSLOG(LOG_INFO, "Ti0_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC 
*              : channel 0 transmit end interrupt is accepted.
*              : The API function R_RIIC_TeiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Tei0_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Tei0_Interrupt()_E");
    R_RIIC_TeiInterrupt(DEVDRV_CH_0);
TOPPERS_SYSLOG(LOG_INFO, "Tei0_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt hander processing to be executed when the RIIC
*              : channel 1 receive data full interrupt is accepted.
*              : The API function R_RIIC_RiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used.)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ri1_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ri1_Interrupt()_E");
    R_RIIC_RiInterrupt(DEVDRV_CH_1);
TOPPERS_SYSLOG(LOG_INFO, "Ri1_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC
*              : channel 1 transmit data empty interrupt is accepted.
*              : The API function R_RIIC_TiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ti1_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ti1_Interrupt()_E");
    R_RIIC_TiInterrupt(DEVDRV_CH_1);
TOPPERS_SYSLOG(LOG_INFO, "Ti1_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC 
*              : channel 1 transmit end interrupt is accepted.
*              : The API function R_RIIC_TeiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Tei1_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Tei1_Interrupt()_E");
    R_RIIC_TeiInterrupt(DEVDRV_CH_1);
TOPPERS_SYSLOG(LOG_INFO, "Tei1_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt hander processing to be executed when the RIIC
*              : channel 2 receive data full interrupt is accepted.
*              : The API function R_RIIC_RiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used.)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ri2_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ri2_Interrupt()_E");
    R_RIIC_RiInterrupt(DEVDRV_CH_2);
TOPPERS_SYSLOG(LOG_INFO, "Ri2_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC
*              : channel 2 transmit data empty interrupt is accepted.
*              : The API function R_RIIC_TiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ti2_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ti2_Interrupt()_E");
    R_RIIC_TiInterrupt(DEVDRV_CH_2);
TOPPERS_SYSLOG(LOG_INFO, "Ti2_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC 
*              : channel 2 transmit end interrupt is accepted.
*              : The API function R_RIIC_TeiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Tei2_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Tei2_Interrupt()_E");
    R_RIIC_TeiInterrupt(DEVDRV_CH_2);
TOPPERS_SYSLOG(LOG_INFO, "Tei2_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt hander processing to be executed when the RIIC
*              : channel 3 receive data full interrupt is accepted.
*              : The API function R_RIIC_RiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used.)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ri3_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ri3_Interrupt()_E");
    R_RIIC_RiInterrupt(DEVDRV_CH_3);
TOPPERS_SYSLOG(LOG_INFO, "Ri3_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC
*              : channel 3 transmit data empty interrupt is accepted.
*              : The API function R_RIIC_TiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Ti3_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Ti3_Interrupt()_E");
    R_RIIC_TiInterrupt(DEVDRV_CH_3);
TOPPERS_SYSLOG(LOG_INFO, "Ti3_Interrupt()_X");
}

/******************************************************************************
* Description  : An interrupt handler processing to be executed when the RIIC 
*              : channel 3 transmit end interrupt is accepted.
*              : The API function R_RIIC_TeiInterrupt is called by this function.
* Arguments    : uint32_t int_sense : Interrupt detection (Not used)
*              :                    :   INTC_LEVEL_SENSITIVE : Level sense
*              :                    :   INTC_EDGE_TRIGGER    : Edge trigger
* Return Value : none
******************************************************************************/
static void Tei3_Interrupt(uint32_t int_sense)
{
TOPPERS_SYSLOG(LOG_INFO, "Tei3_Interrupt()_E");
    R_RIIC_TeiInterrupt(DEVDRV_CH_3);
TOPPERS_SYSLOG(LOG_INFO, "Tei3_Interrupt()_X");
}

/******************************************************************************
* Description  : The INTC setting are executed.
* Arguments    : devdrv_ch_t ch
* Return Value : none
******************************************************************************/
static void set_Interrupt(devdrv_ch_t ch)
{
TOPPERS_SYSLOG(LOG_INFO, "set_Interrupt()_E");
	if ((uint32_t)ch < RIIC_CHANNEL_NUM)
    {
#if 0  //tokuyama@kmg 2016.09.23 割り込み登録は TOPPERS の作法に変更
        /* Register interrupt function */
        R_INTC_RegistIntFunc(userdef_riic_int[ch].intiicri_id, userdef_riic_int[ch].intiicri_func);
        R_INTC_RegistIntFunc(userdef_riic_int[ch].intiicti_id, userdef_riic_int[ch].intiicti_func);
        R_INTC_RegistIntFunc(userdef_riic_int[ch].intiictei_id, userdef_riic_int[ch].intiictei_func);
        /* Set interrupt priority */
        R_INTC_SetPriority(userdef_riic_int[ch].intiicri_id, RIIC_INTIICRI_PRI);
        R_INTC_SetPriority(userdef_riic_int[ch].intiicti_id, RIIC_INTIICTI_PRI);
        R_INTC_SetPriority(userdef_riic_int[ch].intiictei_id, RIIC_INTIICTEI_PRI);
		/* Enable interrupt */
        R_INTC_Enable(userdef_riic_int[ch].intiicri_id);
        R_INTC_Enable(userdef_riic_int[ch].intiicti_id);
        R_INTC_Enable(userdef_riic_int[ch].intiictei_id);
#else
		// 割り込み要求エッジ/レベル
		SOLID_INTC_SetIntConfig(userdef_riic_int[ch].intiicri_id, SOLID_INTC_CONFIG_EDGE_TRIGGERED);
		SOLID_INTC_SetIntConfig(userdef_riic_int[ch].intiicti_id, SOLID_INTC_CONFIG_EDGE_TRIGGERED);
		SOLID_INTC_SetIntConfig(userdef_riic_int[ch].intiictei_id, SOLID_INTC_CONFIG_LEVEL_SENSITIVE);
    // 割り込み enable だけをここで実行します。
        ena_int(userdef_riic_int[ch].intiicri_id);
        ena_int(userdef_riic_int[ch].intiicti_id);
        ena_int(userdef_riic_int[ch].intiictei_id);
#endif /* 0 */
	}
TOPPERS_SYSLOG(LOG_INFO, "set_Interrupt()_X");
}

/******************************************************************************
* Description  : The RIIC initial setting are executed.
* Arguments    : devdrv_ch_t ch
* Return Value : none
******************************************************************************/
static void set_riic_init(devdrv_ch_t ch)
{
    const userdef_riic_reg_t  * reg_addr;
TOPPERS_SYSLOG(LOG_INFO, "set_riic_init(%d)_E",ch);

    if ((uint32_t)ch < RIIC_CHANNEL_NUM)
    {
        reg_addr = &userdef_riic_reg[ch];

        /* SCL and SDA pins: Not driven */
        RZA_IO_RegWrite_8(reg_addr->reg_riicncr1, 0, RIICn_RIICnCR1_ICE_SHIFT, RIICn_RIICnCR1_ICE);
        /* RIIC reset */
        RZA_IO_RegWrite_8(reg_addr->reg_riicncr1, 1, RIICn_RIICnCR1_IICRST_SHIFT, RIICn_RIICnCR1_IICRST);
        /* Internal reset */
        RZA_IO_RegWrite_8(reg_addr->reg_riicncr1, 1, RIICn_RIICnCR1_ICE_SHIFT, RIICn_RIICnCR1_ICE);
        /* Clock source setting for the internal reference clock */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnmr1, RIIC_CKS, RIICn_RIICnMR1_CKS_SHIFT, RIICn_RIICnMR1_CKS);
        /* SCL clock Low width */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnbrl, RIIC_BRL, RIICn_RIICnBRL_BRL_SHIFT, RIICn_RIICnBRL_BRL);
        /* SCL clock High width */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnbrh, RIIC_BRH, RIICn_RIICnBRH_BRH_SHIFT, RIICn_RIICnBRH_BRH);
        /* Noise filter stages : 4 stages */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnmr3, 3, RIICn_RIICnMR3_NF_SHIFT, RIICn_RIICnMR3_NF);
        /* Transfer stop due to the NACK reception during transmission: Enabled */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnfer, 1, RIICn_RIICnFER_NACKE_SHIFT, RIICn_RIICnFER_NACKE);
        /* Digital noise-filter circuit: Used */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnfer, 1, RIICn_RIICnFER_NFE_SHIFT, RIICn_RIICnFER_NFE);
        /* Receive data full interrupt: Enabled */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnier, 1, RIICn_RIICnIER_RIE_SHIFT, RIICn_RIICnIER_RIE);
        /* Transmit data empty interrupt: Enabled */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnier, 1, RIICn_RIICnIER_TIE_SHIFT, RIICn_RIICnIER_TIE);
        /* Transmit end interrupt: Enabled */
        RZA_IO_RegWrite_8(reg_addr->reg_riicnier, 1, RIICn_RIICnIER_TEIE_SHIFT, RIICn_RIICnIER_TEIE);
        /* Clear internal reset */
        RZA_IO_RegWrite_8(reg_addr->reg_riicncr1, 0, RIICn_RIICnCR1_IICRST_SHIFT, RIICn_RIICnCR1_IICRST);
    }
TOPPERS_SYSLOG(LOG_INFO, "set_riic_init(%d)_X",ch);
}


/* End of File */

